# Zephyr Port Plan

Status (2026-10-19): direction accepted. The Zephyr application (`app/`),
the `native_sim` platform and the BCC platform code exist (see
`docs/zephyr_workflow.md`). The Cyphal stack does not exist yet; the
flash settings backend and the BCC USB CDC-ACM console are built but not run on
the board. The phase order below is the plan.

## Goals

- Zephyr is the primary OS. Logging, shell, USB, Cyphal, settings and startup
  run in Zephyr threads: logging and shell over USB, Cyphal over CAN-FD
  (battery case controller and the cart, `docs/system_plan.md`).
- Only the FOC hot path runs in zero-latency IRQs: the sample ISR and the
  control cycle it runs every fourth sample. The observer runs in a
  software-pended normal-priority interrupt. Nothing else runs at zero latency.
- Run the simulation on the host with Zephyr `native_sim`.
- Primary host is Linux. Windows uses WSL2 for the simulation.
- Remove the legacy firmware completely.
- Keep the FOC core in `lib/` untouched. Controllers there are not final, but
  code quality and style (see `lib/units/units.hpp`) are the reference.
- C++20 is the baseline for all project code.

## Decisions

| Topic | Decision |
|---|---|
| Zephyr version | 4.4.0 (released 2026-04-14), pinned in `west.yml`. A 4.5.0 release was not confirmed as of 2026-10-19; upgrade later via the migration guide. |
| Legacy firmware | Removed: the modm firmware, `cmake/Modm.cmake`, `cmake/ToolChain.cmake`, `requirements.txt` and `.github/workflows/ci-firmware.yml` were deleted in `6ca28d1`. The last legacy release stays on `main` until `develop` is merged there. |
| Repository layout | Same repository. Zephyr application in `app/`, `lib/` consumed via `add_subdirectory`. |
| Language standard | C++20 (`CONFIG_STD_CPP20`). `AGENTS.md`, `README.md` and the root `CMakeLists.txt` say C++20. |
| Runtime scope | Zephyr is the primary OS. Only the FOC hot path runs at zero latency. Logging, shell, USB, Cyphal, settings and startup run in threads. |
| Hot-path layout | Decided: 32 kHz PWM, 16 kHz control cycle inside every fourth sample ISR, duty entry 0 into the CCR preload and entries 1 to 3 by DMA, observer in a software-pended normal-priority interrupt (`docs/hotpath_plan.md`). |

## Review Findings Driving the Plan

1. Build inversion: top-level `project()`, executable created in the hardware
   directory, sources added from `src/`. Zephyr needs `find_package(Zephyr)`
   before `project()` and the `app` target.
2. `project_warnings` (`-Werror`, `-Wconversion`, `-Wold-style-cast`) is linked
   into every library and would hit Zephyr headers and glue code. Scope it to
   project code and treat Zephyr includes as system headers.
3. `hardware/host` uses `std::thread`, `<chrono>`, `<fstream>`, an environment
   variable for the settings file and zero-value stubs.
4. `CurrentControlIsr` and `SlowUpdate` are singletons; `samples_ready` is a
   `volatile bool`. `OnControlCycle()` only calls hardware callbacks, which
   fits a zero-latency IRQ, but those callbacks must not log or call kernel APIs.
5. Cyphal: only identifiers, heartbeat serialization and the `Transport`
   interface exist. DSDL, register service, PnP and GetInfo are missing.
6. `lib/cymon` (own library, C++17, no canard dependency) is an uninitialised
   submodule; `add_subdirectory(cymon)` fails without it.
7. Small items: `option(HARDWARE_DIRECTORY ...)` misuse, platform-dependent
   `uint_fast8_t VERSION` in the battery case config, stale skill path in
   `AGENTS.md`, empty `src/logging/CMakeLists.txt`.

## Phases

### Phase 1 - Build foundation
- Add `west.yml` (Zephyr 4.4.0) and `app/` with `CMakeLists.txt`, `prj.conf`,
  Kconfig and board overlays.
- `find_package(Zephyr)` before `project()`; link `lib/` targets into `app`.
- Tests are ztest suites under twister on native_sim (the hosted GoogleTest
  build was removed).
- Scope warnings to project code; mark Zephyr includes as system.
- Initialise or vendor `lib/cymon` and verify the CMake target names.

### Phase 2 - native_sim
- Zephyr hardware layer implementing `HardwareInterface`, bound to the PMSM
  plant from `docs/software_in_loop_plan.md`.
- Fast loop driven from virtual time, not from a real interrupt.
- Settings via flash simulator (replaces the environment-variable file).
- Log and shell on UART (PTY or stdin/stdout) via devicetree `chosen` nodes, so
  application code is identical on target and simulation.
- Default `native_sim` is 32-bit (needs multilib); `native_sim/native/64` is the
  fallback.

### Phase 3 - High-level services
- `HardwareSystemInterface::Log` maps to Zephyr logging.
- Shell commands: settings, startup FSM, status, cymon control.
- `SettingsStorage` key/value callbacks map to the Zephyr settings subsystem on ZMS (`docs/settings_plan.md`, implemented).
- Threads: Cyphal, shell, logging, settings, later the 2 kHz control thread.
  No `main()` wait loop. The observer stays in its pended interrupt.

### Phase 4 - STM32G473 target
- Custom board with devicetree, no legacy code.
- Fast loop: ADC injected conversion (JEOC) and TIM8 via STM32 LL, connected
  with `IRQ_ZERO_LATENCY`. Spike required: the Zephyr ADC API is unlikely to
  cover injected conversions.
- Zero-latency IRQ rules: no kernel calls, no logging, no locks in the ISR path.
- Logging and shell over USB CDC-ACM using the current USB device stack.
  RTT stays optional.

### Phase 5 - Cyphal over CAN-FD
- libcanard 4.x (supports CAN FD; needs a constant-time allocator such as
  O1Heap) plus Nunavut-generated DSDL.
- `CanardTransport : cyphal::Transport` on the Zephyr CAN API.
- Node-ID/PnP, register service to `SettingsStore`, heartbeat, GetInfo.
- cymon traces over Cyphal services; `Tick()` from a timer or task.
- native_sim: SocketCAN snippet (`socketcan-native-sim`) with `vcan0`; CAN FD
  needs a larger MTU on the vcan interface.

### Phase 6 - Cyphal/UDP (simulation fallback only)
- The cart uses Cyphal/CAN-FD and an STM32G4 logic board; an STM32H7 with
  Single Pair Ethernet is a later option (`docs/system_plan.md`, section 6.5).
- libudpard 2.x over Zephyr sockets, only if `vcan` is not available for the
  multi-node simulation. Its upstream is a v3.0 alpha redesign, so pin a
  version deliberately.
- native_sim: TAP driver (`eth_native_tap`) with Zephyr's IP stack. The
  offloaded sockets (NSOS) of 4.4 pass no multicast socket options, which
  Cyphal/UDP needs.

### Phase 7 - CI and documentation
- Done: twister (native_sim run, STM32G4 build) with clang-tidy as Zephyr
  SCA and a clang-format check in CI (`.github/workflows/ci-zephyr.yml`).
- WSL2 instructions for Windows; update `README.md` and `AGENTS.md`.

## Risks and Open Points

- native_sim is Linux only. Whether the default WSL2 kernel provides `vcan` and
  USB/IP modules is unverified. Default path: UART PTY for the console and
  `vcan` for Cyphal/CAN-FD; Cyphal/UDP over TAP if `vcan` is missing; USB as
  an optional extra.
- C++20 with `std::atomic` and `<span>` under Zephyr's C++ library settings
  needs a short spike.
- ZLI constraints on the callbacks behind `HardwareInterface`.
- Zephyr 4.4 -> 4.5 upgrade: expected to be a migration-guide exercise; no
  4.5 feature is currently required.
- **CAN-FD on the BCC:** the transceiver is an SN65HVD230 (classic CAN, no CAN
  FD). Phase 5 needs a CAN-FD transceiver on the board, or it runs at classic
  CAN rates.
- **Observer deadline:** the observer and later the Hall fusion must finish
  within one control cycle (62.5 µs) below the sample ISR, see the budget in
  `docs/hotpath_plan.md` section 4. `state.observer_overruns` counts misses.
