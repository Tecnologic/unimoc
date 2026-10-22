# Zephyr Port Plan

Status (2026-10-21): direction accepted. The Zephyr application (`app/`),
the `native_sim` platform and the BCC platform code exist (see
`docs/zephyr_workflow.md`). The Cyphal/CAN node (Phase 5) runs on
`native_sim` and is built for the BCC; the flash settings backend and FDCAN3
are built but not run on the board. USB and the shell were dropped: the bench
log is on RTT and on the bus. The phase order below is the plan.

## Goals

- Zephyr is the primary OS. Logging, Cyphal, settings and startup run in
  Zephyr threads: Cyphal over CAN-FD is the operating interface (battery case
  controller, the cart and the simulation, `docs/system_plan.md`), the log
  goes to RTT and as `uavcan.diagnostic.Record` on the bus.
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
| Runtime scope | Zephyr is the primary OS. Only the FOC hot path runs at zero latency. Logging, Cyphal, settings and startup run in threads. |
| Operating interface | Cyphal/CAN-FD only (decided 2026-10-21). No shell, no USB console: registers, commands and telemetry over Cyphal, the bench log on RTT. USB comes back for DFU recovery. |
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
   interface existed. Resolved in Phase 5 (the old heartbeat serializer
   packed health and mode into one byte and was removed).
6. `lib/cymon` (own library, no canard dependency) is a submodule. Since
   cymon-lib 0.2 it is C++20 and builds with the UNIMOC warning set; the
   integration steps are in `docs/cymon_integration_plan.md`.
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
- Initialise `lib/cymon` (target `cymon`, alias `cymon::cymon`; `docs/cymon_integration_plan.md`).

### Phase 2 - native_sim
- Zephyr hardware layer implementing `HardwareInterface`, bound to the PMSM
  plant from `docs/software_in_loop_plan.md`.
- Fast loop driven from virtual time, not from a real interrupt.
- Settings via flash simulator (replaces the environment-variable file).
- Log on the console (stdout); Cyphal on SocketCAN and slcan pseudo
  terminals (Phase 5), so application code is identical on target and
  simulation.
- Default `native_sim` is 32-bit (needs multilib); `native_sim/native/64` is the
  fallback.

### Phase 3 - High-level services
- `HardwareSystemInterface::Log` maps to Zephyr logging.
- ~~Shell commands~~: dropped (2026-10-21); settings, status and commands
  are Cyphal registers and ExecuteCommand. Startup FSM and cymon follow over
  Cyphal.
- `SettingsStorage` key/value callbacks map to the Zephyr settings subsystem on ZMS (`docs/settings_plan.md`, implemented).
- Threads: Cyphal, logging, settings, later the 2 kHz control thread.
  No `main()` wait loop. The observer stays in its pended interrupt.

### Phase 4 - STM32G473 target
- Custom board with devicetree, no legacy code.
- Fast loop: ADC injected conversion (JEOC) and TIM8 via STM32 LL, connected
  with `IRQ_ZERO_LATENCY`. Spike required: the Zephyr ADC API is unlikely to
  cover injected conversions.
- Zero-latency IRQ rules: no kernel calls, no logging, no locks in the ISR path.
- Logging over SEGGER RTT. ~~USB CDC-ACM console~~ dropped; USB returns
  for DFU recovery.

### Phase 5 - Cyphal over CAN-FD (implemented on native_sim, built for the BCC)
- Done: libcanard 4.0 with O1Heap, Nunavut 2.3 C serialization generated at
  configure time (`lib/cyphal/CMakeLists.txt`), all pinned in `west.yml` and
  `scripts/requirements-cyphal.txt`.
- Done: `CanardTransport : cyphal::Transport` over `cyphal::CanMedia`; the
  Zephyr CAN API media (`hardware/zephyr_common/zephyr_can_media.cpp`) serves
  the BCC (FDCAN3) and native_sim (SocketCAN `vcan0`). Redundant interfaces
  with their own MTU.
- Done: heartbeat, port list, GetInfo, register List/Access on
  `SettingsStore` (all 77 persistent registers), ExecuteCommand (restart,
  store, factory reset, emergency stop), PnP v1, `uavcan.diagnostic.Record`
  with a Zephyr log backend; drive application in `src/drive/` (readiness,
  current and torque setpoints, telemetry).
- Done: native_sim without `vcan`: slcan on four pseudo terminals
  (`hardware/zephyr_sim/slcan_media.cpp`), checked with yakut 0.14.
- Open: cymon traces over Cyphal (`docs/cymon_integration_plan.md`); firmware update (file service into bank
  2); speed and position setpoints (need the control thread, M7); the startup
  FSM registers; the board run (transceiver, bit timing).

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

- native_sim is Linux only. Whether the default WSL2 kernel provides `vcan` is
  unverified. Default path: `vcan` for Cyphal/CAN-FD; without it the slcan
  pseudo terminals of the simulation (classic CAN, no kernel module).
  Cyphal/UDP over TAP is no longer needed for a single node.
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
