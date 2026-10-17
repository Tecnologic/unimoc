# Zephyr Port Plan

Status: proposal. Architecture and phase order are open for discussion.

## Goals

- Move all high-level functionality to Zephyr: logging and shell over USB,
  Cyphal over CAN-FD (battery case controller), later Cyphal/UDP on an STM32H7.
- Run the simulation on the host with Zephyr `native_sim`.
- Primary host is Linux. Windows uses WSL2 for the simulation.
- Remove modm completely.
- Keep the FOC core in `lib/` untouched. Controllers there are not final, but
  code quality and style (see `lib/units/units.hpp`) are the reference.
- C++20 is the baseline for all project code.

## Decisions

| Topic | Decision |
|---|---|
| Zephyr version | 4.4.0, pinned in `west.yml`. 4.5 is not released yet (working draft, planned for October 2026); upgrade later via the migration guide. |
| modm | Removed. `hardware/batterycasecontroller/modm`, `cmake/Modm.cmake`, `requirements.txt` and `cmake/ToolChain.cmake` go away once the Zephyr target builds. |
| Repository layout | Same repository. Zephyr application in `app/`, `lib/` consumed via `add_subdirectory`. |
| Language standard | C++20 (`CONFIG_STD_CPP20`). Align `CMakeLists.txt` and `AGENTS.md`, which currently say C++23. |

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
   `volatile bool`. `on_jeoc()` only calls hardware callbacks, which fits a
   zero-latency IRQ, but those callbacks must not log or call kernel APIs.
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
- Keep the hosted GoogleTest preset as the fast CI path.
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
- `SettingsStorage` callbacks map to NVS/ZMS.
- Threads: slow update, Cyphal, shell. No `main()` wait loop.

### Phase 4 - STM32G473 target
- Custom board with devicetree, no modm.
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

### Phase 6 - Cyphal/UDP on STM32H7 (later)
- libudpard over Zephyr sockets. Its current documentation describes a v3.0
  alpha redesign, so pin a version deliberately.
- native_sim: offloaded sockets over loopback, no root required.
- Sets up the Python or Rust GUI from `TODO.md`.

### Phase 7 - CI and documentation
- Hosted GoogleTest, native_sim run and STM32G4 build in CI.
- WSL2 instructions for Windows; update `README.md` and `AGENTS.md`.

## Risks and Open Points

- native_sim is Linux only. Whether the default WSL2 kernel provides `vcan` and
  USB/IP modules is unverified. Default path: UART PTY and UDP loopback; CAN and
  USB as optional extras.
- C++20 with `std::atomic` and `<span>` under Zephyr's C++ library settings
  needs a short spike.
- ZLI constraints on the callbacks behind `HardwareInterface`.
- Zephyr 4.4 -> 4.5 upgrade: expected to be a migration-guide exercise; no
  4.5 feature is currently required.
