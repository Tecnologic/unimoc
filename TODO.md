# TODO list for open topics

The overall direction (cart with four wheel drives, central vehicle dynamics
controller, Cyphal/CAN-FD, STM32G4 logic board for the 4850 drive) is drafted
in [docs/system_plan.md](docs/system_plan.md).

### Cart system (see `docs/system_plan.md`)

 - [ ] CAN FD bus test (phase C2): BCC and G4 evaluation boards on a cart-length harness, 1/4 and 1/2 Mbit/s, bus load and latency at 250 and 500 Hz; choose the bit rate and the cycle rate.
 - [ ] Check `vcan` (CAN FD MTU) on WSL2 for the multi-node simulation. A single simulated node already works without it over the slcan pseudo terminals (classic CAN); several nodes on one bus need `vcan` (or Cyphal/UDP over the `native_sim` TAP driver; NSOS passes no multicast socket options in 4.4).
 - [ ] Regenerative braking in the drive: DC-link overvoltage limit and trip, regen limit from the battery state, reported to the VDC; tested in the sim (section 7.1).
 - [ ] Requirements and part choice for the STM32G4 redesign of the hardware_4850 logic board (STM32G473RET6 or G474RET6, pin and ADC check, section 5.2).
 - [ ] Generalize `hardware/zephyr_bcc` to the G4 family so the 4850 logic board shares the hot path (board differences in the devicetree).
 - [ ] Split `app/` into `apps/drive/` and `apps/vdc/` (later `apps/pedal/` and `apps/bms/`) and add `lib/vehicle/`; update `AGENTS.md`, `README.md` and `docs/zephyr_workflow.md`. The VDC firmware lives in this repository (decided); the VDC MCU starts on a G4.
 - [ ] Define the cart messages in the DSDL namespace `dsdl/unimoc/` (wheel command, drive status, pedal state, battery status), every message in one CAN FD frame. The namespace exists with `unimoc.drive.CurrentSetpoint`, `unimoc.drive.Telemetry` and `unimoc.sim.Plant`.
 - [ ] BMS controller with brake chopper as `apps/bms/` with `lib/battery/` (section 5.5): board (BCC or new G4 board), cell monitoring front end, chopper in hardware (G4 comparator to timer), resistor sizing, battery status message.

## Zephyr port and hot path

See [docs/zephyr_port_plan.md](docs/zephyr_port_plan.md) for the decisions and
phases, [docs/zephyr_workflow.md](docs/zephyr_workflow.md) for the build and
interrupt layout, and [docs/hotpath_plan.md](docs/hotpath_plan.md) for the
hot-path design.

There is no board access at the moment. The open items are ordered so that
work verifiable with twister on `native_sim` and the BCC build comes first;
items that need the board are collected below and wait for hardware access.

### P0 - Without hardware (twister on `native_sim`, BCC build)

 - [ ] Connect the startup FSM (`HwStartup`) to Cyphal: the `unimoc.startup.*` registers of `cyphal_interface.hpp` (run, step, external measurements, results) in a register provider, the FSM stepped from the Cyphal thread; test on `native_sim`.
 - [ ] Speed and position setpoints over Cyphal (`uavcan.sub.speed_setpoint`, UDRAL servo); needs the control thread of M7.
 - [ ] cymon traces over Cyphal: steps in [docs/cymon_integration_plan.md](docs/cymon_integration_plan.md). `lib/cymon` is at cymon-lib 0.2 (C++20, interrupt-safe `Tick()`, DSDL `cymon.*.0.1`, CMake 3.20); the host tool is cymon (<https://github.com/Tecnologic/cymon>).
 - [ ] Firmware update over Cyphal: `ExecuteCommand BEGIN_SOFTWARE_UPDATE`, `uavcan.file.Read` into flash bank 2, bootloader swap; USB DFU as recovery.
 - [ ] Simulation-control commands over Cyphal for deterministic runs (reset, fixed-cycle advance, run-until; SIL P2); today the simulation runs in real time.
 - [ ] Live tuning while engaged: register writes reach the controller only by a runtime restart while disengaged (`docs/settings_plan.md`, open item 1).
 - [ ] Write the M0 instrumentation (DWT cycle counters around the sample ISR, control cycle and observer, min/mean/max, overrun counters, output through the log and Cyphal registers); build-tested only until the board is available.
 - [ ] Check whether the hot path (sample ISR, control cycle, buffers) belongs in CCM SRAM on the G4 (`docs/hotpath_plan.md`, "Risks and Open Points"); decide with the M0 timing numbers.
 - [ ] Confirm the CAN-FD transceiver change for the BCC (the board has an SN65HVD230, classic CAN). A board decision, no board access needed.

### P1 - Blocked on hardware access

 - [ ] M0 on the board: timing numbers for the sample ISR, control cycle and observer.
 - [ ] M8 first step with the outputs off: DMA burst, preload, start at ARR/2, parity detection and ADC alignment on the scope.
 - [ ] Cyphal on the BCC: FDCAN3 bit timing at 1 Mbit/s with CAN FD frames through the SN65HVD230, the Rs pin, heartbeat, registers and the RTT log with a USB CAN FD adapter and yakut; plug-and-play with `yakut monitor --plug-and-play`.
 - [ ] Verify ZMS, the flash stall, the dual-bank option bit (DBANK, bank 2 free for Cyphal firmware updates) and the size of the settings partition on the BCC board (settings plan, "Not verified on hardware").
 - [ ] Configure the duty DMA from the devicetree (`dmas` of `foc0`, Zephyr DMA API at init) instead of the fixed DMA1 channel 1 / DMAMUX request 53, and enable its clock through `clock_control` instead of writing `RCC->AHB1ENR`. The zero-latency path keeps reading the channel counter directly. Do this after the M8 first step, so the fixed configuration is a verified reference.
 - [ ] M8 remainder: Hall table, closed current loop with the 50 A limit.
 - [ ] Merge `develop` into `main` as the first Zephyr release (tag it) once the board items above pass; `main` still holds the legacy firmware.

### Done

 - [x] Decide the hot-path layout: 32 kHz PWM, 16 kHz control cycle in the sample ISR, duties by CCR preload plus DMA (variant B), observer in a software-pended normal-priority interrupt (`docs/hotpath_plan.md`).
 - [x] Align the root `CMakeLists.txt` with the C++20 baseline.
 - [x] Replace the RTT wording in the comment of `src/startup/startup_results.hpp` with the Zephyr log.
 - [x] Persist settings per register with the Zephyr settings subsystem on ZMS, live RAM tuning and a gated `SaveToFlash()` ([docs/settings_plan.md](docs/settings_plan.md)).
 - [x] Delete the legacy firmware in `hardware/batterycasecontroller/` together with `cmake/Modm.cmake`, `cmake/ToolChain.cmake`, `requirements.txt` and `ci-firmware.yml`.
 - [x] Hot-path milestones M1 to M3 in code (not run on the board).
 - [x] Hot-path milestones M4 to M7 in code (observer API, Hall driver and sim Hall signals, Hall fusion, control thread with speed and position control), tested with ztest and the sim plant; the Hall capture of the BCC is not run on the board.
 - [x] Bring `src/`, `hardware/zephyr_*` and the tests under clang-tidy (Zephyr SCA through CodeChecker, `scripts/twister_sca.sh`).
 - [x] Bring the `lib/unimoc` headers under clang-tidy (`HeaderFilterRegex`; API renamed to PascalCase methods). `lib/cymon` is still outside: it is not built in the Zephyr workspace yet.
 - [x] Add an `SPDX-License-Identifier` line to the UNIMOC file header (C++ files, CMake and YAML files; file-docs skill updated).
 - [x] Merge `main` into `develop` (`542c501`): the modm and hosted-build files stay deleted, `lib/gitversion` is dropped (the GetInfo version source is part of the Cyphal item), and `hardware/HardwareLimits.hpp` is replaced by the BCC settings profile. Its legacy BCC values (100 A phase, 80 A motor, 50 A battery drive, 20 A charge) were checked against the profile in `hardware/zephyr_bcc/hardware_interface.cpp`: the profile keeps the lower 60 A phase trip and 50 A motor limit, the battery limits are equal. Closing `copilot/fix-pr-10-from-main-to-develop` is left to the repository owner.
 - [x] Settings follow-ups: `PowerStageInterface::Enable()` is held off while a flash save runs, and `SettingsStore` takes a platform lock (Zephyr `k_mutex`) around every operation ([docs/settings_plan.md](docs/settings_plan.md), "Concurrency").
 - [x] Remove the stale references to the deleted legacy firmware `hardware/batterycasecontroller/` (`AGENTS.md`, `docs/zephyr_port_plan.md`, the file-docs skill, `scripts/check_format.sh`, `scripts/codechecker_skip.txt`, `docs/Doxyfile`).
 - [x] ~~Enable the USB CDC-ACM console~~ Superseded (2026-10-21): USB and the shell were dropped; the bench log is on RTT and on the bus. Link limited to the code partition (`CONFIG_USE_DT_CODE_PARTITION`).
 - [x] Cyphal over CAN-FD as the primary interface (2026-10-21): libcanard 4.0 + O1Heap + Nunavut DSDL pinned in `west.yml`; heartbeat, port list, GetInfo, register List/Access on the settings store (77 registers), ExecuteCommand (restart, store, factory reset, emergency stop), PnP v1, diagnostic records from the Zephyr log; drive application (readiness, current and torque setpoints with timeouts and limits, telemetry); shared between BCC (FDCAN3) and `native_sim` (SocketCAN plus slcan pseudo terminals, plant registers and plant trace). Tests `unimoc.lib.cyphal`, `unimoc.sim.cyphal_drive`; checked with yakut 0.14 against the simulation. Not run on the board.
 - [x] pycyphal integration tests of the running simulation (`tests/integration/cyphal_sim`, twister pytest harness, scenario `unimoc.app.cyphal_integration`): node services, registers, arming, setpoints, telemetry, diagnostics, simulation registers, store and restart.
 - [ ] Extend the pycyphal integration tests with every new bus feature (startup FSM registers, speed setpoints, multi-node runs on `vcan` once available in CI).
 - [x] Fix the ExecuteCommand status codes in `cyphal_interface.hpp` (BAD_COMMAND and BAD_STATE were swapped against the specification) and remove the heartbeat serializer that packed health and mode into one byte.
 - [x] `app/boards/native_sim.*` were never applied to the 64-bit build: renamed to `native_sim_native_64.*` (1 us tick, real time).

## Prioritized software-in-the-loop plan

See [docs/software_in_loop_plan.md](docs/software_in_loop_plan.md) for the
architecture, design decisions, implementation phases, risks, and acceptance
criteria.

### P0 - Establish the deterministic runtime

 - [x] Define and test the virtual-time cycle contract around `CurrentControlIsr::OnControlCycle()` and `SlowUpdate::RunOnce()` (`tests/sim/irq_cadence`).
 - [x] Introduce injectable ADC/PWM I/O for the current-control runtime while preserving the target hardware implementation.
 - [x] Extract the current-control runtime into a reusable target independent of firmware `main.cpp` (`current_control::StartRuntime()` in `src/current_control/current_control_runtime.hpp`).
 - [x] Add a HAL-free virtual clock and simulation runtime harness with reset, fast-step, slow-update, advance, and snapshot operations (`hardware/zephyr_sim/sim_control.hpp`: `Reset`, `StepCycle`, `AdvanceCycles`, `VirtualTimeNs`, `TakeSnapshot`).

### P1 - Build the single-motor vertical slice

 - [x] Implement a validated PMSM plant model using `NvmSettings`, existing frame types, and unit types (`lib/sim/`, `MotorSimulation`, with 120 degree Hall sensors).
 - [x] Add plant tests for electrical dynamics, torque direction, pole-pair scaling, mechanics, friction, parameter validation, and repeatability (`lib/sim/tests/`).
 - [x] Connect one host runtime and its ISR I/O to the PMSM plant instead of the zero-value host stubs (`hardware/zephyr_sim`, native_sim).
 - [x] Add a deterministic end-to-end C++ step-response test through the actual current-control runtime (`tests/sim/closed_loop`).
 - [x] Add exact fast-loop/slow-loop cadence tests and finite-state checks for NaN/Inf and unstable output (`tests/sim/closed_loop`).

### P1 - Resolve the Cyphal transport path

 - [x] Feasibility: `native_sim` with pycyphal/yakut over Cyphal/CAN, SocketCAN `vcan0` or the slcan pseudo terminals (no UDP needed for one node).
 - [x] Transport and DSDL: libcanard 4.0 (C) behind `cyphal::Transport`, Nunavut C serialization.
 - [x] Register exchange and telemetry subject confirmed with yakut (register list/read/write, `unimoc.drive.Telemetry`, `unimoc.sim.Plant`, diagnostic records).

### P2 - Add coordinated multi-motor simulation

 - [ ] Add a one-process simulator coordinator with N independent motor nodes and one shared virtual clock.
 - [ ] Define a barriered multi-motor step with coherent timestamps and deterministic ordering.
 - [ ] Add independent constant/viscous load models and tests for isolated motors, unique node IDs, reset, and repeatability.
 - [ ] Add a shared load interface suitable for shaft, wheel, battery, differential, and torque-vectoring models.

### P2 - Expose the simulator over Cyphal (CAN)

 - [ ] Build a host simulator executable around the coordinator.
 - [x] Map standard Cyphal register operations to the settings operation boundary.
 - [ ] Route torque, speed, position, and excitation setpoints into the normal controller input path (torque and d/q current done; speed, position and excitation open).
 - [ ] Publish telemetry and heartbeat with explicit virtual-time semantics (published in real time today; the telemetry carries the control-cycle count).
 - [ ] Add simulation-control commands for reset, fixed-cycle advance, run-until, snapshot, and clock status.
 - [ ] Add protocol and localhost end-to-end tests for single- and multi-node operation (single node done: `unimoc.app.cyphal_integration`).

### P3 - Create the Python command-and-control GUI

 - [ ] Create a separate Python simulation package and dependency file; keep firmware `requirements.txt` focused on firmware tooling.
 - [ ] Implement a `pycyphal` client for node discovery, registers, setpoints, telemetry, reset, and virtual-time stepping.
 - [ ] Build a PySide6/pyqtgraph GUI for single-motor step responses and virtual-time control.
 - [ ] Add scenario definitions, run/pause/reset/step controls, CSV/JSON trace export, and replay.
 - [ ] Keep network I/O and long simulation batches outside the UI thread; support telemetry decimation.
 - [ ] Add headless Python protocol and smoke tests against the host simulator.

### P4 - Expand fidelity and application scope

 - [ ] Add ASM and EESM plants behind the common plant contract (`MotorModel` concept and the `MotorSimulation::Model` variant in `lib/sim/`).
 - [ ] Add startup, observer, field-weakening, homing, and control-mode transition scenarios.
 - [ ] Add vehicle-level models for shared battery/DC-link behavior, wheels, differential load, road load, and torque vectoring.
 - [ ] Add fault injection, profiling, and large-N motor trace tooling.
 - [ ] Consider process-per-motor hosting only after the one-process coordinator is stable and a concrete isolation need exists.

### Validation and documentation

 - [ ] Add twister scenarios (ztest on `native_sim`) for the coordinator and protocol tests as they appear; plant and runtime tests exist in `tests/lib/sim` and `tests/sim/`.
 - [ ] Keep the focused three-phase validation green; ask before widening the validated scope in `AGENTS.md` to the SIL scenarios (`tests/sim/`, `lib/sim`).
 - [ ] Document simulator setup, launch commands, supported model fidelity, and GUI workflows in `README.md`.