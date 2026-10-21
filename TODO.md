# TODO list for open topics

The overall direction (cart with four wheel drives, central vehicle dynamics
controller, Cyphal/CAN-FD, STM32G4 logic board for the 4850 drive) is drafted
in [docs/system_plan.md](docs/system_plan.md).

### Cart system (see `docs/system_plan.md`)

 - [ ] CAN FD bus test (phase C2): BCC and G4 evaluation boards on a cart-length harness, 1/4 and 1/2 Mbit/s, bus load and latency at 250 and 500 Hz; choose the bit rate and the cycle rate.
 - [ ] Check `vcan` (CAN FD MTU) on WSL2 for the multi-node simulation; fallback Cyphal/UDP over the `native_sim` TAP driver (NSOS passes no multicast socket options in 4.4).
 - [ ] Regenerative braking in the drive: DC-link overvoltage limit and trip, regen limit from the battery state, reported to the VDC; tested in the sim (section 7.1).
 - [ ] Requirements and part choice for the STM32G4 redesign of the hardware_4850 logic board (STM32G473RET6 or G474RET6, pin and ADC check, section 5.2).
 - [ ] Generalize `hardware/zephyr_bcc` to the G4 family so the 4850 logic board shares the hot path (board differences in the devicetree).
 - [ ] Split `app/` into `apps/drive/` and `apps/vdc/` (later `apps/pedal/`) and add `lib/vehicle/`; update `AGENTS.md`, `README.md` and `docs/zephyr_workflow.md`. The VDC firmware lives in this repository (decided); the VDC MCU starts on a G4.
 - [ ] Define the DSDL namespace (wheel command, drive status, pedal state) with every message in one CAN FD frame.
 - [ ] Decide whether a brake chopper is needed for braking with a full or disconnected battery.

## Zephyr port and hot path

See [docs/zephyr_port_plan.md](docs/zephyr_port_plan.md) for the decisions and
phases, [docs/zephyr_workflow.md](docs/zephyr_workflow.md) for the build and
interrupt layout, and [docs/hotpath_plan.md](docs/hotpath_plan.md) for the
hot-path design.

There is no board access at the moment. The open items are ordered so that
work verifiable with twister on `native_sim` and the BCC build comes first;
items that need the board are collected below and wait for hardware access.

### P0 - Without hardware (twister on `native_sim`, BCC build)

 - [ ] Implement the hot-path milestones M4 to M7 from `docs/hotpath_plan.md` (observer API, Hall driver and sim Hall signals, Hall fusion, control thread), tested with ztest and the sim plant.
 - [ ] Run a small Cyphal feasibility spike between `native_sim` and `pycyphal`, on `vcan` (CAN FD) with UDP over TAP as the fallback (see the SIL plan below); it settles the DSDL generation for both transports.
 - [ ] Integrate Cyphal (libcanard, DSDL, register service, PnP, heartbeat, GetInfo) and the shell command `settings show|save|reset` (settings open items 1 and 2), tested on `native_sim`. CAN-FD on `native_sim` needs `vcan` in the WSL2 kernel (unverified); CAN-FD on the BCC waits for the transceiver decision.
 - [ ] Write the M0 instrumentation (DWT cycle counters around the sample ISR, control cycle and observer, min/mean/max, overrun counters, output through log and shell); build-tested only until the board is available.
 - [ ] Check whether the hot path (sample ISR, control cycle, buffers) belongs in CCM SRAM on the G4 (`docs/hotpath_plan.md`, "Risks and Open Points"); decide with the M0 timing numbers.
 - [ ] Confirm the CAN-FD transceiver change for the BCC (the board has an SN65HVD230, classic CAN). A board decision, no board access needed.

### P1 - Blocked on hardware access

 - [ ] M0 on the board: timing numbers for the sample ISR, control cycle and observer.
 - [ ] M8 first step with the outputs off: DMA burst, preload, start at ARR/2, parity detection and ADC alignment on the scope; check the USB console (enumeration, CRS lock of the HSI48) and pick a UNIMOC USB VID/PID instead of the Zephyr test IDs.
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
 - [x] Bring `src/`, `hardware/zephyr_*` and the tests under clang-tidy (Zephyr SCA through CodeChecker, `scripts/twister_sca.sh`).
 - [x] Bring the `lib/unimoc` headers under clang-tidy (`HeaderFilterRegex`; API renamed to PascalCase methods). `lib/cymon` is still outside: it is not built in the Zephyr workspace yet.
 - [x] Add an `SPDX-License-Identifier` line to the UNIMOC file header (C++ files, CMake and YAML files; file-docs skill updated).
 - [x] Merge `main` into `develop` (`542c501`): the modm and hosted-build files stay deleted, `lib/gitversion` is dropped (the GetInfo version source is part of the Cyphal item), and `hardware/HardwareLimits.hpp` is replaced by the BCC settings profile. Its legacy BCC values (100 A phase, 80 A motor, 50 A battery drive, 20 A charge) were checked against the profile in `hardware/zephyr_bcc/hardware_interface.cpp`: the profile keeps the lower 60 A phase trip and 50 A motor limit, the battery limits are equal. Closing `copilot/fix-pr-10-from-main-to-develop` is left to the repository owner.
 - [x] Settings follow-ups: `PowerStageInterface::Enable()` is held off while a flash save runs, and `SettingsStore` takes a platform lock (Zephyr `k_mutex`) around every operation ([docs/settings_plan.md](docs/settings_plan.md), "Concurrency").
 - [x] Remove the stale references to the deleted legacy firmware `hardware/batterycasecontroller/` (`AGENTS.md`, `docs/zephyr_port_plan.md`, the file-docs skill, `scripts/check_format.sh`, `scripts/codechecker_skip.txt`, `docs/Doxyfile`).
 - [x] Enable the USB CDC-ACM console and Zephyr logging in the BCC application (`app/boards/bcc.conf`, `bcc.overlay`; HSI48 with CRS on the USB SOF; log also on RTT). Build only; link limited to the code partition (`CONFIG_USE_DT_CODE_PARTITION`).

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

 - [ ] Run a small Cyphal/UDP feasibility spike between `native_sim` and `pycyphal` on localhost (WSL2).
 - [ ] Select the C++ transport and DSDL generation approach for registers, setpoints, telemetry, and heartbeat.
 - [ ] Confirm one register exchange and one telemetry subject before designing the full simulator protocol.

### P2 - Add coordinated multi-motor simulation

 - [ ] Add a one-process simulator coordinator with N independent motor nodes and one shared virtual clock.
 - [ ] Define a barriered multi-motor step with coherent timestamps and deterministic ordering.
 - [ ] Add independent constant/viscous load models and tests for isolated motors, unique node IDs, reset, and repeatability.
 - [ ] Add a shared load interface suitable for shaft, wheel, battery, differential, and torque-vectoring models.

### P2 - Expose the simulator over Cyphal/UDP

 - [ ] Build a host simulator executable around the coordinator.
 - [ ] Map standard Cyphal register operations to the settings operation boundary.
 - [ ] Route torque, speed, position, and excitation setpoints into the normal controller input path.
 - [ ] Publish telemetry and heartbeat with explicit virtual-time semantics.
 - [ ] Add simulation-control commands for reset, fixed-cycle advance, run-until, snapshot, and clock status.
 - [ ] Add protocol and localhost end-to-end tests for single- and multi-node operation.

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