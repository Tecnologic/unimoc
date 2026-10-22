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

 - [x] Settings apply on running hardware: `StartRuntime()` calls `HardwareInterface::Stop()` before it rewrites the control state; on the BCC `foc_stm32g4::Stop()` disables the sample and observer interrupts, stops TIM8 and the duty DMA, stops the injected sequences (JADSTP) and disables the ADCs (ADDIS), and `Initialize()` calls it first (also after a failed init). ztest on `native_sim` (`sim_irq_cadence`), BCC build-tested; verify on the board with M8 (register write while disengaged, then engage).
 - [x] Reject settings with two ports on the same port-ID (`settings_store_internal::HasPortIdCollision()` in `Validate()`): subscriptions and publications on distinct subject-IDs, cymon services on distinct service-IDs; the cymon trigger may publish and subscribe one shared subject (same type, a node does not receive its own transfers). ztest in `unimoc.lib.settings`.
 - [x] Publish the thread-side writes the zero-latency interrupt reads with a defined order: `CurrentControlIsr::ForceDuty()` hands the duties over in a `Mailbox` before an atomic flag (release/acquire); `ReleaseForceDuty()` resets the PI before it clears the flag; the BCC overcurrent window (`SetOvercurrentWindow()`) is one packed atomic word. Not run on the board.
 - [x] Connect the startup FSM (`HwStartup`) to Cyphal: `drive::StartupRegisters` (run, step, external measurements, results), the FSM stepped in `DriveNode::Spin()` in the Cyphal thread; ztests in `sim_cyphal_drive`. The pycyphal test is open (see Validation).
 - [ ] Speed and position setpoints over Cyphal (`uavcan.sub.speed_setpoint`, UDRAL servo); needs the control thread of M7.
 - [x] cymon traces over Cyphal ([docs/cymon_integration_plan.md](docs/cymon_integration_plan.md)): cymon-lib 0.2 linked, `cymon.*` services and trigger event, `Tick()` in the control cycle, plant truth on `native_sim`; the host tool is cymon (<https://github.com/Tecnologic/cymon>).
 - [ ] Measure the cost of the trace `Tick()` in the control cycle on the BCC (M0, target < 1 us); move it to the observer interrupt if it is too slow.
 - [ ] Firmware update over Cyphal: `ExecuteCommand BEGIN_SOFTWARE_UPDATE`, `uavcan.file.Read` into flash bank 2, bootloader swap; USB DFU as recovery.
 - [ ] Simulation-control commands over Cyphal for deterministic runs (reset, fixed-cycle advance, run-until; SIL P2); today the simulation runs in real time.
 - [ ] Live tuning while engaged: register writes reach the controller only by a runtime restart while disengaged (`docs/settings_plan.md`, open item 1).
 - [x] Write the M0 instrumentation: `CONFIG_UNIMOC_HOT_PATH_TIMING` (off by default), `hardware::timing::Probe` (lock-free, writer the interrupt, reset at the next record), DWT cycles around the sample ISR of the boundary, the control cycle and the observer; min/mean/max, duty and observer overruns in the `unimoc.timing.*` registers. ztests on `native_sim` (counts only), BCC build scenario `unimoc.app.build.timing`; the numbers wait for the board (M0 on the board). No log output: the registers are read with yakut.
 - [ ] Check whether the hot path (sample ISR, control cycle, buffers) belongs in CCM SRAM on the G4 (`docs/hotpath_plan.md`, "Risks and Open Points"); decide with the M0 timing numbers.
 - [ ] Confirm the CAN-FD transceiver change for the BCC (the board has an SN65HVD230, classic CAN). A board decision, no board access needed.
 - [x] CI housekeeping (review 2026-10-22): `actions/checkout@v7`, `upload-artifact@v7` and `download-artifact@v8` (Node 24); `ci-docs.yml` only uploads the Doxygen HTML and no longer requests the Pages permissions.
 - [x] One angle convention for the bus: the electrical angle is [0, 2 pi) in `unimoc.drive.Telemetry`, `unimoc.sim.Plant` and the traces (`observer::WrapAnglePositive()`, no libm, usable in the control cycle).

### Unit types throughout the code base (static checks of the physics)

Review 2026-10-22: `lib/units/units.hpp` is used for the settings and the
interfaces, but most formulas unwrap it. Non-test code has about 265
`.Value()` calls in `lib/motor_control/src` (most in `asm_flux_observer`,
`position_controller`, `mechanical_observer`, `current_controller`, the frame
types), about 130 in `lib/settings`, about 80 in `src/` and `hardware/`, and
raw `float` members for physical quantities in `src/` (`CurrentControlState`,
`DriveNode`, `OuterLoop`, `HwStartup`). The root cause is the tag-per-quantity
design: every product or quotient needs a hand-written operator (about 60
today), so a formula without one falls back to raw floats and the compiler
checks nothing. `units.hpp` is in the validated scope (`AGENTS.md`): the
first three items change it and need the owner's go-ahead.

 - [ ] P0: Enforce the boundary in CI: a check (script in `check_format.sh` or a clang-tidy matcher) that counts `.Value()` and raw `unit::X{float}` construction per file outside an allowlist of boundary files (DSDL serialization, cymon float mirror, register encoding, HAL counts), fails when the count grows, and is ratcheted down as the items below land. Effort S.
 - [x] P0: Math helpers in `units.hpp` so typed code needs no escape: `Abs`, `Min`/`Max`, `IsFinite`, `Hypot` (no libm beyond `sqrtf`, an instruction with `-fno-math-errno`); `Clamp(Rep, Rep)` removed and its callers (`mtpa`, `excitation_controller`, `dead_time_compensation`, `field_weakening`) clamp with typed bounds. ztests in `tests/lib/units`. Sine and cosine of an `Angle` are `system::SinCos` (`sin_cos.hpp`), the angle wrap `observer::WrapAngle()`/`WrapAnglePositive()`. `Sqrt` of a squared quantity needs a squared type: it moves to the dimension-exponent item below.
 - [ ] P0 (decision): Replace the tags by dimension exponents (SI base dimensions plus the radian as its own dimension, so that rad/s and Hz stay distinct), keeping the public aliases (`Current`, `Voltage`, ...). Products and quotients are then derived by the compiler, including the controller gains and the observer coefficients that today have no type. The precondition for the items below to stay free of `.Value()`; zero runtime cost (same `float` layout). Effort L.
 - [ ] P1: Frames and transforms without unwrapping: `Stator`/`Rotor` rotation, Clarke/Park in `three_phase_system`, `hfi`, `dead_time_compensation`, `pmsm_flux_observer` (Unit x Ratio exists). Effort M.
 - [ ] P1: Typed controller gains: `kp`/`ki` of `current_controller` (V/A, V/(A s)), `speed_controller`, `position_controller`, `excitation_controller`; the integrators keep their output unit. Effort M.
 - [ ] P1: Observers in units: `mechanical_observer` (torque balance, `dt / J`), `asm_flux_observer` (27 escapes), `excitation_observer`, `field_weakening`, `mtpa`. Effort M.
 - [ ] P1: Application state in units: `CurrentControlState::dt_fast`/`dt_slow` (`Time`), `overcurrent_limit` (`Current`), `ForceDuty(float...)` (`Ratio`), `DriveNode::current_limit_` and `torque_constant_` and `OuterLoop::Parameters::torque_constant` (a torque-constant type, N m/A), `HwStartup` thresholds and results (offsets and noise in A, gains as `Ratio`; the `double` accumulators to `float` or a documented exception), `StartupRegisters::ext_current_`/`ext_vdc_`. Effort M.
 - [ ] P1: Hardware conversions typed at the source: `analog::CountsToCurrent`/`CountsToVoltage`/`CountsToTemperature` return units, the sim `Exceeds()` and the BCC `SetOvercurrentLimit()` compare units. Effort S.
 - [ ] P2: `settings_store_internal::Validate()` compares units with typed limits instead of `.Value()` (about 130 calls). Effort S.
 - [ ] P2: Typed test checks (`UNIMOC_EXPECT_NEAR` for units) and the tests migrated (about 520 `.Value()` calls); document the remaining boundaries (DSDL, cymon, registers) in `CONTEXT.md`. Effort M.

### P1 - Blocked on hardware access

 - [ ] M0 on the board: timing numbers for the sample ISR, control cycle and observer.
 - [ ] M8 first step with the outputs off: DMA burst, preload, start at ARR/2, parity detection and ADC alignment on the scope. Include a settings restart (a register write while disengaged, then engage): `foc_stm32g4::Stop()` and the re-init are not run on the board yet.
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
 - [x] Bring the `lib/unimoc` headers under clang-tidy (`HeaderFilterRegex`; API renamed to PascalCase methods). `lib/cymon` is checked too since it is built (`include/cymon`, `src/cymon`).
 - [x] Add an `SPDX-License-Identifier` line to the UNIMOC file header (C++ files, CMake and YAML files; file-docs skill updated).
 - [x] Merge `main` into `develop` (`542c501`): the modm and hosted-build files stay deleted, `lib/gitversion` is dropped (the GetInfo version source is part of the Cyphal item), and `hardware/HardwareLimits.hpp` is replaced by the BCC settings profile. Its legacy BCC values (100 A phase, 80 A motor, 50 A battery drive, 20 A charge) were checked against the profile in `hardware/zephyr_bcc/hardware_interface.cpp`: the profile keeps the lower 60 A phase trip and 50 A motor limit, the battery limits are equal. Closing `copilot/fix-pr-10-from-main-to-develop` is left to the repository owner.
 - [x] Settings follow-ups: `PowerStageInterface::Enable()` is held off while a flash save runs, and `SettingsStore` takes a platform lock (Zephyr `k_mutex`) around every operation ([docs/settings_plan.md](docs/settings_plan.md), "Concurrency").
 - [x] Remove the stale references to the deleted legacy firmware `hardware/batterycasecontroller/` (`AGENTS.md`, `docs/zephyr_port_plan.md`, the file-docs skill, `scripts/check_format.sh`, `scripts/codechecker_skip.txt`, `docs/Doxyfile`).
 - [x] ~~Enable the USB CDC-ACM console~~ Superseded (2026-10-21): USB and the shell were dropped; the bench log is on RTT and on the bus. Link limited to the code partition (`CONFIG_USE_DT_CODE_PARTITION`).
 - [x] Cyphal over CAN-FD as the primary interface (2026-10-21): libcanard 4.0 + O1Heap + Nunavut DSDL pinned in `west.yml`; heartbeat, port list, GetInfo, register List/Access on the settings store (77 registers), ExecuteCommand (restart, store, factory reset, emergency stop), PnP v1, diagnostic records from the Zephyr log; drive application (readiness, current and torque setpoints with timeouts and limits, telemetry); shared between BCC (FDCAN3) and `native_sim` (SocketCAN plus slcan pseudo terminals, plant registers and plant trace). Tests `unimoc.lib.cyphal`, `unimoc.sim.cyphal_drive`; checked with yakut 0.14 against the simulation. Not run on the board.
 - [x] pycyphal integration tests of the running simulation (`tests/integration/cyphal_sim`, twister pytest harness, scenario `unimoc.app.cyphal_integration`): node services, registers, arming, setpoints, telemetry, diagnostics, simulation registers, store and restart.
 - [ ] Extend the pycyphal integration tests with every new bus feature (startup FSM registers, speed setpoints, multi-node runs on `vcan` once available in CI).
 - [x] Fix the ExecuteCommand status codes in `cyphal_interface.hpp` (BAD_COMMAND and BAD_STATE were swapped against the specification) and remove the heartbeat serializer that packed health and mode into one byte.
 - [x] Review fixes (2026-10-22): a port subscribed twice no longer corrupts the libcanard subscription tree (`CanardTransport::Subscribe()`, ztest); `PowerStageEnable()` checks the break flag again after the release (BCC and sim, not run on the board); CI badge points at `ci-zephyr.yml`; CI uploads the BCC and `native_sim` firmware as artifacts and attaches them to a release for `v*` tags.
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