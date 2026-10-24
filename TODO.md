# TODO list for open topics

**Focus (2026-10-23): Cyphal/CAN-FD for the cart.** The overall direction
(cart with four wheel drives, central vehicle dynamics controller, STM32G4
logic board for the 4850 drive) is in [docs/system_plan.md](docs/system_plan.md);
the hot path in [docs/hotpath_plan.md](docs/hotpath_plan.md), the Zephyr port
in [docs/zephyr_port_plan.md](docs/zephyr_port_plan.md) and
[docs/zephyr_workflow.md](docs/zephyr_workflow.md), the simulation in
[docs/software_in_loop_plan.md](docs/software_in_loop_plan.md), the settings in
[docs/settings_plan.md](docs/settings_plan.md), the traces in
[docs/cymon_integration_plan.md](docs/cymon_integration_plan.md).

There is no board access at the moment. Each section lists first what can be
verified with twister on `native_sim` and the BCC build, then what waits for
hardware. Finished items are collected under "Done" at the end.

## Decisions

### 2026-10-22, PR #26

1. DC-link voltages: regenerative braking derates from 55 V, the drive trips
   at 58 V, the hardware capability is 60 V (BCC and sim). Accepted; check
   the switch and capacitor ratings against the schematic during bring-up.
2. The overvoltage trip switches all transistors off; above the
   line-to-line back-EMF peak the motor still charges the DC link through
   the body diodes. Known and accepted risk until the brake chopper of the
   new 4850 hardware (decision 9).
3. The cymon trigger may publish and subscribe one shared subject: the
   design for triggering several targets at once.
4. The unit types move to dimension exponents (SI base dimensions plus the
   radian); done, see "Unit types".
5. The battery charge limit comes from the register
   `unimoc.battery.limits.charge_current` for now; a BMS may overrule it
   over the bus later.
6. The swing of the regenerative limit after a braking step with an imposed
   shaft speed (sim, about 300 ms) is accepted until the hardware tests.
7. The M0 timing probes stay off by default (`CONFIG_UNIMOC_HOT_PATH_TIMING`
   only in the bring-up build).
8. Live tuning stays conservative: a pending change that needs a restart
   holds back live tuning until the disengage. The drive reports it with a
   `uavcan.diagnostic.Record` ("settings wait for the disengage: <register>",
   then "waiting settings applied") and the register `unimoc.settings.pending`.

### 2026-10-23

9. The new 4850 hardware gets a fourth half bridge (auxiliary output). With
   an EESM it drives the excitation winding; with a PMSM it drives a brake
   resistor on the DC link, so each drive can hold its own DC link when the
   overvoltage trip opens the phases (mitigates decision 2). Details in
   section 5.2 of `docs/system_plan.md`; the electrical design is open.
10. Work focuses on Cyphal/CAN-FD. The plans and this list were consolidated
    around it; the next steps are under "Next".

## Next: Cyphal/CAN-FD (focus)

In this order; all without hardware unless marked.

 - [x] Define the cart messages in `dsdl/unimoc/cart/` (2026-10-23, system plan section 6.2): `unimoc.cart.WheelCommand`, `DriveStatus`, `PedalState`, `BatteryStatus` (version 0.1, `float16` in base units, `@extent` of one frame). ztest `cart_messages` in `unimoc.lib.cyphal`: every type of `dsdl/unimoc/` fits in one CAN FD frame, round trips of the cart messages. The priorities are set with the publishers.
 - [x] Drive side of the cart messages (2026-10-23): registers `unimoc.cart.wheel` and `.reversed`, the wheel command subscribed (the first one after the engage makes the drive a cart drive, applied on reception), the drive status published at `unimoc.cart.status_rate` with the torque limits with and against the motion from the motor limit and `control::DcLinkLimit` (the "reporting to the VDC" of the regenerative braking) and `regen_derated` (`src/drive/cart_control.*`, `OuterStatus::torque_min`/`torque_max`). ztests in `unimoc.drive.cart_control` and `sim_cyphal_drive`, pycyphal test. Also fixed: the settings register provider held 96 of 97 registers (`unimoc.control.command_timeout` was not on the bus).
 - [x] Battery status in the drive (2026-10-23): fresh BMS limits replace `unimoc.battery.limits.*` in the outer loop (`OuterLoop::SetBatteryLimits()`), scaled by `unimoc.cart.battery_share`; none with the contactor open or precharging; the registers again after a BMS timeout (decision 5, system plan section 7).
 - [x] CAN error states in the drive (2026-10-23): `CanMedia::BusStatus()` (Zephyr state-change callback, atomics), the best state of the interfaces in the transport; a bus-off drops every cart input at once (torque ramps to zero), `unimoc.can.bus_off_count`, a record on the return. The controller recovers by itself. Tested with an injected bus-off on `tests/common/can_test_bus.hpp`; the FDCAN callback is not run on the board.
 - [x] Limp-home mode in the drive (2026-10-23): on a VDC timeout (`unimoc.cart.timeout`) the torque comes from the pedal state (brake first, against the motion, fading below `unimoc.cart.standstill_speed`; drive pedal forward only; `unimoc.cart.limp_home_torque`, ramped at `unimoc.cart.torque_ramp`); the readiness timeout waits; back to the VDC only at standstill; without the pedals the torque ramps to zero and the drive disengages. Open: reverse in limp-home (direction selector), the VDC reaction to a lost drive.
 - [x] E-bike pedal assist (2026-10-23, `docs/ebike_plan.md`): `lib/ebike/` (pedal observer, assist law, Kunteng codec, software UART receiver), `src/ebike/` (control), `unimoc.ebike.*` registers, the drive hook, the BCC drivers (cadence pulse PA5, display line on PB15 through TIM15 capture, USART3 transmit on PB10) behind `CONFIG_UNIMOC_EBIKE`, the simulated rider and display. Open: tune the assist (bench, then ride), check the display protocol on a real KT-LCD8 (temperature offset, checksum constant, wheel size mapping, error codes), check the TIM15 both-edge capture and the 5 V levels of the display lines on the board, an assist level on the bus (a pedal-unit node), pytest integration of the simulated rider (registers `unimoc.sim.pedal_*`), persist the learned torque offset, brake and light inputs (not on the BCC connectors).
 - [ ] Multi-node simulation: check `vcan` (CAN FD MTU) on WSL2; one `native_sim` process per node on `vcan`, pycyphal on the same interface; multi-node pycyphal tests in CI. A single node already works over the slcan pseudo terminals (classic CAN). Fallback: Cyphal/UDP over the `native_sim` TAP driver (NSOS passes no multicast socket options in 4.4).
 - [ ] CAN FD bit timing per board in the devicetree: 1 Mbit/s arbitration and 4 Mbit/s data with transmitter delay compensation for the 4850 (TCAN334), 2 Mbit/s data as the fallback; the BCC stays at 1/1 Mbit/s until its transceiver changes (`boards/unimoc/bcc/bcc.dts`).
 - [ ] Firmware update over Cyphal: `ExecuteCommand BEGIN_SOFTWARE_UPDATE`, `uavcan.file.Read` into flash bank 2, bootloader swap; USB DFU as recovery.
 - [ ] Extend the pycyphal integration tests with every new bus feature (done: startup FSM registers, speed, position and servo setpoints, timing registers, simulation pause and advance).
 - [ ] Decide the CAN-FD transceiver change for the BCC (the board has an SN65HVD230, classic CAN). A board decision, no board access needed.
 - [ ] Hardware: Cyphal on the BCC: FDCAN3 bit timing at 1 Mbit/s with CAN FD frames through the SN65HVD230, the Rs pin, heartbeat, registers and the RTT log with a USB CAN FD adapter and yakut; plug-and-play with `yakut monitor --plug-and-play`.
 - [ ] Hardware: CAN FD bus test (phase C2): BCC and G4 evaluation boards on a cart-length harness, 1/4 and 1/2 Mbit/s, bus load and latency at 250 and 500 Hz; choose the bit rate and the cycle rate.

## Cart system

 - [ ] Requirements and part choice for the STM32G4 redesign of the hardware_4850 logic board (STM32G473RET6 or G474RET6, pin and ADC check, system plan section 5.2), including the fourth half bridge (decision 9): its PWM channel synchronized with the phase timer, its current measurement (excitation current, resistor current), the comparator path for the chopper and the board it sits on.
 - [ ] Generalize `hardware/zephyr_bcc` to the G4 family so the 4850 logic board shares the hot path (board differences in the devicetree).
 - [ ] Fourth half bridge in the firmware: a platform-independent auxiliary output next to `hardware/power_stage_interface.hpp` with two roles selected by the motor type, excitation (EESM: `ExcitationController` sets the duty) and brake chopper (PMSM: DC-link threshold with hysteresis, on together with the overvoltage trip; resistor energy and temperature limit in a thread). The chopper reacts in hardware (G4 comparator to timer) where the routing allows it, the firmware sets the thresholds. Simulated first (see "Motor types").
 - [ ] Regenerative braking, open parts (the drive side, the BMS limits and the reporting to the VDC are done): the estimated speed in the limit on a dyno (the observer's torque model expects the shaft to slow; settles within some 300 ms in the sim), the BCC voltage ratings from the schematic, a per-sample overvoltage check in the sample interrupt.
 - [ ] Split `app/` into `apps/drive/` and `apps/vdc/` (later `apps/pedal/` and `apps/bms/`) and add `lib/vehicle/`; update `AGENTS.md`, `README.md` and `docs/zephyr_workflow.md`. The VDC firmware lives in this repository (decided); the VDC MCU starts on a G4.
 - [ ] BMS controller as `apps/bms/` with `lib/battery/` (system plan section 5.5): board (BCC or new G4 board), cell monitoring front end, contactor and precharge, battery status message. Whether it still needs its own brake chopper next to the choppers in the drives is open (system plan section 12).

## Motor types: ASM and EESM in the simulation and the firmware

`lib/motor_control` has the ASM flux observer and controller and the EESM
excitation controller and observer, but the runtime (`src/`, `hardware/`) has
no motor-type selection and runs the PMSM path only, and `lib/sim` has only
`PmsmModel` (`MotorSimulation` rejects other types with
`kUnsupportedMotorType`).

 - [ ] ASM plant in `lib/sim` (`AsmModel`: stator and rotor flux d/q model with rotor resistance and magnetizing inductance) behind the `MotorModel` concept; plant tests (slip, torque direction, flux build-up).
 - [ ] EESM plant in `lib/sim` (`EesmModel`: d/q model with saliency and a field winding, psi_f = L_m i_f, field voltage from `ElectricalInput::field_voltage`); plant tests (field build-up and decay, torque with and without excitation).
 - [ ] Fourth half bridge in the sim inverter: field voltage from its duty (EESM), or a brake resistor on the DC link (PMSM); field or resistor current as a measured channel.
 - [ ] DC-link model in the sim instead of the stiff source set by register: capacitor, battery with internal resistance and a contactor that can open, so the overvoltage trip, the body-diode charging (decision 2) and the chopper can be tested.
 - [ ] Motor-type selection in the runtime: ASM path (flux observer and controller; the `atan2` of `asm_flux_observer::flux_angle` must go without libm before it enters the control cycle) and EESM path (excitation controller and observer, excitation setpoint over Cyphal, the open "excitation" of the SIL setpoint item); closed-loop sim tests per motor type.
 - [ ] Hardware: excitation and chopper on the 4850 bench (after the board exists).

## Hot path and Zephyr port

### Without hardware

 - [ ] Measure the cost of the trace `Tick()` in the control cycle on the BCC (M0, target < 1 us); move it to the observer interrupt if it is too slow.
 - [ ] Check whether the hot path (sample ISR, control cycle, buffers) belongs in CCM SRAM on the G4 (`docs/hotpath_plan.md`, "Risks and Open Points"); decide with the M0 timing numbers.
 - [ ] Simulation-control commands over Cyphal for deterministic runs (SIL P2): pause, fixed-cycle advance and reset done (`unimoc.sim.paused`, `.advance`, `.reset`, `.cycle_count`, `.cycle_period_ns`, pycyphal test); open: run-until, the drive's timeouts in virtual time.

### Blocked on hardware access

 - [ ] M0 on the board: timing numbers for the sample ISR, control cycle and observer.
 - [ ] M8 first step with the outputs off: DMA burst, preload, start at ARR/2, parity detection and ADC alignment on the scope. Include a settings restart (a register write while disengaged, then engage): `foc_stm32g4::Stop()` and the re-init are not run on the board yet.
 - [ ] Verify ZMS, the flash stall, the dual-bank option bit (DBANK, bank 2 free for Cyphal firmware updates) and the size of the settings partition on the BCC board (settings plan, "Not verified on hardware").
 - [ ] Configure the duty DMA from the devicetree (`dmas` of `foc0`, Zephyr DMA API at init) instead of the fixed DMA1 channel 1 / DMAMUX request 53, and enable its clock through `clock_control` instead of writing `RCC->AHB1ENR`. The zero-latency path keeps reading the channel counter directly. Do this after the M8 first step, so the fixed configuration is a verified reference.
 - [ ] M8 remainder: Hall table, closed current loop with the 50 A limit.
 - [x] First Zephyr release: `develop` merged into `main` and published as v0.1.0 (2026-10-24) by the release workflow (`docs/release_plan.md`).
 - [x] Release setup on GitHub: `RELEASE_TOKEN`, merge methods per branch, required checks (check run names) and "Automatically delete head branches" off, which had deleted `develop` after the first release merge.

## Unit types

The tags were replaced by dimension exponents (decision 4); the remaining
escapes are counted in `scripts/unit_escapes_baseline.txt` and may only go
down. The boundaries are documented in `CONTEXT.md` ("Unit Boundaries").

 - [ ] P1: Observers in units, open: the Kalman covariance and correction of `mechanical_observer` (mixed-unit state vector, stays `float`), the `atan2` of `asm_flux_observer::flux_angle` (libm; see "Motor types"), `hall_fusion` (sector timing as `Time`, noise variances), `slow_update` noise. Effort S.
 - [ ] P1: Application state in units, open: `HwStartup` thresholds and results (offsets and noise in A, gains as `Ratio`; the `double` accumulators to `float` or a documented exception), `StartupRegisters::ext_current_`/`ext_vdc_`. Effort M.

## Software-in-the-loop simulation

 - [ ] P2: One-process simulator coordinator with N independent motor nodes and one shared virtual clock; a barriered multi-motor step with coherent timestamps and deterministic ordering.
 - [ ] P2: Independent constant/viscous load models and tests for isolated motors, unique node IDs, reset and repeatability; a shared load interface for shaft, wheel, battery, differential and torque-vectoring models.
 - [ ] P2: Host simulator executable around the coordinator.
 - [ ] P2: Excitation setpoints into the normal controller input path (torque, d/q current, speed, position and the UDRAL servo done); depends on the EESM runtime path.
 - [ ] P2: Telemetry and heartbeat with explicit virtual-time semantics (published in real time today; the telemetry carries the control-cycle count).
 - [ ] P2: Simulation-control commands run-until and snapshot (pause, fixed-cycle advance, reset and clock status done).
 - [ ] P2: Protocol and localhost end-to-end tests for multi-node operation (single node done: `unimoc.app.cyphal_integration`).
 - [ ] P3: Python command-and-control GUI: separate Python package and dependency file, `pycyphal` client (discovery, registers, setpoints, telemetry, reset, virtual-time stepping), PySide6/pyqtgraph step responses, scenarios with run/pause/reset/step, CSV/JSON export and replay, network I/O outside the UI thread, headless protocol and smoke tests.
 - [ ] P4: Startup, observer, field-weakening, homing and control-mode transition scenarios.
 - [ ] P4: Vehicle-level models: shared battery and DC link, wheels, differential load, road load, torque vectoring.
 - [ ] P4: Fault injection, profiling and large-N motor trace tooling; process-per-motor hosting only once the coordinator is stable and a concrete isolation need exists.
 - [ ] Twister scenarios (ztest on `native_sim`) for the coordinator and protocol tests as they appear; plant and runtime tests exist in `tests/lib/sim` and `tests/sim/`.
 - [ ] Keep the focused three-phase validation green; ask before widening the validated scope in `AGENTS.md` to the SIL scenarios (`tests/sim/`, `lib/sim`).
 - [ ] Document simulator setup, launch commands, supported model fidelity and GUI workflows in `README.md`.

## Done

### Cyphal/CAN-FD

 - [x] Cyphal over CAN-FD as the primary interface (2026-10-21): libcanard 4.0 + O1Heap + Nunavut DSDL pinned in `west.yml`; heartbeat, port list, GetInfo, register List/Access on the settings store (77 registers), ExecuteCommand (restart, store, factory reset, emergency stop), PnP v1, diagnostic records from the Zephyr log; drive application (readiness, current and torque setpoints with timeouts and limits, telemetry); shared between BCC (FDCAN3) and `native_sim` (SocketCAN plus slcan pseudo terminals, plant registers and plant trace). Tests `unimoc.lib.cyphal`, `unimoc.sim.cyphal_drive`; checked with yakut 0.14 against the simulation. Not run on the board.
 - [x] SIL transport path: `native_sim` with pycyphal/yakut over Cyphal/CAN (SocketCAN `vcan0` or the slcan pseudo terminals, no UDP needed for one node); libcanard 4.0 behind `cyphal::Transport`, Nunavut C serialization; register exchange and telemetry subject confirmed with yakut; standard register operations mapped to the settings boundary.
 - [x] pycyphal integration tests of the running simulation (`tests/integration/cyphal_sim`, twister pytest harness, scenario `unimoc.app.cyphal_integration`): node services, registers, arming, setpoints, telemetry, diagnostics, simulation registers, store and restart.
 - [x] Connect the startup FSM (`HwStartup`) to Cyphal: `drive::StartupRegisters` (run, step, external measurements, results), the FSM stepped in `DriveNode::Spin()` in the Cyphal thread; ztests in `sim_cyphal_drive`.
 - [x] Speed and position setpoints over Cyphal (`uavcan.sub.speed_setpoint`, `position_setpoint`, UDRAL servo) through the control thread of M7: ztests in `sim_cyphal_drive`, pycyphal speed test against the running simulation.
 - [x] cymon traces over Cyphal ([docs/cymon_integration_plan.md](docs/cymon_integration_plan.md)): cymon-lib 0.2 linked, `cymon.*` services and trigger event, `Tick()` in the control cycle, plant truth on `native_sim`; the host tool is cymon (<https://github.com/Tecnologic/cymon>).
 - [x] Reject settings with two ports on the same port-ID (`settings_store_internal::HasPortIdCollision()` in `Validate()`); the cymon trigger may publish and subscribe one shared subject (decision 3). ztest in `unimoc.lib.settings`.
 - [x] Live tuning while engaged: the gains of the current, speed and position loops, the telemetry rate and the command timeout reach the running loops at once (`DriveNode::TuneLive()`); other registers still wait for the disengage (decision 8, `docs/settings_plan.md`, open item 1). ztest in `sim_cyphal_drive`.
 - [x] One angle convention for the bus: the electrical angle is [0, 2 pi) in `unimoc.drive.Telemetry`, `unimoc.sim.Plant` and the traces (`observer::WrapAnglePositive()`, no libm, usable in the control cycle).
 - [x] Fix the ExecuteCommand status codes in `cyphal_interface.hpp` (BAD_COMMAND and BAD_STATE were swapped against the specification) and remove the heartbeat serializer that packed health and mode into one byte.
 - [x] Review fixes (2026-10-22): a port subscribed twice no longer corrupts the libcanard subscription tree (`CanardTransport::Subscribe()`, ztest).
 - [x] ~~Enable the USB CDC-ACM console~~ Superseded (2026-10-21): USB and the shell were dropped; the bench log is on RTT and on the bus. Link limited to the code partition (`CONFIG_USE_DT_CODE_PARTITION`).

### Drive and regenerative braking

 - [x] Regenerative braking in the drive (2026-10-22): battery drive and charge current limits in the outer loop (`control::DcLinkLimit`, all modes), regenerative derating from `unimoc.battery.limits.regen_derate_voltage` to `trip_voltage` (decisions 1, 2, 5, 6), hard trip in the control cycle (`overvoltage_tripped`, `ClearTrips()`), ztests in `unimoc.lib.motor_control`, `sim_overcurrent` and `sim_cyphal_drive`; the speed controller takes the same power window as output bounds (`DcLinkLimit::AllowedTorque()`), so its integral does not wind up against it.
 - [x] Settings apply on running hardware: `StartRuntime()` calls `HardwareInterface::Stop()` before it rewrites the control state; on the BCC `foc_stm32g4::Stop()` stops the interrupts, TIM8, the duty DMA and the ADCs. ztest on `native_sim` (`sim_irq_cadence`), BCC build-tested; verify on the board with M8.
 - [x] Publish the thread-side writes the zero-latency interrupt reads with a defined order (`CurrentControlIsr::ForceDuty()` mailbox and release/acquire flag, packed atomic overcurrent window). Not run on the board.
 - [x] M0 instrumentation: `CONFIG_UNIMOC_HOT_PATH_TIMING` (off by default, decision 7), `hardware::timing::Probe`, DWT cycles of the sample ISR, the control cycle and the observer in the `unimoc.timing.*` registers; ztests on `native_sim`, BCC build scenario `unimoc.app.build.timing`.
 - [x] Settings follow-ups: `PowerStageInterface::Enable()` is held off while a flash save runs, and `SettingsStore` takes a platform lock (Zephyr `k_mutex`) around every operation.
 - [x] Persist settings per register with the Zephyr settings subsystem on ZMS, live RAM tuning and a gated `SaveToFlash()` ([docs/settings_plan.md](docs/settings_plan.md)).
 - [x] Review fixes (2026-10-22): `PowerStageEnable()` checks the break flag again after the release (BCC and sim, not run on the board).

### Hot path and Zephyr port

 - [x] Decide the hot-path layout: 32 kHz PWM, 16 kHz control cycle in the sample ISR, duties by CCR preload plus DMA (variant B), observer in a software-pended normal-priority interrupt (`docs/hotpath_plan.md`).
 - [x] Hot-path milestones M1 to M7 in code (observer API, Hall driver and sim Hall signals, Hall fusion, control thread with speed and position control), tested with ztest and the sim plant; not run on the board.
 - [x] Align the root `CMakeLists.txt` with the C++20 baseline.
 - [x] Bring `src/`, `hardware/zephyr_*`, the tests and the `lib/unimoc` headers under clang-tidy (Zephyr SCA through CodeChecker, `scripts/twister_sca.sh`); `lib/cymon` is checked too since it is built.
 - [x] Add an `SPDX-License-Identifier` line to the UNIMOC file header.
 - [x] Delete the legacy firmware (`hardware/batterycasecontroller/`, `cmake/Modm.cmake`, `cmake/ToolChain.cmake`, `requirements.txt`, `ci-firmware.yml`) and its stale references; merge `main` into `develop` (`542c501`), the BCC settings profile replaces `hardware/HardwareLimits.hpp`.
 - [x] CI housekeeping: Node 24 actions, `ci-docs.yml` only uploads the Doxygen HTML, CI badge on `ci-zephyr.yml`, BCC and `native_sim` firmware as artifacts and release assets for `v*` tags.
 - [x] `app/boards/native_sim.*` renamed to `native_sim_native_64.*` (1 us tick, real time).

### Unit types

 - [x] Enforce the boundary in CI: `scripts/check_unit_escapes.py` (run by `check_format.sh`) against `scripts/unit_escapes_baseline.txt`.
 - [x] Math helpers in `units.hpp` (`Abs`, `Min`/`Max`, `IsFinite`, `Hypot`, `Sqrt` of a squared quantity), no libm beyond `sqrtf`.
 - [x] Dimension exponents instead of tags (decision 4): `unit::Dimension` (kg, m, s, A, K, rad), one generic `operator*` and `operator/`, the electromechanical coupling as the only explicit rule; zero runtime cost.
 - [x] Frames and transforms, controller gains, the observers except the items above (`mechanical_observer` torque balance, `asm_flux_observer` equations, `excitation_observer`, `field_weakening`, `mtpa`, `OuterLoop::LimitCurrent()`; no `sinf`/`cosf` in the BCC firmware), the hardware conversions, `settings_store_internal::Validate()`, `CurrentControlState` and `DriveNode` in units.
 - [x] Typed test checks: `UNIMOC_EXPECT_FLOAT_EQ`/`UNIMOC_EXPECT_NEAR` take two values of one unit; test escapes from 780 to 118.

### Software-in-the-loop simulation

 - [x] P0: Virtual-time cycle contract (`tests/sim/irq_cadence`), injectable ADC/PWM I/O, reusable runtime (`current_control::StartRuntime()`), virtual clock harness (`hardware/zephyr_sim/sim_control.hpp`).
 - [x] P1: PMSM plant (`lib/sim/`, `MotorSimulation`, 120 degree Hall sensors) with plant tests, connected to `hardware/zephyr_sim`; end-to-end step response, cadence and finite-state tests (`tests/sim/closed_loop`).
