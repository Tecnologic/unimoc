# TODO list for open topics

**Focus (2026-10-23): Cyphal/CAN-FD for the cart.** The overall direction
(cart with four wheel drives, central vehicle dynamics controller, the
4850 drive with an STM32H743VIT6 logic board since 2026-11-01, named Jacobi
since 2026-11-03) is in [docs/system_plan.md](docs/system_plan.md);
the hot path in [docs/hotpath_plan.md](docs/hotpath_plan.md), the Zephyr port
in [docs/zephyr_port_plan.md](docs/zephyr_port_plan.md) and
[docs/zephyr_workflow.md](docs/zephyr_workflow.md), the simulation in
[docs/software_in_loop_plan.md](docs/software_in_loop_plan.md), the settings in
[docs/settings_plan.md](docs/settings_plan.md), the traces in
[docs/cymon_integration_plan.md](docs/cymon_integration_plan.md).

There is no board access at the moment. Each section lists first what can be
verified with twister on `native_sim` and the Jacobi build, then what waits for
hardware. Finished items are collected under "Done" at the end.

## Priorities (2026-10-31)

The open points of this list and the plans, ranked. Tier 1 can be done and
verified in a cloud development container (west workspace, Zephyr SDK,
twister on `native_sim`, the Jacobi build, pycyphal over the slcan pseudo
terminals); that container has no board, no `vcan` (the kernel has no CAN
modules) and no access to cypilot's branches beyond its own repository.

### Tier 1: in the development container, in this order

Items 1 to 5 are done (2026-10-31, one pull request); 6 is dropped and 7 is
done (2026-11-08); 8 is next.

1. Done: **Zephyr 4.5 update** ("Hot path and Zephyr port"). First, so every later
   change lands on the new baseline while the diff against 4.4 is still only
   the migration.
2. Done: **Per-sample overvoltage check in the sample interrupt** ("Cart system",
   regenerative braking). Small, safety-relevant, testable in the sim
   (`tests/sim/`); keeps the zero-latency rules.
3. Done: **DC-link model in the sim** (capacitor, battery with internal resistance,
   contactor; "Motor types"). Makes the overvoltage trip, the body-diode
   charging (decision 2) and later the brake chopper testable; prerequisite
   of 6 and 8.
4. Done: **Virtual time for the simulation**: the drive's timeouts in virtual time
   and run-until ("Hot path and Zephyr port", SIL P2). Deterministic sim and
   pytest runs for everything below.
5. Done (firmware): **Motor identification I0 to I2** (forced angle, the
   `MotorIdentification` FSM and `unimoc.identify.result.*`, M1 with dead time
   in the sim inverter, M2). The next feature line; the decisions are taken
   (plan section 9). I3 (phase-voltage sense in the sim) follows; the
   cypilot side (C0) is in its own repository.
6. Dropped (2026-11-08): **Fourth half bridge**. It leaves the drive plan:
   the EESM excitation and the brake chopper become a separate board (a bus
   module; decisions 9 and 12), so neither the sim inverter nor the drive
   firmware gets an auxiliary output.
7. Done (2026-11-08): **Small bus items**: `.type` registers for every port
   (`docs/gui_plan.md` section 6.4; the drive had them, a pytest checks each
   port), `unimoc.sim.pedal_*` and `unimoc.sim.display_*` registers with the
   pytest scenario of the simulated rider (`unimoc.app.cyphal_integration.ebike`),
   the learned pedal torque offset stored in `unimoc.ebike.torque_offset`
   (`docs/ebike_plan.md`). New pycyphal tests come with each bus feature.
8. **ASM and EESM plants, then the motor-type selection in the runtime**
   ("Motor types"), including the libm-free `atan2` of the ASM flux angle
   and the excitation setpoint (SIL P2).
9. **Cyphal file service and firmware update, sim part**: `uavcan.file.Read`
   and `BEGIN_SOFTWARE_UPDATE` into a simulated bank 2; serving the register
   description file over it (F3). The bank swap and the bootloader wait for
   the board.
10. **Structure for the cart**: split `app/` into `apps/drive/` and
    `apps/vdc/` with `lib/vehicle/` (the VDC, pedal unit and BMS boards are
    STM32H7 based and share `hardware/zephyr_stm32h7`).
11. **SIL coordinator** (one process, N motor nodes, shared virtual clock,
    load models, the host executable; SIL P2) and the simulator chapter of
    `README.md`. Multi-node over `vcan` cannot run in the container.

### Tier 2: decisions, no hardware needed (yours)

 - The data bit rate of the bus (Jacobi's TCAN334 handles CAN FD rates; all
   boards run 1/1 Mbit/s today).
 - License of `dsdl/unimoc/` (`docs/gui_plan.md` section 2).
 - VDC part (STM32H7; `docs/system_plan.md` section 12); the EESM excitation module (section 5.2.1).
 - Limp-home reverse (direction selector) and the VDC reaction to a lost
   drive.

### Tier 3: needs another machine

 - `vcan` (CAN FD MTU) on WSL2, `scripts/sim_cluster.sh` and the multi-node
   pycyphal tests on it; then multi-node tests in CI if the runner has `vcan`.
 - cypilot C0 against the next UNIMOC release.

### Tier 4: blocked on the board

M0 timing with the flux observer and Hall fusion running (then the DTCM
decision), M8 first step and remainder, ZMS and the dual-bank option, Cyphal
on the NUCLEO-H743ZI and Jacobi and the CAN
FD bus test C2 (then the bit rate and the Jacobi bit timing), identification
I7, regenerative braking on a dyno.

## Decisions

### 2026-10-22, PR #26

1. DC-link voltages: regenerative braking derates from 55 V, the drive trips
   at 58 V, the hardware capability is 60 V (Jacobi and sim). Accepted; check
   the switch and capacitor ratings against the schematic during bring-up.
   2026-11-08: both H7 power stages carry 60 V FETs, so the H7 boards take
   their limits from the devicetree (foc0 `dc-link-millivolts`: derate 45 V,
   trip 48 V, at most 50 V); the simulation keeps 55 / 58 / 60 V.
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
   Superseded 2026-11-03 (decision 12) and 2026-11-08: the drive has three
   phases only; excitation and brake chopper are a separate board.
10. Work focuses on Cyphal/CAN-FD. The plans and this list were consolidated
    around it; the next steps are under "Next".

### 2026-11-01

11. The 4850 logic board gets an STM32H743VIT6 (LQFP100) instead of the
    STM32G473/G474: the 512 KB G4 parts are not available, the firmware does
    not fit into 128 KB (BCC image 194 KB), and the H7 leaves headroom for a
    faster current loop. Pin proposal and port in
    `docs/drive4850_h7_plan.md`; a second hot-path platform
    `hardware/zephyr_stm32h7`.

### 2026-11-03

12. Hardware is named after pioneers of electric drives and computing
    (`docs/system_plan.md` section 5.0). The H7 logic board is **Jacobi**
    (`boards/unimoc/jacobi/`, board `unimoc_jacobi`): three phases only, phase
    voltage taps in place of the AC current paths; the EESM excitation becomes
    a second bus module, the brake chopper stays in the BMS.
13. The battery case controller (BCC, STM32G473) is removed: board, platform
    `hardware/zephyr_bcc`, the G4 and e-bike bindings. The e-bike control
    remains for the simulation only. A revived BCC gets an STM32H743 too
    (decision 2026-11-03, `docs/system_plan.md` section 5.1). 2026-11-08:
    all boards are STM32H7 based; the firmware builds only the H7 boards and
    `native_sim`.

## Next: Cyphal/CAN-FD (focus)

In this order; all without hardware unless marked.

 - [x] Define the cart messages in `dsdl/unimoc/cart/` (2026-10-23, system plan section 6.2): `unimoc.cart.WheelCommand`, `DriveStatus`, `PedalState`, `BatteryStatus` (version 0.1, `float16` in base units, `@extent` of one frame). ztest `cart_messages` in `unimoc.lib.cyphal`: every type of `dsdl/unimoc/` fits in one CAN FD frame, round trips of the cart messages. The priorities are set with the publishers.
 - [x] Drive side of the cart messages (2026-10-23): registers `unimoc.cart.wheel` and `.reversed`, the wheel command subscribed (the first one after the engage makes the drive a cart drive, applied on reception), the drive status published at `unimoc.cart.status_rate` with the torque limits with and against the motion from the motor limit and `control::DcLinkLimit` (the "reporting to the VDC" of the regenerative braking) and `regen_derated` (`src/drive/cart_control.*`, `OuterStatus::torque_min`/`torque_max`). ztests in `unimoc.drive.cart_control` and `sim_cyphal_drive`, pycyphal test. Also fixed: the settings register provider held 96 of 97 registers (`unimoc.control.command_timeout` was not on the bus).
 - [x] Battery status in the drive (2026-10-23): fresh BMS limits replace `unimoc.battery.limits.*` in the outer loop (`OuterLoop::SetBatteryLimits()`), scaled by `unimoc.cart.battery_share`; none with the contactor open or precharging; the registers again after a BMS timeout (decision 5, system plan section 7).
 - [x] CAN error states in the drive (2026-10-23): `CanMedia::BusStatus()` (Zephyr state-change callback, atomics), the best state of the interfaces in the transport; a bus-off drops every cart input at once (torque ramps to zero), `unimoc.can.bus_off_count`, a record on the return. The controller recovers by itself. Tested with an injected bus-off on `tests/common/can_test_bus.hpp`; the FDCAN callback is not run on the board.
 - [x] Limp-home mode in the drive (2026-10-23): on a VDC timeout (`unimoc.cart.timeout`) the torque comes from the pedal state (brake first, against the motion, fading below `unimoc.cart.standstill_speed`; drive pedal forward only; `unimoc.cart.limp_home_torque`, ramped at `unimoc.cart.torque_ramp`); the readiness timeout waits; back to the VDC only at standstill; without the pedals the torque ramps to zero and the drive disengages. Open: reverse in limp-home (direction selector), the VDC reaction to a lost drive.
 - [x] E-bike pedal assist (2026-10-23, `docs/ebike_plan.md`): `lib/ebike/` (pedal observer, assist law, Kunteng codec, software UART receiver), `src/ebike/` (control), `unimoc.ebike.*` registers, the drive hook, the BCC drivers (cadence pulse PA5, display line on PB15 through TIM15 capture, USART3 transmit on PB10) behind `CONFIG_UNIMOC_EBIKE`, the simulated rider and display. Open: tune the assist (bench, then ride), check the display protocol on a real KT-LCD8 (temperature offset, checksum constant, wheel size mapping, error codes), check the TIM15 both-edge capture and the 5 V levels of the display lines on the board, an assist level on the bus (a pedal-unit node), brake and light inputs (not on the BCC connectors). Done 2026-11-08: the pytest of the simulated rider (`unimoc.sim.pedal_*`, `unimoc.sim.display_*`, scenario `unimoc.app.cyphal_integration.ebike`) and the learned torque offset stored in `unimoc.ebike.torque_offset`.
 - [ ] Multi-node simulation: check `vcan` (CAN FD MTU) on WSL2; one `native_sim` process per node on `vcan`, pycyphal on the same interface; multi-node pycyphal tests in CI. A single node already works over the slcan pseudo terminals (classic CAN). Fallback: Cyphal/UDP over the `native_sim` TAP driver (NSOS passes no multicast socket options in 4.4).
 - [ ] CAN FD bit timing per board in the devicetree: 1 Mbit/s arbitration and 4 Mbit/s data with transmitter delay compensation for the 4850 (TCAN334), 2 Mbit/s data as the fallback; Jacobi and the NUCLEO-H743ZI run 1/1 Mbit/s until then (`boards/unimoc/jacobi/unimoc_jacobi.dts`).
 - [ ] Firmware update over Cyphal: `ExecuteCommand BEGIN_SOFTWARE_UPDATE`, `uavcan.file.Read` into flash bank 2, bootloader swap; USB DFU as recovery.
 - [ ] Extend the pycyphal integration tests with every new bus feature (done: the commissioning commands and status message of the startup aid, START_HOMING and the homing state, RESET_TIMING, speed, position and servo setpoints, timing registers, simulation pause and advance).
 - [ ] Hardware: Cyphal on the NUCLEO-H743ZI, then Jacobi: FDCAN1 bit timing at 1 Mbit/s with CAN FD frames, the TCAN334 standby pin, heartbeat, registers and the log with a USB CAN FD adapter and yakut; plug-and-play with `yakut monitor --plug-and-play`.
 - [ ] Hardware: CAN FD bus test (phase C2): NUCLEO-H743ZI and evaluation boards on a cart-length harness, 1/4 and 1/2 Mbit/s, bus load and latency at 250 and 500 Hz; choose the bit rate and the cycle rate.

## Motor identification

Plan: [docs/motor_identification_plan.md](docs/motor_identification_plan.md) (algorithms of the old firmware carried over, an assisted flow in cypilot like the startup aid). Decisions in its section 9 first.

 - [ ] C0 in cypilot: the startup card, `cypilot-ctl startup` and the demo device on the commissioning commands and status message (event-driven), Home (START_HOMING, `cypilot-ctl run --home`) and `reset-timing`; needs a UNIMOC release with `unimoc.drive.CommissioningStatus.0.1` and the homing state for `DEFAULT_UNIMOC_REF`.
 - [x] I0, firmware (2026-10-31): forced angle in `SlowUpdate`, online accumulation of the measurement windows, `MotorIdentification` FSM on C0 (START_IDENTIFICATION `M1,M2`, ABORT, the status message) with the result registers `unimoc.identify.result.*` (plan section 8.1). Open: the cypilot card and `cypilot-ctl identify` (cypilot repository).
 - [x] I1 (2026-10-31): M1 stator resistance and dead time (regression over five levels of each sign at three angles, without the dead-time compensation); dead time in the sim inverter. Sim: R and t_dead within 0.1 % of the plant.
 - [x] I2 (2026-10-31): M2 L_d and L_q (a sine at f_pwm / 64 per axis on the held PI voltage, single-bin phasors with the exact phase of every sample, Z = U / I). Sim: within 0.2 %. Open: the saturation option (L_d at three biases).
 - [ ] Identification on the board (I7 covers the bench comparison): the rotor alignment at each M1 angle with a free shaft, the settle time, the M2 amplitude adaptation on a real winding.
 - [ ] I3: phase voltages in `ControlCycleSamples` (STM32H7: `va`, `vb`, `vc`, on Jacobi from the new taps; the sim plant with open outputs), their calibration in the startup aid's DUTY_FORCE steps; M3 flux linkage by coast-down.
 - [ ] I4: M4 pole pairs (measured, or entered and checked); M5 Hall offset and order.
 - [ ] I5: M6 inertia and friction (the coast of M3, torque steps).
 - [ ] I6: M7 derived gains; review, comparison and approval in cypilot, apply and store, motor profiles.
 - [ ] I7 (hardware): bench comparison with LCR meter, 4-wire R and a scope coast-down; the gate-driver idle polarity for the coast.

## Cart system

 - [ ] Jacobi schematic (Tecnologic/unimoc_jacobi, MCU drawn 2026-11-03): the reserved functions (overcurrent comparator, angle sensor, CAN STB), the *(RM0433)* points, board size in the enclosure. The fourth half bridge is dropped (separate module).
 - [x] (2026-11-08) Jacobi devicetree to the schematic (`docs/drive4850_h7_plan.md` section 4 note): PWM pins, ADC channels, LEDs, `PWM_EN` = break PE15 (drop the PE7 `pwm-enable-gpios`), CAN STB PD2, PWR_DET PE4, +12P sense PC5. The binding now takes one to four conversions per ADC by signal name (both temperatures on ADC1, vdc on ADC2). Open: LED polarity; the inputs below.
 - [x] (2026-11-08) Jacobi board inputs (`docs/drive4850_h7_plan.md` section 4.3, decisions there): PWR_DET (PE4) and +12P (PC5, 10k / 2k2) refuse the engage (BLOCKED, health WARNING) and trip an engaged drive (+12P below 10 V for 5 ms); the accelerator (PC0) and the brake lever (PC4) are the local torque setpoint of a drive without the e-bike (`unimoc.accel.*`); `unimoc.board.*` registers; Vdc moved to ADC3. Sim registers `unimoc.sim.power_board`, `gate_supply`, `accelerator`, `brake`; ztests (`tests/drive/accelerator_control`, `sim_cyphal_drive`) and pytest.
 - [ ] Hardware: the board inputs on Jacobi: PWR_DET with and without the power board, the +12P reading against a meter (divider R109/R110), the accelerator and brake lever voltages of the real sensors (released and pressed, for `unimoc.accel.*_min`/`_max`), the 11k / 100n settling against the 16.5-cycle sample time.
 - [x] (2026-11-08, foc0 `max-phase-current-amps`, `max-motor-current-amps`, `dc-link-millivolts`, `shunt-nano-ohm`, `current-amplifier-gain`, `voltage-divider-ohms`) Limits per board (`docs/drive4850_h7_plan.md` open point 9): the STM32H7 profile has the legacy 4850 values (48 V, trip 58 V, 60 A trip); Jacobi has 60 V FETs (36 V systems, not for 13S), ±160 A sensing, trip 120 to 130 A, 100 A peak. Take them from the devicetree or a board profile; the current scaling (0.5 mΩ, INA240A1 gain 20) differs from the legacy board too.
 - [x] H7 platform `hardware/zephyr_stm32h7` on a NUCLEO-H743ZI rev. Y (2026-11-01, `docs/drive4850_h7_plan.md` section 7): H7 ADC and calibration, non-cacheable duty DMA buffer, dual-bank flash layout, 400 MHz; builds and passes clang-tidy, not run on the board.
 - [ ] NUCLEO-H743ZI on the bench (plan section 7, steps 1 to 5): boot, settings, PWM and break, ADC, timing, Cyphal, then the 4850 power board through an adapter.
 - [x] H7 hot path in ITCM and the control cycle on every sample (2026-11-03, `hardware/zephyr_stm32h7/hot_path_itcm.ld`, `docs/hotpath_plan.md` section 4); the Jacobi board exists (`boards/unimoc/jacobi/`).
 - [ ] Regenerative braking, open parts (the drive side, the BMS limits and the reporting to the VDC are done): the estimated speed in the limit on a dyno (the observer's torque model expects the shaft to slow; settles within some 300 ms in the sim), the Jacobi voltage ratings from the schematic. Done 2026-10-31: the per-sample overvoltage check in the sample interrupt (raw DC-link counts, `SetOvervoltageCounts()`; not run on the board).
 - [ ] Split `app/` into `apps/drive/` and `apps/vdc/` (later `apps/pedal/` and `apps/bms/`) and add `lib/vehicle/`; update `AGENTS.md`, `README.md` and `docs/zephyr_workflow.md`. The VDC firmware lives in this repository (decided); the VDC MCU is an STM32H7.
 - [ ] BMS controller as `apps/bms/` with `lib/battery/` (system plan section 5.5): board (a new STM32H7 board, Volta), cell monitoring front end, contactor and precharge, battery status message. It needs its own brake chopper: the drives have none (system plan section 5.2.1).

## GUI

The operator GUI and CLI for one to many nodes is
[cypilot](https://github.com/Tecnologic/cypilot) (`cypilot`, `cypilot-ctl`,
Python with pycyphal and PySide6), in its own repository with its own plan
(`docs/plan.md` there; G0, the core and `cypilot-ctl` against the v0.1.0
simulation, done 2026-10-24). [docs/gui_plan.md](docs/gui_plan.md) keeps the
contract with the firmware. The firmware side, in this repository:

 - [x] F1 (2026-10-24): vendor status bits 4 to 7 (trip latched, E-stop latched, settings pending, local control) in the heartbeat; the readiness subject follows the node-ID (65534: 1024 + node-ID) until written; ztest and pytest. The GetInfo name convention was dropped (the name is the user's role label, `docs/gui_plan.md`, section 6).
 - [x] F2 (2026-10-24): register description file: unit, limits, effect (live-tunable, disengaged, restart) and help next to each register, `--describe-registers` of the simulation, ztests of the descriptions, a pytest against the registers on the bus, the CI and release asset `unimoc-registers-<version>.json`; `scripts/sim_cluster.sh` for N drives on `vcan0` (`--cart-wheel`). Fixed with it: the register list ended at 128 settings, so `unimoc.ebike.rearm_time` and `unimoc.ebike.display_timeout` were not on the bus.
 - [x] Startup aid from cypilot (2026-10-29, cypilot branch `ccr-03adfb43-5avw34`): a Startup aid card in the Control view on the existing `unimoc.startup.*` registers; STOP ALL, the E-stop and the operator's lease abort a run (readiness does not stop it). Firmware side: the abort is a no-op outside a run, the aid reaches DONE. The CONNECT_MOTOR barrier holds the outputs off instead of asking for a power cycle (which restarted the drive and ended the run, 2026-10-29).
 - [ ] F3: serve the description file over `uavcan.file.Read` (with the Cyphal file service of the firmware update).
 - [ ] Decide the license of `dsdl/unimoc/` (proposed: permissive, so the GUI repository can carry it; plan section 2.1).

## Motor types: ASM and EESM in the simulation and the firmware

`lib/motor_control` has the ASM flux observer and controller and the EESM
excitation controller and observer, but the runtime (`src/`, `hardware/`) has
no motor-type selection and runs the PMSM path only, and `lib/sim` has only
`PmsmModel` (`MotorSimulation` rejects other types with
`kUnsupportedMotorType`).

 - [ ] ASM plant in `lib/sim` (`AsmModel`: stator and rotor flux d/q model with rotor resistance and magnetizing inductance) behind the `MotorModel` concept; plant tests (slip, torque direction, flux build-up).
 - [ ] EESM plant in `lib/sim` (`EesmModel`: d/q model with saliency and a field winding, psi_f = L_m i_f, field voltage from `ElectricalInput::field_voltage`); plant tests (field build-up and decay, torque with and without excitation).
 - [x] DC-link model in the sim (2026-10-31, `lib/sim/src/dc_link_model.hpp`): capacitor, battery with internal resistance and a contactor (`unimoc.sim.dc_link_capacitance`, `.battery_resistance`, `.contactor_closed`; the ideal source stays the default), the body-diode charging of the open bridge as an averaged six-step rectifier (decision 2), the dead time of the bridge. Open: the chopper (with the fourth half bridge), a test of the regenerative derating against the capacitor.
 - [ ] Motor-type selection in the runtime: ASM path (flux observer and controller; the `atan2` of `asm_flux_observer::flux_angle` must go without libm before it enters the control cycle) and EESM path (excitation controller and observer, excitation setpoint over Cyphal, the open "excitation" of the SIL setpoint item); closed-loop sim tests per motor type.
 - [ ] Hardware: the excitation and chopper board (separate module, not the drive) on the bench, after it exists.

## Hot path and Zephyr port

### Without hardware

 - [x] Update to the Zephyr 4.5 release (done 2026-10-31, `docs/zephyr_port_plan.md`, "Zephyr 4.5 migration"; NSOS of 4.5 maps the multicast options, source check only): bump the revision in `west.yml` and check the module allowlist, work through the 4.5 migration guide and release notes (devicetree bindings, Kconfig renames, the STM32 FDCAN and ADC drivers, `native_sim` and NSOS, ZMS and settings, ztest and twister, the C++ library settings), the Zephyr SDK version required by 4.5 in `docs/zephyr_workflow.md`, the CI workflows (`ci-zephyr.yml`, `build-firmware.yml`) and `README.md` ("Requirements"); `check_format.sh` and `twister_sca.sh` green on `native_sim` and `bcc`, the map file of the BCC build compared before and after (hot-path placement, flash and RAM use). Recheck whether NSOS in 4.5 passes the multicast socket options (the Cyphal/UDP fallback of the multi-node simulation). No 4.5 feature is required (`docs/zephyr_port_plan.md`, "Risks and Open Points").
 - [ ] Measure the cost of the trace `Tick()` with an armed capture on the STM32H7 (M0, target < 1 us; idle it is inside the measured 5.7 us worst sample interrupt); move it to the observer interrupt if it is too slow.
 - [ ] Decide whether the hot-path data (control state, sample) goes into DTCM on the STM32H7 and whether the hot path is built with `-O2` (`docs/hotpath_plan.md`, section 4); M0 on the Nucleo from ITCM: 1727 cycles mean, 1908 worst at `-Os`, 1426 / 1679 at `-O2`.
 - [ ] Raise the factory current PI gains per motor to the bandwidth of the running timing (kp = L * w_c, w_c = (pi / 6) / T_d; 2.7 kHz at 32 kHz PWM, 5.3 kHz at 64 kHz, `docs/hotpath_plan.md` section 3); the hoverboard default is still 2000 rad/s. Derive them from L, R and the timing automatically once the identification results are applied.
 - [x] Low-side shunts in the hot path (2026-11-03, `docs/hotpath_plan.md` section 2, "Low-side shunts"): foc0 `current-sense = "low-side"` runs the phases in PWM mode 2 (low-side pulses centered on the valley sample), `settings::CurrentSense` and `min_low_side_time` in the profile; `ResolveControlTiming()` skips divider 1 and caps the duty at 1 − f_pwm · `min_low_side_time` (`unimoc.timing.pwm`, fifth value). Sim scenario `unimoc.sim.closed_loop.low_side`, build check `unimoc.app.build.low_side` (Jacobi with an overlay). Open: no board has them; measure `min_low_side_time` (STM32H7 estimate 6 µs with 1 µs dead time) and the amplifier sign; cap only the middle phase and reconstruct the highest (more voltage).
 - [x] Control divider and PWM frequency from the inductance (2026-11-03, `lib/settings/control_timing.hpp`, `docs/hotpath_plan.md`, the header and sections 1 to 3): the control step every 1, 2, 4 or 8 half periods (`unimoc.control.divider`, 0 the smallest the board's `max_control_frequency` allows): divider 1 after every sample with an update at every boundary (RCR = 0), else after the valley samples with the update at the peak only (RCR = 1, repetition counter started at 0, measured on the Nucleo); the observer after 1 to 8 steps (at most 16 kHz), `pwm_frequency` up to 64 kHz and 0 from the inductance (30 % ripple target), `unimoc.timing.pwm` and `.pwm_suggestion`. Nucleo: 64.3 k steps/s at 32 kHz (divider 1), 64.4 k at 64 kHz (divider 2), no overrun.
 - [ ] 64 kHz PWM on Jacobi: the dead time down to about 300 ns (devicetree `dead-time-ns`, now 1000), the hot path at `-O2` (0.9 us margin before the peak at `-Os`), then raise the factory frequency. The Jacobi power stage is designed for 200 ns dead time, and the legacy 4850 power board is capable of 200 ns too (2026-11-08): measure 200 ns on the scope with each stage, then set `dead-time-ns = <200>` in `unimoc_jacobi.dts` and the Nucleo overlay and revisit `svm_duty_min/max` and the low-side estimate (`min_low_side_time` follows `dead-time-ns`).
 - [x] Factory `dtc_dead_time` of the STM32H7 profile from the devicetree `dead-time-ns` (2026-11-08); M1 measures the effective value.
 - [x] Simulation-control commands over Cyphal for deterministic runs (SIL P2): pause, fixed-cycle advance, reset (`unimoc.sim.paused`, `.advance`, `.reset`, `.cycle_count`, `.cycle_period_ns`), run-until (`unimoc.sim.run_until`, stops at a trip) and the drive's timeouts in virtual time while paused (2026-10-31, `CyphalPlatformDriveTimeUs()`, pycyphal test).

### Blocked on hardware access

 - [x] M0 on the board (2026-11-03, NUCLEO-H743ZI, outputs off): per-sample control step 1537 cycles mean, sample interrupt 2276 worst of 6250, no overrun (`docs/hotpath_plan.md`, section 4).
 - [ ] M0 with the flux observer and Hall fusion running (outputs released on a power stage).
 - [ ] M8 first step with the outputs off: preload timing, start at ARR/2, peak/valley detection from `CNT` and ADC alignment on the scope. Include a settings restart (a register write while disengaged, then engage): `foc_stm32g4::Stop()` and the re-init are not run on the board yet.
 - [ ] Verify ZMS, the flash stall, the dual-bank option bit (DBANK, bank 2 free for Cyphal firmware updates) and the size of the settings partition on the NUCLEO-H743ZI and Jacobi (settings plan, "Not verified on hardware").
 - [ ] M8 remainder: Hall table, closed current loop with the 50 A limit.
 - [x] First Zephyr release: `develop` merged into `main` and published as v0.1.0 (2026-10-24) by the release workflow (`docs/release_plan.md`).
 - [x] Release setup on GitHub: `RELEASE_TOKEN`, merge methods per branch, required checks (check run names) and "Automatically delete head branches" off, which had deleted `develop` after the first release merge.
 - [ ] Repository settings (by hand): add the check `firmware (nucleo_h743zi)` of `.github/workflows/build-firmware.yml` to the required checks of the `develop` ruleset, next to the other firmware builds.

## Unit types

The tags were replaced by dimension exponents (decision 4); the remaining
escapes are counted in `scripts/unit_escapes_baseline.txt` and may only go
down. The boundaries are documented in `CONTEXT.md` ("Unit Boundaries").


## Software-in-the-loop simulation

 - [ ] P2: One-process simulator coordinator with N independent motor nodes and one shared virtual clock; a barriered multi-motor step with coherent timestamps and deterministic ordering.
 - [ ] P2: Independent constant/viscous load models and tests for isolated motors, unique node IDs, reset and repeatability; a shared load interface for shaft, wheel, battery, differential and torque-vectoring models.
 - [ ] P2: Host simulator executable around the coordinator.
 - [ ] P2: Excitation setpoints into the normal controller input path (torque, d/q current, speed, position and the UDRAL servo done); depends on the EESM runtime path.
 - [ ] P2: Telemetry and heartbeat with explicit virtual-time semantics (published in real time today; the telemetry carries the control-cycle count).
 - [ ] P2: Simulation-control command snapshot (pause, fixed-cycle advance, reset, run-until and clock status done).
 - [ ] P2: Protocol and localhost end-to-end tests for multi-node operation (single node done: `unimoc.app.cyphal_integration`).
 - [ ] P3: The command-and-control GUI moved to cypilot (section "GUI" below).
 - [ ] P4: Startup, observer, field-weakening, homing and control-mode transition scenarios.
 - [ ] P4: Vehicle-level models: shared battery and DC link, wheels, differential load, road load, torque vectoring.
 - [ ] P4: Fault injection, profiling and large-N motor trace tooling; process-per-motor hosting only once the coordinator is stable and a concrete isolation need exists.
 - [ ] Twister scenarios (ztest on `native_sim`) for the coordinator and protocol tests as they appear; plant and runtime tests exist in `tests/lib/sim` and `tests/sim/`.
 - [ ] Keep the focused three-phase validation green; ask before widening the validated scope in `AGENTS.md` to the SIL scenarios (`tests/sim/`, `lib/sim`).
 - [ ] Document simulator setup, launch commands, supported model fidelity and GUI workflows in `README.md`.

## Done

### Hardware

 - [x] Jacobi board definition and BCC removal (2026-11-03): `boards/unimoc/jacobi/` (`unimoc_jacobi`, STM32H743VIT6, pins of the `docs/drive4850_h7_plan.md` proposal, phase voltages on PA7/PB1/PC3_C, motor temperature on PC5, TCAN334 on FDCAN1, RTT log, 8 MHz HSE assumed); the app builds (197 KB flash, 84 KB RAM). The BCC board, `hardware/zephyr_bcc`, the `unimoc,foc-stm32g4` and `unimoc,ebike` bindings and `app/boards/bcc.conf` removed; CI, twister and the firmware artifacts build `unimoc_jacobi` in place of `bcc`. Open: the schematic, `PWM_EN` (PE7) driven by the platform, the phase voltage dividers, the HSE, the GitHub ruleset check `firmware / firmware (jacobi)`.

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
 - [x] C0: the commissioning bus contract (2026-10-29, plan section 5.4, decision 6): ExecuteCommand codes 0x0200 .. 0x0205 (START_STARTUP_AID, CONTINUE, SET_READING, ABORT; the identification codes reserved, BAD_COMMAND), the answer with its reason in `output`; `unimoc.drive.CommissioningStatus.0.1` on subject 215 (`uavcan.pub.commissioning.id`) on every change, at 2 Hz in a run and 0.5 Hz otherwise. The startup aid on it: the `unimoc.startup.run`, `step`, `ext_current_A` and `ext_vdc_V` registers removed (`unimoc.startup.results` and the calibration stay), the old `CMD_MEASURE_*` codes and `meas_*` subjects 208 .. 210 removed. CONTINUE is refused while a step measures or waits for its reading, and needs `motor-connected` at CONNECT_MOTOR. ztests `lib_cyphal`, `sim_cyphal_drive`, `sim_overcurrent`, the pycyphal integration test.
 - [x] One-shot actions as commands (2026-10-29): `unimoc.timing.reset` replaced by the vendor command RESET_TIMING (`0x0300`, diagnostics range `0x0300` .. `0x03FF`, allowed in every state); the never implemented `CMD_GET_*` codes `0x0001` .. `0x0008` removed (telemetry and the drive status carry the values); `0x0001` .. `0x01FF` (also the former `CMD_MEASURE_*`) are retired, not reused. Enabling stays on the readiness subject (it must be refreshed, a lost master disengages the drive), stopping stays EMERGENCY_STOP and readiness SLEEP. ztests `lib_cyphal`, `sim_cyphal_drive`, the pycyphal integration test.
 - [x] Homing as a command (2026-10-29): START_HOMING (`0x0400`, motion range `0x0400` .. `0x04FF`) in place of the never subscribed `uavcan.sub.homing_trigger` (105); refused with its reason unless engaged in position control, with homing off or while a run searches. The outer loop runs the shaft at `unimoc.control.pos.homing_speed` with at most the new `unimoc.control.pos.homing_current` (2 A, 0: off) until it stalls at the stop (0.1 s at the bound, below half the speed), takes the stop as zero and holds it until the setpoint changes; a disengage or another mode ends the run in FAULT. The state on `uavcan.pub.homing_state` (204, `uavcan.primitive.scalar.Natural8.1.0`) on every change, at 2 Hz searching and 0.5 Hz otherwise. ztests `sim_speed_control`, settings; the pycyphal integration test. Open: a limit switch or index pulse as the home event (the BCC has no input for it), homing on hardware.

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
 - [x] Current PI on every sample (2026-11-03): the control step runs in every sample ISR (64 kHz) and writes the CCR preload directly; the duty DMA is gone; the observer hands one angle reference per cycle that the steps rotate forward; HFI removed (Hall sensors at low speed). Delay 1.5 instead of 4.5 half periods: about 3x the current bandwidth (`docs/hotpath_plan.md`, sections 2 and 3).
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
 - [x] Startup aid phase 2 on a motor (2026-10-29): PHASE_ADC_ALIGNMENT forced 0.8/0.2/0.2 duty (about 190 A in a 0.1 Ohm motor at 48 V, an overcurrent trip on any real motor); it now injects the small alpha voltage of CURRENT_SENSE_CALIBRATION. The overcurrent check of the run covers phase 1 too (a connected motor faults it). ztests `sim_cyphal_drive`: phase 2 on the plant's motor to DONE with gains 1, a motor in phase 1 faults the run. Open on the board: the 0.2 % injection against the dead time of the BCC (no dead-time compensation while the duties are forced).
 - [x] Startup calibration applied (2026-10-29): the control cycle corrects every sample, `(i_raw - adc_offset) * adc_gain` per phase and `v_raw * adc_gain_vdc`; control, outer loop, telemetry, cart status and traces use the calibrated values, the startup aid the raw ones. Phase C is calibrated too (`unimoc.startup.adc_offset_c`, `gain_c`, new registers): Clarke uses all three phases. The unused `phase_balance_*` (`unimoc.motor.balance.gain_a/b/c`, CMD_MEASURE_BALANCE, subject 211) were removed (2026-10-29, decision 2 of `docs/motor_identification_plan.md`).
 - [x] A second abort of the startup aid (`unimoc.startup.run` = 0 in FAULT) forced the neutral duties again and nobody released them: the next engage ran without the current loop. `RequestAbort()` is a no-op unless a run is active (2026-10-29, ztest in `sim_cyphal_drive`).
 - [x] Startup aid reaches DONE (2026-10-29): the step after CURRENT_SENSE_CALIBRATION ran `TransitionTo(kDone)` instead of `RunDone()`, so the calibration was never committed and the outputs stayed released at DONE. The current gains now subtract the zero offset of ADC_OFFSET_CAL. ztests in `sim_overcurrent` and `sim_cyphal_drive` (a run to DONE over the bus, then engage).
 - [x] P1: Application state in units (2026-10-29): `HwStartup` thresholds, results and intermediate values (offsets, noise, OC limit in A, the DC link in V, gains and tolerances as `Ratio`), `SetExtCurrent()`/`SetExtVdc()` and `StartupRegisters::ext_current_`/`ext_vdc_` typed; the bus still carries `real32` in A and V. The `double` accumulators stay, a documented exception (CONTEXT.md, "Unit Boundaries": float loses the alignment-sweep variance). Results bit-identical (checked against the previous code); ztests in `sim_overcurrent` (a run through every step) and `sim_cyphal_drive`. Unit escapes 311 to 309.
 - [x] P1: Observers in units (2026-10-29): the measurement noise as typed variances (`unit::AngleVariance`, `unit::AngularVelocityVariance`, `unit::AngleVarianceSpeedWeight`, literals `_rad2`, `_rad4_per_s2`) in `MechanicalObserver::UpdateAngle()`/`UpdateSpeed()` and `R`, in `hall_fusion` (with the sector timing as `unit::Time`) and in `SlowUpdate::FluxNoise()`; the settings `mech_obs_r`, `hall_r_edge`, `hall_r_sector`, `pmsm_flux_obs_r_speed` in units (same register and storage encoding). Bit-identical results; unit escapes 314 to 311. Stays as it is: the Kalman covariance of `mechanical_observer` (mixed-unit state vector, `float`), the `atan2` of `asm_flux_observer::flux_angle` (libm; see "Motor types").

### Software-in-the-loop simulation

 - [x] P0: Virtual-time cycle contract (`tests/sim/irq_cadence`), injectable ADC/PWM I/O, reusable runtime (`current_control::StartRuntime()`), virtual clock harness (`hardware/zephyr_sim/sim_control.hpp`).
 - [x] P1: PMSM plant (`lib/sim/`, `MotorSimulation`, 120 degree Hall sensors) with plant tests, connected to `hardware/zephyr_sim`; end-to-end step response, cadence and finite-state tests (`tests/sim/closed_loop`).
 - [x] pycyphal tests in virtual time (2026-10-30): `wait`, `hold`, `publish_for` and the settle time of `latest` in `tests/integration/cyphal_sim/sim_client.py` count the control cycles of the telemetry instead of the wall clock. The simulation runs only about 1.4 times faster than real time, so on a loaded CI runner it fell behind and the readiness timeout and the holds were too short (10 of 25 failed on PR #40, on `develop` too under the same load). The wheel command and limp-home tests run with `unimoc.cart.timeout` at 0.3 s: the default 0.1 s is shorter than the pauses of the test client on a loaded runner.
