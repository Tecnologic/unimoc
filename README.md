# UNIMOC — Universal Motor Control

[![Zephyr CI (main)](https://github.com/Tecnologic/unimoc/actions/workflows/ci-zephyr.yml/badge.svg?branch=main&event=push)](https://github.com/Tecnologic/unimoc/actions/workflows/ci-zephyr.yml?query=branch%3Amain+event%3Apush)
[![Zephyr CI (develop)](https://github.com/Tecnologic/unimoc/actions/workflows/ci-zephyr.yml/badge.svg?branch=develop&event=push)](https://github.com/Tecnologic/unimoc/actions/workflows/ci-zephyr.yml?query=branch%3Adevelop+event%3Apush)
[![Release](https://img.shields.io/github/v/release/Tecnologic/unimoc?include_prereleases)](https://github.com/Tecnologic/unimoc/releases/latest)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)

**UNIMOC** (**UNI**versal **MO**tor **C**ontrol) is a platform-independent
C++ library and [Zephyr RTOS](https://zephyrproject.org/) firmware for
Field-Oriented Control (FOC) of multi-phase electric motors.  It targets
embedded micro-controllers (STM32 series, first the STM32G473 battery case
controller).  The algorithm code is portable and tested with Zephyr's ztest
and twister on `native_sim`.

Zephyr is the primary OS: logging, Cyphal/CAN FD, and settings run in
Zephyr threads.  Only the FOC hot path runs in zero-latency interrupts.  The
same firmware runs on Linux through Zephyr's `native_sim` with a motor
simulation.

---

## Table of Contents

1. [Features](#features)
2. [Supported Motor Types](#supported-motor-types)
3. [Library Structure](#library-structure)
4. [Cyphal (UAVCAN v1) Interface](#cyphal-uavcan-v1-interface)
   - [Node ID and Plug-and-Play](#node-id-and-plug-and-play)
   - [Node Identity](#node-identity)
   - [Settings via Registers](#settings-via-registers)
   - [Setpoints via Subjects](#setpoints-via-subjects)
   - [Telemetry Subjects](#telemetry-subjects)
5. [NVM-Backed Settings](#nvm-backed-settings)
6. [Hardware Bring-Up Guide](#hardware-bring-up-guide)
   - [Equipment Required](#equipment-required)
   - [Phase 1 — No Motor Connected](#phase-1--no-motor-connected)
   - [Phase 2 — Motor Connected](#phase-2--motor-connected)
   - [Calibration Results and NVM Commit](#calibration-results-and-nvm-commit)
   - [Aborting or Recovering from FAULT](#aborting-or-recovering-from-fault)
7. [Getting Started](#getting-started)
   - [Running the Tests](#running-the-tests)
   - [Generating the API Documentation](#generating-the-api-documentation)
   - [Building Firmware (Zephyr)](#building-firmware-zephyr)
8. [Contributing](#contributing)
9. [License](#license)

---

## Features

| Category | Capability |
|---|---|
| **Motor types** | PMSM, ASM (induction), EESM (wound-rotor synchronous) |
| **Observers** | Back-EMF / PLL (PMSM), HFI sensorless (IPMSM), ASM rotor-flux Luenberger, EESM excitation LPF |
| **Control** | MTPA, field-weakening, ASM flux PI, EESM excitation PI (current or flux mode) |
| **Position** | Absolute multi-turn tracker (±4096+ rev), cascaded P+PI position controller, homing state machine |
| **Modulation** | Space Vector PWM (SVPWM), dead-time compensation |
| **Network** | Cyphal (UAVCAN v1) over CAN FD — every setting a register, arming, setpoints, telemetry, traces (cymon) and the log on the bus, also for the simulation (see [Cyphal](#cyphal-uavcan-v1-interface)) |
| **Persistence** | Live RAM settings for tuning; one flash entry per changed register (Zephyr settings on ZMS); saving only while the power stage is disabled |
| **Platform** | HAL-free algorithms; tested on Linux x86-64 with GTest; Zephyr firmware for the BCC (STM32G473) and `native_sim` |
| **Runtime** | Zephyr threads for logging, Cyphal, and settings; zero-latency IRQs only for the FOC hot path |

---

## Supported Motor Types

### PMSM — Permanent-Magnet Synchronous Motor
Surface-mount and interior-PM motors.
Control path: MechanicalObserver (back-EMF PLL) → MTPA → FieldWeakening → SVM.
Low-speed sensorless: HFI observer (requires inductance saliency, IPMSM only).

### ASM — Asynchronous (Induction) Motor
Squirrel-cage induction motors.
Control path: AsmFluxObserver (Luenberger) → AsmFluxController (PI) → MechanicalObserver (shared PLL) → SVM.

### EESM — Electrically Excited Synchronous Machine
Wound-rotor synchronous motors with external excitation (slip-rings or brushless exciter).
Stator control path identical to PMSM.  Additionally:
- **ExcitationController** — PI loop on rotor current I_f (CurrentMode) or flux ψ_f = L_m · I_f (FluxMode).
- **ExcitationObserver** — first-order low-pass filter on measured I_f → adaptive ψ_f estimate fed into MTPA.
- Excitation setpoint and mode received via Cyphal subject.

---

## Library Structure

```
lib/
├── motor_control/
│   ├── src/
│   │   ├── asm_flux_controller.hpp  # ASM rotor-flux PI controller
│   │   ├── dead_time_compensation.hpp  # Inverter dead-time compensation
│   │   ├── excitation_controller.hpp  # EESM rotor excitation PI (current / flux mode)
│   │   ├── field_weakening.hpp      # Voltage-headroom field-weakening integrator
│   │   ├── mtpa.hpp                 # Maximum-Torque-Per-Ampere (IPMSM)
│   │   ├── position_controller.hpp  # Cascaded P+PI position/speed controller + homing FSM
│   │   ├── svm.hpp                   # Space Vector PWM modulator
│   │   ├── asm_flux_observer.hpp    # ASM full-order Luenberger rotor-flux observer
│   │   ├── excitation_observer.hpp  # EESM excitation current LPF -> flux estimate
│   │   ├── hfi.hpp                   # 4-step High-Frequency Injection (IPMSM standstill)
│   │   ├── mechanical_observer.hpp  # Back-EMF observer + PLL (PMSM / shared with ASM)
│   │   ├── position_tracker.hpp     # Absolute multi-turn position + homing
│   │   └── *_system.hpp              # Rotor, stator, and three-phase coordinate systems
├── cyphal/
│   ├── can_media.hpp             # CAN interface boundary (frames in and out)
│   ├── canard_transport.hpp      # Cyphal/CAN transport on libcanard + O1Heap
│   ├── control_mode.hpp          # Cyphal setpoint-to-control-mode mapping
│   ├── cyphal_interface.hpp      # Register names + subject port IDs + command codes
│   ├── node.hpp                  # Node services: heartbeat, GetInfo, registers, commands, PnP, log
│   ├── node_identity.hpp         # Node name + hw/sw version
│   ├── register_description.hpp  # Unit, limits, effect and help of a register (registers.json)
│   ├── register_value.hpp        # uavcan.register.Value helpers, register provider boundary
│   └── transport.hpp             # Transfer-level transport contract (CAN, later UDP, tests)
├── settings/
│   ├── nvm_settings.hpp          # Typed aggregate of NVM-backed parameters
│   ├── settings_registers.hpp    # Register table with descriptions + byte encoding of one value
│   ├── settings_profile.hpp      # Immutable target capabilities and defaults
│   ├── settings_storage.hpp      # Platform key/value storage contract
│   ├── settings_store.hpp        # Live settings, snapshot, save gate, authorized mutation API
│   └── unit_symbol.hpp           # SI symbol of a unit type (registers.json)
├── system/
│   ├── rotor_angle.hpp           # Q1.31 electrical angle + revolution counter, sin/cos LUT
│   ├── rotor_system.hpp          # d/q rotating reference frame
│   ├── stator_system.hpp         # α/β stationary reference frame
│   └── three_phase_system.hpp    # Three-phase (a/b/c) vector
└── units/
    └── units.hpp                 # SI units with dimension exponents
```

---

## Firmware Architecture

The firmware is one Zephyr application (`app/`). At 32 kHz PWM the execution
contexts are (details in [`docs/hotpath_plan.md`](docs/hotpath_plan.md)):

| Context | Runs | Latency |
|---|---|---|
| Sample ISR (ADC end of injected sequence, every half PWM period, 64 kHz) | Raw current copy, overcurrent check on raw counts | Zero-latency IRQ |
| Control cycle (every fourth sample, 16 kHz) | FOC hot path: mean current, Park, PI once, four duty entries (inverse Park, HFI step, SVM), entry 0 into the CCR preload, entries 1 to 3 by DMA | Zero-latency IRQ |
| Observer (after each control cycle, 16 kHz) | Flux observer, Kalman, HFI, angles of the next cycle (`SlowUpdate`) | Software-pended normal-priority IRQ |
| Zephyr threads | Speed and position control (about 2 kHz, planned), logging (RTT), the Cyphal node, settings, startup sequencing | Normal scheduling |

Zero-latency handlers may not call kernel APIs, log, or lock.  The
hardware-specific parts are selected by the board and Kconfig, not by
`#ifdef`.  The integration status and the open items are in
[`docs/zephyr_port_plan.md`](docs/zephyr_port_plan.md) and
[`docs/zephyr_workflow.md`](docs/zephyr_workflow.md).

## Cyphal (UAVCAN v1) Interface

UNIMOC is configured, commanded and traced over
[Cyphal](https://opencyphal.org/) v1.0 on CAN FD.  The same node code runs on
the battery case controller (FDCAN3) and on `native_sim`; only the CAN
interfaces and the simulation registers are platform specific.

> **Status:** integrated and tested on `native_sim` (ztest scenarios
> `unimoc.lib.cyphal` and `unimoc.sim.cyphal_drive`, and by hand with yakut).
> The BCC builds with the node on FDCAN3 but has not run on the board; its
> SN65HVD230 is a classic CAN transceiver, so CAN FD frames run at 1 Mbit/s
> without bit rate switch (`boards/unimoc/bcc/bcc.dts`).  Not implemented
> yet: firmware update over Cyphal.  Speed, position and UDRAL servo
> setpoints run the outer loop in the control thread; the startup FSM runs
> over the commissioning commands and status message (Hardware Bring-Up Guide).  The cymon traces run on both
> platforms; their cost in the control cycle is not yet measured on the board.

### Stack

| Layer | Where | Content |
|---|---|---|
| DSDL | `dsdl/unimoc/`, `lib/cymon/dsdl/cymon/` + OpenCyphal public types | Vendor types `unimoc.drive.CurrentSetpoint.0.1`, `unimoc.drive.Telemetry.0.1`, `unimoc.sim.Plant.0.1`, the cart types `unimoc.cart.*.0.1`, the trace types `cymon.*.0.1`; Nunavut generates the C serialization at configure time |
| Transport | `lib/cyphal/canard_transport.*` | libcanard 4.0 with an O1Heap arena, one TX queue per CAN interface (redundant interfaces, mixed MTU) |
| Node | `lib/cyphal/node.*` | Heartbeat, `uavcan.node.port.List`, GetInfo, `uavcan.register.List/Access`, ExecuteCommand, plug-and-play (`uavcan.pnp.NodeIDAllocationData.1.0`), `uavcan.diagnostic.Record`, application services on register-assigned IDs (`AddServer()`) |
| Drive application | `src/drive/` | Settings registers, arming, setpoints, telemetry, commands, the cart wheel drive (`cart_control.*`) |
| Traces | `src/trace/`, `lib/cymon/` (cymon-lib 0.2) | cymon services, float mirror of the control state sampled in the control cycle |
| Zephyr glue | `hardware/zephyr_common/` | Cyphal thread, CAN interface on the chosen `zephyr,canbus`, log backend to `uavcan.diagnostic.Record` |
| Platform | `hardware/zephyr_bcc/`, `hardware/zephyr_sim/` | BCC: transceiver pin. Sim: slcan pseudo terminals, plant registers and plant trace, the register description file (`register_file.cpp`) |

libcanard, O1Heap and the public regulated data types are pinned in
`west.yml`; Nunavut is a Python requirement
(`pip install -r unimoc/scripts/requirements-cyphal.txt`).

### Node ID, identity and plug-and-play

`uavcan.node.id` is 0 on a fresh BCC: the node then requests an ID with
`uavcan.pnp.NodeIDAllocationData.1.0` (an allocator such as
`yakut monitor --plug-and-play <db>` answers) and stores the allocated ID.
The simulation starts with node-ID 10 (`--node-id=<n>` overrides it per
process, `--cart-wheel=<n>` the cart wheel).  GetInfo reports the name from `uavcan.node.description`, the
hardware version of the platform, the software version from `app/VERSION`, the
git hash as `software_vcs_revision_id` and the hwinfo device ID as unique-ID
(also the register `uavcan.node.unique_id`).  A changed node-ID or port-ID
takes effect after a restart.

### Settings via registers

Every persistent setting (`ForEachRegister()` in
`lib/settings/settings_registers.hpp`, 132 registers) is a Cyphal register
under the same name; `yakut rl <node>` lists them.  Units and floats are
`real32[1]`, counts and enums `natural8[1]`, `uavcan.node.id` and the port-IDs
`natural16[1]`, the name a string; writes accept any numeric type.  A write is
validated against the hardware capabilities and changes the RAM settings
only; a rejected write answers with the unchanged value.  The drive applies
new settings to the controller while it is disengaged (at the latest when it
engages).  While engaged, the gains of the current, speed and position loops
(`unimoc.control.current.*`, `unimoc.control.speed.*`, the position gains,
limits and tolerances), the telemetry rate and the command timeout take
effect at once without a restart (live tuning); any other change waits for
the disengage and also holds back live tuning until then.  Such a write is
reported with a `uavcan.diagnostic.Record` warning ("settings wait for the
disengage: <register>", then "waiting settings applied"), and the read-only
`unimoc.settings.pending` (bit) reads 1 while changes wait.  `ExecuteCommand STORE_PERSISTENT_STATES` (65530) saves them to
flash, refused (`BAD_STATE`) while the power stage is enabled.

### Register description file

Every register is described next to its definition: the settings in
`ForEachRegister()`, the other registers in their provider
(`RegisterProvider::Describe()`). The description holds the unit (taken from
the unit type of a setting where it has one), the limits of the validation
that hold for the register alone, when a write takes effect (`immediate`,
also while engaged; `disengaged`; `restart`) and one line of help. The
simulation writes all of it as JSON and exits:

```sh
unimoc/app/build/sim/zephyr/zephyr.exe -flash=/tmp/fresh.bin --describe-registers=registers.json
```

One entry per register in the order of `uavcan.register.List`: name, value
type and size, `mutable`, `persistent`, the value of a fresh simulated node as
`default`, `unit`, `min`, `max`, `applies` and `help`; the schema is in
`docs/settings_plan.md`. The firmware CI and every release attach it as
`unimoc-registers-<version>.json`. Tools such as
[cypilot](https://github.com/Tecnologic/cypilot) pick it by the software
version and VCS revision of GetInfo. The limits are a hint: the node's
validation decides, also on limits between registers or of the hardware
(named in the help).

### Ports of the drive

| Register | Default | Type | Function |
|---|---|---|---|
| `uavcan.sub.readiness.id` | 65534: 1024 + node-ID | `reg.udral.service.common.Readiness.0.1` | `ENGAGED` (3) releases the power stage, `SLEEP`/`STANDBY` disable it; one subject per drive (below) |
| `uavcan.sub.current_setpoint.id` | 107 | `unimoc.drive.CurrentSetpoint.0.1` | d/q current setpoint [A] |
| `uavcan.sub.torque_setpoint.id` | 100 | `uavcan.si.unit.torque.Scalar.1.0` | Torque [N m], converted to i_q with 3/2 p ψ_PM |
| `uavcan.sub.speed_setpoint.id` | 101 | `uavcan.si.unit.angular_velocity.Scalar.1.0` | Mechanical speed [rad/s], speed loop of the control thread, limited to the motor speed range |
| `uavcan.sub.position_setpoint.id` | 102 | `uavcan.si.unit.angle.Scalar.1.0` | Mechanical position [rad] from home (the estimate at start, or the stop of the last homing run) |
| `uavcan.sub.servo_setpoint.id` | 108 | `reg.udral.physics.dynamics.rotation.Planar.0.1` | UDRAL servo: the commanded (non-NaN) field selects position, speed or torque |
| `uavcan.pub.telemetry.id` | 212 | `unimoc.drive.Telemetry.0.1` | Current controller trace at `unimoc.telemetry.rate` (20 Hz) |
| `uavcan.sub.wheel_command.id` | 110 | `unimoc.cart.WheelCommand.0.1` | Cart: torque per wheel from the VDC; the drive takes `unimoc.cart.wheel` |
| `uavcan.sub.pedal_state.id` | 111 | `unimoc.cart.PedalState.0.1` | Cart: pedals for limp-home mode |
| `uavcan.sub.battery_status.id` | 112 | `unimoc.cart.BatteryStatus.0.1` | Cart: current limits of the BMS, replace `unimoc.battery.limits.*` while fresh |
| `uavcan.pub.drive_status.id` | 214 | `unimoc.cart.DriveStatus.0.1` | Cart: wheel speed, torque, drive and brake torque limits, DC link, flags at `unimoc.cart.status_rate` (50 Hz) |
| `uavcan.pub.commissioning.id` | 215 | `unimoc.drive.CommissioningStatus.0.1` | Commissioning (startup aid): step, state, needs, progress, passed and failed steps; on every change, 2 Hz in a run, 0.5 Hz otherwise ([Commands and Status](#commands-and-status)) |
| `uavcan.pub.homing_state.id` | 204 | `uavcan.primitive.scalar.Natural8.1.0` | Homing: 0 IDLE, 1 SEARCHING, 3 DONE, 4 FAULT; on every change, 2 Hz while searching, 0.5 Hz otherwise ([Homing](#homing)) |
| `uavcan.pub.sim_plant.id` (sim only) | 213 | `unimoc.sim.Plant.0.1` | Plant truth: phase currents, shaft angle and speed, torque, Hall code |
| `uavcan.srv.cymon_variables.id` | 120 | `cymon.GetVariableList.0.1` | Trace variables: index -> name and unit |
| `uavcan.srv.cymon_buffer_info.id` | 121 | `cymon.GetBufferInfo.0.1` | Capture limits and the tick period (62.5 us) |
| `uavcan.srv.cymon_setup.id` | 122 | `cymon.SetupCapture.0.1` | Channels, sample period, samples, pre-trigger depth |
| `uavcan.srv.cymon_arm.id` | 123 | `cymon.ArmTrigger.0.1` | Arm (rising, falling, hysteresis, immediate) or disarm |
| `uavcan.srv.cymon_read.id` | 124 | `cymon.ReadSamples.0.1` | Capture state; once complete, the frames page by page |
| `uavcan.pub.cymon_trigger.id` | 65535 | `cymon.TriggerEvent.0.1` | Published when a capture triggers (disabled by default) |
| `uavcan.sub.cymon_trigger.id` | 65535 | `cymon.TriggerEvent.0.1` | Triggers the armed capture on another node's event (disabled by default) |

Each port has a read-only `.type` register.  Two enabled ports may not share
a subject-ID (or two services a service-ID): such a write is rejected, so to
swap two IDs, disable one port (65535) first.  The cymon trigger is the
exception: publishing and subscribing one shared trigger subject lets every
node of a bus trigger the others.  Readiness and setpoints must be
refreshed within `unimoc.control.command_timeout` (0.5 s): a missing
readiness disengages the drive, a missing setpoint falls back to zero
current.  The current vector is limited to `unimoc.motor.limits.i_max`.

The battery current is limited as well, in every mode: the drive estimates
it from the mechanical power of the q current (k_t i_q ω / V_dc, losses
ignored) and scales i_q to `unimoc.battery.limits.drive_current` when
motoring and to `unimoc.battery.limits.charge_current` when braking.  The
charge limit falls linearly from `unimoc.battery.limits.regen_derate_voltage`
(55 V) to zero at `unimoc.battery.limits.trip_voltage` (58 V, at most the
hardware's 60 V), where the control cycle trips the power stage.  The trip
switches all transistors off: above the line-to-line back-EMF peak the
motor still feeds the DC link through the body diodes, so the trip level
must stay above the battery voltage and a brake chopper has to hold the
DC link (section 7.1 of `docs/system_plan.md`).  The BCC has none (an
accepted risk); the new 4850 hardware gets a fourth half bridge that
drives a brake resistor with a PMSM or the excitation winding of an EESM
(section 5.2.1).  The BCC voltages
are accepted values, to be checked against its schematic during bring-up.

### Cart wheel drive

The cart messages (`dsdl/unimoc/cart/`, section 6.2 of
`docs/system_plan.md`) make the drive one wheel of the cart:

- **Wheel command.** The first wheel command after the engage makes the
  drive a cart drive until it disengages: the torque of its wheel
  (`unimoc.cart.wheel`, 0 front left to 3 rear right; `unimoc.cart.reversed`
  for a mirrored wheel) replaces the other setpoints and applies on
  reception, limited like a torque setpoint.  Arming stays with the readiness.
- **Limp-home.** Without a wheel command for `unimoc.cart.timeout` (0.1 s,
  at most the command timeout) the torque comes from the pedal state:
  brake pedal first, against the motion and fading out below
  `unimoc.cart.standstill_speed` (1 rad/s), drive pedal forward only, both
  up to `unimoc.cart.limp_home_torque` (5 N m) and ramped at
  `unimoc.cart.torque_ramp` (50 N m/s).  The readiness timeout waits during
  limp-home.  Without the pedals too the torque ramps to zero and then the
  readiness timeout disengages.  A fresh wheel command takes over again
  only at standstill.
- **Battery limits.** A fresh battery status replaces
  `unimoc.battery.limits.drive_current` and `.charge_current` by the BMS
  limits times `unimoc.cart.battery_share` (the drives of one battery share
  them); with the contactor open or precharging the drive takes no battery
  current.  Without the BMS the registers apply again.
- **Drive status.** Wheel speed and torque estimate in the cart frame, the
  torque the drive can apply now with and against the motion (motor
  current limit and battery power window), DC-link voltage and estimated
  battery current, temperatures in K (NaN when not measured), the power
  stage, engaged, limp-home, regen derated and trip flags.
- **Bus-off.** The CAN controller leaves bus-off on its own; the drive
  drops every cart input at once (the torque ramps to zero), counts it in
  `unimoc.can.bus_off_count` and logs the return to the bus.

An overcurrent or overvoltage trip or a power stage fault while engaged disengages the drive;
it engages again only after a readiness other than `ENGAGED` (re-arm).
`STANDBY` while disengaged acknowledges a latched trip or fault without
releasing the outputs; the settings can only be stored without one.

### E-bike (battery case controller)

With `CONFIG_UNIMOC_EBIKE=y` (off by default) the drive becomes the motor of a
pedal-assist e-bike ([`docs/ebike_plan.md`](docs/ebike_plan.md)). The crank torque and cadence sensor (E-Rider
T9N) and the display (KT-LCD8, Kunteng protocol) feed the assist law (`lib/ebike/`, `src/ebike/`):

- Low speed: the motor torque follows the pedal torque (`unimoc.ebike.torque_ratio`); above the crossover speed
  the motor power follows the pedal power. The assist ratio is the base ratio of the display level
  (`unimoc.ebike.gain_1` to `gain_5`) times `exp(pedal power / unimoc.ebike.power_scale)`, limited to
  `unimoc.ebike.max_ratio`. It fades out below the speed limit of the display, stays within the torque limit of
  the drive and ramps (`rise_time`, `fall_time`).
- The e-bike control engages the drive on demand: while the crank turns at a display level above 0, and for
  `unimoc.ebike.engage_hold` afterwards. The readiness `ENGAGED` of the bus no longer engages it; `SLEEP` and
  `STANDBY` release it and keep it released until the pedals stood still for `unimoc.ebike.rearm_time`, as a trip
  or a refused engage do. Without a display request for `unimoc.ebike.display_timeout` the level counts as 0.
- Registers (all persistent): `unimoc.ebike.pulses_per_rev`, `wheel_circumference` [m], `gear_ratio`,
  `battery_cells`, `nominal_voltage`, `gain_1` to `gain_5`, `power_scale`, `max_ratio`, `torque_ratio`,
  `torque_deadband`, `min_cadence`, `speed_taper`, `rise_time`, `fall_time`, `engage_hold`, `rearm_time`,
  `display_timeout`; the direction is `unimoc.cart.reversed`. A change while engaged waits for the disengage. The
  defaults are untuned starting values: try them on the bench first.
- BCC wiring (from the schematic): the display transmit line is PB10 (USART3_TX); the display receive line is
  on PB15, which has no UART receiver on the STM32G473, so TIM15 channel 2 captures its edges and a software
  receiver decodes the bytes; the cadence pulse is PA5; the crank torque is a channel of the injected ADC
  sequence behind a 2/3 divider.
- `native_sim`: a simulated rider and display (`hardware/zephyr_sim/sim_ebike.hpp`) drive the control in the
  tests (`sim_cyphal_drive` attaches the control itself); the simulation app keeps the bus readiness for the
  pytest integration tests, so `CONFIG_UNIMOC_EBIKE` is off there too.

### Readiness per drive and the heartbeat status

The factory value of `uavcan.sub.readiness.id` is 65534, "follows the
node-ID": the drive subscribes the readiness on subject 1024 + node-ID (1034
for node 10), so each drive of a bus is armed on its own subject, and a tool
or the VDC arms several drives by publishing on each of their subjects.
Writing another subject-ID fixes it (65535 disables it), writing 65534
returns to the node-ID; the register keeps the written value. A drive
without a node-ID (plug-and-play) subscribes once the ID is allocated. Two
ports on one subject are rejected, including the readiness subject of the
node-ID. Settings saved before this scheme keep their readiness subject
until a factory reset.

The vendor-specific status code of the heartbeat gives the drive state
without any other subscription (`cyphal::vendor_status`,
`docs/gui_plan.md`, section 3):

| Bits | Meaning |
|---|---|
| 0 to 2 | Power stage: 0 disabled, 1 enabled, 2 fault, 3 blocked (external enable missing) |
| 3 | Engaged |
| 4 | Trip latched (overcurrent or overvoltage); `STANDBY` acknowledges it |
| 5 | Emergency stop latched until a restart (health CAUTION as well) |
| 6 | Settings wait for the disengage (`unimoc.settings.pending`) |
| 7 | Local control: the e-bike control engages the drive, the bus readiness does not |

### Commands (uavcan.node.ExecuteCommand)

| Command | Effect |
|---|---|
| 65535 RESTART | Disengages and restarts (the simulation re-executes itself) |
| 65532 FACTORY_RESET | Factory settings in RAM and flash; refused while engaged |
| 65531 EMERGENCY_STOP | Disengages; engaging is refused until a restart |
| 65530 STORE_PERSISTENT_STATES | Saves the RAM settings to flash; refused while engaged |
| 768 (`0x0300`) RESET_TIMING | New statistics of the interrupt timing (below), in every state |
| 1024 (`0x0400`) START_HOMING | Starts a homing run ([Homing](#homing)); refused unless engaged in position control |
| 512..517 (`0x0200`..`0x0205`) | The commissioning commands of the startup aid and the motor identification ([Commands and Status](#commands-and-status)) |

Codes below `0x8000` are UNIMOC's own: `0x0200`..`0x02FF` commissioning,
`0x0300`..`0x03FF` diagnostics, `0x0400`..`0x04FF` motions; `0x0001`..`0x01FF` held the former
`CMD_GET_*` and `CMD_MEASURE_*` codes and are not reused.  A one-shot action is a command, not a
register write; enabling stays on the readiness subject, which the drive
needs refreshed (a lost master disengages it), and stopping is
EMERGENCY_STOP or readiness SLEEP.

### Homing

The position counts from the estimate at the start until a homing run sets
the home at a mechanical stop.  START_HOMING (`yakut cmd 10 1024`), while
engaged in position control, runs the shaft at `unimoc.control.pos.homing_speed`
(5 rad/s; the sign is the direction) with at most
`unimoc.control.pos.homing_current` (2 A; 0 turns homing off).  When the
shaft stalls against the stop, 0.1 s at that current below half the homing
speed, the stop becomes position zero and the drive holds it until the
position setpoint changes; the next setpoint is in the new frame.  A
disengage or another control mode ends a run in FAULT; a new run may start
at any time.  A restart of the controller (settings applied, power cycle)
returns the home to the start position (IDLE).

### Traces (cymon)

The drive records up to 8 signals of the control cycle into a RAM ring
buffer (8 KB, `CONFIG_UNIMOC_TRACE_*`) and a tool reads the capture back over
the bus: the cymon protocol of [cymon-lib](lib/cymon) (`lib/cymon`, 0.2),
driven by the **Scope** page of the host tool
[cymon](https://github.com/Tecnologic/cymon).  The control cycle copies its
state into a float mirror (`src/trace/trace_signals.hpp`) and samples it with
`cymon::Device::Tick()` at 16 kHz (zero latency: no lock, no kernel call, no
function pointer); the services run in the Cyphal thread
(`src/trace/trace_node.*`).

| IDs | Variables |
|---|---|
| 0..11 | `i_d`, `i_q`, `i_d_ref`, `i_q_ref` [A], `u_d`, `u_q` [V], `electrical_angle` [rad, 0 to 2 pi], `electrical_speed` [rad/s] (observer, one cycle late), `dc_link_voltage` [V], `i_a`, `i_b`, `i_c` [A] (newest sample) |
| 12.. (sim only) | Plant truth: `sim.i_a`..`sim.i_c`, `sim.i_d`, `sim.i_q`, `sim.electrical_angle`, `sim.electrical_speed`, `sim.mechanical_speed`, `sim.torque` |

The sample period is a whole multiple of the control cycle; a capture of
8 channels by 256 frames takes about 0.4 s of bus time at 1 Mbit/s.

### Interrupt timing (M0)

With `CONFIG_UNIMOC_HOT_PATH_TIMING=y` the platform measures the sample
interrupt of each control boundary (including the control cycle), the control
cycle alone and the observer in cycles of the DWT counter (170 MHz on the
BCC; `hardware/hot_path_timing.hpp`).  Read-only registers:

| Register | Type | Content |
|---|---|---|
| `unimoc.timing.sample_isr`, `.control_cycle`, `.observer` | natural32[4] | min, mean, max [cycles], number of records |
| `unimoc.timing.overruns` | natural32[2] | duty plans applied late, control cycles that started while the observer still ran |
| `unimoc.timing.clock_hz` | natural32[1] | the timestamp clock |

```sh
west build -b bcc unimoc/app -d unimoc/app/build/bcc -- -DCONFIG_UNIMOC_HOT_PATH_TIMING=y
yakut call 10 uavcan.register.access '{name: {name: unimoc.timing.control_cycle}}'
yakut cmd 10 768                               # RESET_TIMING: new statistics
```

The times start at the first instruction of the handler (the 12 cycles of
the exception entry are not included).  On `native_sim` the counts are real,
the times are 0: simulated time stands still inside a cycle.

```sh
cymon serve -i socketcan:vcan0 --dsdl unimoc/dsdl/unimoc --open      # simulation
cymon serve -i canable:/dev/ttyACM0 --dsdl unimoc/dsdl/unimoc --open # BCC, CANable 2.0
```

### Logging

The bench log of the BCC goes to SEGGER RTT; USB and the shell are not used.
Every Zephyr log line up to `CONFIG_UNIMOC_CYPHAL_LOG_LEVEL` (info) and the
drive events (engaged, disengaged and why) are also published as
`uavcan.diagnostic.Record`.

### Simulation over Cyphal

The simulation (`native_sim`) runs in real time with the PMSM plant and has
two kinds of CAN interfaces:

- **SocketCAN `vcan0`** (CAN FD), the same Zephyr CAN driver path as the
  board.  Needs the `vcan` kernel module; `--can-if=<name>` selects another
  interface.
- **slcan pseudo terminals** (classic CAN), for hosts without `vcan` (WSL2
  kernels without CAN): `uart_1` to `uart_4`, one tool process each; the
  terminals form one bus.

```sh
pip install -r unimoc/scripts/requirements-cyphal-tools.txt   # yakut, python-can
west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim

# with vcan (once, as root): modprobe vcan; ip link add vcan0 type vcan;
#                            ip link set vcan0 mtu 72 up
unimoc/app/build/sim/zephyr/zephyr.exe        # prints "uart_1 connected to pseudotty: /dev/pts/N"

. unimoc/scripts/sim_yakut_env.sh             # vcan0, or: . unimoc/scripts/sim_yakut_env.sh /dev/pts/N
yakut call 10 uavcan.node.getinfo
yakut rl 10                                   # all registers
yakut reg 10 unimoc.sim.shaft_mode 1          # imposed speed (0 rad/s: locked rotor)
yakut pub --period 0.05 1034:reg.udral.service.common.Readiness '{value: 3}' \
              107:unimoc.drive.CurrentSetpoint '{i_d: 0, i_q: 5}'    # second terminal
yakut sub 212:unimoc.drive.Telemetry 213:unimoc.sim.Plant           # third terminal
yakut sub uavcan.diagnostic.Record
yakut cmd 10 65530                            # store the settings (flash.bin in the working directory)
```

Several drives on one bus: `unimoc/scripts/sim_cluster.sh` starts N drives
(default 4) on `vcan0`, drive k with node-ID 10 + k, its own flash file and
unique-ID, and cart wheel k mod 4 (`--cart-wheel=<n>`, like `--node-id`, sets
`unimoc.cart.wheel` of one process); `--setup-vcan` creates the interface
with sudo, `--help` lists the options. Ctrl+C stops all of them.

With slcan, each yakut process needs its own terminal (`/dev/pts/N`,
`N+1`, ...); the terminal numbers can change when the simulation restarts.
Simulation registers (volatile): `unimoc.sim.shaft_mode` (0 free, 1 imposed
speed), `unimoc.sim.imposed_speed` [rad/s], `unimoc.sim.load_torque` [N m],
`unimoc.sim.dc_link_voltage` [V], `unimoc.sim.external_enable`, and
`unimoc.sim.plant_follows_settings` (the plant takes the motor parameters of
the settings when they are applied).

Deterministic runs: `unimoc.sim.paused` (bit) stops the cycle timer, also
across a settings restart of the controller; then a write of
`unimoc.sim.advance` (natural32, at most 16000 = 1 s) runs that many control
cycles before the response.  `unimoc.sim.cycle_count` and
`unimoc.sim.cycle_period_ns` (62500 at 32 kHz PWM) give the virtual time; the
count restarts with the controller.  A write of 1 to `unimoc.sim.reset`
(refused while the power stage is enabled) puts the plant back to
standstill, the virtual time to 0 and restarts the controller with the
applied settings; register inputs and a pause stay.  The drive's readiness and setpoint
timeouts still run in real time, so keep publishing the readiness while
paused.

The rotor angle of the controller comes from its observer only (the Hall
fusion is milestone M5/M6 of `docs/hotpath_plan.md`); at standstill the
estimate drifts, so the current loop is meaningful in the controller frame
(telemetry), not yet as a torque on the simulated shaft.

---

## NVM-Backed Settings

All settings are aggregated in `unimoc::settings::NvmSettings`
(`lib/settings/nvm_settings.hpp`).  Runtime ownership is provided by
`unimoc::settings::SettingsStore`.  The design is:

- **Validated** by target capability and cross-field checks, before a change
   becomes live and when the stored registers are loaded. Unusable stored
   registers are replaced by the target factory profile.
- **Complete** — every tunable parameter from every controller and observer
  lives in a single flat struct.
- **Live and persistent** — operations change the RAM copy at once (live
  tuning, no flash access). `SettingsOperations::SaveToFlash()` writes only the
  registers that changed since the last load or save, one flash entry per
  register under its Cyphal register name; a register that is back at its
  factory value is erased. It is refused (`kPersistBlocked`) unless the power
  stage is disabled, because a flash write can stall the zero-latency control
  cycle. A register that is not stored has its factory value, so new firmware can
  add registers without invalidating stored settings.
- **Factory motor** — the motor defaults describe the cart wheel: a 6.5"
  36 V hoverboard hub motor with a 3.50-6 pneumatic tyre (15 pole pairs,
  0.15 Ω, 0.35 mH, ψ 23 mWb, k_t 0.52 N m/A, 15 A, current PI at about
  320 Hz). These are typical values, not a measurement of the actual motor;
  the platform profiles leave them as they are and only set the power stage
  limits. The simulation runs this motor too; the sim tests that were written
  for a smaller motor load it from `tests/common/sim_test_motor.hpp`.
- **Platform-agnostic** — the hardware layer supplies an immutable profile and
   a key/value storage adapter. The Zephyr platforms use the settings
   subsystem on ZMS in the `storage_partition` (`hardware/zephyr_common/
   zephyr_settings_storage.cpp`); on `native_sim` that is the flash simulator.
- **Read-only to application code** — control code receives a
   `SettingsSnapshot`; only an explicitly passed `SettingsOperations` capability
   can commit changes.
- **Explicitly serialized** — unit wrappers are encoded through their numeric
   values as little-endian floats, one value per register
   (`lib/settings/settings_registers.hpp`). The C++ object layout is not persisted.

```cpp
#include "hardware_interface.hpp"
#include "settings_store.hpp"

unimoc::settings::SettingsStore settings_store{
      unimoc::hardware::kSettings.GetSettingsProfile(),
      unimoc::hardware::kSettings.GetSettingsStorage()};

const auto load_status = settings_store.Load();
const auto settings = settings_store.GetSnapshot();

// Application consumers receive only a const settings record.
current_control.Init(settings.Get(), timer_clock_hz);

// Operation endpoints such as Cyphal receive the mutation capability.
const auto operations = settings_store.GetOperations();
operations.SetMotorCurrentLimit(unimoc::unit::Current{20.0F});
if (load_status == unimoc::system::SettingsStatus::kStorageError) {
      // Handle a storage failure before enabling the drive.
}
```

The target profile is compiled into the hardware adapter as a `const` object,
so stock parameters and immutable safety capabilities are included in the
firmware image. The writable settings are separate and are never allowed
to raise those capabilities.

---

## Hardware Bring-Up Guide

> **Status:** runs on `native_sim` (ztests `unimoc.sim.cyphal_drive` and
> `unimoc.sim.overcurrent`, the pycyphal integration test); not yet run on the board.  The FSM is driven by
> the commissioning commands (`uavcan.node.ExecuteCommand`) and stepped in the Cyphal thread.  Log lines go to the
> Zephyr log on SEGGER RTT and as `uavcan.diagnostic.Record` on the bus.

`HwStartup` is a step-by-step bring-up wizard for commissioning a new UNIMOC
board.  It verifies the ADC signal chain, gate drivers, and DC-link voltage
measurement **before** any motor is connected, then performs current-sense and
ADC-trigger calibration with the motor attached.  All calibration results are
committed to NVM automatically when the sequence completes.

The FSM **never advances on its own**.  Each step completes its measurement,
logs the result through the Zephyr log, then pauses and waits for the command
CONTINUE before moving on.  This gives you time to review scope
traces and enter external meter readings.

The motor parameters (R, L, flux, Hall offset, inertia) are entered by hand
for now; their measurement is planned as a second assisted flow
(`docs/motor_identification_plan.md`).

**From cypilot** (the operator app): select the drive, open the Control view
and use the **Startup aid** card (take control first). It shows the step and
what to do, takes the meter readings, and asks for the confirmation that the
motor is connected before phase 2. STOP ALL, the E-stop and a lost control
lease abort the run. The commands below are the same over yakut.

While the aid runs it owns the outputs: it releases the power stage itself,
so readiness SLEEP does not stop it. Abort it with the command ABORT,
the emergency stop or a restart.

### Commands and Status

The aid is driven by `uavcan.node.ExecuteCommand` (service 435) with the
codes of `unimoc.drive.CommissioningStatus.0.1`. The answer says at once
whether the action was taken; a refusal carries the reason in the output text
(for example `the step is still measuring`, `engaged: disengage first`,
`outputs not released: external enable missing`).

| Command | Code | Parameter | Effect |
|---------|------|-----------|--------|
| START_STARTUP_AID | `0x0200` (512) | none | starts from IDLE, DONE or FAULT; refused while engaged, while it runs or after an emergency stop |
| CONTINUE | `0x0202` (514) | the confirmation the step needs, else none | next step; refused while the step measures or waits for its reading |
| SET_READING | `0x0203` (515) | `vdc=<V>` or `current=<A>` | an external meter reading in V or A |
| ABORT | `0x0204` (516) | none | FAULT, outputs off; no change when nothing runs |

With yakut (node 42 as an example):

```
y cmd 42 512                       # START_STARTUP_AID
y cmd 42 514                       # CONTINUE
y cmd 42 514 motor-connected       # CONTINUE at CONNECT_MOTOR
y cmd 42 515 vdc=48.2              # SET_READING
y cmd 42 516                       # ABORT
```

The progress is the status message `unimoc.drive.CommissioningStatus.0.1` on
subject 215 (`uavcan.pub.commissioning.id`), published on every change, at
2 Hz while the aid runs and at 0.5 Hz otherwise: the step, its state
(MEASURING, WAITING, NEEDS_READING, DONE, FAILED, ABORTED), what it needs
(a reading, `motor-connected`), the progress of the measurement in %, one bit
per passed and per failed step, and the reason of a fault. Follow it with
`y sub 215:unimoc.drive.CommissioningStatus.0.1`.

### Equipment Required

| Item | Used in step(s) |
|------|-----------------|
| Cyphal/UAVCAN v1 bus + tool (e.g. **Yakut**, **Cymon**) | All steps |
| Oscilloscope (optional but recommended) | PWM_DISABLE, DUTY_FORCE_\* |
| Digital multimeter — DC voltage | DC_LINK_VOLTAGE_CHECK |
| AC/DC clamp-meter (or shunt + oscilloscope) | CURRENT_SENSE_CALIBRATION |
| Motor with UVW + PE leads, motor shaft free to rotate | Phase 2 steps |

---

### Phase 1 — No Motor Connected

> **Caution:** Do **not** connect a motor until the `CONNECT_MOTOR` step
> instructs you to do so.

#### Step 0 — Start the FSM

Power on the drive without a motor connected.  Using your Cyphal tool, send
START_STARTUP_AID:

```
y cmd 42 512
```

The firmware logs:

```
[STARTUP] Hardware bring-up sequence started.
[STARTUP] Phase 1 — NO MOTOR CONNECTED.
[STARTUP] Continue each step with the commissioning
[STARTUP]   command CONTINUE (0x0202)
```

The FSM immediately enters **PWM_DISABLE**.

---

#### Step 1 — PWM_DISABLE

All gate-driver outputs are held at 50 % neutral duty.  The log confirms
the duty is applied and invites you to verify on a scope.

**Action:** Confirm PWM signals are toggling correctly on the scope, then
advance:

```
y cmd 42 514
```

---

#### Step 2 — ADC_OFFSET_CAL

The firmware samples 1024 ADC readings on both current-sense channels
(phases A and B) with zero current flowing and computes the mean offset.

The log reports:

```
[STARTUP] ADC_OFFSET_CAL: offset_a=<value> A, offset_b=<value> A
```

A warning is printed if either offset exceeds `offset_threshold_A` (default
0.5 A).  The results are stored in `StartupResults::adc_offset_a/b` and
written to `NvmSettings::adc_offset_a/b` at the end of the sequence.

**Action:** Review the offsets.  Values > ±0.5 A suggest a hardware problem.
Advance:

```
y cmd 42 514
```

---

#### Step 3 — ADC_NOISE_FLOOR

1024 samples are collected again.  The standard deviation (RMS noise) is
computed for each channel and compared against `noise_threshold_A` (default
0.1 A).

The log reports:

```
[STARTUP] ADC_NOISE_FLOOR: rms_a=<value> A, rms_b=<value> A
```

A warning is printed if noise exceeds the threshold.

**Action:** Review the noise figures.  High noise may indicate a power-supply
or layout problem.  Advance:

```
y cmd 42 514
```

---

#### Steps 4–6 — DUTY_FORCE_LOW / MID / HIGH

Three forced-duty steps (5 %, 50 %, 95 %) check that the gate driver and
bootstrap circuit produce valid PWM across the full duty range.  Each step
holds the duty for 1000 samples, then waits.

The log reports the applied duty:

```
[STARTUP] DUTY_FORCE_LOW: duty=0.05 — verify current/scope. Press NEXT to continue.
[STARTUP] DUTY_FORCE_MID: duty=0.50 ...
[STARTUP] DUTY_FORCE_HIGH: duty=0.95 ...
```

**Action:** Observe gate-drive waveforms or check that the DC-link current
response looks correct for each duty level.  Advance after each step:

```
y cmd 42 514
```

---

#### Step 7 — DC_LINK_VOLTAGE_CHECK

The firmware reads the DC-link voltage ADC channel while the PWM is running.
Before continuing you must enter a reference measurement from a voltmeter;
the status shows NEEDS_READING (DC_LINK_VOLTAGE) until then.

**Action:**

1. Measure the DC-link voltage with a calibrated multimeter.
2. Write the reading:
   ```
   y cmd 42 515 vdc=48.2            # the measured voltage in V
   ```
3. Advance:
   ```
   y cmd 42 514
   ```

The firmware computes `gain_vdc = adc_vdc / vdc` and reports it.  A
warning is printed if the error exceeds `vdc_gain_tolerance` (default 2 %).
The correction factor is stored in `NvmSettings::adc_gain_vdc` at DONE.

---

#### Step 8 — GATE_DRIVER_ENABLE_CHECK

The firmware briefly disables then re-enables the gate-driver enable signal
while measuring the ADC response on the current-sense inputs.  This confirms
that the enable pin is wired correctly and that the gate driver is live.

The log reports:

```
[STARTUP] GATE_DRIVER_ENABLE_CHECK: delta_ia=<value> A — gate driver responding.
```

A warning is printed if the response is smaller than twice the measured noise
floor (gate driver may not be wired or may be faulty).

**Action:** Advance:

```
y cmd 42 514
```

---

#### CONNECT_MOTOR barrier

The aid switches the outputs **off** at this barrier and keeps the logic
running: the run survives connecting the motor, a power cycle would end it
(the drive restarts in IDLE).

> ⚠️ **Before advancing past this step:**
>
> 1. **Connect** the motor (UVW + protective earth) with the outputs off.
>    Keep the logic powered; switch the DC link off only if the logic has a
>    supply of its own. Pulling the external enable while wiring is fine.
> 2. Ensure the motor shaft is **free to rotate**.

The log prints a prominent warning:

```
[STARTUP] !!  PHASE 1 COMPLETE - PREPARE FOR PHASE 2       !!
[STARTUP] !!  The outputs are OFF now.                      !!
[STARTUP] !!    1. CONNECT the motor (UVW + PE). Keep the   !!
[STARTUP] !!       logic powered: a power cycle ends the    !!
[STARTUP] !!       run. ...                                 !!
[STARTUP] !!    2. CONTINUE with motor-connected            !!
```

Then confirm with CONTINUE and the parameter `motor-connected` (a CONTINUE
without it is refused); the aid releases the outputs again at neutral duty
(acknowledging a break of the external enable during the barrier) and starts
phase 2, or goes to FAULT when the external enable is still missing:

```
y cmd 42 514 motor-connected
```

---

### Phase 2 — Motor Connected

> ⚠️ **Caution:** Phase 2 injects live voltages into the motor windings.  An
> over-current (OC) trip limit equal to `oc_fraction × max_phase_current_A`
> (default 10 %) is active throughout.  The FSM transitions immediately to
> FAULT if this limit is exceeded.  Independently, the fast ISR trips the power
> stage at the hard limit `max_phase_current` (60 A on the BCC) and latches a
> fault until it is cleared.

---

#### Step 10 — PHASE_ADC_ALIGNMENT

The firmware sweeps the ADC trigger offset by ±5 µs in 0.5 µs steps
(21 positions) while injecting a small voltage, measuring the current-sense
noise at each position.  The injection is that of step 11 (`v_cal_fraction`,
0.2 % duty on the alpha axis, about 1 A in a 0.1 Ohm motor at 48 V).

In every step of a run the phase currents are checked against 10 % of the
hardware current limit (`oc_fraction`): a current in phase 1 means a motor is
still connected, and the run goes to FAULT.  The position with the lowest noise is selected as the
optimal ADC sampling instant.

The log reports:

```
[STARTUP] PHASE_ADC_ALIGNMENT: best_pos=<n>, optimal_ticks=<value>
[STARTUP] Suggest writing adc_trigger_offset=<value> to hardware HAL.
```

The optimal offset is stored in `StartupResults::adc_trigger_offset_optimal`
for the application to commit to the hardware timer configuration.

**Action:** Note the suggested `adc_trigger_offset` value.  Advance:

```
y cmd 42 514
```

---

#### Step 11 — CURRENT_SENSE_CALIBRATION

A small voltage (fraction `v_cal_fraction` of V_dc, default 0.2 %) is forced
on the α-axis to produce a small but measurable current.  The firmware reads
the phase-A ADC while a reference clamp-meter provides the true current.

**Action:**

1. Attach a clamp-meter (or calibrated shunt + scope) to phase A.
2. Write the clamp-meter reading:
   ```
   y cmd 42 515 current=0.85        # the clamp-meter reading in A
   ```
   The reading may come before or after the measurement; the status shows
   NEEDS_READING (PHASE_CURRENT) once the step has measured without it.
3. The firmware computes `gain_a = (adc_mean_ia - adc_offset_a) / current`
   (phase B against `-current / 2`) and reports it; the stored correction
   is `adc_gain_a = 1 / gain_a`.
   A warning is printed if the error exceeds `gain_tolerance` (default 5 %).
4. Advance:
   ```
   y cmd 42 514
   ```

The gain correction factors are stored in `NvmSettings::adc_gain_a` and
`adc_gain_b` at DONE.

> **Note:** CONTINUE is refused until the reading is there (`the step needs
> its meter reading`); ABORT ends the run without a gain correction.

---

### Calibration Results and NVM Commit

When the sequence reaches **DONE** the firmware automatically commits the
calibration through its authorized `SettingsOperations` capability:

1. Builds one candidate settings record containing:

   | NvmSettings field | Source |
   |---|---|
   | `adc_offset_a` | Mean phase-A ADC reading at zero current |
   | `adc_offset_b` | Mean phase-B ADC reading at zero current |
   | `adc_offset_c` | Mean phase-C ADC reading at zero current |
   | `adc_gain_vdc` | `1 / gain_vdc` computed in DC_LINK_VOLTAGE_CHECK |
   | `adc_gain_a` | `1 / gain_a` computed in CURRENT_SENSE_CALIBRATION |
   | `adc_gain_b` | `1 / gain_b` computed in CURRENT_SENSE_CALIBRATION |
   | `adc_gain_c` | `1 / gain_c` computed in CURRENT_SENSE_CALIBRATION |

   The control cycle applies them to every sample from the next settings
   restart (the next engage): `i = (i_raw - adc_offset) * adc_gain` per phase,
   `v = v_raw * adc_gain_vdc`. The aid itself measures the raw samples, so a
   second run measures from scratch instead of correcting the first.

2. Validates the candidate and applies it to the live settings as one
   transaction. A rejected candidate leaves the active settings unchanged and puts
   the startup FSM into `FAULT`. The calibration is not written to flash: save it
   with `SettingsOperations::SaveToFlash()` once the power stage is disabled.

3. Prints a full summary to the Zephyr log:
   ```
   [STARTUP] HARDWARE STARTUP AID — SUMMARY
   [STARTUP]   PWM_DISABLE:               PASS
   [STARTUP]   ADC_OFFSET_CAL:            PASS
   ...
   [STARTUP] adc_offset_a  = 0.012 A
   [STARTUP] adc_offset_b  = -0.007 A
   [STARTUP] gain_vdc      = 1.003
   [STARTUP] gain_a        = 0.997
   [STARTUP] adc_trig_opt  = 176 ticks
   [STARTUP] NVM calibration fields updated.
   ```

Individual step results can also be read back at any time via:

```
unimoc.startup.results      # uint8 array; 1=pass, 0=fail/pending per step index
unimoc.startup.gain_a
unimoc.startup.gain_b
unimoc.startup.gain_vdc
unimoc.startup.adc_offset_a
unimoc.startup.adc_offset_b
```

---

### Aborting or Recovering from FAULT

To **abort** the sequence at any time, send ABORT:

```
y cmd 42 516
```

The FSM enters FAULT and disables the power stage; the drive releases the
forced duties and returns to its control loop. ABORT when no run is active
changes nothing, so a tool may abort every drive on a stop.

To **restart** from scratch after a fault or abort, send START_STARTUP_AID:

```
y cmd 42 512
```

This resets all accumulators and results, then re-enters PWM_DISABLE from the
beginning.

---

## Getting Started

### Building Firmware (Zephyr)

Requirements: Zephyr 4.5.0 with the Zephyr SDK 1.0.1 (`arm-zephyr-eabi`),
CMake ≥ 3.28 (Zephyr's minimum since 4.5), Ninja, Python 3 with `west`.  On Windows, run the build in WSL2.
The setup steps are in [`docs/zephyr_workflow.md`](docs/zephyr_workflow.md).

Run these from the west workspace root, which contains `unimoc/` and `zephyr/`:

```bash
# Linux host simulation (native_sim)
west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim

# Battery Case Controller (STM32G473)
west build -b bcc unimoc/app -d unimoc/app/build/bcc

```

The BCC has not run on hardware yet.

### Running the Tests

All tests run under twister on `native_sim`; there is no hosted test build.
`tests/lib/<library>/` (ztest) covers the libraries, `tests/sim/` (ztest) runs
the application core against the simulated drive, and
`tests/integration/cyphal_sim/` (pytest with pycyphal) drives the running
simulation over Cyphal. From the west workspace root:

```bash
# All scenarios on native_sim and bcc, with clang-tidy as static analysis (as in CI)
unimoc/scripts/twister_sca.sh

# clang-format check of all C/C++ sources (--fix to apply)
unimoc/scripts/check_format.sh

# pycyphal integration tests of the simulation (needs scripts/requirements-cyphal-tools.txt)
west twister -T unimoc/app -p native_sim/native/64 --scenario unimoc.app.cyphal_integration

# One suite without static analysis, while iterating
west twister -T unimoc/tests/lib/motor_control -p native_sim/native/64
```

Prebuilt firmware: every CI run uploads the BCC image (`unimoc-bcc`) and the
`native_sim` executable (`unimoc-native_sim`) as artifacts of the
[Zephyr workflow](https://github.com/Tecnologic/unimoc/actions/workflows/ci-zephyr.yml).
Releases (below) attach the same files with checksums, the twister report and
the API documentation.

### Releases

A release is made by merging a pull request from `develop` into `main` (merge
commit). Nothing is tagged by hand ([docs/release_plan.md](docs/release_plan.md)):

- The version is in the Zephyr version file `app/VERSION`. While the
  `develop` to `main` pull request is open, the *Release prepare* workflow
  commits the next version and `CHANGELOG.md` to `develop` and comments the
  release on the pull request. The label `release-as:X.Y.Z` forces a version.
- Versions follow [semantic versioning](https://semver.org) and come from the
  [conventional commits](https://www.conventionalcommits.org) of the squash
  merges into `develop` (the pull request title is checked):

  | Title | Version (below 1.0) |
  |---|---|
  | `fix: ...`, `perf: ...`, `refactor: ...` | patch |
  | `feat: ...`, `feat!: ...` | minor |
  | `docs:`, `test:`, `ci:`, `build:`, `chore:` | none |

  A change of the DSDL types or of register and subject IDs is always `feat!`
  or `fix!`, because it changes the bus protocol.
- The merge runs the Zephyr workflow on `main`. After the tests and all
  firmware builds passed, its `release` job creates the tag `vX.Y.Z` and the
  GitHub release with the change log and every build (BCC, `native_sim`,
  later ones), `SHA256SUMS`, the twister report and the Doxygen HTML. The
  release is a step of that CI run, so the badge of `main` shows it.

`twister_sca.sh` needs `clang-tidy` (LLVM 18) and CodeChecker
(`pip install codechecker`) in addition to the Zephyr setup;
`check_format.sh` needs exactly clang-format 22.1.1
(`pip install clang-format==22.1.1`).

### Generating the API Documentation

Requirements: Doxygen and Graphviz (`dot`).

```bash
mkdir -p build && doxygen docs/Doxyfile
```

The generated HTML documentation is placed in `build/doxygen/html/` and
contains only UNIMOC-owned C++ code.

---

## Contributing

Contributions are welcome!  Please:

1. Fork the repository and create a feature branch.
2. Follow the existing code style (`.clang-format` is provided).
3. Add ztest coverage for new algorithm headers in `tests/lib/`.
4. Run `scripts/check_format.sh` and `scripts/twister_sca.sh` before opening a
   pull request — CI runs the same two steps.
5. Open a pull request against `main`.

---

## License

UNIMOC is free software released under the
**GNU General Public License v3.0**.  See the [`LICENSE`](LICENSE) file for
the full text.
