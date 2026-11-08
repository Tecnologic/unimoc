# System Plan: Manned Cart with Four Wheel Drives

> **Status (2026-10-21):** draft to sort the overall direction. Nothing in
> this file is built or verified on hardware. Decided so far:
>
> - Cyphal/CAN-FD for the cart; Single Pair Ethernet is a later option.
> - (2026-11-01, replaces the STM32G4 choice) The new 4850 logic board uses an
>   STM32H743VIT6 (LQFP100, 2 MB dual-bank flash): the 512 KB G4 parts are not
>   available, and the H7 leaves headroom for a faster current loop. Pin
>   proposal and port in `docs/drive4850_h7_plan.md`.
> - The VDC firmware lives in this repository.
> - A mechanical emergency brake, otherwise pure regenerative braking.
> - A BMS controller with brake chopper is another bus node and another
>   application in this repository.
> - The steering servo is out of scope for now; the first rides use a
>   steering angle sensor only.
> - (2026-10-23, changed 2026-11-03) ~~The new 4850 hardware gets a fourth
>   half bridge~~: the drive has three phases only. The EESM excitation
>   becomes a second module on the bus, the brake chopper stays in the BMS
>   controller (section 5.2.1).
> - (2026-11-03) Hardware is named after pioneers of electric drives and
>   computing (section 5.0). The H7 redesign of the 4850 logic board is
>   **Jacobi** (`boards/unimoc/jacobi/`, board `unimoc_jacobi`). All boards
>   are STM32H7 based (one platform, `hardware/zephyr_stm32h7`); the
>   NUCLEO-H743ZI is the bring-up board.
> - (2026-10-23) Current focus: Cyphal/CAN-FD (cart messages, bus error
>   handling, multi-node simulation; the order is in `TODO.md`, "Next").

## 1. Goal

A manned cart with up to four direct wheel drives, torque vectoring and
drift assist, and later an electric steering assist. UNIMOC supplies the
wheel drives (and later the steering servo); a central vehicle dynamics
controller (VDC) coordinates them. Standard modules are reused where they
connect to the VDC directly (IMU, GNSS, steering angle sensor, display).

## 2. Repositories

| Repository | Role in this plan |
|---|---|
| [Tecnologic/unimoc](https://github.com/Tecnologic/unimoc) (this repository, branch `develop`) | Firmware on Zephyr for all bus nodes: wheel drives, VDC, pedal unit and BMS controller. FOC hot path, observers, settings, vehicle dynamics, `native_sim` simulation, tests. |
| [Tecnologic/hardware_4850](https://github.com/Tecnologic/hardware_4850) | 48 V / 50 A drive in an RND 455-00370 enclosure. [`power/`](https://github.com/Tecnologic/hardware_4850/tree/master/power): power stage, kept unchanged. [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic): logic board, redesigned with an STM32H743VIT6 (section 5.2). |
| [Tecnologic/unimoc_old_software, branch `electric_quad`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad) | Legacy firmware (ChibiOS, STM32F730) of the 1.6 kW children's quad on the 4850 hardware. Source for proven settings and features to port (section 9); not built any more. |
| `lib/cymon` (submodule) | Tracing and monitoring over Cyphal (`docs/cymon_integration_plan.md`); host tool: cymon. |

### 2.1 What the 4850 hardware is (from the KiCad schematics)

- **Power stage (`power/phase.sch`, three sheets):** per phase MOSFETs
  (the ordered boards: NCEP6080AG, 60 V, per the BOM in
  `unimoc_jacobi/production/legacy_kicad5/power`; one prototype carries the
  BSC040N08NS5, 80 V), an LM5109B half-bridge driver, three 2 mΩ shunts in
  parallel with an INA240A1 inline current amplifier, NTC on the heat sink. Proven over years
  in the quad (1.6 kW) and kept as it is.
- **Logic (`logic/`):** STM32F730R8 (LQFP64), TCAN334 CAN transceiver
  (CAN FD capable), USB mini-B, 24LC04 EEPROM, Hall and motor temperature
  inputs, GS8632 op-amp filters for the current and voltage channels, LM5008
  and LMR12010 supplies from 48 V.

The redesign replaces only the logic board. Interfaces to the power
board (PWM, ADC, supply) stay the same, so the power stage keeps its track
record.

## 3. System Architecture

```text
   IMU (SPI)   GNSS (UART)   Steering angle sensor   Display
        \          |                 |                 /
         +---------+-----------------+----------------+
         |        Vehicle dynamics controller (VDC)    |   STM32H7, Zephyr
         +----------------------+----------------------+
                                |
     ======================= CAN FD (Cyphal/CAN) ========================
        |            |            |            |             |              |
   Drive FL      Drive FR     Drive RL     Drive RR      Pedal unit     BMS controller
   UNIMOC        UNIMOC       UNIMOC       UNIMOC        (drive + brake (cells, contactor,
   Jacobi        Jacobi       Jacobi       Jacobi        pedal, H7)     brake chopper, H7)

   (later: steering assist as another UNIMOC node)

   USB CAN FD adapter ──► PC tools (pycyphal) on the same bus
   Hard-wired E-stop loop ──► external enable / break input of every power stage
   Mechanical emergency brake ──► independent of bus and electronics
```

| Node | Hardware | Function |
|---|---|---|
| VDC | STM32H7, Zephyr | Driver inputs, vehicle state estimation (IMU, GNSS, wheel speeds, steering angle), torque distribution, torque vectoring, drift assist, display, logging |
| Wheel drives (2 or 4) | Jacobi: 4850 power stage + new H743 logic | Torque control per wheel (drive and regenerative braking), wheel speed, limp-home mode (section 7) |
| Pedal unit | Small STM32H7 board | Publishes drive and brake pedal positions (two sensor channels per pedal); sends no commands |
| BMS controller | STM32H7 board in the battery (section 5.5) | Cell monitoring, contactor and precharge, state of charge, charge and discharge current limits, brake chopper on the DC link |
| Steering angle sensor | Standard sensor on the VDC (for example the AS5048 of the legacy firmware) | Steering angle for the vehicle dynamics; no bus node |
| IMU, GNSS, display | Standard modules on the VDC | No bus node |
| Steering assist | Later: UNIMOC | Out of scope until a steering concept exists |
| EESM excitation | Later: own module (section 5.2.1) | Field current of an EESM drive, commanded by its drive over the bus |
| NUCLEO-H743ZI (`app/boards/nucleo_h743zi.*`) | ST evaluation board | Bring-up of the STM32H7 platform before Jacobi exists; not part of the cart |

## 4. Decisions

| Topic | Decision | Reason |
|---|---|---|
| Control structure | One central VDC. If it fails, the drives fall back to the pedal unit (section 7). | The vehicle dynamics need all sensors in one place; a VDC failure must still let the driver brake. |
| Brakes | Mechanical emergency brake; the service brake is regenerative only, through the drives. | Consequences in section 7.1. |
| BMS and brake chopper | **One node, one more application in this repository** (`apps/bms/`): the BMS controller also drives the brake chopper. The chopper reacts to the DC-link voltage locally, without the bus. | Braking energy the battery cannot take (full battery, cold battery, open contactor) goes into the resistor; the BMS knows when that is the case. |
| Steering | Mechanical steering. First rides: steering angle sensor on the VDC only. A steering assist (UNIMOC) follows later with its own plan. | No complete steering concept yet. |
| Protocol | **Cyphal v1.0**, libraries pinned in `west.yml`: libcanard 4.x, Nunavut for DSDL, pycyphal for tools and tests. | All bus nodes are own nodes; publish/subscribe like DDS without a broker; settings already use register names (`unimoc.motor.pole_pairs`). Cyphal v1.1 (cy, libcanard 5) is alpha and wire-compatible with Cyphal/CAN v1.0; revisit after its release. |
| Cart transport | **Cyphal/CAN-FD**, one bus for all nodes, bit rates in section 6.3. | Mature in Zephyr (STM32 FDCAN driver) and in Cyphal (libcanard), robust next to the drives, priority arbitration, cheap transceivers, enough bandwidth (section 6.3). Section 6.5 compares it with SPE. |
| Message size | Every message fits in **one CAN FD frame** (at most 63 bytes payload). | No multi-frame transfers in the control loop; a later move to UDP or SPE only swaps the transport. |
| Single Pair Ethernet | Later option, not in the first cart (section 6.5). | The Zephyr drivers and the Cyphal/UDP library are young. |
| Simulation and PC transport | Cyphal/CAN-FD as well: `native_sim` on SocketCAN `vcan`, PC tools on a USB CAN FD adapter. Fallback if WSL2 has no `vcan`: Cyphal/UDP over the `native_sim` TAP driver (section 6.4). | One transport everywhere; the simulation tests the transport of the cart. |
| Logic board MCU | **STM32H743VIT6** for the 4850 logic board (2026-11-01, section 5.2, `docs/drive4850_h7_plan.md`); | The 512 KB G4 parts are not available or far too expensive, and 128 KB does not hold the firmware (194 KB on the former BCC). Three ADCs for simultaneous phase currents, FDCAN, TIM1/TIM8 and DMAMUX as on the G4, 2 MB dual-bank flash, headroom for the control cycle on every sample. Costs a second hot-path platform (`hardware/zephyr_stm32h7`). |
| VDC firmware | **In this repository**, as a second Zephyr application (`apps/vdc/`) with a portable `lib/vehicle/`. The MCU is an STM32H7 (section 5.3); a Raspberry Pi or an ESP32-P4 are out, they do not run this repository's Zephyr build. | One toolchain, one CI, shared libraries and DSDL, the whole cart in the simulation. |
| Cyphal/serial | Not used. | No maintained C implementation (libserard). |
| USB | Firmware update (DFU) only, later. The CDC-ACM console and the shell were dropped (2026-10-21): the bench log is on RTT, the log also goes out as `uavcan.diagnostic.Record`. | One operating interface; USB stays the recovery path. |
| Settings | Cyphal registers on the existing `SettingsStore` (`docs/settings_plan.md`). | Already designed that way. |
| Fourth half bridge on the 4850 | ~~One auxiliary half bridge per drive~~ (2026-10-23). **Changed 2026-11-03: Jacobi has three phases only**; the EESM excitation is a second module on the bus, the brake chopper stays in the BMS (section 5.2.1). | Keeps the logic board and the power board connector simple. Costs the local chopper: above the back-EMF peak a PMSM keeps charging the DC link through the body diodes after a trip (section 7.1), so the BMS chopper must hold the DC link. |
| Firmware update | Cyphal file service into flash bank 2 (dual-bank flash: 2 MB H743, `docs/settings_plan.md`); USB DFU as recovery. An image carries a UNIMOC header (magic, version, size, board name, CRC32 of the payload), checked after the download, before the bank swap (decided 2026-11-08; signatures come with the bootloader). Status 2026-11-08: the download and the check run on `native_sim` into a RAM bank (`lib/cyphal/file_service.*`, `unimoc.update.*`); the flash writer and the bootloader are open. | Update all nodes over the bus without opening the cart. |

## 5. Hardware Plan

### 5.0 Hardware names

Every hardware gets the name of a pioneer of electric drives or computing
(2026-11-03). The Zephyr board is `unimoc_<name>` in `boards/unimoc/<name>/`,
the Cyphal node name `org.unimoc.<name>`; later board revisions use Zephyr
board revisions (`unimoc_jacobi@B`). The ratings stay in the board's
documentation, not in the name. Only Jacobi exists; the others are proposals.

| Name | Hardware | After |
|---|---|---|
| **Jacobi** | 36 V / 45 A drive (100 A peak phase current, 60 V FETs; Tecnologic/unimoc_jacobi), the H7 redesign of the 4850 logic board (section 5.2) | Moritz Hermann Jacobi, first practical electric motor (1834), electric boat on the Neva (1838) |
| Barlow | small drive (steering, pedal generator) | Peter Barlow, Barlow's wheel (1822) |
| Dolivo | high-power drive | Michail Dolivo-Dobrowolsky, three-phase motor and system (1889) |
| Zuse | VDC | Konrad Zuse, first working programmable computer |
| Drais | pedal unit or pedal generator | Karl Drais, Laufmaschine (1817) |
| Mercator | steering | Gerardus Mercator, the map everyone navigates by |
| Gutenberg | display | Johannes Gutenberg |
| Volta | BMS controller | Alessandro Volta, the voltaic pile (1800) |
| Otto | throttle | Nikolaus Otto |

### 5.1 One MCU family

Decision 2026-11-03: every board of the system is STM32H7 based (one
platform, `hardware/zephyr_stm32h7`, one timing baseline). The earlier G4
target is removed; its code is in the git history before 2026-11-03.

### 5.2 Jacobi: new 4850 logic board (STM32H743VIT6)

Requirements for the redesign of [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic):

- **MCU:** STM32H743VIT6 (LQFP100, decided 2026-11-01).
  Pin proposal, flash layout and the firmware port in
  `docs/drive4850_h7_plan.md`: TIM1 for the phases with the break input,
  one phase current per ADC (ADC1 to ADC3) for simultaneous injected
  sampling, phase voltage taps, FDCAN1, USB, SWD, Hall inputs, temperatures.
- **Current sensing:** the INA240A1 inline phase amplifiers stay
  (foc0 `current-sense = "inline"`, the default); they measure the phase current
  independent of the switching state; check the gain, the offset and the
  filter of the GS8632 stages against the H743 ADC range.
- **CAN FD:** keep the TCAN334 (CAN FD capable), with a split termination
  that can be populated at the bus ends.
- **Keep:** USB for DFU, SWD, Hall and temperature inputs, the power board
  connector and the supply topology.
- **Replace:** the 24LC04 EEPROM is no longer needed (settings on ZMS in the
  MCU flash, `docs/settings_plan.md`).
- **Prepare for the later steering variant:** an SPI or I2C connector for an
  absolute angle sensor.
- **Phase voltages:** three divider taps on the power board connector
  (2026-11-03), sampled with the phase currents (`va`, `vb`, `vc` of the
  foc0 node).
- **No fourth half bridge** (2026-11-03, section 5.2.1).
- **Platform layer:** `hardware/zephyr_stm32h7`, with the board differences
  (pins, ADC channels, timer, dead time, sample times) in the devicetree
  (`foc0` node). Board definition `boards/unimoc/jacobi/`
  (`unimoc_jacobi`); bring-up first on a NUCLEO-H743ZI.

#### 5.2.1 Fourth half bridge: excitation or brake chopper

Changed 2026-11-03: Jacobi has no fourth half bridge. The EESM excitation
becomes a second module on the bus that the drive commands over Cyphal; the
brake chopper of the PMSM case stays in the BMS controller (section 5.5). The
requirements below (decided 2026-10-23 for a half bridge on the drive) now
apply to that module.

Decided 2026-11-08 for the drive-to-module link:

- **The field current loop runs in the excitation module**
  (`ExcitationController`, `ExcitationObserver`). The drive publishes a
  field current command (`unimoc.excitation.Command.0.1`: field current,
  enable) and subscribes the module's status (`unimoc.excitation.Status.0.1`:
  measured field current, field voltage, fault). The `native_sim` platform
  runs a simulated module that feeds the field voltage of the EESM plant.
- **The drive decides the field current** from its operating point (the
  rated field current, reduced in field weakening, within
  `unimoc.excitation.i_f_min`/`i_f_max`); an excitation setpoint on the bus
  overrides it while fresh, like the other setpoints (tests, the VDC).
- **ASM sensorless first:** the induction motor runs without a rotor sensor
  (an adaptive full-order flux observer, implemented 2026-11-08); a sensored
  path (Hall or encoder) is a later option.

The role of the extra half bridge follows the motor type (`unimoc.motor.type`):

| Motor | Load on the half bridge | Function |
|---|---|---|
| EESM | Excitation (field) winding | `ExcitationController` (current or flux mode) sets the duty; `ExcitationObserver` filters the measured field current. |
| PMSM | Brake resistor on the DC link | Brake chopper: switches on above a DC-link threshold (hysteresis or PWM) and while the overvoltage trip holds the phases off, so the body-diode charging of section 7.1 goes into the resistor. |
| ASM | none (or brake resistor) | The ASM has no excitation winding; the output can serve as a chopper as with the PMSM. |

Requirements to settle in the schematic (nothing is designed yet):

- **Placement:** on the new logic board or as an addition to the power board,
  which otherwise stays unchanged (section 2.1). Same parts as the phases
  (LM5109B) where the rating fits.
- **PWM:** a fourth channel synchronized with the phase timer (for example a
  spare channel of TIM1/TIM8; on the H743 TIM8 CH2/CH2N slaved to TIM1), with the same break
  input, so the E-stop and the overcurrent break also stop it.
- **Measurement:** the current of the half bridge (field current for the
  excitation loop, resistor current for the chopper energy), the DC-link
  voltage at the bridge.
- **Chopper without the CPU:** like the BMS chopper (section 5.5), an H7
  comparator with a DAC threshold drives the half bridge directly; the
  firmware sets the thresholds and watches the resistor energy and
  temperature in a thread. If the routing does not allow it, the chopper runs
  in the zero-latency sample ISR (no kernel calls, no locks).
- **Ratings:** field current and field voltage of the intended EESM; resistor
  power and energy from the braking case (section 12, question 7), per drive.
- **Field decay (EESM):** with the field current at zero the EESM has no
  back-EMF, so a trip cannot charge the DC link once the field has decayed.
  A single half bridge can only freewheel the field current, which decays
  slowly; whether a faster de-excitation (asymmetric H-bridge, a second half
  bridge, a freewheeling resistor) is needed depends on the field time
  constant. Not checked against a real machine.
- **Firmware:** an auxiliary output next to `hardware/power_stage_interface.hpp`
  and the hot-path bindings; the simulation models it first (`lib/sim`, ASM and
  EESM plants and a DC-link model, `TODO.md`, "Motor types").

### 5.3 VDC

The VDC has no zero-latency hot path: a 250 to 500 Hz control loop, sensor
drivers, the display and logging. It runs Zephyr and its firmware lives in
this repository.

**Decision 2026-11-08:** the VDC is an STM32H7 board, like every board of
the system (section 5.1): a Jacobi logic board without power stage, a small
own board, or an evaluation board for a start. It shares the platform
`hardware/zephyr_stm32h7`; display and logging (LTDC on some parts, SDMMC)
pick the part.

**Impact on this repository:**

- `app/` becomes `apps/drive/` and `apps/vdc/` (two Zephyr applications,
  decided), later `apps/pedal/` and `apps/bms/`. The applications share `lib/units`,
  `lib/settings`, `lib/cyphal`, the DSDL namespace, `hardware/zephyr_common`
  and CI.
- New portable library `lib/vehicle/` (torque distribution, yaw-rate
  control, vehicle state estimation) with ztest suites in `tests/lib/vehicle/`.
- New board definition for the VDC (or a devicetree variant of the 4850
  logic board) and a VDC platform layer.
- `lib/sim` needs the vehicle model (four wheels, tyres, steering angle,
  shared battery); P4 item in `TODO.md`.
- `AGENTS.md`, `README.md` and `docs/zephyr_workflow.md` describe one
  application today; they need updating.
- More twister scenarios and CI time.

### 5.4 Pedal unit

- Small STM32H7 board with a CAN FD transceiver, two independent sensor
  channels per pedal (plausibility check as in automotive pedal modules).
- Firmware in this repository (`apps/pedal/`), or a role of the VDC board
  placed at the pedals.

### 5.5 BMS controller with brake chopper

- **Board:** STM32H7 with a CAN FD transceiver, a cell monitoring front end
  (part to choose), current measurement in the battery path, contactor and
  precharge drivers, and a brake chopper switch (MOSFET or half bridge) with
  a resistor on the DC link. The drives have no chopper of their own
  (section 5.2.1), so the BMS chopper is required; an EESM drive also relies
  on the de-excitation by its excitation module.
- **Firmware:** `apps/bms/` in this repository, with a portable
  `lib/battery/` (state of charge, current limits from cell voltage and
  temperature, chopper control) and ztest suites in `tests/lib/battery/`.
- **Brake chopper control:**
  - The chopper switches on the DC-link voltage measured on the BMS board,
    not on a bus message, so it also works without the VDC and the bus.
  - First stage in hardware: an H7 comparator (COMP) with a DAC threshold
    drives the chopper timer directly (hysteresis or PWM without the CPU).
    The firmware sets the thresholds and monitors the resistor temperature
    and energy. Whether this fits the H7 comparator and timer routing is to
    be checked in the schematic.
  - The chopper is the backstop: the drives limit the regenerative torque to
    the charge limit the BMS publishes, so the chopper only takes what the
    battery cannot (section 7.1).
  - Software in the chopper path follows the same rules as the hot path if
    it runs in an interrupt: no kernel calls, no logging, no locks.
- **Resistor:** sized from the cart mass, the top speed and the longest
  braking (open, section 12); an overtemperature limit protects it.
- **Contactor:** the BMS announces an opening in advance where it can
  (overcurrent and cell faults may need an immediate opening); the drives
  then ramp the regenerative torque down and the chopper holds the DC link.

## 6. Network

### 6.1 Bus

- One CAN FD bus through the cart: VDC, two or four drives, pedal unit, later
  the steering; a USB CAN FD adapter for the PC. Line topology, short stubs,
  120 Ω termination at both ends (split termination).
- Node IDs are static (settings register per node), no plug-and-play
  allocation at runtime, so the cart starts without an allocator node.

### 6.2 Messages

The DSDL namespace is `dsdl/unimoc/` (2026-10-21: `unimoc.drive.CurrentSetpoint`,
`unimoc.drive.Telemetry`, `unimoc.sim.Plant`, port-IDs as registers with
defaults in `lib/cyphal/cyphal_interface.hpp`). The cart messages are
defined in `dsdl/unimoc/cart/` (2026-10-23, version 0.1). The drive
subscribes the wheel command, the pedal state and the battery status and
publishes the drive status (README, "Cart wheel drive"); the VDC, the pedal
unit and the BMS do not exist yet. Every message fits in one CAN FD frame; a
ztest in `unimoc.lib.cyphal` checks it for every type of the namespace.

| Message | Type | From → to | Rate | Payload | Content |
|---|---|---|---|---|---|
| Wheel command | `unimoc.cart.WheelCommand.0.1` | VDC → drives | 250 to 500 Hz | 8 bytes | Torque per wheel (front left, front right, rear left, rear right) in the forward direction of the cart, negative brakes; one message for all wheels, applied on reception |
| Drive status | `unimoc.cart.DriveStatus.0.1` | each drive → VDC | same | 18 bytes | Wheel index, wheel speed, torque estimate, drive and brake torque limits, DC-link voltage and current, temperatures, power stage state, engaged, limp-home, regen derated, trips |
| Pedal state | `unimoc.cart.PedalState.0.1` | pedal unit → VDC and drives | same | 9 bytes | Drive and brake pedal travel 0 to 1, both channels, plausibility flags |
| Battery status | `unimoc.cart.BatteryStatus.0.1` | BMS → VDC and drives | 50 to 100 Hz | 15 bytes | Pack voltage and current, state of charge, charge and discharge current limits, cell temperature, contactor state (with the announced opening), chopper state and temperature |
| Heartbeat | all | 1 Hz (standard) | Health, mode |
| Registers, GetInfo, file update | VDC/PC ↔ all | on demand, low priority | Settings, version, firmware update |
| Diagnostics | all | event, low priority | `uavcan.diagnostic.Record` instead of the USB log |

The wheel command gets the highest Cyphal priority, then the pedal state and
the drive status; services, file transfers and diagnostics run at low
priority and cannot delay the control loop.

Design of the cart messages:

- **`float16` values** in SI base units (N m, rad/s, V, A, K; pedal travel
  and state of charge as ratios), NaN for a value a node does not measure.
  `float16` keeps the frames short; its 11 significant bits (0.05 % of the
  value) are finer than the measurements. The drive-internal
  `unimoc.drive.Telemetry` stays `float32`.
- **Directions in the cart frame:** positive torque and speed drive the cart
  forward. Each drive maps them to its own rotor direction (a settings
  register of the drive, with the wheel index; defined with the drive side).
- **Arming stays with `reg.udral.service.common.Readiness`**; the wheel
  command carries no mode. Each drive has its own readiness subject, 1024 +
  node-ID by default (2026-10-24), so the VDC arms the drives one by one.
  Limp-home is decided by each drive (section 7) and reported in the drive
  status.
- **`@extent 63 * 8`** instead of `@sealed`: a later minor version can add
  fields within the one frame and older receivers still accept it.
- **Own types instead of the UDRAL battery service** (`reg.udral.service.battery`):
  it spreads the state over several subjects and carries more than the
  drives need; one battery status frame is enough for the cart.
- Port-IDs are registers (`uavcan.sub.*.id`, `uavcan.pub.*.id`) as for the
  existing types; they are defined with the node that uses a message.
- Field names do not start with `is` or `to` followed by a lowercase letter:
  C reserves them and Nunavut escapes them with a leading `_` (hence
  `wheel_torque` and `estimated_torque`).

### 6.3 Bit rates and bus load (estimate, to verify by measurement)

One extended CAN FD frame with a 16-byte payload takes about 100 µs at
1 Mbit/s arbitration and 4 Mbit/s data (about 150 µs at 2 Mbit/s data),
including stuff bits and the inter-frame space. Six frames per cycle (one
command, four drive status, pedal; the later steering adds one). The battery
status at 50 to 100 Hz adds less than 1 % and is not in the table:

| Cycle rate | 1 / 4 Mbit/s | 1 / 2 Mbit/s | Classic CAN 1 Mbit/s (for comparison) |
|---|---|---|---|
| 250 Hz | about 15 % | about 25 % | about 30 % |
| 500 Hz | about 30 % | about 45 % (tight) | about 60 % (too high) |

- Start with 1 Mbit/s arbitration and 4 Mbit/s data on the cart (the TCAN334
  and the STM32H7 FDCAN support it; transmitter delay compensation on), and
  2 Mbit/s data as the fallback if the signal quality on the cart harness is
  not good enough.
- 250 Hz for the vehicle dynamics loop as the start; 500 Hz if the
  measurements in phase C2 leave room. Keep the bus load below about 50 %.
- Status messages may run at a lower rate than the command if the load is
  tight.

### 6.4 Simulation and PC tools

- **Single-process tests** (`tests/sim/`): no network, the existing virtual
  time harness.
- **Multi-node simulation:** one `native_sim` process per node on a SocketCAN
  `vcan` interface (`docs/zephyr_port_plan.md`, Phase 5: the
  `socketcan-native-sim` snippet, a larger MTU on `vcan` for CAN FD). pycyphal
  connects to the same interface. Creating `vcan` needs root once; whether the
  WSL2 kernel has the `vcan` module is unverified.
- **Fallback without `vcan`:** the simulation offers slcan (Lawicel) on four
  pseudo terminals that form one classic CAN bus; pycyphal and yakut use them
  with their python-can slcan media (implemented 2026-10-21). Cyphal/UDP with
  the TAP driver and libudpard stays an option for multi-node simulations;
  the offloaded sockets (NSOS) of Zephyr 4.4 passed no multicast socket
  options; those of 4.5 map them (source check), so NSOS may carry
  Cyphal/UDP without TAP.
- **Cart:** pycyphal on a USB CAN FD adapter (SocketCAN on Linux).

### 6.5 Comparison with Single Pair Ethernet (10BASE-T1S)

| | 10BASE-T1S (SPE) | CAN FD |
|---|---|---|
| Maturity | Standard from 2019; first automotive designs now | In production cars for years |
| Zephyr 4.4 | Young drivers: LAN865x only for Rev.B0/B1 and without multicast filtering; LAN867x on RMII without an in-tree STM32 board | Mature STM32 FDCAN driver (H7) |
| Cyphal library | libudpard 2.x, while upstream rewrites it for v1.1 | libcanard 4.x, the most used Cyphal library |
| Bandwidth | 10 Mbit/s shared, about 25 % load at 500 Hz | about 30 % at 500 Hz (1/4 Mbit/s) |
| Payload | Up to about 1400 bytes | 63 bytes per frame |
| Access | PLCA: round robin, all nodes equal; the coordinator node is a single point (fallback to CSMA/CD) | Priority arbitration; error counters and bus-off take a faulty node off the bus |
| Next to 1.6 kW drives | Little field experience | Decades of experience |
| Parts | PHY plus coupling network | Transceiver; the 4850 logic already has a TCAN334 |
| MCU | Ethernet MAC (H7) or a MAC-PHY on SPI | Any STM32H7 |
| PC tools | T1S USB adapter, Wireshark | Many USB CAN FD adapters, SavvyCAN, pycyphal |

**Zephyr v4.4.0 SPE status (checked in the source tree, not on hardware),
for the later option:**

- LAN8650/LAN8651 MAC-PHY on SPI: `drivers/ethernet/eth_lan865x.c` with the
  OPEN Alliance TC6 layer; Rev.B0/B1 only; accepts all multicast frames
  (hash table all ones); RX and TX in a driver thread over SPI. In-tree
  example on an STM32G474: `samples/net/zperf/boards/nucleo_g474re.overlay`.
- LAN8670/LAN8671/LAN8672 PHY on MII/RMII: `phy_microchip_t1s.c`
  (Rev.C1/C2/D0); no in-tree STM32 board uses it.
- PLCA: `phy_oa_tc14_plca.c`, devicetree properties `plca-enable`,
  `plca-node-id`, `plca-node-count`, `plca-burst-count`, `plca-burst-timer`,
  `plca-to-timer`, runtime change through `ETHERNET_CONFIG_TYPE_T1S_PARAM`.
- Others: NXP T1S PHY (`phy_nxp_t1s.c`), ADIN1110/ADIN2111 10BASE-T1L,
  TJA1103/TJA11xx 100BASE-T1.
- Reported outside the tree: no IPv6 with the LAN865x driver, crashes on
  ESP32-C3/S3, LAN8651 TX lock-ups.

SPE becomes interesting again when the cart needs more bandwidth (camera,
lidar, a Linux display node) or when the Zephyr drivers and Cyphal/UDP have
matured. Because every message fits in one CAN FD frame and the application
sits behind `lib/cyphal`'s `Transport`, the change is a transport swap plus
hardware. A Linux display or logging node can also join later through a
CAN-to-Ethernet gateway without changing the drives.

## 7. Fault Reactions

The pedal unit only publishes measurements. Each drive decides on its own:

| Situation | Reaction |
|---|---|
| VDC alive | Drives follow the VDC torque (drive and regenerative braking). |
| VDC times out (for example three missed cycles) | Drives: limp-home mode, torque from the pedals, brake pedal has priority, the same torque on all wheels, reduced limit, no torque vectoring, ramped. |
| VDC comes back | Back to normal only at standstill. |
| Pedal unit lost, VDC alive | VDC ramps the drive torque to zero and brakes gently by regeneration. |
| VDC and pedal unit lost | Drives ramp the torque to zero; the driver uses the mechanical emergency brake. |
| One drive lost | VDC removes the opposite wheel of the axle too (or redistributes the torque without a yaw moment), limits the speed, shows the fault. |
| Drive in bus-off | The drive treats it like a VDC and pedal timeout: torque to zero, ramped; it tries to rejoin the bus. |
| Steering angle sensor lost | VDC switches off torque vectoring and drift assist, equal torque distribution. |
| BMS lost (timeout) | Drives fall back to a conservative regenerative limit stored in their settings; VDC limits the speed and shows the fault. The chopper keeps working locally. |
| BMS opens the contactor | Drives ramp the regenerative torque to zero; the chopper holds the DC link; the VDC shows the fault and the driver uses the mechanical brake. |
| Chopper overtemperature | BMS lowers the published charge limit; drives brake less, the mechanical brake covers the rest. |
| E-stop | Hard-wired to the external enable and break input of every power stage; independent of the bus. The mechanical brake stops the cart. |

The fallback logic runs in a Zephyr thread of the drive, never in the hot
path (`AGENTS.md`, zero-latency rules).

Implemented in the drive (2026-10-23, `src/drive/cart_control.*`, tested in
the simulation, not on hardware):

- **VDC timeout:** limp-home after `unimoc.cart.timeout` without a wheel
  command, while the pedal state is fresh. Each drive applies the pedal
  torque to its own wheel (the same torque on all wheels when all drives
  have the same `unimoc.cart.limp_home_torque`). The readiness timeout waits
  during limp-home, so the drive stays engaged without the VDC; the cart
  timeout must not exceed the command timeout (settings validation).
- **VDC comes back:** a fresh wheel command takes over again only at
  standstill (`unimoc.cart.standstill_speed`).
- **VDC and pedal unit lost:** the torque ramps to zero, then the readiness
  timeout disengages the drive.
- **Bus-off:** every cart input counts as lost at once (the torque ramps to
  zero, no limp-home on old pedal states). The controller rejoins the bus
  by itself (no `CONFIG_CAN_MANUAL_RECOVERY_MODE`); the drive counts it in
  `unimoc.can.bus_off_count` and logs the return.
- **BMS lost:** the battery limits of the settings apply again; the BMS
  limits are scaled by `unimoc.cart.battery_share` for the drives of one
  battery. With the contactor open or precharging the drive takes no
  battery current; an announced opening (`CONTACTOR_OPENING`) keeps the
  published limits, which the BMS ramps down itself.
- Open: the VDC reaction to a lost drive, and the limp-home with a
  direction selector (the drive pedal only drives forward).

### 7.1 Consequences of pure regenerative braking

- **Bus or VDC failure:** the brake pedal must keep working through the
  limp-home path (pedal unit → drives directly). That path becomes a safety
  function, not a comfort feature.
- **Full battery or opened contactor:** the battery cannot take the braking
  current (the Jacobi profile allows only 20 A charge). The drives limit the
  regenerative torque to the charge limit the BMS publishes; the brake
  chopper of the BMS takes what is left (section 5.5); the mechanical brake
  covers the rest.
- **DC-link overvoltage:** with the battery disconnected during braking the
  DC link rises fast; the H7 power stages carry 60 V FETs (NCEP6080AG) and run 36 V systems. The
  chopper limits the voltage, and each drive still needs its own
  overvoltage limit in the current controller and a hard trip, verified on
  the bench with the chopper disconnected. Implemented in the drive
  (2026-10-22): regenerative derating from 55 V, trip at 58 V (since 2026-11-08 per board
  from the devicetree: 45 V / 48 V on the 60 V H7 power stages). The
  trip switches all transistors off, so at high speed the motor still
  charges the DC link through the body diodes; accepted as a known risk on
  Jacobi. The brake chopper of the BMS (section 5.5) and, with an EESM, the
  de-excitation by the excitation module mitigate it, section
  5.2.1.
- **Low speed and standstill:** regeneration fades near zero speed and
  cannot hold the cart; holding torque at standstill costs battery energy.
  Parking needs the mechanical brake (or a parking brake).
- **Power stage off (fault, E-stop):** no electric braking at all; the
  mechanical emergency brake must stop the cart from top speed on its own.

## 8. Software Architecture

| Part | Where | Content |
|---|---|---|
| FOC core | `lib/motor_control`, `src/current_control` | Unchanged, shared by all UNIMOC drives |
| Drive application | `app/` (later `apps/drive/`) | Torque mode with regenerative braking, wheel speed, DC-link limits, limp-home mode, Cyphal node |
| VDC application | `apps/vdc/` with `lib/vehicle/` | Torque distribution, vehicle state estimation, display, logging |
| Pedal application | `apps/pedal/` | Pedal sensors, plausibility, publishing |
| BMS application | `apps/bms/` with `lib/battery/` | Cell monitoring, contactor and precharge, state of charge, current limits, brake chopper |
| Platforms | `hardware/zephyr_stm32h7` (Jacobi, NUCLEO-H743ZI), `hardware/zephyr_sim` | Hot path per MCU, the rest shared through `hardware/zephyr_common` |
| Cyphal layer | `lib/cyphal` | `Transport` with a CAN-FD implementation (libcanard), register service on `SettingsStore`, heartbeat, GetInfo, file update; a UDP implementation (libudpard) only for the simulation fallback |
| Simulation | `native_sim` | Single-process deterministic tests (`tests/sim/`), plus multi-process runs: one `native_sim` per node on `vcan` (section 6.4) |
| Vehicle model | `lib/sim` (P4 in `TODO.md`) | Four wheels, tyres, steering angle, shared battery; needed for VDC tests |
| Tools | Python, pycyphal | Register access, telemetry, scenarios, the operator GUI and CLI [cypilot](https://github.com/Tecnologic/cypilot) (contract: `docs/gui_plan.md`) over CAN FD |
| Steering assist | Later | Own plan once the steering concept exists |

## 9. Porting from the `electric_quad` Firmware

| Legacy ([`src/`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/src)) | In this plan |
|---|---|
| Motor and battery parameters in [`settings.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/settings.cpp) (for example Rs 0.15 Ω, L 0.22/0.25 mH, ψ 1.9 mWb, 7 pole pairs, 40 A, 32 kHz PWM, 1000 ns dead time; to check against the actual quad motor) | Starting point for the 4850 settings profile and the sim motor |
| "High power mode" in [`management.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/management.cpp) (80 A, unlocked by a switch sequence at power-up) | A settings register or profile selected by the VDC, within the hardware capabilities |
| Throttle mapping and dead zones (`SetThrottleSetpoint`) | Pedal unit and VDC (pedal map); limp-home mode in the drive |
| PAS ([`pas.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/pas.cpp)) | Not needed in the cart |
| Hall, flux and HFI observers, mechanical observer ([`observer.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/observer.cpp)) | Already in `lib/motor_control`; Hall fusion is milestones M5/M6; HFI is not ported (removed 2026-11-03), the Hall sensors cover standstill |
| AS5048 encoder ([`as5048b.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/as5048b.cpp)) | Steering angle sensor driver on the VDC |
| FreeMASTER over USB | Cyphal registers and cymon traces |
| EEPROM settings (24LC04) | Zephyr settings on ZMS (`docs/settings_plan.md`) |
| Board pinning ([`logic/pinning`](https://github.com/Tecnologic/hardware_4850/tree/master/logic/pinning), [`hardware/4850`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/hardware/4850)) | Reference for the old 4850 pinout (STM32F730); the Jacobi pins are in the unimoc_jacobi schematic |

## 10. Roadmap

Each phase keeps `check_format.sh` and `twister_sca.sh` green.

| Phase | Content | Hardware | Exit criterion |
|---|---|---|---|
| A | Hot path M4 to M7 (`docs/hotpath_plan.md`), sim made interactive (motor parameters, setpoints, telemetry) | none | Sim tests: speed step, Hall failure |
| B | Cyphal v1.0 spike: `native_sim` on `vcan` with pycyphal (fallback UDP over TAP); DSDL namespace; register service, heartbeat, GetInfo | none | One register exchange and one telemetry subject with pycyphal |
| C | ~~G4 bring-up~~ dropped 2026-11-03: M0 and M8, Cyphal/CAN-FD move to the NUCLEO-H743ZI and Jacobi (D) | NUCLEO-H743ZI | Closed current loop with the 50 A limit; registers over CAN FD |
| C2 | CAN FD bus test: the NUCLEO-H743ZI and evaluation boards on a cart-length harness, 1/4 and 1/2 Mbit/s, bus load and latency at 250 and 500 Hz | NUCLEO-H743ZI, evaluation boards, USB CAN FD adapter | Measured load and latency; bit rate and cycle rate chosen |
| D | Jacobi (4850 H743 logic board): schematic, layout, H7 platform (`hardware/zephyr_stm32h7`, bring-up on a NUCLEO-H743ZI, board `unimoc_jacobi`), regenerative braking with DC-link limits, simulated first | Jacobi, quad motor | Same tests as C on the 4850; braking into a full battery and with the battery disconnected on the bench |
| E | VDC (H7) and pedal unit hardware, steering angle sensor; `apps/` split | VDC, pedal unit | Pedal and angle published on the bus, VDC commands a drive on the bench |
| E2 | BMS controller with brake chopper: board, `apps/bms/`, `lib/battery/`, chopper thresholds, contactor sequence | BMS board, battery, resistor | Regenerative braking into a full battery and with an open contactor on the bench, DC link held below the limit |
| F | VDC firmware, multi-node sim with the vehicle model, fault reactions of section 7 in the sim | none, then VDC | All fault cases of section 7 pass in the sim |
| G | Cart integration: two wheels first, then four; torque vectoring and drift assist | cart | Driving tests with the fault cases on a closed area |
| H | Tools and GUI over Cyphal/CAN-FD (`docs/software_in_loop_plan.md`, P3) | PC, USB CAN FD adapter | Parameter set and telemetry plots from the GUI |
| later | Steering assist as a UNIMOC node; SPE if the bandwidth is needed (section 6.5) | | Own plans |

Phases A and B run in parallel without hardware; C2 and the schematic of D
can start while C runs.

## 11. Safety Outline

Not a safety concept; a list of what one must cover.

- E-stop and power-stage cutoff hard-wired, never only over the bus.
- Mechanical emergency brake able to stop the cart from top speed on its own
  (section 7.1).
- The regenerative service brake keeps working without the VDC (limp-home
  path); its limits (full battery, disconnected battery, low speed) are known
  to the VDC and shown to the driver.
- Steering stays mechanical.
- Setpoint timeouts in every receiver; ramps instead of steps on mode changes.
- Pedals with two channels and a plausibility check.
- Check the applicable standard for a manned light vehicle (for example
  EN 17128 or ISO 13849) before the cart phase G.

## 12. Open Questions

1. **VDC MCU:** STM32H7 (decided 2026-11-08: all boards are H7 based); the part is open.
2. **4850 logic MCU:** decided 2026-11-01, STM32H743VIT6. Open: board size
   with the LQFP100 in the enclosure, the silicon revision of the bought
   parts, the points of `docs/drive4850_h7_plan.md` section 8.
3. **Bit rates and cycle rate:** 1/4 or 1/2 Mbit/s, 250 or 500 Hz; from the
   bus test C2.
4. **`vcan` on WSL2:** available, or the UDP-over-TAP fallback for the
   simulation.
5. **BMS board (Volta):** a new STM32H7 board; cell monitoring
   front end; cell count and chemistry.
6. **Display:** connection to the VDC and content.
7. **Brake resistor:** power and energy rating from the cart mass, top speed
   and longest braking; chopper switch and its cooling.
8. **Steering assist:** concept, torque sensor, interface (later).
9. ~~**Fourth half bridge (section 5.2.1)**~~: dropped from the drive
   (2026-11-03, confirmed 2026-11-08); the excitation and chopper are a
   separate board with its own open points (placement, comparator routing,
   field and resistor ratings, de-excitation of the EESM).

## 13. Risks

- **Cyphal library churn:** every library is moving to v1.1 at the same time.
  Mitigation: libcanard 4.x pinned (Cyphal/CAN v1.1 stays wire-compatible
  with v1.0), the application behind `lib/cyphal`'s `Transport` and a thin own
  protocol layer.
- **Bus load and signal quality at 4 Mbit/s data:** measured in phase C2 on a
  cart-length harness; fallback 2 Mbit/s data and 250 Hz.
- **Simulation network:** `vcan` on WSL2 unverified; fallback UDP over TAP.
- **Pure regenerative braking:** the service brake depends on the drives,
  the battery state and the bus; covered by the limp-home path, the brake
  chopper and the mechanical emergency brake (section 7.1).
- **Chopper in hardware:** the comparator-to-timer path of the H743 has to be
  confirmed in the schematic (BMS and the separate excitation and chopper
  board);
  otherwise the chopper needs a fast interrupt.
- **ASM and EESM only in the sim:** plants and runtime paths exist since
  2026-11-08 (sensorless ASM, EESM with a simulated excitation module); no
  real machine has run on them, and the sensorless ASM at standstill depends
  on exact motor parameters.
- **H743 logic board:** the LQFP100 needs more board area than the LQFP64,
  a second hot-path platform is to maintain, and the H7 adds cache, MPU and
  power-supply details (`docs/drive4850_h7_plan.md`).
- **Vehicle model effort:** the VDC cannot be tested in the sim without it
  (phase F).
