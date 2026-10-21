# System Plan: Manned Cart with Four Wheel Drives

> **Status (2026-10-21):** draft to sort the overall direction. Nothing in
> this file is built or verified on hardware. Decided so far:
>
> - Cyphal/CAN-FD for the cart; Single Pair Ethernet is a later option.
> - The new 4850 logic board uses an STM32G4, the same MCU family as the BCC.
> - The VDC firmware lives in this repository.
> - A mechanical emergency brake, otherwise pure regenerative braking.
> - A BMS controller with brake chopper is another bus node and another
>   application in this repository.
> - The steering servo is out of scope for now; the first rides use a
>   steering angle sensor only.

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
| [Tecnologic/hardware_4850](https://github.com/Tecnologic/hardware_4850) | 48 V / 50 A drive in an RND 455-00370 enclosure. [`power/`](https://github.com/Tecnologic/hardware_4850/tree/master/power): power stage, kept unchanged. [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic): logic board, redesigned with an STM32G4 (section 5.2). |
| [Tecnologic/unimoc_old_software, branch `electric_quad`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad) | Legacy firmware (ChibiOS, STM32F730) of the 1.6 kW children's quad on the 4850 hardware. Source for proven settings and features to port (section 9); not built any more. |
| `lib/cymon` (submodule) | Tracing and monitoring; transported over Cyphal later (`docs/zephyr_port_plan.md`, Phase 5). |

### 2.1 What the 4850 hardware is (from the KiCad schematics)

- **Power stage (`power/phase.sch`, three sheets):** per phase BSC040N08
  MOSFETs (80 V), an LM5109B half-bridge driver, 2 mΩ shunts with an
  INA240A1 inline current amplifier, NTC on the heat sink. Proven over years
  in the quad (1.6 kW) and kept as it is.
- **Logic (`logic/`):** STM32F730R8 (LQFP64), TCAN334 CAN transceiver
  (CAN FD capable), USB mini-B, 24LC04 EEPROM, Hall and motor temperature
  inputs, GS8632 op-amp filters for the current and voltage channels, LM5008
  and LMR12010 supplies from 48 V.

The G4 redesign replaces only the logic board. Interfaces to the power
board (PWM, ADC, supply) stay the same, so the power stage keeps its track
record.

## 3. System Architecture

```text
   IMU (SPI)   GNSS (UART)   Steering angle sensor   Display
        \          |                 |                 /
         +---------+-----------------+----------------+
         |        Vehicle dynamics controller (VDC)    |   STM32G4 or H7, Zephyr
         +----------------------+----------------------+
                                |
     ======================= CAN FD (Cyphal/CAN) ========================
        |            |            |            |             |              |
   Drive FL      Drive FR     Drive RL     Drive RR      Pedal unit     BMS controller
   UNIMOC        UNIMOC       UNIMOC       UNIMOC        (drive + brake (cells, contactor,
   G4 + 4850     G4 + 4850    G4 + 4850    G4 + 4850     pedal, G4)     brake chopper, G4)

   (later: steering assist as another UNIMOC node)

   USB CAN FD adapter ──► PC tools (pycyphal) on the same bus
   Hard-wired E-stop loop ──► external enable / break input of every power stage
   Mechanical emergency brake ──► independent of bus and electronics
```

| Node | Hardware | Function |
|---|---|---|
| VDC | STM32G4 (or H7 if the display or logging need it), Zephyr | Driver inputs, vehicle state estimation (IMU, GNSS, wheel speeds, steering angle), torque distribution, torque vectoring, drift assist, display, logging |
| Wheel drives (2 or 4) | 4850 power stage + new G4 logic | Torque control per wheel (drive and regenerative braking), wheel speed, limp-home mode (section 7) |
| Pedal unit | Small STM32G4 board | Publishes drive and brake pedal positions (two sensor channels per pedal); sends no commands |
| BMS controller | STM32G4 board in the battery (section 5.5) | Cell monitoring, contactor and precharge, state of charge, charge and discharge current limits, brake chopper on the DC link |
| Steering angle sensor | Standard sensor on the VDC (for example the AS5048 of the legacy firmware) | Steering angle for the vehicle dynamics; no bus node |
| IMU, GNSS, display | Standard modules on the VDC | No bus node |
| Steering assist | Later: UNIMOC | Out of scope until a steering concept exists |
| G4 BCC (`boards/unimoc/bcc`) | Existing STM32G473 board | Development and bring-up platform; the reference for the G4 hot path; not part of the cart |

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
| Logic board MCU | **STM32G4** for the 4850 logic board (section 5.2), the same family as the BCC. | One hot-path platform: `hardware/zephyr_bcc` is the base, no H7 port. FDCAN on chip. |
| VDC firmware | **In this repository**, as a second Zephyr application (`apps/vdc/`) with a portable `lib/vehicle/`. The MCU is a Zephyr-supported STM32 (G4 or H7, section 5.3); a Raspberry Pi or an ESP32-P4 are out, they do not run this repository's Zephyr build. | One toolchain, one CI, shared libraries and DSDL, the whole cart in the simulation. |
| Cyphal/serial | Not used. | No maintained C implementation (libserard). |
| USB | Firmware update (DFU) only on the cart. The CDC-ACM log stays until Cyphal runs on the board, then logs go out as `uavcan.diagnostic.Record`. | One operating interface; USB stays the recovery path. |
| Settings | Cyphal registers on the existing `SettingsStore` (`docs/settings_plan.md`). | Already designed that way. |
| Firmware update | Cyphal file service into flash bank 2 (512 KB dual-bank G4, `docs/settings_plan.md`); USB DFU as recovery. | Update all nodes over the bus without opening the cart. |

## 5. Hardware Plan

### 5.1 G4 BCC (existing)

- Development target for the hot path (milestones M0 to M8 in
  `docs/hotpath_plan.md`) and for Cyphal/CAN-FD.
- Board change: CAN-FD transceiver instead of the SN65HVD230 (open item in
  `TODO.md`).
- Its hot path is the reference for the 4850 logic board.

### 5.2 New 4850 logic board (STM32G4)

Requirements for the redesign of [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic):

- **MCU:** an STM32G473 or STM32G474 with 512 KB dual-bank flash (for the
  firmware update into bank 2), in LQFP64 like the current STM32F730R8
  (for example STM32G473RET6). Check against the current interfaces: TIM1 or
  TIM8 with the six PWM outputs and the break input, enough ADC channels on
  at least three ADCs for simultaneous injected sampling of the three phase
  currents and the DC link (the BCC uses ADC1 to ADC4), FDCAN1 pins, USB,
  SWD, Hall inputs, temperatures.
- **Current sensing:** the INA240A1 inline phase amplifiers stay. Like the
  BCC (`kCurrentSenseIsLowSide = false`), they measure the phase current
  independent of the switching state; check the gain, the offset and the
  filter of the GS8632 stages against the G4 ADC range.
- **CAN FD:** keep the TCAN334 (CAN FD capable), with a split termination
  that can be populated at the bus ends.
- **Keep:** USB for DFU, SWD, Hall and temperature inputs, the power board
  connector and the supply topology.
- **Replace:** the 24LC04 EEPROM is no longer needed (settings on ZMS in the
  G4 flash, `docs/settings_plan.md`).
- **Prepare for the later steering variant:** an SPI or I2C connector for an
  absolute angle sensor.
- **Platform layer:** generalize `hardware/zephyr_bcc` to the G4 family, with
  the board differences (pins, ADC channels, timer, dead time, sample times)
  in the devicetree (`foc0` node, as in `boards/unimoc/bcc/bcc.dts`). A new
  board definition `boards/unimoc/drive4850/` (name to decide).

### 5.3 VDC

The VDC has no zero-latency hot path: a 250 to 500 Hz control loop, sensor
drivers, the display and logging. It runs Zephyr and its firmware lives in
this repository.

| | STM32G4 | STM32H7 |
|---|---|---|
| Hardware | The 4850 G4 logic board without power stage, or a small own board | Own board (or an evaluation board for a start) |
| Compute | 170 MHz with FPU; enough for a 250 to 500 Hz vehicle dynamics loop | 480 to 550 MHz, more RAM |
| Display and logging | Small display on SPI, logging to an SD card on SPI or to flash | Parallel display (LTDC on some parts), SDMMC |
| Platform effort | Shares the G4 board support | A second MCU family to support |

**Recommendation:** start with the G4 (the logic board design and board
support are shared). Move to an H7 only if the display or the logging need
it; the firmware is portable between both.

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

- Small STM32G4 board with a CAN FD transceiver, two independent sensor
  channels per pedal (plausibility check as in automotive pedal modules).
- Firmware in this repository (`apps/pedal/`), or a role of the VDC board
  placed at the pedals.

### 5.5 BMS controller with brake chopper

- **Board:** STM32G4 with a CAN FD transceiver, a cell monitoring front end
  (part to choose), current measurement in the battery path, contactor and
  precharge drivers, and a brake chopper switch (MOSFET or half bridge) with
  a resistor on the DC link. Whether the existing BCC board can take this
  role is open (section 12).
- **Firmware:** `apps/bms/` in this repository, with a portable
  `lib/battery/` (state of charge, current limits from cell voltage and
  temperature, chopper control) and ztest suites in `tests/lib/battery/`.
- **Brake chopper control:**
  - The chopper switches on the DC-link voltage measured on the BMS board,
    not on a bus message, so it also works without the VDC and the bus.
  - First stage in hardware: a G4 comparator (COMP) with a DAC threshold
    drives the chopper timer directly (hysteresis or PWM without the CPU).
    The firmware sets the thresholds and monitors the resistor temperature
    and energy. Whether this fits the G4 comparator and timer routing is to
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

### 6.2 Messages (first draft)

Subject and register IDs are not assigned yet; they are fixed in a DSDL
namespace in this repository before any firmware uses them. Every message
fits in one CAN FD frame.

| Message | From → to | Rate | Content |
|---|---|---|---|
| Wheel command | VDC → drives | 250 to 500 Hz | Torque per wheel, drive or brake (one message for all wheels, applied on reception), mode |
| Drive status | each drive → VDC | same | Wheel speed, torque estimate, DC link, temperatures, regen limit, fault flags |
| Pedal state | pedal unit → VDC and drives | same | Drive and brake pedal, both channels, plausibility flag |
| Battery status | BMS → VDC and drives | 50 to 100 Hz | Pack voltage and current, state of charge, charge and discharge current limits, contactor state, chopper state and temperature |
| Heartbeat | all | 1 Hz (standard) | Health, mode |
| Registers, GetInfo, file update | VDC/PC ↔ all | on demand, low priority | Settings, version, firmware update |
| Diagnostics | all | event, low priority | `uavcan.diagnostic.Record` instead of the USB log |

The wheel command gets the highest Cyphal priority, then the pedal state and
the drive status; services, file transfers and diagnostics run at low
priority and cannot delay the control loop.

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
  and the STM32G4 FDCAN support it; transmitter delay compensation on), and
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
- **Fallback:** Cyphal/UDP with the `native_sim` TAP driver (`eth_native_tap`)
  and libudpard 2.x. The offloaded sockets (NSOS) of Zephyr 4.4 pass no
  multicast socket options, so they cannot carry Cyphal/UDP.
- **Cart:** pycyphal on a USB CAN FD adapter (SocketCAN on Linux).

### 6.5 Comparison with Single Pair Ethernet (10BASE-T1S)

| | 10BASE-T1S (SPE) | CAN FD |
|---|---|---|
| Maturity | Standard from 2019; first automotive designs now | In production cars for years |
| Zephyr 4.4 | Young drivers: LAN865x only for Rev.B0/B1 and without multicast filtering; LAN867x on RMII without an in-tree STM32 board | Mature STM32 FDCAN driver (G4 and H7) |
| Cyphal library | libudpard 2.x, while upstream rewrites it for v1.1 | libcanard 4.x, the most used Cyphal library |
| Bandwidth | 10 Mbit/s shared, about 25 % load at 500 Hz | about 30 % at 500 Hz (1/4 Mbit/s) |
| Payload | Up to about 1400 bytes | 63 bytes per frame |
| Access | PLCA: round robin, all nodes equal; the coordinator node is a single point (fallback to CSMA/CD) | Priority arbitration; error counters and bus-off take a faulty node off the bus |
| Next to 1.6 kW drives | Little field experience | Decades of experience |
| Parts | PHY plus coupling network | Transceiver; the 4850 logic already has a TCAN334 |
| MCU | Ethernet MAC (H7) or a MAC-PHY on SPI | Any STM32G4 |
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

### 7.1 Consequences of pure regenerative braking

- **Bus or VDC failure:** the brake pedal must keep working through the
  limp-home path (pedal unit → drives directly). That path becomes a safety
  function, not a comfort feature.
- **Full battery or opened contactor:** the battery cannot take the braking
  current (the BCC profile allows only 20 A charge). The drives limit the
  regenerative torque to the charge limit the BMS publishes; the brake
  chopper of the BMS takes what is left (section 5.5); the mechanical brake
  covers the rest.
- **DC-link overvoltage:** with the battery disconnected during braking the
  DC link rises fast; the BSC040N08 are 80 V parts on a 48 V system. The
  chopper limits the voltage, and each drive still needs its own
  overvoltage limit in the current controller and a hard trip, verified on
  the bench with the chopper disconnected.
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
| Platforms | `hardware/zephyr_bcc` (generalized to the G4 family), `hardware/zephyr_sim` | Hot path per MCU, the rest shared through `hardware/zephyr_common` |
| Cyphal layer | `lib/cyphal` | `Transport` with a CAN-FD implementation (libcanard), register service on `SettingsStore`, heartbeat, GetInfo, file update; a UDP implementation (libudpard) only for the simulation fallback |
| Simulation | `native_sim` | Single-process deterministic tests (`tests/sim/`), plus multi-process runs: one `native_sim` per node on `vcan` (section 6.4) |
| Vehicle model | `lib/sim` (P4 in `TODO.md`) | Four wheels, tyres, steering angle, shared battery; needed for VDC tests |
| Tools | Python, pycyphal | Register access, telemetry, scenarios, later the GUI (`docs/software_in_loop_plan.md`, P3) over CAN FD |
| Steering assist | Later | Own plan once the steering concept exists |

## 9. Porting from the `electric_quad` Firmware

| Legacy ([`src/`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/src)) | In this plan |
|---|---|
| Motor and battery parameters in [`settings.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/settings.cpp) (for example Rs 0.15 Ω, L 0.22/0.25 mH, ψ 1.9 mWb, 7 pole pairs, 40 A, 32 kHz PWM, 1000 ns dead time; to check against the actual quad motor) | Starting point for the 4850 settings profile and the sim motor |
| "High power mode" in [`management.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/management.cpp) (80 A, unlocked by a switch sequence at power-up) | A settings register or profile selected by the VDC, within the hardware capabilities |
| Throttle mapping and dead zones (`SetThrottleSetpoint`) | Pedal unit and VDC (pedal map); limp-home mode in the drive |
| PAS ([`pas.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/pas.cpp)) | Not needed in the cart |
| Hall, flux and HFI observers, mechanical observer ([`observer.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/observer.cpp)) | Already in `lib/motor_control`; Hall fusion is milestones M5/M6, HFI later for standstill |
| AS5048 encoder ([`as5048b.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/as5048b.cpp)) | Steering angle sensor driver on the VDC |
| FreeMASTER over USB | Cyphal registers and cymon traces |
| EEPROM settings (24LC04) | Zephyr settings on ZMS (`docs/settings_plan.md`) |
| Board pinning ([`logic/pinning`](https://github.com/Tecnologic/hardware_4850/tree/master/logic/pinning), [`hardware/4850`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/hardware/4850)) | Reference for the G4 pinout and the devicetree of the new board |

## 10. Roadmap

Each phase keeps `check_format.sh` and `twister_sca.sh` green.

| Phase | Content | Hardware | Exit criterion |
|---|---|---|---|
| A | Hot path M4 to M7 (`docs/hotpath_plan.md`), sim made interactive (motor parameters, setpoints, telemetry) | none | Sim tests: speed step, Hall failure |
| B | Cyphal v1.0 spike: `native_sim` on `vcan` with pycyphal (fallback UDP over TAP); DSDL namespace; register service, heartbeat, GetInfo | none | One register exchange and one telemetry subject with pycyphal |
| C | G4 bring-up: M0 and M8 on the BCC, CAN-FD transceiver, Cyphal/CAN-FD | BCC | Closed current loop with the 50 A limit; registers over CAN FD |
| C2 | CAN FD bus test: the BCC and G4 evaluation boards on a cart-length harness, 1/4 and 1/2 Mbit/s, bus load and latency at 250 and 500 Hz | BCC, evaluation boards, USB CAN FD adapter | Measured load and latency; bit rate and cycle rate chosen |
| D | 4850 G4 logic board: schematic, layout, G4 platform generalized from the BCC, regenerative braking with DC-link limits | new logic board, quad motor | Same tests as C on the 4850; braking into a full battery and with the battery disconnected on the bench |
| E | VDC (G4) and pedal unit hardware, steering angle sensor; `apps/` split | VDC, pedal unit | Pedal and angle published on the bus, VDC commands a drive on the bench |
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

1. **VDC MCU:** STM32G4 (recommended start) or STM32H7.
2. **G4 part for the 4850 logic:** STM32G473RET6 or STM32G474RET6 (LQFP64,
   512 KB); pin and ADC check against the current interfaces.
3. **Bit rates and cycle rate:** 1/4 or 1/2 Mbit/s, 250 or 500 Hz; from the
   bus test C2.
4. **`vcan` on WSL2:** available, or the UDP-over-TAP fallback for the
   simulation.
5. **BMS board:** the existing BCC or a new STM32G4 board; cell monitoring
   front end; cell count and chemistry.
6. **Display:** connection to the VDC and content.
7. **Brake resistor:** power and energy rating from the cart mass, top speed
   and longest braking; chopper switch and its cooling.
8. **Steering assist:** concept, torque sensor, interface (later).

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
- **Chopper in hardware:** the comparator-to-timer path of the G4 has to be
  confirmed in the schematic; otherwise the chopper needs a fast interrupt on
  the BMS controller.
- **G4 pin and ADC budget in LQFP64:** checked before the schematic of
  phase D; a larger package changes the logic board layout.
- **Vehicle model effort:** the VDC cannot be tested in the sim without it
  (phase F).
