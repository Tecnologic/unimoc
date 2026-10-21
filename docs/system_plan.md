# System Plan: Manned Cart with Four Wheel Drives

> **Status (2026-10-21):** draft to sort the overall direction. Nothing in
> this file is built or verified on hardware. Where it changes an earlier
> decision (Cyphal/CAN-FD only, shell and log over USB), the earlier plans
> (`docs/zephyr_port_plan.md`, `TODO.md`) are updated once this draft is
> accepted.

## 1. Goal

A manned cart with up to four direct wheel drives, an electric steering
assist, torque vectoring and drift assist. UNIMOC supplies all actuators
(four wheel drives and the steering servo); a central vehicle dynamics
controller (VDC) coordinates them. Standard modules are reused where they
connect to the VDC directly (IMU, GNSS, display).

## 2. Repositories

| Repository | Role in this plan |
|---|---|
| [Tecnologic/unimoc](https://github.com/Tecnologic/unimoc) (this repository, branch `develop`) | Drive firmware on Zephyr: FOC hot path, observers, settings, `native_sim` simulation, tests. Target for the drive, steering and (open, see section 12) VDC firmware. |
| [Tecnologic/hardware_4850](https://github.com/Tecnologic/hardware_4850) | 48 V / 50 A drive in an RND 455-00370 enclosure. [`power/`](https://github.com/Tecnologic/hardware_4850/tree/master/power): power stage, kept unchanged. [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic): logic board, redesigned with an STM32H7 and Single Pair Ethernet (section 5.2). |
| [Tecnologic/unimoc_old_software, branch `electric_quad`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad) | Legacy firmware (ChibiOS, STM32F730) of the 1.6 kW children's quad on the 4850 hardware. Source for proven settings and features to port (section 9); not built any more. |
| `lib/cymon` (submodule) | Tracing and monitoring; transported over Cyphal later (`docs/zephyr_port_plan.md`, Phase 5). |

### 2.1 What the 4850 hardware is (from the KiCad schematics)

- **Power stage (`power/phase.sch`, three sheets):** per phase BSC040N08
  MOSFETs (80 V), an LM5109B half-bridge driver, 2 mΩ shunts with an
  INA240A1 inline current amplifier, NTC on the heat sink. Proven over years
  in the quad (1.6 kW) and kept as it is.
- **Logic (`logic/`):** STM32F730R8, TCAN334 CAN transceiver (CAN FD
  capable), USB mini-B, 24LC04 EEPROM, Hall and motor temperature inputs,
  GS8632 op-amp filters for the current and voltage channels, LM5008 and
  LMR12010 supplies from 48 V.

The H7 redesign replaces only the logic board. Interfaces to the power
board (PWM, ADC, supply) stay the same, so the power stage keeps its track
record.

## 3. System Architecture

```text
                 IMU (SPI)   GNSS (UART)   Display (UART/CAN/…)
                      \          |           /
                       +---------+----------+
                       |  Vehicle dynamics   |   central, STM32H7 + SPE
                       |  controller (VDC)   |
                       +----------+----------+
                                  |
     ===================== Single Pair Ethernet (Cyphal/UDP) ====================
        |            |            |            |             |             |
   Drive FL      Drive FR     Drive RL     Drive RR     Steering       Pedal unit
   UNIMOC        UNIMOC       UNIMOC       UNIMOC       UNIMOC         (drive + brake
   H7 + 4850     H7 + 4850    H7 + 4850    H7 + 4850    assist +       pedal, 2 channels
                                                        angle sensor   each)

   Hard-wired E-stop loop ──► external enable / break input of every power stage
```

| Node | Hardware | Function |
|---|---|---|
| VDC | H7 logic design with SPE (same as the drive logic, without power stage) or a separate board | Driver inputs, vehicle state estimation (IMU, GNSS, wheel speeds, steering angle), torque distribution, torque vectoring, drift assist, display, logging |
| Wheel drives (2 or 4) | 4850 power stage + new H7 logic | Torque control per wheel, wheel speed, limp-home mode (section 7) |
| Steering servo | UNIMOC (4850 or smaller power stage) | Assist torque on a mechanical steering column, absolute steering angle |
| Pedal unit | Small MCU with SPE | Publishes drive and brake pedal positions (two sensor channels per pedal); sends no commands |
| IMU, GNSS, display | Standard modules on the VDC | No bus node; standard drivers on the VDC |
| G4 BCC (`boards/unimoc/bcc`) | Existing STM32G473 board | Development and bring-up platform with Cyphal/CAN-FD; not part of the cart |

## 4. Decisions

| Topic | Decision | Reason |
|---|---|---|
| Control structure | One central VDC. If it fails, the drives fall back to the pedal unit (section 7). | The vehicle dynamics need all sensors in one place; a VDC failure must still let the driver stop. |
| Protocol | **Cyphal v1.0**, libraries pinned in `west.yml`: libcanard 4.x (CAN FD), libudpard 2.x (UDP), Nunavut for DSDL, pycyphal for tools and tests. | All bus nodes are own nodes; publish/subscribe like DDS without a broker; settings already use register names (`unimoc.motor.pole_pairs`). Cyphal v1.1 (cy, libudpard 3, libcanard 5) is alpha; revisit after its release. |
| Cart transport | **Cyphal/UDP over Single Pair Ethernet**, preferably 10BASE-T1S (section 6.1). | More bandwidth than CAN FD, bus wiring like CAN, same transport as the `native_sim` simulation. |
| G4 transport | Cyphal/CAN-FD; the SN65HVD230 on the BCC is replaced by a CAN-FD transceiver. | The BCC is the hardware at hand; the application layer is the same as on UDP. |
| Cyphal/serial | Not used. | No maintained C implementation (libserard). |
| USB | Firmware update (DFU) only on the cart. The CDC-ACM log stays until Cyphal runs on the board, then logs go out as `uavcan.diagnostic.Record`. | One operating interface; USB stays the recovery path. |
| Steering | Assist on a mechanical column, not steer-by-wire; the servo goes passive on any fault. | A VDC or servo failure leaves the driver with mechanical steering. |
| Settings | Cyphal registers on the existing `SettingsStore` (`docs/settings_plan.md`). | Already designed that way. |
| Firmware update | Cyphal file service into the second flash bank (G4: bank 2, `docs/settings_plan.md`); USB DFU as recovery. | Update all nodes from the VDC or a PC without opening the cart. |

## 5. Hardware Plan

### 5.1 G4 BCC (existing)

- Development target for the hot path (milestones M0 to M8 in
  `docs/hotpath_plan.md`) and for Cyphal/CAN-FD.
- Board change: CAN-FD transceiver instead of the SN65HVD230 (open item in
  `TODO.md`).

### 5.2 New 4850 logic board (STM32H7 + SPE)

Requirements for the redesign of [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic):

- **MCU:** an STM32H7 with Ethernet MAC, FDCAN, an advanced timer (TIM1 or
  TIM8) and at least two ADCs with simultaneous injected sampling for three
  phase currents and the DC link (candidates: STM32H723/H725/H730/H735;
  check ADC count, package and pins against the current interfaces).
- **SPE:** 10BASE-T1S PHY on RMII (for example LAN8670/LAN8671/LAN8672) or a
  MAC-PHY over SPI (LAN8650/LAN8651). Check the Zephyr 4.4 driver support
  (OA TC6 / LAN865x, PLCA configuration) before choosing.
- **Keep:** CAN FD transceiver (TCAN334 footprint, optional population) for
  bench tests with the BCC tools, USB for DFU, SWD, Hall and temperature
  inputs, the power board connector and the supply topology.
- **Add for the steering variant:** absolute angle sensor input (SPI or I2C,
  for example the AS5048 already supported in the legacy firmware) and,
  if the assist needs it, a steering torque sensor input (section 12).
- **Keep the hot-path assumptions of the G4** (sample per half period,
  control cycle every fourth sample, duty DMA) or document where the H7
  differs; the platform layer is a new `hardware/zephyr_h7/` next to
  `hardware/zephyr_bcc/`.

### 5.3 VDC and pedal unit

- **VDC:** reuse the H7 + SPE logic design without the power stage, plus
  connectors for IMU (SPI), GNSS (UART) and display. One design and one
  board support package for drive and VDC.
- **Pedal unit:** small MCU with SPE (MAC-PHY over SPI allows an MCU
  without an Ethernet MAC), two independent sensor channels per pedal
  (plausibility check as in automotive pedal modules).

## 6. Network

### 6.1 SPE variant

| Variant | Topology | Fit |
|---|---|---|
| **10BASE-T1S with PLCA** | Multidrop bus (8 or more nodes, about 25 m), no switch | **Preferred**: wiring like CAN, bounded access latency through PLCA, 10 Mbit/s shared |
| 100BASE-T1 | Point to point | Needs an automotive switch in the VDC; for 1 kHz command rates |
| 10BASE-T1L | Point to point, long reach | Range not needed in a cart |

### 6.2 Messages (first draft)

Subject and register IDs are not assigned yet; they are fixed in a DSDL
namespace in this repository before any firmware uses them.

| Message | From → to | Rate | Content |
|---|---|---|---|
| Wheel and steering command | VDC → drives, steering | 250 to 500 Hz | Torque per wheel (one message for all wheels, applied on reception), steering assist torque, mode |
| Drive status | each drive → VDC | same | Wheel speed, torque estimate, DC link, temperatures, fault flags |
| Steering status | steering → VDC | same | Steering angle, assist torque, fault flags |
| Pedal state | pedal unit → VDC and drives | same | Drive and brake pedal, both channels, plausibility flag |
| Heartbeat | all | 1 Hz (standard) | Health, mode |
| Registers, GetInfo, file update | VDC/PC ↔ all | on demand | Settings, version, firmware update |
| Diagnostics | all | event | `uavcan.diagnostic.Record` instead of the USB log |

### 6.3 Bandwidth estimate (to verify by measurement)

One Cyphal/UDP frame with a short payload is about 100 bytes on the wire
(Ethernet, IP, UDP and Cyphal headers, preamble and inter-frame gap), about
85 µs at 10 Mbit/s. Seven frames per cycle (one command, four drive status,
steering status, pedal):

| Cycle rate | 10BASE-T1S load | Classic CAN 1 Mbit/s | CAN FD 1/5 Mbit/s |
|---|---|---|---|
| 250 Hz | about 15 % | about 35 % | low |
| 500 Hz | about 30 % | about 70 % (too high) | about 20 % |
| 1 kHz | about 60 % (tight) | not possible | about 40 % |

250 to 500 Hz is enough for torque vectoring and drift assist; 1 kHz would
need 100BASE-T1.

## 7. Fault Reactions

The pedal unit only publishes measurements. Each drive decides on its own:

| Situation | Reaction |
|---|---|
| VDC alive | Drives follow the VDC torque; steering follows the VDC assist command. |
| VDC times out (for example three missed cycles) | Drives: limp-home mode, equal torque from the pedals, reduced limit, no torque vectoring, ramped. Steering: assist off (passive), mechanical steering remains. |
| VDC comes back | Back to normal only at standstill. |
| Pedal unit lost, VDC alive | VDC ramps the drive torque to zero and brakes gently. |
| VDC and pedal unit lost | Drives ramp the torque to zero. |
| One drive lost | VDC removes the opposite wheel of the axle too (or redistributes the torque without a yaw moment), limits the speed, shows the fault. |
| Steering lost | VDC switches off torque vectoring and drift assist. |
| E-stop | Hard-wired to the external enable and break input of every power stage; independent of the bus. |

The fallback logic runs in a Zephyr thread of the drive, never in the hot
path (`AGENTS.md`, zero-latency rules).

## 8. Software Architecture

| Part | Where | Content |
|---|---|---|
| FOC core | `lib/motor_control`, `src/current_control` | Unchanged, shared by all UNIMOC nodes |
| Drive application | `app/`, a "wheel drive" role | Torque mode, wheel speed, limp-home mode, Cyphal node |
| Steering application | `app/`, a "steering" role (Kconfig or settings) | Assist torque, angle sensor, passive on fault |
| Platforms | `hardware/zephyr_bcc`, `hardware/zephyr_sim`, new `hardware/zephyr_h7` | Hot path per MCU, the rest shared through `hardware/zephyr_common` |
| Cyphal layer | `lib/cyphal` | `Transport` with a CAN-FD (libcanard) and a UDP (libudpard) implementation, register service on `SettingsStore`, heartbeat, GetInfo, file update |
| VDC firmware | open (section 12) | Torque distribution, vehicle state estimation, display |
| Simulation | `native_sim` | Single-process deterministic tests (`tests/sim/`), plus multi-process runs: one `native_sim` per node on UDP loopback |
| Vehicle model | `lib/sim` (P4 in `TODO.md`) | Four wheels, tyres, steering, shared battery; needed for VDC tests |
| Tools | Python, pycyphal | Register access, telemetry, scenarios, later the GUI (`docs/software_in_loop_plan.md`, P3) |

## 9. Porting from the `electric_quad` Firmware

| Legacy ([`src/`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/src)) | In this plan |
|---|---|
| Motor and battery parameters in [`settings.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/settings.cpp) (for example Rs 0.15 Ω, L 0.22/0.25 mH, ψ 1.9 mWb, 7 pole pairs, 40 A, 32 kHz PWM, 1000 ns dead time; to check against the actual quad motor) | Starting point for the 4850 settings profile and the sim motor |
| "High power mode" in [`management.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/management.cpp) (80 A, unlocked by a switch sequence at power-up) | A settings register or profile selected by the VDC, within the hardware capabilities |
| Throttle mapping and dead zones (`SetThrottleSetpoint`) | Pedal unit and VDC (pedal map); limp-home mode in the drive |
| PAS ([`pas.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/pas.cpp)) | Not needed in the cart |
| Hall, flux and HFI observers, mechanical observer ([`observer.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/observer.cpp)) | Already in `lib/motor_control`; Hall fusion is milestones M5/M6, HFI later for standstill |
| AS5048 encoder ([`as5048b.cpp`](https://github.com/Tecnologic/unimoc_old_software/blob/electric_quad/src/as5048b.cpp)) | Steering angle sensor driver in the steering role |
| FreeMASTER over USB | Cyphal registers and cymon traces |
| EEPROM settings (24LC04) | Zephyr settings on ZMS (`docs/settings_plan.md`) |
| Board pinning ([`logic/pinning`](https://github.com/Tecnologic/hardware_4850/tree/master/logic/pinning), [`hardware/4850`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/hardware/4850)) | Reference for the H7 pinout and the devicetree of the new board |

## 10. Roadmap

Each phase keeps `check_format.sh` and `twister_sca.sh` green.

| Phase | Content | Hardware | Exit criterion |
|---|---|---|---|
| A | Hot path M4 to M7 (`docs/hotpath_plan.md`), sim made interactive (motor parameters, setpoints, telemetry) | none | Sim tests: speed step, Hall failure |
| B | Cyphal v1.0 spike on `native_sim` over UDP with pycyphal; DSDL namespace; register service, heartbeat, GetInfo | none | One register exchange and one telemetry subject with pycyphal |
| C | G4 bring-up: M0 and M8 on the BCC, CAN-FD transceiver, Cyphal/CAN-FD | BCC | Closed current loop with the 50 A limit; registers over CAN FD |
| D | 4850 H7 + SPE logic board: schematic, layout, `hardware/zephyr_h7`, Cyphal/UDP on 10BASE-T1S | new logic board, quad motor | Same tests as C on the 4850, quad drives on the bench |
| E | Steering role: angle sensor, assist torque, passive on fault | second 4850 or smaller stage | Assist and angle on the bench |
| F | VDC and pedal unit: firmware, multi-node sim with the vehicle model, fault reactions of section 7 in the sim | VDC board, pedal unit | All fault cases of section 7 pass in the sim |
| G | Cart integration: two wheels first, then four; torque vectoring and drift assist | cart | Driving tests with the fault cases on a closed area |
| H | Tools and GUI over Cyphal/UDP (`docs/software_in_loop_plan.md`, P3) | PC | Parameter set and telemetry plots from the GUI |

Phases A and B run in parallel without hardware; D can start (schematic)
while C runs.

## 11. Safety Outline

Not a safety concept; a list of what one must cover.

- E-stop and power-stage cutoff hard-wired, never only over the bus.
- Brakes: a brake that works without the bus and without the power stages
  (open, section 12).
- Steering stays mechanical; the assist fails passive.
- Setpoint timeouts in every receiver; ramps instead of steps on mode changes.
- Pedals with two channels and a plausibility check.
- Check the applicable standard for a manned light vehicle (for example
  EN 17128 or ISO 13849) before the cart phase G.

## 12. Open Questions

1. **Brakes:** is there a hydraulic or mechanical brake, or only regenerative
   braking through the drives?
2. **Steering torque sensor:** does the assist need a torsion-bar torque
   sensor, or is it derived from the angle and the motor current?
3. **VDC firmware location:** this repository (a second Zephyr application)
   or a separate repository that shares `lib/` and the DSDL namespace?
4. **SPE variant and parts:** 10BASE-T1S PHY or MAC-PHY; Zephyr driver
   support; PLCA node count and cable length in the cart.
5. **H7 part:** exact type and package for the 4850 logic outline.
6. **Display:** connection to the VDC (UART, CAN, SPI) and content.
7. **Battery:** BMS and its interface (VDC input or bus node).
8. **Cycle rate:** 250 or 500 Hz for the VDC loop, measured against the bus load.
9. **Cyphal/UDP versions:** libudpard 2.x implements Cyphal/UDP v1.0; the
   v1.1 stack only claims wire compatibility for CAN. Pin the versions and
   decide on v1.1 after its release.

## 13. Risks

- **Cyphal library churn:** every library is moving to v1.1 at the same time.
  Mitigation: pinned versions, the application behind `lib/cyphal`'s
  `Transport` and a thin own protocol layer.
- **SPE driver maturity in Zephyr:** to check before the board layout.
- **H7 hot path:** a new platform layer; the G4 milestones M0/M8 are the
  reference measurements.
- **Bus load at higher cycle rates:** measured in phase D; fallback 100BASE-T1.
- **Vehicle model effort:** the VDC cannot be tested in the sim without it
  (phase F).
