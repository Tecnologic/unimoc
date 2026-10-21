# System Plan: Manned Cart with Four Wheel Drives

> **Status (2026-10-21):** draft to sort the overall direction. Nothing in
> this file is built or verified on hardware. Where it changes an earlier
> decision (Cyphal/CAN-FD only, shell and log over USB), the earlier plans
> (`docs/zephyr_port_plan.md`, `TODO.md`) are updated once this draft is
> accepted. Decided so far: a mechanical emergency brake, otherwise pure
> regenerative braking; the steering servo is out of scope for now, the
> first rides use a steering angle sensor only.

## 1. Goal

A manned cart with up to four direct wheel drives, torque vectoring and
drift assist, and later an electric steering assist. UNIMOC supplies the
wheel drives (and later the steering servo); a central vehicle dynamics
controller (VDC) coordinates them. Standard modules are reused where they
connect to the VDC directly (IMU, GNSS, steering angle sensor, display).

## 2. Repositories

| Repository | Role in this plan |
|---|---|
| [Tecnologic/unimoc](https://github.com/Tecnologic/unimoc) (this repository, branch `develop`) | Drive firmware on Zephyr: FOC hot path, observers, settings, `native_sim` simulation, tests. Possibly also the VDC firmware (section 5.3). |
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
   IMU (SPI)   GNSS (UART)   Steering angle sensor   Display
        \          |                 |                 /
         +---------+-----------------+----------------+
         |        Vehicle dynamics controller (VDC)    |   central, platform open (5.3)
         +----------------------+----------------------+
                                |
     ================= Single Pair Ethernet (Cyphal/UDP) =================
        |            |            |            |             |
   Drive FL      Drive FR     Drive RL     Drive RR      Pedal unit
   UNIMOC        UNIMOC       UNIMOC       UNIMOC        (drive + brake pedal,
   H7 + 4850     H7 + 4850    H7 + 4850    H7 + 4850     2 channels each)

   (later: steering assist as another UNIMOC node)

   Hard-wired E-stop loop ──► external enable / break input of every power stage
   Mechanical emergency brake ──► independent of bus and electronics
```

| Node | Hardware | Function |
|---|---|---|
| VDC | Open (section 5.3) | Driver inputs, vehicle state estimation (IMU, GNSS, wheel speeds, steering angle), torque distribution, torque vectoring, drift assist, display, logging |
| Wheel drives (2 or 4) | 4850 power stage + new H7 logic | Torque control per wheel (drive and regenerative braking), wheel speed, limp-home mode (section 7) |
| Pedal unit | Small MCU with SPE | Publishes drive and brake pedal positions (two sensor channels per pedal); sends no commands |
| Steering angle sensor | Standard sensor on the VDC (for example the AS5048 of the legacy firmware) | Steering angle for the vehicle dynamics; no bus node |
| IMU, GNSS, display | Standard modules on the VDC | No bus node |
| Steering assist | Later: UNIMOC | Out of scope until a steering concept exists |
| G4 BCC (`boards/unimoc/bcc`) | Existing STM32G473 board | Development and bring-up platform with Cyphal/CAN-FD; not part of the cart |

## 4. Decisions

| Topic | Decision | Reason |
|---|---|---|
| Control structure | One central VDC. If it fails, the drives fall back to the pedal unit (section 7). | The vehicle dynamics need all sensors in one place; a VDC failure must still let the driver brake. |
| Brakes | Mechanical emergency brake; the service brake is regenerative only, through the drives. | Decided; consequences in section 7.1. |
| Steering | Mechanical steering. First rides: steering angle sensor on the VDC only. A steering assist (UNIMOC) follows later with its own plan. | No complete steering concept yet. |
| Protocol | **Cyphal v1.0**, libraries pinned in `west.yml`: libcanard 4.x (CAN FD), libudpard 2.x (UDP), Nunavut for DSDL, pycyphal for tools and tests. | All bus nodes are own nodes; publish/subscribe like DDS without a broker; settings already use register names (`unimoc.motor.pole_pairs`). Cyphal v1.1 (cy, libudpard 3, libcanard 5) is alpha; revisit after its release. |
| Cart transport | **Cyphal/UDP over Single Pair Ethernet**, preferably 10BASE-T1S with PLCA (section 6). | More bandwidth than CAN FD, bus wiring like CAN, the same transport as the simulation. Zephyr support checked in section 6.4. |
| G4 transport | Cyphal/CAN-FD; the SN65HVD230 on the BCC is replaced by a CAN-FD transceiver. | The BCC is the hardware at hand; the application layer is the same as on UDP. |
| Cyphal/serial | Not used. | No maintained C implementation (libserard). |
| USB | Firmware update (DFU) only on the cart. The CDC-ACM log stays until Cyphal runs on the board, then logs go out as `uavcan.diagnostic.Record`. | One operating interface; USB stays the recovery path. |
| Settings | Cyphal registers on the existing `SettingsStore` (`docs/settings_plan.md`). | Already designed that way. |
| Firmware update | Cyphal file service into the second flash bank (G4: bank 2, `docs/settings_plan.md`); USB DFU as recovery. | Update all nodes from the VDC or a PC without opening the cart. |
| VDC firmware | **In this repository**, as a second Zephyr application (`apps/vdc/`) with a portable `lib/vehicle/`. The MCU is a Zephyr-supported STM32 (H7 or G4, section 5.3); a Raspberry Pi or an ESP32-P4 are out, they do not run this repository's Zephyr build. | One toolchain, one CI, shared libraries and DSDL, the whole cart in the simulation. |

## 5. Hardware Plan

### 5.1 G4 BCC (existing)

- Development target for the hot path (milestones M0 to M8 in
  `docs/hotpath_plan.md`) and for Cyphal/CAN-FD.
- Board change: CAN-FD transceiver instead of the SN65HVD230 (open item in
  `TODO.md`).
- Optional SPE test: a LAN8651 MAC-PHY on SPI works on an STM32G4 in Zephyr
  (in-tree example `samples/net/zperf/boards/nucleo_g474re.overlay`), so the
  BCC could join an SPE bench network through an SPI adapter board.

### 5.2 New 4850 logic board (STM32H7 + SPE)

Requirements for the redesign of [`logic/`](https://github.com/Tecnologic/hardware_4850/tree/master/logic):

- **MCU:** an STM32H7 with Ethernet MAC, FDCAN, an advanced timer (TIM1 or
  TIM8) and at least two ADCs with simultaneous injected sampling for three
  phase currents and the DC link (candidates: STM32H723/H725/H730/H735;
  check ADC count, package and pins against the current interfaces).
- **SPE:** a LAN8670/LAN8671 10BASE-T1S PHY on the RMII of the H7, or a
  LAN8650/LAN8651 MAC-PHY on SPI. Both are supported by Zephyr 4.4 with
  limitations (section 6.4). The RMII PHY is the preferred choice on the H7:
  it uses the MAC and its DMA and keeps the SPI traffic off the CPU.
- **Keep:** CAN FD transceiver (TCAN334 footprint, optional population) for
  bench tests with the BCC tools, USB for DFU, SWD, Hall and temperature
  inputs, the power board connector and the supply topology.
- **Prepare for the later steering variant:** an SPI or I2C connector for an
  absolute angle sensor.
- **Keep the hot-path assumptions of the G4** (sample per half period,
  control cycle every fourth sample, duty DMA) or document where the H7
  differs; the platform layer is a new `hardware/zephyr_h7/` next to
  `hardware/zephyr_bcc/`.

### 5.3 VDC platform

The VDC has no zero-latency hot path: a 250 to 500 Hz control loop, sensor
drivers, the display and logging. Any of the candidates can run it; they
differ in boot time, determinism and how much of this repository they share.

| | STM32H7 with Zephyr | Raspberry Pi (Linux) | ESP32-P4 |
|---|---|---|---|
| Same hardware as the drives | Yes: the H7 + SPE logic design without power stage | No | No |
| OS and build | Zephyr, this repository's toolchain and twister | Linux (PREEMPT_RT for the loop), CMake or Python | Not supported by Zephyr 4.4 (no `esp32p4` SoC, the Zephyr Ethernet driver covers only the original ESP32); ESP-IDF instead |
| SPE | RMII PHY (LAN867x) or MAC-PHY (LAN865x), Zephyr drivers in tree | MAC-PHY (LAN865x) on SPI with the Linux OA TC6 driver (to verify on the Pi kernel) | ESP-IDF drivers (to verify) |
| Boot time, determinism | Under a second, deterministic | Tens of seconds, jitter, SD card on power loss | Fast, deterministic |
| Display, logging, GUI | Small display (SPI or LTDC), logging to flash or SD | HDMI/DSI display, large logs, Python | MIPI-DSI display |
| SIL | The same binary on `native_sim` | Runs natively on the host | Separate simulation |
| Impact on this repository | See below | Shares `lib/` and the DSDL through a separate repository | Separate repository |

**Recommendation:** the VDC on the STM32H7 with Zephyr, using the same logic
board as the drives: one board design, one toolchain, deterministic, fast
boot, and the whole cart runs in the simulation. If a large display, long
logs or a GUI in the cart are wanted later, add a Raspberry Pi as a
non-safety-relevant Cyphal/UDP node (display and logger) that the cart can
drive without.

**Impact of a Zephyr VDC on this repository:**

- `app/` becomes `apps/drive/` and `apps/vdc/` (two Zephyr applications,
  decided). The VDC shares `lib/units`, `lib/settings`,
  `lib/cyphal`, the DSDL namespace, `hardware/zephyr_common` and CI.
- New portable library `lib/vehicle/` (torque distribution, yaw-rate
  control, vehicle state estimation) with ztest suites in `tests/lib/vehicle/`.
- New board definition for the VDC (or a devicetree variant of the H7
  logic board) and a VDC platform layer.
- `lib/sim` needs the vehicle model (four wheels, tyres, steering angle,
  shared battery); P4 item in `TODO.md`.
- `AGENTS.md`, `README.md` and `docs/zephyr_workflow.md` describe one
  application today; they need updating.
- More twister scenarios and CI time.

A Linux VDC would not fit this repository: `AGENTS.md` excludes a hosted
build. It would live in its own repository and share the portable `lib/`
code and the DSDL as a submodule.

### 5.4 Pedal unit

- Small MCU with SPE (a LAN8651 MAC-PHY on SPI also works without an
  Ethernet MAC), two independent sensor channels per pedal (plausibility
  check as in automotive pedal modules).
- Could be an STM32G4 with Zephyr in this repository (a third, small
  application), or a role of the VDC board placed at the pedals.

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
| Wheel command | VDC → drives | 250 to 500 Hz | Torque per wheel, drive or brake (one message for all wheels, applied on reception), mode |
| Drive status | each drive → VDC | same | Wheel speed, torque estimate, DC link, temperatures, regen limit, fault flags |
| Pedal state | pedal unit → VDC and drives | same | Drive and brake pedal, both channels, plausibility flag |
| Heartbeat | all | 1 Hz (standard) | Health, mode |
| Registers, GetInfo, file update | VDC/PC ↔ all | on demand | Settings, version, firmware update |
| Diagnostics | all | event | `uavcan.diagnostic.Record` instead of the USB log |

### 6.3 Bandwidth estimate (to verify by measurement)

One Cyphal/UDP frame with a short payload is about 100 bytes on the wire
(Ethernet, IP, UDP and Cyphal headers, preamble and inter-frame gap), about
85 µs at 10 Mbit/s. Six frames per cycle (one command, four drive status,
pedal; the later steering adds one):

| Cycle rate | 10BASE-T1S load | Classic CAN 1 Mbit/s | CAN FD 1/5 Mbit/s |
|---|---|---|---|
| 250 Hz | about 15 % | about 30 % | low |
| 500 Hz | about 25 % | about 60 % (too high) | about 20 % |
| 1 kHz | about 50 % (tight) | not possible | about 35 % |

250 to 500 Hz is enough for torque vectoring and drift assist; 1 kHz would
need 100BASE-T1.

### 6.4 Zephyr support for SPE (checked in the v4.4.0 tree)

Checked in the Zephyr v4.4.0 source tree (the version pinned in `west.yml`);
not tested on hardware.

| Part | Zephyr 4.4 driver | Notes |
|---|---|---|
| LAN8650/LAN8651 (10BASE-T1S MAC-PHY, SPI) | `drivers/ethernet/eth_lan865x.c` with the OPEN Alliance TC6 layer `oa_tc6.c`, binding `microchip,lan865x` | Silicon Rev.B0/B1 only (the driver rejects other revisions). Accepts all multicast frames (hash table all ones), so every node receives all Cyphal/UDP traffic and filters in software. RX and TX run in a driver thread over SPI (CPU and SPI load). Capabilities: 10BASE and promiscuous mode only. |
| LAN8670/LAN8671/LAN8672 (10BASE-T1S PHY, MII/RMII) | `drivers/ethernet/phy/phy_microchip_t1s.c`, binding `microchip,t1s-phy` | Rev.C1/C2/D0. Used with a MAC such as the STM32 Ethernet MAC (`phy-handle`). No in-tree STM32 board uses it; the H7 + LAN867x combination is unproven in tree. |
| PLCA | `phy_oa_tc14_plca.c`, devicetree properties `plca-enable`, `plca-node-id`, `plca-node-count`, `plca-burst-count`, `plca-burst-timer`, `plca-to-timer`; runtime change through `ETHERNET_CONFIG_TYPE_T1S_PARAM` | The node ID can be set from the devicetree or at runtime (for example from a settings register). |
| NXP 10BASE-T1S PHY | `phy_nxp_t1s.c`, binding `nxp,t1s-phy` | Alternative PHY vendor. |
| ADIN1110 / ADIN2111 (10BASE-T1L, SPI) | `eth_adin2111.c`, `phy_adin2111.c` | Point to point only; not preferred. |
| TJA1103 / TJA11xx (100BASE-T1) | `phy_tja1103.c`, `phy_tja11xx.c` | Used on NXP boards (`mr_canhubk3`, `vmu_rt1170`); the fallback for 1 kHz. |
| STM32H7 Ethernet MAC | `eth_stm32_hal_v2.c`, multicast hash filter (`CONFIG_ETH_STM32_MULTICAST_FILTER`) | Mature driver; filters Cyphal multicast groups in hardware. |

Known issues reported outside the tree (to recheck against 4.4): the
LAN865x driver did not work with IPv6 (Cyphal/UDP uses IPv4 multicast, so
not a blocker), crashes on ESP32-C3/S3 with the LAN865x, and LAN8651 TX
lock-ups that an external SDK recovers from.

**Simulation transport.** The `native_sim` offloaded sockets (NSOS) in 4.4
pass no multicast socket options (`IP_ADD_MEMBERSHIP` and the like) to the
host, so Cyphal/UDP multicast does not work over NSOS. Use the TAP driver
(`eth_native_tap`) with Zephyr's own IP stack instead: it needs a TAP
interface on the host (root or `CAP_NET_ADMIN` once, Zephyr's `net-setup.sh`),
which also has to be checked on WSL2. This replaces "offloaded sockets over
loopback, no root required" in `docs/zephyr_port_plan.md`, Phase 6.

**Conclusion:** Zephyr is ready enough for a 10BASE-T1S bench setup today
(LAN8651 on SPI has an in-tree STM32G4 example). For the cart, the
H7 + LAN867x on RMII is the better hardware choice but has no in-tree
reference board, so a bring-up spike on an evaluation board (for example
a NUCLEO-H7 with an EVB-LAN8670-RMII) comes before the logic board layout.

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
| Steering angle sensor lost | VDC switches off torque vectoring and drift assist, equal torque distribution. |
| E-stop | Hard-wired to the external enable and break input of every power stage; independent of the bus. The mechanical brake stops the cart. |

The fallback logic runs in a Zephyr thread of the drive, never in the hot
path (`AGENTS.md`, zero-latency rules).

### 7.1 Consequences of pure regenerative braking

- **Bus or VDC failure:** the brake pedal must keep working through the
  limp-home path (pedal unit → drives directly). That path becomes a safety
  function, not a comfort feature.
- **Full battery or opened BMS:** the battery cannot take the braking
  current (the BCC profile allows only 20 A charge). The drives must limit
  the regenerative torque to what the DC link can take, and report the
  limit to the VDC; the mechanical brake covers the rest. Check whether a
  brake chopper with a resistor is needed.
- **DC-link overvoltage:** with the battery disconnected during braking the
  DC link rises fast; the BSC040N08 are 80 V parts on a 48 V system. The
  drive needs an overvoltage limit in the current controller and a hard
  trip, verified on the bench.
- **Low speed and standstill:** regeneration fades near zero speed and
  cannot hold the cart; holding torque at standstill costs battery energy.
  Parking needs the mechanical brake (or a parking brake).
- **Power stage off (fault, E-stop):** no electric braking at all; the
  mechanical emergency brake must stop the cart from top speed on its own.

## 8. Software Architecture

| Part | Where | Content |
|---|---|---|
| FOC core | `lib/motor_control`, `src/current_control` | Unchanged, shared by all UNIMOC nodes |
| Drive application | `app/` (later `apps/drive/`) | Torque mode with regenerative braking, wheel speed, DC-link limits, limp-home mode, Cyphal node |
| Platforms | `hardware/zephyr_bcc`, `hardware/zephyr_sim`, new `hardware/zephyr_h7` | Hot path per MCU, the rest shared through `hardware/zephyr_common` |
| Cyphal layer | `lib/cyphal` | `Transport` with a CAN-FD (libcanard) and a UDP (libudpard) implementation, register service on `SettingsStore`, heartbeat, GetInfo, file update |
| VDC firmware | Section 5.3 (recommended: `apps/vdc/` with `lib/vehicle/`) | Torque distribution, vehicle state estimation, display |
| Simulation | `native_sim` | Single-process deterministic tests (`tests/sim/`), plus multi-process runs: one `native_sim` per node on a TAP network (section 6.4) |
| Vehicle model | `lib/sim` (P4 in `TODO.md`) | Four wheels, tyres, steering angle, shared battery; needed for VDC tests |
| Tools | Python, pycyphal | Register access, telemetry, scenarios, later the GUI (`docs/software_in_loop_plan.md`, P3) |
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
| Board pinning ([`logic/pinning`](https://github.com/Tecnologic/hardware_4850/tree/master/logic/pinning), [`hardware/4850`](https://github.com/Tecnologic/unimoc_old_software/tree/electric_quad/hardware/4850)) | Reference for the H7 pinout and the devicetree of the new board |

## 10. Roadmap

Each phase keeps `check_format.sh` and `twister_sca.sh` green.

| Phase | Content | Hardware | Exit criterion |
|---|---|---|---|
| A | Hot path M4 to M7 (`docs/hotpath_plan.md`), sim made interactive (motor parameters, setpoints, telemetry) | none | Sim tests: speed step, Hall failure |
| B | Cyphal v1.0 spike on `native_sim` over UDP (TAP) with pycyphal; DSDL namespace; register service, heartbeat, GetInfo | none | One register exchange and one telemetry subject with pycyphal |
| C | G4 bring-up: M0 and M8 on the BCC, CAN-FD transceiver, Cyphal/CAN-FD | BCC | Closed current loop with the 50 A limit; registers over CAN FD |
| C2 | SPE spike: Cyphal/UDP on 10BASE-T1S between an STM32H7 evaluation board with a LAN867x on RMII and a LAN8651 node, PLCA, multicast, bus load | evaluation boards | Measured latency and load at 500 Hz with six nodes |
| D | 4850 H7 + SPE logic board: schematic, layout, `hardware/zephyr_h7`, regenerative braking with DC-link limits | new logic board, quad motor | Same tests as C on the 4850, braking into a full battery and with the battery disconnected on the bench |
| E | VDC platform decision (section 5.3), pedal unit, steering angle sensor | VDC, pedal unit | Platform chosen, pedal and angle published on the bus |
| F | VDC firmware, multi-node sim with the vehicle model, fault reactions of section 7 in the sim | none, then VDC | All fault cases of section 7 pass in the sim |
| G | Cart integration: two wheels first, then four; torque vectoring and drift assist | cart | Driving tests with the fault cases on a closed area |
| H | Tools and GUI over Cyphal/UDP (`docs/software_in_loop_plan.md`, P3) | PC | Parameter set and telemetry plots from the GUI |
| later | Steering assist as a UNIMOC node | steering hardware | Own plan |

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

1. **VDC MCU:** STM32H7 or STM32G4 with Zephyr (the firmware lives in this
   repository, decided).
2. **SPE parts:** LAN867x on RMII (preferred) or LAN865x on SPI; PLCA node
   count and cable length in the cart; result of the spike C2.
3. **H7 part:** exact type and package for the 4850 logic outline.
4. **Brake chopper:** needed for braking with a full or disconnected battery?
5. **Display:** connection to the VDC and content.
6. **Battery:** BMS and its interface (VDC input or bus node); charge current
   limit during regenerative braking.
7. **Cycle rate:** 250 or 500 Hz for the VDC loop, measured against the bus load.
8. **Cyphal/UDP versions:** libudpard 2.x implements Cyphal/UDP v1.0; the
   v1.1 stack only claims wire compatibility for CAN. Pin the versions and
   decide on v1.1 after its release.
9. **Steering assist:** concept, torque sensor, interface (later).

## 13. Risks

- **Cyphal library churn:** every library is moving to v1.1 at the same time.
  Mitigation: pinned versions, the application behind `lib/cyphal`'s
  `Transport` and a thin own protocol layer.
- **SPE in Zephyr:** working drivers, but young; no in-tree STM32 board with
  a T1S PHY on RMII; the LAN865x driver accepts all multicast frames. The
  spike C2 settles it before the board layout.
- **Simulation network:** Cyphal/UDP multicast needs the TAP driver on
  `native_sim` (root once); WSL2 support to check.
- **Pure regenerative braking:** the service brake depends on the drives,
  the battery state and the bus; covered by the limp-home path and the
  mechanical emergency brake (section 7.1).
- **H7 hot path:** a new platform layer; the G4 milestones M0/M8 are the
  reference measurements.
- **Bus load at higher cycle rates:** measured in spike C2; fallback 100BASE-T1.
- **Vehicle model effort:** the VDC cannot be tested in the sim without it
  (phase F).
