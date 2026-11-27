# Zuse: VDC Board and the Cart Periphery

> **Status (2026-11-26, updated 2026-11-27):** draft. Nothing is drawn or built. Zuse is the
> vehicle dynamics controller (VDC) board of the cart (`docs/system_plan.md`,
> sections 3, 5.0 and 5.3). Its firmware is `apps/vdc/`, today on the
> NUCLEO-H743ZI and `native_sim`. This file also collects what was decided
> on the same day for the nodes around it: the display (Gutenberg), the
> pedal unit (Drais) and the 12 V supply.

## 0. Decisions of 2026-11-26

| Topic | Decision | Section |
|---|---|---|
| Battery | **24 V** pack for now; Zuse and the 12 V supply are laid out for **48 V** nominal. A 48 V Jacobi (80 to 100 V FETs) comes later. | 3 |
| 12 V supply | A **12 V auxiliary net** from a bought **XL7015 buck module**, separate (not on Volta): **no MCU, no bus node**; replaced by a stronger module if needed. Jacobi keeps its own supply from the DC link. | 3 |
| Display | Panlee **WT32-SC01 Plus** (3.5", ESP32-S3) on **RS-485** to Zuse, a **lean own Cyphal/serial transport**, firmware on **Zephyr**. | 4 |
| Pedal unit | Drais, an own Cyphal/CAN-FD node on an **STM32G473CBT6** (128 KB), firmware update over Cyphal (RAM staging proposed, section 6.2), USB DFU as recovery. Two three-wire Hall pedals (1 to 4.6 V), one channel each, range-checked. Limp-home on the pedals stays. | 6 |
| Steering angle sensor | AS5048A on a small passive board at the steering column, cable to Zuse; **not** a bus node, not on the Zuse board. | 5 |
| E-stop loop | Zuse **senses** it with the optocoupler input of Jacobi; only the E-stop buttons open it, **Zuse has no contact in the loop**. | 2.1 |
| Lights | **Switched by Zuse**: Infineon BTS high-side switches (PROFET) with current sense; the part is chosen with the schematic. | 2.2 |
| IMU | **TDK ICM-42688-P**. | 2.4 |
| Connectors | **JST PH** (2.0 mm) for signals as on Jacobi, **Molex Micro-Fit 3.0** for the supply and the lights. | 2.3 |
| Enclosure | 3D printed in **ABS / ASA**. | 2.5 |
| Names | The applications keep their names (`apps/drive/`, `apps/vdc/`); the board code names are in the documentation (`apps/*/README.md`). | 8 |
| MCU | Zuse gets the **STM32H743VIT6** of Jacobi (one part, one platform). | 2 |
| Boards (2026-11-27) | Zuse is a **stack of two boards**: the logic board and a light board with the high-side switches. Made at **JLCPCB**, components on **one side, 0603** and larger; the boards are sized for that. | 2, 2.2 |
| Mounting (2026-11-27) | **Zuse at the bottom, in the middle of the cart**; **Drais at the front between the pedals**. Both run from the **12 V aux net**. | 2.5, 6 |
| CAN connector (2026-11-27) | **The same connector and pinout on every node** (that of Jacobi), so one harness fits all. | 2 |
| GNSS (2026-11-27) | A **u-blox M10 drone module**, mounted on top of the cart with a cable to Zuse. | 2.6 |
| microSD (2026-11-27) | **Yes**, for logging. | 2 |
| Lamps (2026-11-27) | 12 V LED garden lamps as headlights; indicators and tail lights to buy (12 V LED); **no horn**; the indicators **blink in software**. | 2.2 |
| Controls (2026-11-27) | Light, indicators, gear (forward, neutral, reverse) and mode **on Zuse**, and **optionally from the display**. | 2.7 |

Checked on 2026-11-26 (builds, nothing run on hardware): the pedal
application fits a G4 with 128 KB, not one with 64 KB (section 6.3), and Zephyr 4.5 has every driver the
SC01 Plus needs (section 4.4). No blocker, if the pedal G4 has 128 KB.

## 1. Function

- Cyphal/CAN-FD node (node-ID 1), the VDC application `apps/vdc/`.
- Driver inputs on Zuse: key switch, engage, gear, mode, light, indicators,
  the switch of the mechanical brake (section 2.7). The pedals come over the bus
  (`unimoc.cart.PedalState` from Drais).
- Vehicle state: IMU on the board, GNSS module, steering angle, wheel speeds
  from the drive status.
- Torque distribution, later torque vectoring and drift assist
  (`lib/vehicle/`).
- Lights and indicators (on the light board of the stack).
- The display link (Gutenberg) and logging to a microSD card.
- Senses the E-stop loop; never opens, closes or bridges it.

## 2. Interfaces

| Connector | Signals | Level / protection | MCU peripheral (STM32H743VIT6) |
|---|---|---|---|
| Supply in (Micro-Fit) | +VIN, GND | 9 to 60 V (12 V aux net, or the pack directly on the bench), reverse polarity protection, TVS, fuse | ADC: VIN sense |
| CAN FD ×2 (same bus, daisy chain) | CANH, CANL, GND | Transceiver with standby pin, switchable split termination 2 × 60 Ω, ESD | FDCAN1 |
| CAN 2 (optional) | CANH, CANL, GND | Footprint only | FDCAN2 |
| E-stop sense | Loop in, loop return | Optocoupler circuit of Jacobi (section 2.1) | GPIO (EXTI) |
| Display (SC01 Plus) | RS-485 A, B, GND, +5 V | Half-duplex transceiver, switchable 120 Ω, ESD; +5 V switched and current limited (≥ 0.5 A) | USART1 with hardware DE, GPIO for the 5 V switch |
| GNSS (M10 drone module) | TX, RX, +5 V, GND; PPS if the module has it; I2C for its compass (optional) | Series resistors, ESD; switched supply | USART2, TIM input capture (PPS) |
| Steering angle sensor | +5 V, GND, PWM (baseline); SPI lines reserved | Shielded cable, RC filter, ESD; switched supply | TIM input capture; SPI2 option |
| Driver inputs | Key switch (wake), engage, gear (forward / reverse; neither: neutral), mode, light, indicator left, indicator right, mechanical brake switch, 2 spare: 11 (section 2.7) | 12 to 60 V tolerant, switched to GND | GPIO |
| Stack connector to the light board | 12 V and GND for the lamps, the enable of each output, the current sense lines | Board-to-board header | GPIO, ADC (sense) |
| microSD | SDMMC 4 bit | Card detect | SDMMC1 |
| Debug | SWD, SWO, NRST | Tag-Connect TC2050 (as Jacobi) | SWD (RTT log) |
| USB-C | USB FS | DFU recovery | OTG_FS |
| Status LEDs | RUN, ERROR, CAN, MODE | | GPIO |

The pin assignment follows in CubeMX once this list is agreed. The budget
fits the LQFP100 with room to spare: one FDCAN (two with the option), two
SPI (IMU, steering option), three U(S)ARTs (display, GNSS, spare), SDMMC1,
USB, about 10 ADC channels, two input captures and about 25 GPIOs. Check
that SDMMC1 (PC8 to PC12, PD2) does not collide with the FDCAN and USART
pins.

### 2.1 E-stop loop

The loop stays hard-wired to the break input of every power stage
(`docs/system_plan.md`, sections 3 and 7) and is opened by the E-stop
buttons only. Zuse has no contact in the loop: a fault of Zuse can never
stop the drives through the loop, and the drives keep their limp-home on
the pedal unit (section 6).

Zuse senses the loop with the optocoupler input of Jacobi (the `PWM_EN`
optocoupler on PE15, schematic in Tecnologic/unimoc_jacobi) at the end of
the loop, so it sees "loop open" no matter which button opened it, shows it
on the display and logs it. The sense input loads the loop like one more
drive.

### 2.2 Light board (second board of the stack)

The light outputs sit on their own board under or above the logic board,
connected by a board-to-board header (12 V in, the enables and the current
sense lines). The lamp currents then stay off the logic board, and a
burnt output costs the small board only. The lamp connectors (Micro-Fit)
are on the light board.

Six protected high-side outputs at 12 V (aux net):

| Output | Lamp |
|---|---|
| 1 | Headlights (12 V LED garden lamps, at hand) |
| 2 | Tail lights (12 V LED, to buy) |
| 3 | Brake lights (12 V LED, to buy; may share the tail lamp with two brightness levels by PWM) |
| 4 | Indicators left (front and rear) |
| 5 | Indicators right (front and rear) |
| 6 | Spare |

- **Switches:** Infineon BTS high-side switches (PROFET family, dual
  channel) with the current sense (IS) pin on an ADC input of Zuse. The LED
  loads draw little (a few hundred mA per output), so the smallest parts
  of the family do; their current sense is coarse at such currents, enough
  to tell a missing lamp from a working one, which the indicators need
  (a fast blink shows a broken indicator, as in a car).
- **Indicators blink in software** (about 1.5 Hz), with the hazard lights
  (both sides) on a fault, for example the E-stop loop open.
- **The brake light** follows the brake pedal and the regenerative brake
  torque, not only a pedal switch.
- No horn.

### 2.3 Connectors

- **Signals: JST PH (2.0 mm)** as on Jacobi. PH locks by friction only; in
  the vehicle the enclosure holds the plugs (a rib above them) or a drop of
  hot glue does. PH carries 2 A per contact.
- **Supply and lights: Molex Micro-Fit 3.0** (3.0 mm, latching, up to about
  5 A per contact depending on the wire and the pin count; parts at hand).
- **CAN: the same connector and pinout on every node**, those of Jacobi
  (two per board, in and out, for the daisy chain).
- **Assembly:** JLCPCB, all parts on one side, 0603 and larger.

### 2.4 IMU: TDK ICM-42688-P

Low gyro noise, an anti-alias filter in the part, SPI up to 24 MHz, known
from flight controllers; Zephyr driver `invensense,icm42688` (check against
the Zephyr version in `west.yml`). The MPU-9250 breakout at hand serves the
first bench steps (Zephyr driver `invensense,mpu9250`); it is discontinued
and not for the board. No magnetometer: the heading comes from GNSS and the
gyro.

Placement: in the middle of the board, away from mounting holes and
connectors, axes on the silkscreen, the interrupt on an EXTI or timer pin.
The firmware gets a mounting rotation register (calibrated at standstill
from gravity, the heading axis from a straight acceleration), so the board
does not need to sit level. Mount the board rigidly near the centre of
gravity of the cart.

### 2.5 Enclosure, water and heat

Printed in ABS or ASA (ASA for outdoor use: UV stable). The board stack
sits on screws in brass heat-set inserts, not on foam, so the IMU sees the
cart and not the enclosure. Zuse is mounted at the bottom in the middle of
the cart (close to the centre of gravity, good for the IMU; exposed to
spray water and dirt from the road); Drais at the front between the
pedals.

**Water:** a printed part is not watertight by itself; the water creeps
between the layers. Splash water (about IP54) is reachable, immersion is
not. What helps:

- Thick walls (four or more perimeters, 100 % infill near the sealing
  faces), printed hot and slow for good layer bonding; ASA and ABS can be
  smoothed with acetone vapour, which closes the surface, or coated (epoxy
  coating).
- A lid with a groove and a gasket (printed TPU or an O-ring cord), and
  screws close together.
- The cables leave through cable glands (PG7 / M12), the JST PH and
  Micro-Fit plugs stay inside; or sealed connectors in the wall (Deutsch
  DT, Superseal) for the harness. USB-C and the microSD inside, under the
  lid.
- A pressure-equalisation vent (a membrane vent), so that warming and
  cooling do not pump humid air in through the seals; openings at the
  bottom, not at the top.
- Conformal coating of the boards as a second line, except the connectors
  and the IMU.

**Heat:** no cooling needed. Zuse dissipates about 2 to 3 W (the H743 at
400 MHz well below 1 W, the 5 V buck and the transceivers about 1 W, the
high-side switches with LED loads almost nothing; the 12 V converter
outside). In a closed box under the cart that raises the inside by about
10 to 20 K; ABS and ASA soften only near 100 °C. Keep the 12 V converter in
its own (potted) housing outside the box.

### 2.6 GNSS: u-blox M10 drone module

A drone module (u-blox M10, often with a compass on I2C): 5 V, UART at 3.3 V
(38400 or 115200 baud, UBX protocol), PPS only on some modules. Under the
cart it would see little sky, so the module sits on top of the cart (the
body or the seat back), with a cable to Zuse (JST PH, 4 to 6 wires). The
compass of such modules is not used (motors and steel nearby).

### 2.7 Driver controls

The controls are switches to GND on the driver inputs of Zuse: key
(wake), engage, gear (forward / reverse; neither is neutral), mode, light,
indicator left, indicator right, mechanical brake switch. The display can
do the same over its Cyphal link (light, indicators, mode, and the gear),
so the switches can be fewer later. The display is a request, not a
command: Zuse takes a gear change only at standstill with the brake
pressed, as from the switch, and the switches win over the display.

### 2.8 Option: a flight controller as the logic board

Drone flight controllers with an STM32H743 have much of what Zuse needs:
for example the Matek H743 family (H743-WING, H743-SLIM; to check per
model): the same STM32H743VIT6, an ICM-42688-P, a microSD slot, many
UARTs, ADC inputs and, on some models, a CAN FD transceiver. Zephyr has no
board definition for them; one is written like Jacobi's (pins from the
vendor's ArduPilot or Betaflight target).

- **For:** a finished, tested board for the bench and the first rides,
  with a vibration-damped IMU; no logic board to design first.
- **Against:** no RS-485 to the display, no E-stop optocoupler, no
  protected 12 V inputs for the switches, small connectors (JST GH 1.25 mm)
  not made for a vehicle harness, CAN only on some models, and its supply
  range (2S to 6S or 8S) to check against the pack.

**Proposal:** a flight controller as an intermediate step instead of the
NUCLEO, with the light board extended into an I/O board (RS-485, the
E-stop sense, the input protection, the light switches) in the stack; the
own logic board later, or never if the flight controller holds up. The
firmware stays the same, only the board definition differs.

## 3. Power

- **12 V aux net** for Zuse, Drais, the display, the sensors and the
  lights. Budget (estimate, 2026-11-27, LED lamps, no horn): Zuse 2 to 3 W,
  Drais 0.5 W, display 1 W, GNSS 0.5 W, lights 10 to 20 W (headlights, tail
  and brake lights, indicators); about 15 to 25 W, so 12 V / 2 A, with
  margin 3 A. To check with the real lamps.
- **A bought XL7015 buck module, separate, not on Volta; no MCU, no bus
  node.** Zuse and the display hang on this net; a converter regulated by
  firmware would take them down on every crash, update or reset of its MCU.
  Zuse measures the 12 V itself (VIN sense) and knows the load of every
  light output.
- **XL7015 and the budget:** the XL7015 takes up to 80 V, which covers a
  48 V pack, but it is rated about 0.8 A output (XLSEMI datasheet; check the
  module's own rating): about 10 W at 12 V. That carries the electronics
  (Zuse, Drais, display, GNSS: about 5 W) but not the lights on top.
  Proposal: replace it by one potted vehicle DC/DC (24 / 48 V to 12 V,
  3 to 5 A, waterproof housing), which also solves its own cooling and
  water protection.
- Its input is a diode OR of the pack (after the key switch, fused) and the
  DC link, so it runs before the contactor closes and without a battery
  when the pedal generator holds the link (`docs/system_plan.md`, section
  7.2.3). Input range 16 to 60 V (a 100 V controller for the regenerative
  rise). It sits alone, not on the BMS board (Volta).
- **Zuse takes 9 to 60 V**, so it runs from the 12 V net or straight from
  the pack on the bench. On board: a wide-input buck to 5 V (display,
  steering sensor, GNSS), then 3.3 V.
- **Jacobi keeps its own supply from the DC link** (LM5008 / LMR12010 of
  the 4850 logic, +12P gate supply). It is not redundant with the 12 V net:
  - the pedal generator boots the drives from the DC link alone at about
    10 V (`docs/system_plan.md`, section 7.2.3);
  - the gate supply stays local to the power stage, not on a long harness
    with the switching currents of four drives;
  - a failed aux converter would otherwise stop all four drives at once.
  A 12 V aux input on a later Jacobi revision (diode OR into the logic
  supply, for configuration and updates without the DC link) is an option.
- **24 V now, 48 V later:** Jacobi has 60 V FETs (NCEP6080AG) and is rated
  36 V. A 24 V pack (7S, 29.4 V full) is within the rating. A 13S pack
  reaches 54.6 V when full, plus the rise during regenerative braking, so
  48 V needs a Jacobi power board variant with 80 to 100 V FETs and
  capacitors (`docs/system_plan.md`, open question 11). The voltage
  thresholds of section 7.2 of the system plan (33 / 30 / 38 / 42 V) are
  for a 36 V system and have to be scaled to the pack.

Display budget: 4.7 to 5.5 V, 170 to 190 mA at full backlight from the
datasheet (WT32-SC01 Plus V1.3, table 9); WLAN peaks come on top, so the
5 V output to the display is rated for at least 0.5 A.

## 4. Display (Gutenberg = WT32-SC01 Plus)

From the datasheet (V1.3): ESP32-S3 (WT32-S3-WROVER-N16R2: 16 MB flash,
2 MB PSRAM), 480 × 320 IPS, ST7796UI over 8 bit 8080, FT6336U capacitive
touch over I2C, I2S amplifier 2.5 W at 4 Ω, microSD over SPI, USB-C.
The RS-485 connector (table 6) carries A, B, GND and +5 V ("power supply
or output"); the ESP32-S3 drives it with RXD GPIO 1, TXD GPIO 42 and RTS
(driver enable) GPIO 2. Six spare 3.3 V GPIOs on the extended IO
connector. Not in the datasheet: the RS-485 transceiver part and whether
the display terminates the line.

### 4.1 Why RS-485 and not the cart CAN

The ESP32-S3 has a classic CAN controller (TWAI), no CAN FD. A classic CAN
node on the cart bus would answer every FD frame with an error frame. So
the display stays off the cart bus and talks to Zuse point to point.

### 4.2 Link protocol: lean Cyphal/serial

This replaces "Cyphal/serial: not used" of `docs/system_plan.md`, section 4
(no maintained C implementation, libserard): instead of a library, an own
lean transport in `lib/cyphal`.

- Single-frame transfers only (every UNIMOC message fits in 63 bytes
  anyway): the Cyphal/serial header with its CRC, COBS framing and the
  CRC-32C of the payload, behind the `Transport` of `lib/cyphal`.
- The display is a Cyphal node like any other: it subscribes to the
  messages Zuse sends it, publishes its inputs, and has registers, GetInfo
  and the firmware update. Zuse forwards the subjects the display needs
  between the CAN bus and the serial link (no transparent bridge).
- RS-485 is half duplex: **Zuse is the master.** It sends its frames
  cyclically (10 to 20 Hz) and the display answers only in the window after
  them (touch inputs, mode selection). Baud rate 115200 to 1 Mbit/s, fixed
  with the bench test.
- Tested with ztest on the frame codec and on `native_sim` over a pseudo
  terminal, as the slcan ports.

The display shows and asks; no safety function depends on it.

On the ESP32-S3 the RS-485 driver enable is the UART's own half-duplex
mode: the Zephyr UART driver switches RTS (GPIO 2) by itself
(`hw-rs485-hd-mode` of the ESP32 UART binding). On Zuse the STM32 USART
drives DE in hardware the same way.

### 4.3 Display firmware on Zephyr

A third Zephyr application (Gutenberg) with LVGL for the screens; it shares
`lib/cyphal`, `lib/units`, `lib/settings`, the DSDL and the release
version with the other applications. To do:

- A board definition for the SC01 Plus: the pins of section 4 (8080 bus,
  touch I2C, RS-485 UART with RTS as driver enable, microSD over SPI, I2S).
- Check against the Zephyr version in `west.yml`: the ST7796 driver on the
  8080 (MIPI DBI) bus of the ESP32-S3, the FT6336 touch (FocalTech FT5336
  driver family), the PSRAM for the LVGL frame buffers.
- The ESP32 toolchain (`xtensa-espressif_esp32s3_zephyr-elf` of the Zephyr
  SDK) and the Espressif HAL as west projects; CI builds the display too.
- The screens show what a driver needs at a glance (speed, battery, mode,
  faults of the drives, the BMS and the E-stop loop), and the settings of
  the cart behind a menu.

### 4.4 Zephyr check (2026-11-26)

Zephyr 4.5 (the revision of `west.yml`) with `hal_espressif` and `lvgl`:
the LVGL sample built for `esp32s3_devkitc/esp32s3/procpu` with a
devicetree overlay of the SC01 Plus pins (datasheet V1.3), 601 KB flash.
Built only, not run.

| Part | Zephyr | Result |
|---|---|---|
| ST7796UI panel | `sitronix,st7796s` (`CONFIG_ST7796S`) | Builds |
| 8 bit 8080 bus | `espressif,esp32-lcd-cam-mipi-dbi` on the LCD-CAM block, `MIPI_DBI_MODE_8080_BUS_8_BIT`, DMA; the pins through the GPIO matrix (`LCD_CAM_DATA_OUT*`, `LCD_CAM_LCD_CLK_GPIO47`, `LCD_CAM_DC_GPIO0`) | Builds; **no board in the Zephyr tree uses this driver yet**, so it is the bring-up risk |
| FT6336U touch | `focaltech,ft5336` (`CONFIG_INPUT_FT5336`), as the M5Stack Core2 and CoreS3 do for their FT6336U | Builds |
| 2 MB PSRAM | `psram0`, `CONFIG_ESP_SPIRAM` | Builds |
| RS-485 | ESP32 UART with `hw-rs485-hd-mode` (RTS as driver enable) | In the driver |
| LVGL | `lvgl` module | Builds |

Open: the panel's init sequence (gamma, inversion, orientation) from the
vendor's example code, GPIO 0 and 46 are boot strapping pins used by the
bus (the vendor board works with that), the workspace needs `hal_espressif`
and `lvgl` in the `west.yml` allowlist and `esptool` for the build. If the
8080 driver fails on the board, a bitbang MIPI DBI driver
(`zephyr,mipi-dbi-bitbang`) is the slow fallback.

## 5. Steering angle sensor

The steering column is neither horizontal nor vertical, so the board does
not sit on the column (the IMU would be tilted and far from the centre of
gravity). The sensor is a small passive board at the end of the column:
AS5048A (as in the legacy firmware; breakout at hand) with the diametric
magnet on the column. No MCU, not a bus node.

- **Baseline: the PWM output** of the AS5048A (12 bit, about 1 kHz) over a
  shielded three-wire cable (+5 V, GND, PWM) into a timer input capture of
  Zuse. Robust over a metre or two and enough for the 250 to 500 Hz loop.
- **Option: SPI over RS-422** line drivers on both ends, for the 14 bit
  value and the diagnostics.
- A second sensor channel for a plausibility check is optional; a lost or
  implausible sensor switches torque vectoring and drift assist off
  (`docs/system_plan.md`, section 7). Zero and end stops are calibrated in
  registers.

## 6. Pedal unit (Drais) on the STM32G473CBT6

### 6.1 Pedals

Two bought pedals (drive and brake), each a three-wire Hall sensor: +5 V,
GND and the signal, about 1 V released to 4.6 V fully pressed (user's
measurement, 2026-11-26). One channel per pedal, so the plausibility check
of two channels per pedal is not possible with them. Instead:

- **Range check:** with a pull-down at the input an open wire reads 0 V and
  a short to the supply 5 V; below about 0.5 V or above about 4.8 V the
  pedal is faulty and its travel counts as 0 (drive) or as a fault the VDC
  shows (brake). The thresholds and the released and pressed voltages are
  registers, calibrated on the real pedals.
- **Supply check:** the 5 V sensor supply (switched, current limited, one
  per pedal) is measured too; a sensor reading is only valid while its
  supply is.
- **Both pedals:** drive pedal and brake pedal pressed together counts as
  brake (the drive request goes to 0), as in the drive's limp-home.
- **ADC:** a divider from the 0 to 5 V signal to the 3.3 V ADC (for example
  10 kΩ / 20 kΩ: 4.6 V gives 3.07 V), a small RC filter, ESD.
- `unimoc.cart.PedalState` carries both channels; the second channel is NaN
  and the flags report the range check.

Limp-home on the pedals after a VDC failure stays as in section 7 of the
system plan. A second sensor per pedal (a second Hall throttle on the same
pedal, opposite slope) is the later upgrade if the single channel proves
too weak for the safety of the brake.

### 6.2 MCU and firmware update

The G4 of the former BCC is an exception to "every board is STM32H7"
(`docs/system_plan.md`, section 5.1) for small nodes **without a hot
path**: no hot-path platform (the removed `hardware/zephyr_bcc` was one),
only a board definition, Zephyr's own drivers (FDCAN, ADC) and a
`cyphal_platform.cpp`.

**The part: STM32G473CBT6** (2026-11-27): 128 KB flash, 128 KB SRAM,
LQFP48, FDCAN, USB. (A 64 KB G4 would not hold the node, section 6.3.)

**Firmware update over Cyphal (proposal 2026-11-27).** Two MCUboot slots
do not fit into 128 KB, but the update does not need them:

- **Proposal: stage the image in RAM.** The download client of
  `lib/cyphal` (`FirmwareUpdate`) writes into a `FirmwareBank`; on the G4
  that bank is a buffer in SRAM, as on `native_sim`. The header and the
  CRC32 are checked there, before anything in flash changes. `activate()`
  then runs a short routine from RAM (`__ramfunc`, interrupts off): erase
  the application pages, copy the buffer, reset. No bootloader of our own,
  the whole flash minus the settings pages for the application.
  - RAM: the image (about 60 to 70 KB) plus the running node. The node
    must shrink to about 50 KB RAM (measured 59 KB, 38 KB with a smaller
    libcanard heap and Cyphal stack, section 6.3); the buffer gets the rest.
  - Risk: a power loss during the copy (about a second) leaves no
    application. The ROM bootloader is always there, so the node is
    recovered over USB DFU; the cart is parked for an update anyway (no
    pedal state while it runs).
  - The settings pages stay outside the copied range.
- **Alternative: a single-slot Cyphal bootloader** (for example
  [Kocherga](https://github.com/Zubax/kocherga)) that survives a power loss
  during the update, at the cost of 16 to 32 KB flash; the application then
  has 88 to 104 KB, which still fits.

**USB DFU** of the STM32 ROM bootloader (BOOT0 button, or an
ExecuteCommand that jumps to it) is the first flash and the recovery in
both cases; SWD on the bench.

### 6.3 Size check (2026-11-26)

The VDC application stands in for the pedal application: both are a Cyphal
node without a hot path (heartbeat, registers, subjects, the log on the
bus); the pedal unit adds the ADC and the range check and takes out the
torque split. Built for `nucleo_g474re` (same core and FDCAN as every G4),
with the STM32H7 platform choice opened to the G4 for the measurement only
(not committed):

| Build | Flash | RAM |
|---|---|---|
| VDC on the NUCLEO-H743ZI (reference, debug log) | 70.5 KB | 59.9 KB |
| G474, debug log | 64.7 KB | 59.0 KB |
| G474, warning log, no asserts, LTO | 58.8 KB | 58.8 KB |
| G474, as before, with the settings on ZMS | 66.4 KB | 59.1 KB |
| G474, minimal: error log on the bus only, no console, no printk, nano printf, LTO, settings on ZMS | 55.9 KB | 58.8 KB |
| as minimal, libcanard heap 8 KB, Cyphal stack 4 KB, smaller log buffer | 55.9 KB | 37.8 KB |

What this means for the part:

- **128 KB or more (G473CB and up):** fits with a wide margin, also with
  the debug log.
- **64 KB:** the minimal build plus the ADC and the pedal logic comes to
  about 58 to 60 KB, and ZMS needs at least two 2 KB pages for the
  settings: about 0 to 2 KB left. **Too tight**; it would only fit without
  the settings subsystem (the pedal calibration in one own flash page) and
  with the log off, and every later feature hurts.
- **32 KB SRAM (G431):** the RAM above (38 KB even trimmed) does not fit
  without shrinking the Cyphal node further.

Decided 2026-11-27: the STM32G473CBT6 (128 KB flash, 128 KB SRAM).

## 7. Order of work

Code and schematic run side by side; the layout waits for the bench.

1. **Now, without new hardware:** the Cyphal/serial transport (section
   4.2), the display application on the SC01 Plus (a finished board,
   section 4.3), the pedal logic (range check, pedal map) in a library with
   ztests, the RAM-staged update (section 6.2) on `native_sim`.
2. **Bench on the NUCLEO-H743ZI with breakouts:** IMU (MPU-9250 first,
   ICM-42688-P later), AS5048A (PWM on a timer capture), GNSS, RS-485 to the
   SC01 Plus, CAN FD transceiver (`apps/vdc/boards/nucleo_h743zi.overlay`).
   This proves the Zephyr drivers and the pin functions before they are
   frozen in copper.
3. **Pin assignment as a devicetree:** the Zuse and Drais board
   definitions (`boards/unimoc/zuse/`, `boards/unimoc/drais/`) written from
   the CubeMX pinout before the schematic is final; twister builds them, so
   a pin conflict shows up as a build error.
4. **Schematics** of both boards (the pedal board first, it is small), in
   parallel with 2 and 3.
5. **Layout** after the bench has confirmed the interfaces and the
   enclosure sizes are known.

## 7.1 Information still needed

Answered on 2026-11-27: JLCPCB, one side 0603, 12 V for both boards, the
same CAN connector, the mounting places, microSD, GNSS, lamps, controls
(section 0). Still open:

- The enclosure designs and from them the board outlines (Zuse at the
  bottom in the middle, Drais between the pedals).
- The E-stop loop: its voltage and current (Jacobi's `PWM_EN` input needs
  12 V, `docs/system_plan.md` section 7.2.1) and who feeds it.
- The current of the real lamps (garden LEDs, the indicators and tail
  lights to buy), for the BTS parts and the 12 V converter.
- The exact M10 module (with or without PPS).
- KiCad version and the symbol and footprint library (the one of Jacobi,
  proposal); layer count (proposal: 2 layers for Drais and the light
  board, 4 for the Zuse logic board).
- Flight controller as the first Zuse logic (section 2.8): yes or no.

## 7.2 Hardware repositories

Proposal: one repository per board, like Tecnologic/unimoc_jacobi:
`unimoc_zuse` and `unimoc_drais`, each with the KiCad project, the BOM and
the production files; the symbols and footprints shared with Jacobi
(moved into a library repository used as a git submodule, or copied once).
The firmware (board definitions, devicetrees) stays in this repository.

## 8. Applications and boards

The applications keep their names; the board code names
(`docs/system_plan.md`, section 5.0) are in their READMEs.

| Application | Node | Boards |
|---|---|---|
| `apps/drive/` | Wheel drive (and the pedal generator) | Jacobi (`unimoc_jacobi`), NUCLEO-H743ZI, `native_sim`; later Barlow, Dolivo |
| `apps/vdc/` | VDC | Zuse (planned, STM32H743VIT6), NUCLEO-H743ZI, `native_sim` |
| planned | Pedal unit | Drais (STM32G473CBT6) |
| planned | Display | Gutenberg (WT32-SC01 Plus) |
| planned | BMS controller | Volta |

## 9. Open Points

1. The 12 V converter: a potted vehicle DC/DC of 3 to 5 A in place of the
   XL7015 (section 3).
2. The BTS part for the lights, after the lamp currents (section 2.2).
3. The SC01 Plus on the bench: the 8080 driver and the panel init
   sequence (section 4.4).
4. The update of the pedal unit: RAM staging (proposal) or a single-slot
   bootloader (section 6.2).
5. The information of section 7.1 and the hardware repositories
   (section 7.2).
6. Later: a Jacobi power board variant for 48 V (80 to 100 V FETs).
