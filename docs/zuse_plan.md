# Zuse: VDC Board and the Cart Periphery

> **Status (2026-11-26):** draft. Nothing is drawn or built. Zuse is the
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
| Pedal unit | Drais, an own Cyphal/CAN-FD node on an **STM32G473CBT6** (the small G4 of the former BCC), **firmware update over Cyphal**. Limp-home on the pedals stays. | 6 |
| Steering angle sensor | AS5048A on a small passive board at the steering column, cable to Zuse; **not** a bus node, not on the Zuse board. | 5 |
| E-stop loop | Zuse **senses** it with the optocoupler input of Jacobi; only the E-stop buttons open it, **Zuse has no contact in the loop**. | 2.1 |
| Lights | **Switched by Zuse**: Infineon BTS high-side switches (PROFET) with current sense; the part is chosen with the schematic. | 2.2 |
| IMU | **TDK ICM-42688-P**. | 2.4 |
| Connectors | **JST PH** (2.0 mm) for signals as on Jacobi, **Molex Micro-Fit 3.0** for the supply and the lights. | 2.3 |
| Enclosure | 3D printed in **ABS / ASA**. | 2.5 |
| Names | The applications keep their names (`apps/drive/`, `apps/vdc/`); the board code names are in the documentation (`apps/*/README.md`). | 8 |
| MCU | Zuse gets the **STM32H743VIT6** of Jacobi (one part, one platform). | 2 |

Checked on 2026-11-26 (builds, nothing run on hardware): the pedal
application fits the G4 (section 6.1), and Zephyr 4.5 has every driver the
SC01 Plus needs (section 4.4). No blocker.

## 1. Function

- Cyphal/CAN-FD node (node-ID 1), the VDC application `apps/vdc/`.
- Driver inputs on Zuse: key switch, engage, direction, mode, the switch of
  the mechanical brake. The pedals come over the bus
  (`unimoc.cart.PedalState` from Drais).
- Vehicle state: IMU on the board, GNSS module, steering angle, wheel speeds
  from the drive status.
- Torque distribution, later torque vectoring and drift assist
  (`lib/vehicle/`).
- Lights and horn.
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
| GNSS | TX, RX, PPS, +5 V / 3.3 V, GND | Series resistors, ESD; switched supply | USART2, TIM input capture (PPS) |
| Steering angle sensor | +5 V, GND, PWM (baseline); SPI lines reserved | Shielded cable, RC filter, ESD; switched supply | TIM input capture; SPI2 option |
| Driver inputs | Key switch (wake), engage, direction forward / reverse, mode, mechanical brake switch, 1 spare | 12 to 60 V tolerant, switched to GND | GPIO |
| Lights and horn (Micro-Fit) | 6 switched outputs (section 2.2) | Protected high-side switches with current sense | GPIO, ADC (sense) |
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

### 2.2 Lights and horn

Six protected high-side outputs at 12 V (aux net): headlight, tail light,
brake light, indicator left, indicator right, horn. Infineon BTS high-side
switches (PROFET family, single or dual channel) with the current sense
(IS) pin on an ADC input, so Zuse detects a broken lamp (the brake light
above all) and a short. The part follows from the lamp currents when the
schematic is drawn. The brake light follows the
brake pedal and the regenerative brake torque, not only a pedal switch.

### 2.3 Connectors

- **Signals: JST PH (2.0 mm)** as on Jacobi. PH locks by friction only; in
  the vehicle the enclosure holds the plugs (a rib above them) or a drop of
  hot glue does. PH carries 2 A per contact.
- **Supply and lights: Molex Micro-Fit 3.0** (3.0 mm, latching, up to about
  5 A per contact depending on the wire and the pin count; parts at hand).

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

### 2.5 Enclosure

Printed in ABS or ASA (ASA for outdoor use: UV stable). The board sits on
screws in brass heat-set inserts, not on foam, so the IMU sees the cart and
not the enclosure. Openings for the connectors and the USB-C socket; the
JST PH plugs held by a rib (section 2.3).

## 3. Power

- **12 V aux net** for Zuse, the display, the sensors, the lights and the
  horn. Budget (estimate): Zuse 2 W, display 1 W, GNSS 0.5 W, lights
  20 to 30 W, horn 10 W; about 50 W, so 12 V / 5 A.
- **A bought XL7015 buck module, separate, not on Volta; no MCU, no bus
  node.** Zuse and the display hang on this net; a converter regulated by
  firmware would take them down on every crash, update or reset of its MCU.
  Zuse measures the 12 V itself (VIN sense) and knows the load of every
  light output.
- **XL7015 and the budget:** the XL7015 takes up to 80 V, which covers a
  48 V pack, but it is rated about 0.8 A output (XLSEMI datasheet; check the
  module's own rating): about 10 W at 12 V. That carries the electronics
  (Zuse, display, GNSS, sensors: about 4 to 5 W) but not the lights and the
  horn (30 to 40 W). Either the lights get their own, stronger converter
  (a 24 / 48 V to 12 V vehicle DC/DC of 5 to 10 A), or the XL7015 is
  replaced by one such converter for everything.
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

Two channels per pedal (opposite slopes or separate supplies), plausibility
check, `unimoc.cart.PedalState` on the bus (`docs/system_plan.md`,
section 5.4). Limp-home on the pedals after a VDC failure stays as in
section 7 of the system plan.

The G4 is an exception to "every board is STM32H7" (`docs/system_plan.md`,
section 5.1) for small nodes **without a hot path**: no hot-path platform
(the removed `hardware/zephyr_bcc` was one), only a board definition,
Zephyr's own drivers (FDCAN, ADC) and a `cyphal_platform.cpp`.

**Firmware update over Cyphal, one slot.** The 128 KB flash holds no two
MCUboot slots. Instead:

- A small **Cyphal bootloader** stays in the first sectors and is never
  updated over the bus. The application answers `BEGIN_SOFTWARE_UPDATE`
  with a restart into the bootloader; the bootloader reads the image with
  `uavcan.file.Read` from the commanding node (the same file service and
  UNIMOC image header as the drives) straight into the one application
  slot, checks it and starts it.
- A failed or interrupted download leaves the bootloader running, with its
  heartbeat in mode SOFTWARE_UPDATE, until a good image arrives: the node
  cannot brick itself over the bus. There is no rollback to the old image,
  and while the bootloader runs there is no pedal state, so the update runs
  with the cart parked.
- Candidates: [Kocherga](https://github.com/Zubax/kocherga) (a header-only
  Cyphal bootloader library, single slot, CAN and serial transports), or an
  own one on `lib/cyphal`. Budget: bootloader at most 32 KB, application
  96 KB.
- USB DFU of the STM32 ROM bootloader (BOOT0 button) stays the recovery and
  the first flash.

### 6.1 Size check (2026-11-26)

The VDC application stands in for the pedal application: both are a Cyphal
node without a hot path (heartbeat, registers, subjects, the log on the
bus); the pedal unit adds the ADC and the plausibility check and takes out
the torque split. Built for `nucleo_g474re` (STM32G474RE, same core and
FDCAN as the G473), with the STM32H7 platform choice opened to the G4 for
the measurement only (not committed):

| Build | Flash | RAM |
|---|---|---|
| VDC on the NUCLEO-H743ZI (reference, debug log) | 70.5 KB | 59.9 KB |
| VDC on the G474, debug log | 64.7 KB | 59.0 KB |
| VDC on the G474, warning log, no asserts, LTO | 58.8 KB | 58.8 KB |
| as before, with the settings on ZMS | 66.4 KB | 59.1 KB |

So the pedal application needs about 65 to 75 KB flash. The 128 KB split:
bootloader 32 KB, settings 8 KB (ZMS, 2 KB pages), application 88 KB:
about 15 to 20 KB margin. RAM: 59 KB of 128 KB, most of it the libcanard
heap (24 KB) and the thread stacks.

The bootloader is the tighter part: Zephyr's kernel, drivers and libcanard
alone take some 25 KB of the image above, so a Zephyr-based bootloader
would be close to 32 KB. Kocherga on the bare FDCAN (ST LL drivers) stays
well below; to measure when it is chosen. If the margin melts: the
STM32G473CCT6 (256 KB, same package).

## 7. Bring-up order

1. NUCLEO-H743ZI with breakout boards: IMU (MPU-9250 first, ICM-42688-P
   later), AS5048A (PWM), GNSS, RS-485 transceiver to the SC01 Plus, CAN FD
   transceiver (`apps/vdc/boards/nucleo_h743zi.overlay`).
2. The Cyphal/serial transport (section 4.2), then the display application
   on the SC01 Plus (section 4.3).
3. Size check of the pedal application on the G4 and the bootloader choice
   (section 6).
4. Schematic and layout of Zuse, then `boards/unimoc/zuse/` (board
   `unimoc_zuse`).

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

1. The supply of the lights: a second, stronger converter next to the
   XL7015, or one vehicle DC/DC for everything (section 3).
2. The BTS part for the lights (section 2.2).
3. The SC01 Plus on the bench: the 8080 driver and the panel init
   sequence (section 4.4).
4. The bootloader of the pedal unit: Kocherga or own, its size
   (section 6.1).
5. Later: a Jacobi power board variant for 48 V (80 to 100 V FETs).
