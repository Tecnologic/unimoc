# Zuse: VDC Board Plan

> **Status (2026-11-26):** draft. Nothing is drawn or built. Zuse is the
> vehicle dynamics controller (VDC) board of the cart (`docs/system_plan.md`,
> sections 3, 5.0 and 5.3). The firmware is `apps/vdc/`, today on the
> NUCLEO-H743ZI and `native_sim`.
>
> Decided by the user (2026-11-26):
>
> - The display is the Panlee **WT32-SC01 Plus** (3.5", ESP32-S3), connected
>   to Zuse over its **RS-485** port with a **lean own Cyphal/serial
>   transport** (section 4).
> - The battery is **24 V** for now; Zuse supports up to **48 V** nominal.
> - The **pedals go to a pedal unit (Drais) on an STM32G473CBT6** (the small
>   G4 of the former BCC), a Cyphal node of its own. Limp-home on the pedals
>   stays as planned (section 6).
> - A **12 V auxiliary supply** for the low-power consumers: a buck
>   controller IC or a bought module, **no MCU and no bus node** (section 3).
> - **Zuse switches the lights** (section 2.2).
> - The steering angle sensor is mechanically independent of the board, and
>   it is **not** a CAN node of its own (section 5). AS5048A breakout at hand.
> - The E-stop loop: Zuse **only senses** whether the loop is open, with the
>   optocoupler circuit of Jacobi. The loop is opened by the E-stop buttons
>   only, never by Zuse (section 2.1).
> - Connectors: **JST PH** (2.0 mm) as on Jacobi (section 2.3).
> - Enclosure: **printed in ABS / ASA**.
> - The pedal unit has one firmware image, flashed over USB (section 6).
> - The display firmware is **Zephyr** (`apps/display/`, section 4.3).
> - A 48 V variant of Jacobi is for later (section 3).
>
> Proposed here, to confirm: the IMU part (section 2.4).

## 1. Function

- Cyphal/CAN-FD node `org.unimoc.zuse` (node-ID 1), the VDC application.
- Driver inputs on Zuse: key switch, engage, direction, mode, the switch of
  the mechanical brake. The pedals come over the bus (`unimoc.cart.PedalState`
  from Drais).
- Vehicle state: IMU on the board, GNSS module, steering angle, wheel speeds
  from the drive status.
- Torque distribution, later torque vectoring and drift assist
  (`lib/vehicle/`).
- Lights and horn.
- Display (Gutenberg = the SC01 Plus) and logging to a microSD card.
- Senses the E-stop loop; never opens, closes or bridges it.

## 2. Interfaces

| Connector | Signals | Level / protection | MCU peripheral (STM32H743VIT6) |
|---|---|---|---|
| Supply in | +VIN, GND | 9 to 60 V (12 V aux net, or 24 / 48 V battery directly on the bench), reverse polarity protection, TVS, fuse | ADC: VIN sense |
| CAN FD ×2 (same bus, daisy chain) | CANH, CANL, GND | Transceiver with standby pin (as Jacobi), switchable split termination 2 × 60 Ω, ESD | FDCAN1 |
| CAN 2 (optional) | CANH, CANL, GND | Footprint only | FDCAN2 |
| E-stop sense | Loop in, loop return | Optocoupler circuit of Jacobi (section 2.1) | GPIO (EXTI) |
| Display (SC01 Plus) | RS-485 A, B, GND, +5 V | Half-duplex transceiver, switchable 120 Ω, ESD; +5 V switched and current limited (≥ 0.5 A) | USART1 with hardware DE, GPIO for the 5 V switch |
| GNSS | TX, RX, PPS, +5 V / 3.3 V, GND | Series resistors, ESD; switched supply | USART2, TIM input capture (PPS) |
| Steering angle sensor | +5 V, GND, PWM (baseline); SPI lines reserved | Shielded cable, RC filter, ESD; switched supply | TIM input capture; SPI2 option |
| Driver inputs | Key switch (wake), engage, direction forward / reverse, mode, mechanical brake switch, 1 spare | 12 to 60 V tolerant, to GND switched | GPIO |
| Lights and horn | 6 switched outputs (section 2.2) | Protected high-side switches with current sense | GPIO, ADC (sense) |
| microSD | SDMMC 4 bit | Card detect | SDMMC1 |
| Debug | SWD, SWO, NRST | Tag-Connect TC2050 (as Jacobi) | SWD (RTT log) |
| USB-C | USB FS | Later DFU (`docs/system_plan.md`, "USB") | OTG_FS |
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
buttons only. Zuse has no contact in the loop (decided 2026-11-26): a fault
of Zuse can never stop the drives through the loop, and the drives keep
their limp-home on the pedal unit (section 6).

Zuse senses the loop with the optocoupler input of Jacobi (the `PWM_EN`
optocoupler on PE15, schematic in Tecnologic/unimoc_jacobi) at the end of
the loop, so it sees "loop open" no matter which button opened it, shows it
on the display and logs it. The sense input loads the loop like one more
drive.

### 2.2 Lights and horn

Six protected high-side outputs at 12 V (aux net): headlight, tail light,
brake light, indicator left, indicator right, horn. High side with current
sense, so Zuse detects a broken lamp (the brake light above all) and a
short; candidates are dual or quad smart switches of the automotive range
(Infineon PROFET, ST VIPower; part to choose). The brake light follows the
brake pedal and the regenerative brake torque, not only the pedal switch.

### 2.3 Connectors

JST PH (2.0 mm) as on Jacobi (decided 2026-11-26). PH only locks by
friction; in the vehicle the enclosure holds the plugs (a rib above them)
or a drop of hot glue does. The light outputs and the supply input carry
more than the 2 A of PH: use a larger crimp connector there (proposal
JST VH, 3.96 mm, up to 10 A).

### 2.4 IMU

In the middle of the board, away from mounting holes and connectors, axes
on the silkscreen, SPI and an interrupt on an EXTI or timer pin. The
firmware gets a mounting rotation register (calibrated at standstill from
gravity, the heading axis from a straight acceleration), so the board does
not need to sit level. Mount the board rigidly (screws into brass inserts
of the printed enclosure, no foam) near the centre of gravity of the cart.

| Part | For | Against |
|---|---|---|
| **TDK ICM-42688-P** (proposal) | Low gyro noise, anti-alias filter in the part, SPI up to 24 MHz, cheap, common on breakouts, Zephyr driver `invensense,icm42688` | Consumer part (−40 to 85 °C) |
| Bosch BMI088 | Built for vibration (drones), separate accelerometer and gyro, Zephyr driver `bosch,bmi08x-*` | Higher gyro noise, two chip selects |
| ST ISM330DHCX | Industrial (−40 to 105 °C), machine learning core unused here, Zephyr driver `st,ism330dhcx` | More expensive |

The MPU-9250 at hand works for the bench (Zephyr driver
`invensense,mpu9250`), but it is discontinued; not for the board. The
magnetometer is not needed: the heading comes from GNSS and the gyro.
Driver names to check against the Zephyr version in `west.yml`.

## 3. Power

- **12 V aux net** for Zuse, the display, the sensors, the lights and the
  horn. Budget (estimate): Zuse 2 W, display 1 W, GNSS 0.5 W, lights
  20 to 30 W, horn 10 W; about 50 W, so 12 V / 5 A.
- Its input is a diode OR of the pack (after the key switch, fused) and the
  DC link, so it runs before the contactor closes and without a battery when
  the pedal generator holds the link (`docs/system_plan.md`, section 7.2.3).
  Input range 16 to 60 V for a 24 V or 48 V pack (a 100 V controller for the
  regenerative rise).
- **A buck controller IC or a bought module, no MCU, no bus node**
  (decided 2026-11-26; candidates: 100 V synchronous buck controllers such
  as the TI LM5146, or a DC/DC module for 24 / 48 V vehicles). Reasons:
  - Zuse and the display hang on this net. A converter regulated by
    firmware would take them down on every crash, update or reset of its
    MCU.
  - Zuse measures the 12 V itself (VIN sense) and knows the load of every
    light output; a converter node adds little to that.
  - One board fewer on the bus, no firmware to maintain for it.
- **Zuse takes 9 to 60 V** anyway, so it runs from the 12 V net or straight
  from the pack on the bench. On board: a wide-input buck to 5 V (display,
  steering sensor, GNSS), then 3.3 V.
- **Jacobi keeps its own supply from the DC link** (LM5008 / LMR12010 of
  the 4850 logic, +12P gate supply). It is not redundant with the 12 V net:
  - the pedal generator boots the drives from the DC link alone at about
    10 V (section 7.2.3);
  - the gate supply stays local to the power stage, not on a long harness
    with the switching currents of four drives;
  - a failed aux converter would otherwise stop all four drives at once.
  A 12 V aux input on a later Jacobi revision (diode OR into the logic
  supply, for configuration and updates without the DC link) is an option.
- **48 V and Jacobi (later):** Jacobi has 60 V FETs (NCEP6080AG) and is rated 36 V.
  A 13S pack reaches 54.6 V when full, plus the rise during regenerative
  braking, so 48 V needs a power board variant with 80 to 100 V FETs and
  capacitors. A 24 V pack (7S, 29.4 V full) is within the rating. The
  voltage thresholds of section 7.2 (33 / 30 / 38 / 42 V) are for the 36 V
  system and have to be scaled for the pack. The 48 V variant of Jacobi is
  planned for later (`docs/system_plan.md`, section 12); Zuse and the 12 V
  supply are laid out for 48 V from the start.

Display budget: 4.7 to 5.5 V, 170 to 190 mA at full backlight from the
datasheet (WT32-SC01 Plus V1.3, table 9); WLAN peaks come on top, so the
5 V output to the display is rated for at least 0.5 A.

## 4. Display (Gutenberg = WT32-SC01 Plus)

From the datasheet (V1.3): ESP32-S3 (WT32-S3-WROVER-N16R2: 16 MB flash,
2 MB PSRAM), 480 × 320 IPS, ST7796UI over 8 bit 8080, FT6336U capacitive
touch over I2C, I2S amplifier 2.5 W at 4 Ω, microSD over SPI, USB-C.
The RS-485 connector (table 6) carries A, B, GND and +5 V ("power supply
or output"); the ESP32-S3 drives it with RXD GPIO 1, TXD GPIO 42 and RTS
(driver enable) GPIO 2. Six spare 3.3 V GPIOs on the extended IO connector.
Not in the datasheet: the RS-485 transceiver part and whether the display
terminates the line.

### 4.1 Why RS-485 and not the cart CAN

The ESP32-S3 has a classic CAN controller (TWAI), no CAN FD. A classic CAN
node on the cart bus would answer every FD frame with an error frame. So
the display stays off the cart bus and talks to Zuse point to point.

### 4.2 Link protocol: lean Cyphal/serial (decided 2026-11-26)

This revises `docs/system_plan.md`, section 4, "Cyphal/serial: not used"
(no maintained C implementation, libserard): instead of a library, an own
lean transport in `lib/cyphal`.

- Single-frame transfers only (every UNIMOC message fits in 63 bytes
  anyway): the Cyphal/serial header with its CRC, COBS framing and the
  CRC-32C of the payload, behind the `Transport` of `lib/cyphal`.
- The display is a Cyphal node like any other: it subscribes to the
  messages Zuse sends it, publishes its inputs, and has registers, GetInfo
  and later the firmware update. Zuse forwards the subjects the display
  needs between the CAN bus and the serial link (no transparent bridge).
- RS-485 is half duplex: **Zuse is the master.** It sends its frames
  cyclically (10 to 20 Hz) and the display answers only in the window after
  them (touch inputs, mode selection). Baud rate 115200 to 1 Mbit/s, fixed
  with the bench test.
- Tested with ztest on the frame codec and on `native_sim` over a pseudo
  terminal, as the slcan ports.

The display shows and asks; no safety function depends on it.

### 4.3 Display firmware (Zephyr, decided 2026-11-26)

A third Zephyr application, `apps/display/` (Gutenberg), with LVGL for the
screens; it shares `lib/cyphal`, `lib/units`, `lib/settings`, the DSDL and
the release version with the other applications. To do:

- A board definition for the SC01 Plus (`boards/unimoc/gutenberg/` or an
  overlay on an ESP32-S3 board): the pins of section 4 (8080 bus, touch
  I2C, RS-485 UART with RTS as driver enable, microSD over SPI, I2S).
- Check against the Zephyr version in `west.yml`: the ST7796 driver on the
  8080 (MIPI DBI) bus of the ESP32-S3, the FT6336 touch (FocalTech FT5336
  driver family), the PSRAM for the LVGL frame buffers.
- The ESP32 toolchain (`xtensa-espressif_esp32s3_zephyr-elf` of the Zephyr
  SDK) and the Espressif HAL as west projects; CI builds the display too.
- The screens show what a driver needs at a glance (speed, battery, mode,
  faults of the drives, the BMS and the E-stop loop), and the settings of
  the cart behind a menu.

## 5. Steering angle sensor

The steering column is neither horizontal nor vertical, so the board does
not sit on the column (the IMU would be tilted and far from the centre of
gravity). The sensor is a small passive board at the end of the column:
AS5048A (as the legacy firmware) with the diametric magnet on the column.
No MCU, not a bus node.

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

This revises `docs/system_plan.md`, section 5.1 ("every board is STM32H7
based"): the G4 is back for small nodes **without a hot path**. They need
no hot-path platform (the removed `hardware/zephyr_bcc` was one), only a
board definition, Zephyr's own drivers (FDCAN, ADC) and a
`cyphal_platform.cpp`.

**Firmware update (decided 2026-11-26): one image, no MCUboot, no update
over the bus.** The pedal unit is flashed over USB with the DFU of the STM32
system bootloader in ROM, so the whole 128 KB flash holds the application.
The system bootloader is entered with BOOT0 (a button on the board) or from
the firmware: an ExecuteCommand jumps to the ROM bootloader, so the
enclosure needs no opening when its USB-C socket is reachable from outside.
Still check the size of a minimal pedal application for a G4 board
(NUCLEO-G474RE) before the layout.

Jacobi and Zuse keep the update over the bus (MCUboot in the 2 MB of the
H743, already running on the NUCLEO-H743ZI): it updates the assembled cart
without reaching every board. USB DFU stays their recovery path.

## 7. Bring-up order

1. NUCLEO-H743ZI with breakout boards: IMU (MPU-9250 first, ICM-42688-P
   later), AS5048A (PWM), GNSS, RS-485 transceiver to the SC01 Plus, CAN FD
   transceiver (`apps/vdc/boards/nucleo_h743zi.overlay`).
2. The Cyphal/serial transport (section 4.2) and the display link; the
   display application on the SC01 Plus (section 4.3).
3. Size check of the pedal unit on the G4 (section 6).
4. Schematic and layout of Zuse, then `boards/unimoc/zuse/` (board
   `unimoc_zuse`).

## 8. Open Points

1. IMU part (section 2.4).
2. 12 V supply: the controller IC or the module, and where it sits.
3. Zephyr support for the SC01 Plus (section 4.3).
4. Size check of the pedal application on the G4 (section 6).
5. Later: a Jacobi power board variant for 48 V (80 to 100 V FETs).
