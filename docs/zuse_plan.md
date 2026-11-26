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
> - A **12 V auxiliary supply** for the low-power consumers; the STM32G473CBT6
>   is a candidate for it too (section 3).
> - **Zuse switches the lights** (section 2.2).
> - The steering angle sensor is mechanically independent of the board, and
>   it is **not** a CAN node of its own (section 5). AS5048A breakout at hand.
> - The E-stop loop: Zuse **senses** whether the loop is open and has a
>   contact to **open** it, both with the optocoupler circuit of Jacobi
>   (section 2.1).
> - Connectors: **JST** as on Jacobi (pitch to confirm, section 2.3).
> - Enclosure: **printed in ABS / ASA**.
>
> Proposed here, to confirm: the 12 V supply without an MCU (section 3),
> the IMU part (section 2.4), the default state of the E-stop open contact
> (section 2.1).

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
- Senses the E-stop loop and can open it; never closes or bridges it.

## 2. Interfaces

| Connector | Signals | Level / protection | MCU peripheral (STM32H743VIT6) |
|---|---|---|---|
| Supply in | +VIN, GND | 9 to 60 V (12 V aux net, or 24 / 48 V battery directly on the bench), reverse polarity protection, TVS, fuse | ADC: VIN sense |
| CAN FD ×2 (same bus, daisy chain) | CANH, CANL, GND | Transceiver with standby pin (as Jacobi), switchable split termination 2 × 60 Ω, ESD | FDCAN1 |
| CAN 2 (optional) | CANH, CANL, GND | Footprint only | FDCAN2 |
| E-stop sense | Loop in, loop return | Optocoupler circuit of Jacobi (section 2.1) | GPIO (EXTI) |
| E-stop open | Contact in series with the loop | Solid-state relay (section 2.1) | GPIO |
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
(`docs/system_plan.md`, sections 3 and 7). Zuse gets two parts of the
Jacobi circuit (the `PWM_EN` optocoupler on PE15, schematic in
Tecnologic/unimoc_jacobi):

- **Sense:** the same optocoupler input at the end of the loop, so Zuse
  sees "loop open" no matter which switch opened it, shows it and logs it.
- **Open:** a solid-state relay (an optically isolated MOSFET switch) in
  series with the loop. Its rating follows the loop current: the optocoupler
  inputs of four drives, a few mA each, so a small signal relay part does.
  **Proposal: normally closed (form B).** A dead or resetting Zuse then
  leaves the loop closed, and the drives keep their limp-home on the pedal
  unit (section 6). Zuse opens the loop only on a fault it detects itself
  (for example an implausible yaw rate, a stuck torque). A normally open
  part would stop the cart on every VDC reset.

### 2.2 Lights and horn

Six protected high-side outputs at 12 V (aux net): headlight, tail light,
brake light, indicator left, indicator right, horn. High side with current
sense, so Zuse detects a broken lamp (the brake light above all) and a
short; candidates are dual or quad smart switches of the automotive range
(Infineon PROFET, ST VIPower; part to choose). The brake light follows the
brake pedal and the regenerative brake torque, not only the pedal switch.

### 2.3 Connectors

JST as on Jacobi. **To confirm: JST XH has a 2.5 mm pitch; 2.0 mm is
JST PH.** Both only lock by friction; in the vehicle the enclosure holds
the plugs (a rib above them) or a drop of hot glue does. The light outputs
and the supply input carry more than the 2 to 3 A of PH / XH: use a larger
crimp connector there (JST VH, 3.96 mm, up to 10 A).

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
- **Proposal: a buck controller IC, no MCU, no CAN** (100 V synchronous
  buck controllers such as the TI LM5146, or a bought module). Reasons:
  - Zuse and the display hang on this net. A converter regulated by
    firmware takes them down on every crash, update or reset of its MCU.
  - Zuse measures the 12 V itself (VIN sense) and knows the load of every
    light output; a converter node adds little to that.
  - One board fewer on the bus, no firmware to maintain for it.
  A G473 on the converter makes sense only for something an IC cannot do
  (several switched outputs with a power budget, a charger). Then it would
  supervise and switch, while an IC still regulates.
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
- **48 V and Jacobi:** Jacobi has 60 V FETs (NCEP6080AG) and is rated 36 V.
  A 13S pack reaches 54.6 V when full, plus the rise during regenerative
  braking, so 48 V needs a power board variant with 80 to 100 V FETs and
  capacitors. A 24 V pack (7S, 29.4 V full) is within the rating. The
  voltage thresholds of section 7.2 (33 / 30 / 38 / 42 V) are for the 36 V
  system and have to be scaled for the pack.

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

The display shows and asks; no safety function depends on it. Its
firmware (Zephyr with LVGL as `apps/display/`, or ESP-IDF) is open: Zephyr
supports the ESP32-S3; the ST7796 over 8080 and the FT6336 drivers are to
check.

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

**Risk: 128 KB flash.** The firmware update uses MCUboot with two slots
(`docs/system_plan.md`, section 4). Bootloader plus two slots of a Zephyr
Cyphal node with registers is tight in 128 KB. First step: build a minimal
pedal application for a G4 board (NUCLEO-G474RE) and read the size. If it
does not fit: the STM32G473CCT6 (256 KB) or an update without a second slot.

## 7. Bring-up order

1. NUCLEO-H743ZI with breakout boards: IMU (MPU-9250 first, ICM-42688-P
   later), AS5048A (PWM), GNSS, RS-485 transceiver to the SC01 Plus, CAN FD
   transceiver (`apps/vdc/boards/nucleo_h743zi.overlay`).
2. The Cyphal/serial transport (section 4.2) and the display link.
3. Size check of the pedal unit on the G4 (section 6).
4. Schematic and layout of Zuse, then `boards/unimoc/zuse/` (board
   `unimoc_zuse`).

## 8. Open Points

1. 12 V supply: controller IC (proposal) or a G473 node (section 3).
2. IMU part (section 2.4).
3. E-stop open contact: normally closed (proposal) or normally open.
4. JST pitch: XH 2.5 mm or PH 2.0 mm (section 2.3).
5. The display firmware (Zephyr or ESP-IDF).
6. A Jacobi power board variant for 48 V (80 to 100 V FETs).
7. Flash size of the pedal unit (section 6).
