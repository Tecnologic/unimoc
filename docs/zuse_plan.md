# Zuse: VDC Board Plan

> **Status (2026-11-26):** draft. Nothing is drawn or built. Zuse is the
> vehicle dynamics controller (VDC) board of the cart (`docs/system_plan.md`,
> sections 3, 5.0 and 5.3). The firmware is `apps/vdc/`, today on the
> NUCLEO-H743ZI and `native_sim`.
>
> Decided by the user (2026-11-26):
>
> - The display is the Panlee **WT32-SC01 Plus** (3.5", ESP32-S3), connected
>   to Zuse over its **RS-485** port (section 4).
> - The battery is **24 V** for now; Zuse supports up to **48 V** nominal.
> - **The pedals connect to Zuse**, no separate pedal unit. Limp-home on the
>   pedals after a VDC failure is given up for this variant (section 6).
> - The steering angle sensor is mechanically independent of the board, and
>   it is **not** a CAN node of its own (section 5).
>
> Proposed here, to confirm: the 12 V auxiliary supply (section 3), the
> link protocol of the display (section 4.2), the board I/O (section 2).

## 1. Function

- Cyphal/CAN-FD node `org.unimoc.zuse` (node-ID 1), the VDC application.
- Driver inputs: drive and brake pedal (two channels each), key switch,
  engage, direction, mode.
- Vehicle state: IMU on the board, GNSS module, steering angle, wheel speeds
  from the drive status.
- Torque distribution, later torque vectoring and drift assist
  (`lib/vehicle/`).
- Display (Gutenberg = the SC01 Plus) and logging to a microSD card.
- Monitors the E-stop loop; never closes or bridges it.

## 2. Interfaces

| Connector | Signals | Level / protection | MCU peripheral (STM32H743VIT6) |
|---|---|---|---|
| Supply in | +VIN, GND | 9 to 60 V (12 V aux net, or 24 / 48 V battery directly), reverse polarity protection, TVS, fuse | ADC: VIN sense |
| CAN FD ×2 (same bus, daisy chain) | CANH, CANL, GND (+12 V aux pass-through optional) | Transceiver with standby pin (as Jacobi), switchable split termination 2 × 60 Ω, ESD | FDCAN1 |
| CAN 2 (optional) | CANH, CANL, GND | Footprint only | FDCAN2 |
| E-stop sense | Loop + (after the E-stop switches), loop − | Optocoupler, 12 to 60 V tolerant; reads "loop closed" | GPIO (EXTI) |
| E-stop open (option) | Normally open contact in series with the loop | Relay or solid-state switch, opens when unpowered; Zuse can open the loop, never close it | GPIO |
| Display (SC01 Plus) | RS-485 A, B, GND, +5 V | Half-duplex transceiver, switchable 120 Ω, ESD; +5 V switched and current limited (≥ 0.5 A) | USART1 with hardware DE, GPIO for the 5 V switch |
| GNSS | TX, RX, PPS, +5 V / 3.3 V, GND | Series resistors, ESD; switched supply | USART2, TIM input capture (PPS) |
| Steering angle sensor | +5 V, GND, PWM (baseline); SPI or RS-422 lines reserved | Shielded cable, RC filter, ESD; switched supply | TIM input capture; SPI2 option |
| Drive pedal | 2 × signal, 2 × +5 V sensor supply, GND | Separate current-limited supplies per channel (plausibility), RC filters, ESD | 2 ADC channels on different ADCs, supply sense |
| Brake pedal | 2 × signal, 2 × +5 V sensor supply, GND | as above | 2 ADC channels, supply sense |
| Driver inputs | Key switch (wake), engage, direction forward / reverse, mode, mechanical brake switch, 1 spare | 12 to 60 V tolerant, pull to GND or to VIN selectable | GPIO, key switch also on the supply enable |
| Low-side outputs ×3 | Brake light, horn / buzzer, spare | Protected low-side switches (≥ 1 A), freewheel diode | GPIO / TIM PWM |
| microSD | SDMMC 4 bit | Card detect | SDMMC1 |
| Debug | SWD, SWO, NRST | Tag-Connect TC2050 (as Jacobi) | SWD (RTT log) |
| USB-C | USB FS | Later DFU (`docs/system_plan.md`, "USB") | OTG_FS |
| Status LEDs | RUN, ERROR, CAN, MODE | | GPIO |

The pin assignment follows in CubeMX once this list is agreed. The budget
fits the LQFP100 with room to spare: one FDCAN (two with the option), two
SPI (IMU, steering option), three U(S)ARTs (display, GNSS, spare), SDMMC1,
USB, about 8 ADC channels, two input captures, and about 20 GPIOs.
Check that SDMMC1 (PC8 to PC12, PD2) does not collide with the FDCAN and
USART pins, and that the pedal channels of one pedal sit on different ADCs.

**IMU:** in the middle of the board, away from mounting holes and
connectors, axes on the silkscreen, interrupt on an EXTI or timer pin, SPI.
A second footprint for an alternative part (vibration-robust candidates:
BMI088, ISM330DHCX; Zephyr drivers to check). The firmware gets a mounting
rotation register (calibrated at standstill from gravity, the heading axis
from a straight acceleration), so the board does not need to sit level.
Mount the board rigidly near the centre of gravity of the cart.

## 3. Power

- **Proposal: a 12 V auxiliary net** for Zuse, the display, the sensors,
  the lights and the horn. Its DC/DC converter is fed through a diode OR from
  the pack (after the key switch, fused) and from the DC link. The pack
  feeds it before the contactor closes. The DC link feeds it without a
  battery, when the pedal generator holds the link (`docs/system_plan.md`,
  section 7.2.3). Where it sits: on the BMS board (Volta), which sees both
  voltages, or a bought converter module. To decide.
- **Zuse takes 9 to 60 V** anyway, so it runs from the 12 V net or straight
  from a 24 V or 48 V pack on the bench. On board: a wide-input buck to 5 V
  (display, pedals, steering sensor, GNSS), then 3.3 V.
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

### 4.2 Link protocol (to decide)

`docs/system_plan.md`, section 4, decided **"Cyphal/serial: not used"** (no
maintained C implementation, libserard). The display link needs that
decision revisited:

- **A. Cyphal/serial, own implementation (proposal).** Single-frame
  transfers only (every UNIMOC message fits in 63 bytes anyway): the
  Cyphal/serial header, COBS framing and CRC-32C, behind the `Transport` of
  `lib/cyphal`. The display is then a Cyphal node like any other: it
  subscribes to the cart messages Zuse forwards, publishes its inputs, and
  gets registers, GetInfo and the firmware update. Testable on `native_sim`
  over the slcan-like pseudo terminals.
- **B. Own small frame protocol**, Zuse as the gateway. Less code on the
  display, but a second protocol, no registers or update over the bus.

Either way RS-485 is half duplex: **Zuse is the master.** It sends the
display frame cyclically (10 to 20 Hz) and the display answers only in the
window after it (touch inputs, mode selection). Baud rate 115200 to
1 Mbit/s, to fix with the bus test.

The display shows and asks; no safety function depends on it. Its
firmware (Zephyr with LVGL as `apps/display/`, or ESP-IDF) is open: Zephyr
supports the ESP32-S3, the ST7796 over 8080 and the FT6336 drivers are to
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

## 6. Pedals on Zuse

Two channels per pedal (opposite slopes or separate supplies), plausibility
check in `lib/vehicle/` as planned for the pedal unit. Zuse publishes
`unimoc.cart.PedalState` itself, so the logging, the drive code and the PC
tools stay unchanged.

**Consequence:** the pedal state dies with Zuse. After a VDC failure the
drives see "VDC and pedal unit lost" (`docs/system_plan.md`, section 7):
the torque ramps to zero and **only the mechanical emergency brake stops
the cart**; there is no regenerative service brake left. This contradicts
the safety outline (section 11: "the regenerative service brake keeps
working without the VDC"), which needs an update; the mechanical brake must
be able to stop the cart from top speed on its own (section 7.1).
Limp-home stays useful for the pedal generator variant (Drais), where the
rider's pedals replace the drive and brake pedal.

## 7. Bring-up order

1. NUCLEO-H743ZI with breakout boards: IMU, AS5048A (PWM), GNSS, RS-485
   transceiver to the SC01 Plus, CAN FD transceiver
   (`apps/vdc/boards/nucleo_h743zi.overlay`).
2. The display link (section 4.2) and the pedal inputs in `apps/vdc/`.
3. Schematic and layout of Zuse, then `boards/unimoc/zuse/` (board
   `unimoc_zuse`).

## 8. Open Points

1. 12 V aux converter: on Volta (BMS) or a bought module (section 3).
2. Display link protocol A or B (section 4.2), and the display firmware.
3. IMU part.
4. Connector family (locking, vehicle grade) and enclosure.
5. The E-stop open contact on Zuse: yes or no.
6. A Jacobi power board variant for 48 V (80 to 100 V FETs).
