# Plan: VDC Hardware (Zuse)

> **Status (2026-11-23):** collection for the schematic, nothing drawn. The
> VDC runs on a NUCLEO-H743ZI until this board exists
> (`apps/vdc/boards/nucleo_h743zi.overlay`). The pins in section 10 are a
> proposal to check in STM32CubeMX before the layout. The cart around it is
> in `docs/cart_mechanics_plan.md`, the system in `docs/system_plan.md`
> (section 5.3).

## 1. Job of the board

The vehicle dynamics controller of the cart: driver inputs (gear buttons,
steering angle), vehicle state (IMU, GNSS, wheel speeds from the drives),
the torque request and its split onto the four drives over Cyphal/CAN FD,
display and logging. No power stage, no zero-latency hot path; a 250 to
500 Hz control loop in Zephyr threads.

Not on this board: the pedals (pedal unit Drais, an own node so the drives
keep their limp-home path without the VDC), the E-stop loop (hard-wired to
the enable inputs of the drives), the mechanical brake.

## 2. Block overview

```
 DC link / key switch (10 to 60 V)
        │
  fuse, reverse polarity, TVS ── buck 5 V ──┬── LDO / buck 3.3 V ── MCU, CAN, IMU, sensors
        │                                    ├── lamps of the gear buttons
     voltage sense (ADC)                     ├── display, GNSS, radio bridge (5 V)
                                             └── steering angle sensor (5 V or 3.3 V)

 STM32H743VIT6 ── FDCAN1 ── TCAN334 ── CAN in / CAN out (split termination)
      ├── SPI1 ── steering angle sensor AS5048A (connector at the column)
      ├── SPI2 ── IMU (on board)
      ├── SPI4 ── display (Gutenberg, connector) + backlight PWM
      ├── USART2 ── GNSS module (connector, PPS)
      ├── USART3 ── radio bridge for the phone (ESP32 module, connector)
      ├── GPIO ── 5 gear buttons (connector) / 5 lamp drivers
      ├── SDMMC1 ── microSD (logging, optional)
      ├── USB FS ── DFU (USB-C)
      └── SWD/SWO ── debug (RTT log)
```

## 3. MCU

- **STM32H743VIT6** (LQFP100), as Jacobi (`docs/drive4850_h7_plan.md`):
  one part, one platform (`hardware/zephyr_stm32h7`), one BOM. The VDC uses
  only part of it; the rest is headroom for torque vectoring and logging.
- **Clock:** 20 MHz oscillator in bypass on PH0, as Jacobi (FDCAN from
  PLL2 Q, 80 MHz). 32.768 kHz LSE optional (log time stamps).
- **Supply of the H743:** LDO mode (VCAP pins with their capacitors), VREF+
  filtered, decoupling as in the data sheet; VBAT to 3.3 V (or a coin cell
  if the RTC should run).
- **Boot:** BOOT0 to ground with a button or jumper (system DFU), NRST button.

## 4. Supply

| Item | Value | Note |
|---|---|---|
| Input | 10 to 60 V | LFP 8S 20 to 29.2 V, a 36 V pack up to 42 V, the pedal generator from about 10 V (system plan 7.2.4). The drives' FETs are 60 V parts. |
| Protection | fuse (or polyfuse), reverse polarity (P-FET), TVS | TVS standoff above the highest pack voltage (42 V), clamp below the buck's maximum input |
| Buck | 60 V input class, 5 V / 1 A or more | display, GNSS, radio bridge, lamps; check the start-up on a slowly rising supply (generator) |
| 3.3 V | LDO or second buck from 5 V, about 500 mA | MCU (H743 supply current well above 100 mA), CAN, IMU, sensors |
| Voltage sense | divider to an ADC pin | supply voltage for the display and the empty-battery state |
| Key switch | in the supply line of the VDC | the cart is off when the key is off; the VDC boots last (system plan 7.2) |

## 5. CAN FD

- **TCAN334** at 3.3 V, as Jacobi, FDCAN1; standby pin to a GPIO (or tied
  to normal mode as on Jacobi rev 0.3).
- **Split termination** (2 × 60 Ω, 4.7 nF to ground) that can be enabled
  with a jumper or solder bridge: the VDC may sit at a bus end.
- **ESD** diode on CANH/CANL (CAN FD capable, low capacitance).
- **Two connectors** (in and out) for the daisy chain, with the supply
  pins if the harness carries it.
- 1 Mbit/s in both phases (as all nodes, `docs/system_plan.md` 6.3).

## 6. Gear buttons and lamps

Five illuminated buttons, one per gear (R, N, 1, 2, 3), on the dashboard
next to the steering column (`lib/vehicle/src/gear_buttons.hpp`,
`hardware/zephyr_common/vdc_gear_io.cpp`).

**Inputs:**
- Button contact between the input and ground (active low).
- 10 kΩ pull-up to 3.3 V on the board (the wires are long; do not rely on
  the internal pull-ups), 1 kΩ in series and 100 nF to ground at the pin
  (RC about 0.1 ms against spikes; the debounce of 20 ms is in software).
- ESD protection per line (TVS array).
- The firmware ignores two buttons at once and a button held longer than
  2 s (stuck, ADVISORY).

**Lamps:**
- Arcade buttons with an LED module (5 V or 12 V variant, series resistor
  inside); choose the 5 V variant, so the 5 V rail feeds them.
- One low-side N-MOSFET per lamp (logic-level, for example a 2N7002 or
  BSS138 class part for about 20 mA per lamp), 100 Ω gate resistor, 100 kΩ
  gate pull-down so the lamps are dark during reset.
- Lamp supply through a polyfuse; a short in the harness must not take the
  VDC down.
- Lamp pins on timer channels where possible, so the lamps can be dimmed
  later (for example "released but not active").
- What the lamps show: lit = active gear, slow blinking = requested and
  waiting (reverse while the cart rolls), fast blinking for a second =
  pressed but not released, dark = the rest.

**Connector:** 12 pins (5 inputs, ground, 5 lamp outputs, lamp supply), a
locking type for vibration (for example Micro-Fit 3.0 2 × 6 or JST XH).

**Devicetree:** `zephyr,user` with `gear-button-gpios` and
`gear-lamp-gpios`, five entries each in the order R, N, 1, 2, 3, the
active level in the flags.

## 7. Sensors

- **Steering angle sensor:** AS5048A (SPI, 14 bit) on a magnet at the
  lower end of the steering column; connector with SCK, MISO, MOSI, CS,
  3.3 V, ground (the legacy firmware used the I2C variant AS5048B; SPI
  is preferred for the cable length and the diagnostics bits). ESD on the
  lines; series resistors (33 Ω) at the MCU for the cable.
- **IMU:** on the board (SPI2, data-ready interrupt), close to the board
  centre; mark the axis directions on the silkscreen and mount the board
  flat on the frame (yaw rate for torque vectoring and drift assist).
- **GNSS:** module on a connector (USART2, 5 V or 3.3 V, PPS to a timer
  input), antenna with a view of the sky; for speed over ground and the
  log.

## 8. Display and phone

- **Display (Gutenberg):** a module on a connector, SPI4 with DC, reset and
  a backlight PWM, 5 V supply, optional touch on I2C1. Shows speed, state
  of charge, gear, faults.
- **Phone:** the VDC has no radio. The gear release and the parameters are
  registers; a small radio bridge (ESP32 module with BLE or Wi-Fi) on
  USART3 can relay them from a phone app. Reserve the connector (5 V,
  ground, TX, RX, EN, BOOT); the protocol is open (section 12).

## 9. Debug, update, logging, misc

- **SWD with SWO** on a 10-pin Cortex debug connector (or Tag-Connect);
  the log runs over RTT.
- **USB-C FS** (PA11/PA12, VBUS sense on PA9) for DFU; ESD on D+/D-; CC
  resistors for a device.
- **microSD** on SDMMC1 (4 bit) for logging, optional.
- **Status LEDs:** two (heartbeat, fault).
- **E-stop sense (optional):** the E-stop loop stays hard-wired to the
  drives; the VDC may read its state through a divider or optocoupler to
  show it on the display. It never switches the loop.

## 10. Pin proposal (STM32H743VIT6, LQFP100)

To check in STM32CubeMX (alternate functions, LQFP100 pin-out, routing).

| Function | Pins | Peripheral |
|---|---|---|
| HSE | PH0 | 20 MHz oscillator, bypass |
| CAN FD | PD0 (RX), PD1 (TX); standby PD3 | FDCAN1 (AF9) |
| Steering angle sensor | PA5 (SCK), PA6 (MISO), PA7 (MOSI), PA4 (CS) | SPI1 (AF5) |
| IMU | PB13 (SCK), PB14 (MISO), PB15 (MOSI), PB12 (CS), PC4 (INT) | SPI2 (AF5) |
| Display | PE12 (SCK), PE13 (MISO), PE14 (MOSI), PE11 (CS), PE15 (DC), PE10 (RST), PE9 (backlight) | SPI4 (AF5), TIM1_CH1 (AF1) |
| Touch, spare I2C | PB6 (SCL), PB7 (SDA) | I2C1 (AF4) |
| GNSS | PD5 (TX), PD6 (RX), PD7 (PPS) | USART2 (AF7) |
| Radio bridge | PD8 (TX), PD9 (RX), PB0 (EN), PB1 (BOOT) | USART3 (AF7) |
| Gear buttons R, N, 1, 2, 3 | PE2, PE3, PE4, PE5, PE6 | GPIO inputs, external pull-ups |
| Gear lamps R, N, 1, 2, 3 | PD10, PD11, PD12, PD13, PD14 | GPIO outputs (PD12 to PD14 also TIM4_CH1 to CH3) |
| microSD | PC8 to PC12, PD2 | SDMMC1 (AF12) |
| USB DFU | PA11 (DM), PA12 (DP), PA9 (VBUS) | OTG FS |
| SWD | PA13, PA14, SWO PB3 | |
| Supply sense | PC0 | ADC (INP10) |
| E-stop sense | PC2 | GPIO input |
| Status LEDs | PE7, PE8 | GPIO |
| Boot | BOOT0 | button or jumper |

The NUCLEO-H743ZI uses other pins for the gear buttons for now (PE9, PE11,
PE13, PF14, PF15; lamps PE14, PE12, PE10, PF12, PF13).

## 11. Mechanics

- Board in a closed housing behind the steering column, cable glands or
  sealed connectors (water spray, dirt).
- Mounted flat and stiff on the frame (IMU); note the orientation.
- Connectors with locking; strain relief for the column cables.

## 12. Open points

1. Radio bridge for the phone: module, protocol (Cyphal register access
   over BLE, or a small own protocol), and who may write the release.
2. Display module and its size; touch or not.
3. Whether the IMU sits on this board or on a small board at the centre of
   gravity.
4. Physical gear switch on the bus for a later steering node (Mercator):
   would need a DSDL field; for now the buttons are local to the VDC.
5. Lamp dimming ("released but not active") on the timer channels.
