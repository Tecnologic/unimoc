# Plan: Pedal Assist on the Battery Case Controller

Status: 2026-10-23. Steps 1 to 5 are implemented, tested under twister on `native_sim`
(`lib/ebike/`, `src/ebike/`, `tests/lib/ebike/`, `tests/ebike/`, the e-bike tests of
`tests/sim/cyphal_drive`) and compiled for the BCC (`unimoc.app.build.ebike`). Nothing is
verified on the BCC hardware or on a real KT-LCD8.

## 1. Goal

The battery case controller (BCC) drives a hub or mid-drive motor of an e-bike.
Besides the drive, the BCC has the connectors of an E-Rider T9N torque and
cadence sensor in the bottom bracket and of a KT-LCD8 display. The assist is
computed on the BCC itself: pedal torque at low speed, pedal power at higher
speed, with a gain that rises exponentially with the pedal power. The base
assist level comes from the display.

## 2. Decisions

- **One firmware, one extra layer.** The drive (FOC, startup, settings,
  Cyphal) stays the motor node. The e-bike layer runs in Zephyr threads and
  hands its torque setpoint to the outer loop in process, through the same
  setpoint path the Cyphal setpoint uses. No Cyphal round trip to the own MCU.
  Cyphal stays for telemetry, registers and cymon.
- **Portable logic in `lib/ebike/`** (no Zephyr, ztest coverage), drivers in
  `hardware/zephyr_bcc/`, fake pedal and display inputs for `native_sim`.
  `lib/motor_control` is not changed.
- **Nothing in the hot path.** The crank torque is already part of the
  injected ADC sequence (PB14, ADC4, `crank-torque`); the thread side
  averages it. The cadence pulse (PA5, `PULSE_CRK`) is captured by a
  normal-priority GPIO interrupt with a timestamp, like the Hall edges.
- **The display stays a replaceable driver.** The KT-LCD8 (Kunteng UART
  protocol) first, behind an interface of telemetry out and user input in
  (assist level, light, wheel size, speed limit, current limit). The protocol
  is implemented from its frame layout; the EBiCS firmware is a reference only,
  it carries no license file, so its code is not copied.

## 3. Sensor

E-Rider T9N (ebikes.ca product page):

| | |
|---|---|
| Torque | 1.5 V to 3.5 V, 70 Nm/V, both pedals |
| Cadence | single wire, 36 poles per revolution (10 degrees); a vendor text gives 18, so the pulse count is a parameter |
| Direction | not available (single wire), so no back-pedal detection |

BCC wiring (schematic, Tecnologic/battery-case-controller, connector J38 CRANK): pin 1 is +12 V (the sensor
accepts 5 to 15 V), pin 2 the cadence pulse (a 1k pull-up to 3V3, 3k3 series, 2n2: PA5), pin 3 the torque
(R84 || R85 in series with R86 to ground: a 2/3 divider, so 3.5 V at the sensor is 2.3 V at the ADC; PB14, a
channel of the injected sequence), pin 4 ground. `analog_scaling.hpp` undoes the divider and converts to torque
above the nominal 1.5 V rest voltage (it ignored the divider before). The offset of the individual sensor is
trimmed by the observer.

## 4. Pedal observer (`lib/ebike/src/pedal_observer.hpp`)

- **Cadence:** Kalman filter on crank angle and cadence. Each pulse is an angle
  measurement (pulse index times pulse angle, modulo one revolution). The
  platform hands in the age of each pulse, so the estimate does not depend on
  the step jitter. An overdue pulse limits the cadence, so it decays during a
  stop; without pulses for 0.5 s the crank counts as standing still.
- **Torque:** the samples are averaged per pulse interval; the mean over the
  last half revolution cancels the twice-per-revolution pedal-stroke ripple at
  any cadence (delay up to half a stroke, in angle, not time). A plain
  low-pass (20 ms) is provided for display and diagnosis.
- **Power:** mean torque times cadence.
- **Offset:** follows the reading while the crank stands still, only within
  3 Nm of the current offset (a foot on a pedal is not a zero), time constant
  30 s after 1 s of standstill. The value can be stored and restored
  (`SetZeroOffset`); storing it as a setting is pending.

## 5. Assist law (`lib/ebike/src/assist_controller.hpp`)

With the pedal torque after a deadband, the cadence and the pedal power
`P_p = torque * cadence`:

- Assist ratio `g = min(g_level * exp(P_p / P_scale), g_max)`, with a base
  ratio per display level (level 0 is off).
- Motor torque `g * torque * rho`, `rho = min(rho_max, cadence / motor_speed)`.
  At low speed the motor torque follows the pedal torque (`rho_max`, "torque
  ratio"); above the crossover speed `cadence / rho_max` the motor power equals
  `g` times the pedal power. The two regimes join without a step.
- Fades out over a speed range below the display's speed limit, limited to
  the torque limit (display current limit and drive limits), ramps slowly up
  (0.5 s to the limit) and quickly down (0.15 s).
- Zero without pedaling, below the minimum cadence, at level 0, and without a
  valid speed or torque limit.

The defaults are a starting point (level gains 0, 0.3, 0.6, 1.0, 1.5, 2.0;
`P_scale` 300 W; maximum ratio 4; torque ratio 0.5; deadband 2 Nm), none is
tuned. They are the `unimoc.ebike.*` registers (README, "E-bike"). The speed
limit of the display (km/h) becomes a motor shaft speed with the wheel
circumference and the gear ratio. The display's current limit code is not used
yet: the torque limit is the drive's own window (motor and battery limits).

## 6. Display protocol (`lib/ebike/src/kunteng_protocol.hpp`)

The Kunteng protocol of the KT-LCD3/5/8 family, implemented from its frame
layout (the EBiCS firmware was read as a reference, nothing was copied).

- Display to controller, 13 bytes about every 100 ms: assist level, light,
  wheel size code, speed limit, motor characteristic, power assist mode, current
  limit code. End bytes `0x32` or `0x37`, then `0x0E`. Checksum: xor of all bytes
  except byte 5, xored with a constant (10 for most displays; the decoder adopts
  another constant after two frames in a row show it).
- Controller to display, 12 bytes as the answer to each valid request: battery
  bars, nominal voltage, wheel period (the display derives the speed with its
  own wheel size, so the wheel size setting of the display and the
  controller's circumference have to match), error code, mode bits, battery
  current in quarter amps, motor temperature minus 15.
- `DisplayDecoder` finds frames in the UART byte stream with a sliding 13-byte
  window, so it needs no DMA length tricks and resynchronizes by itself.

Unverified against a real KT-LCD8: the temperature offset, the wheel size
code mapping (kept raw), the error codes (any fault reports code 1) and the
checksum constant of this display.

## 7. Integration

**Engagement on demand** (`src/ebike/ebike_control.*`, `DriveNode::SetLocalControl()`): the e-bike asks for
the drive while the crank turns at a display level above 0 and for the engage hold (2 s) afterwards. Its
torque replaces the other setpoints while the drive is engaged (limited by `i_max` and the outer loop). The
readiness `ENGAGED` of the bus is ignored; `SLEEP` or `STANDBY` release the drive and latch the e-bike off until
the pedals stood still for the re-arm time (1 s). A trip, a power stage fault or a refused engage latch it the
same way, so a persistent fault cannot re-engage the drive in a loop. A silent display (2 s) counts as level 0,
so there is no assist without a display. The assist torque ramps up in 0.5 s and down in 0.15 s.

**BCC hardware** (connector J32 Display: pin 1 +BATT, 2 DCDC_VIN, 3 GND, 4 DISP_TX, 5 DISP_RX):

- DISP_TX is PB10, USART3_TX (Zephyr UART driver, interrupt driven, 9600 8N1).
- DISP_RX is PB15, which has **no UART receive function** on the STM32G473, and PB11, the receive pin of USART3,
  is the second break input. The edges of PB15 are captured by TIM15 channel 2 (both edges, an interrupt each,
  stamped with the DWT counter) and `EdgeUartDecoder` rebuilds the bytes in the Cyphal thread. EXTI is not used
  for it: EXTI line 15 is also the line of Hall C (PC15) in the Hall fusion plan. A swap of the two display
  lines on a later board revision would give a normal UART.
- The cadence pulse PA5 is a GPIO interrupt (falling edge) that stamps the DWT counter.
- The crank torque is read from the injected ADC results (a single reading per update, the observer averages).
- Not verified on the board: the TIM15 both-edge capture (CC2NP on TIM15), the logic levels of the display
  (5 V on DISP_RX?), the interrupt latencies.

**Switch.** `CONFIG_UNIMOC_EBIKE` is off on the BCC by default, because the assist parameters are untuned and
the control engages the power stage by itself; enable it in `app/boards/bcc.conf` for the bench
(`unimoc.app.build.ebike` builds it). native_sim has it on.

## 8. Steps

1. Torque scaling and zero offset. **Done** (scaling and the input divider fixed, offset trim in the observer).
2. `PedalObserver` with tests. **Done.**
3. Assist law with tests. **Done.**
4. Kunteng frame codec with tests. **Done.**
5. BCC drivers, devicetree nodes, the drive hook, the registers and the native_sim rider. **Done**, except
   the open points below.

Open: tuning on the bench and on the road; the display on a real KT-LCD8; the board check of the TIM15 capture and
the logic levels; brake, light and throttle inputs (no pins on the BCC connectors); the `unimoc.sim.pedal_*`
registers for pytest and yakut; persisting the learned torque offset; the display's current limit code.
