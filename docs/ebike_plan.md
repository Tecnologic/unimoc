# Plan: Pedal Assist on the Battery Case Controller

Status: 2026-10-23, draft. Steps 1 to 4 are implemented and tested under twister on
`native_sim` (`lib/ebike/`, `tests/lib/ebike/`). Nothing is verified on the BCC
hardware or on a real KT-LCD8.

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

Open points: the T9N output reaches 3.5 V, the BCC ADC reference is 3.3 V
(check the input network of PB14 in the schematic); the sensor needs 5 to 15 V
(check the supply of the connector).

`analog_scaling.hpp` converts counts to torque above the nominal 1.5 V rest
voltage. The offset of the individual sensor is trimmed by the observer.

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
tuned. They become settings (registers) with the integration. Open: the
conversion of the display's speed limit and the torque limit to the motor
shaft, which needs the wheel circumference and the gearing.

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
code mapping (kept raw), and the checksum constant of this display.

## 7. Steps

1. Torque scaling and zero offset. **Done** (scaling fixed, offset trim in the observer).
2. `PedalObserver` with tests. **Done.**
3. Assist law with tests. **Done.**
4. Kunteng frame codec with tests. **Done.**
5. BCC drivers (cadence interrupt, display UART, devicetree nodes), the
   setpoint hook-up, the register set for the parameters above, and fake inputs
   for `native_sim`. **Open.**
