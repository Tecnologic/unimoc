# Plan: Pedal Assist on the Battery Case Controller

Status: 2026-10-23, draft. Steps 1 and 2 are implemented and tested on the host
(`lib/ebike/`, `tests/lib/ebike/`). Nothing is verified on the BCC hardware.

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

## 5. Assist law (next step)

- Low speed: motor torque follows the pedal torque times the level gain.
- Higher speed: motor power follows the pedal power; the torque is that power
  divided by the motor speed. Smooth crossover between both.
- Gain: `g_base(level) * exp(P_pedal / P_scale)`, clamped.
- Limits: display speed and current limits, the battery and drive limits, a
  torque ramp and a deadband. The assist ends a few hundred milliseconds after
  the pedaling stops.
- Open: `P_scale`, the crossover speed, the level table.

## 6. Steps

1. Torque scaling and zero offset. **Done** (scaling fixed, offset trim in the observer).
2. `PedalObserver` with tests. **Done.**
3. Assist law with tests.
4. Kunteng frame parser and encoder with tests.
5. BCC drivers (cadence interrupt, display UART, devicetree nodes) and the
   setpoint hook-up; fake inputs for `native_sim`.
