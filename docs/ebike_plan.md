# Plan: Pedal Assist

> **2026-11-03:** no board carries the e-bike inputs. The e-bike control runs
> in the simulation only; `CONFIG_UNIMOC_EBIKE` depends on `native_sim`. A
> drive with a crank sensor and a display port adds an `ebike_io.cpp` to its
> STM32H7 platform. The first driver (cadence capture, display line, UART) of
> the removed G4 board is in the git history before 2026-11-03.

Status: 2026-10-23. Steps 1 to 5 are implemented, tested under twister on `native_sim`
(`lib/ebike/`, `src/ebike/`, `tests/lib/ebike/`, `tests/ebike/`, the e-bike tests of
`tests/sim/cyphal_drive`; the pytest scenario `unimoc.app.drive.cyphal_integration.ebike` with the simulated rider
on the bus, 2026-11-08). Nothing is verified on hardware or on a real KT-LCD8.

## 1. Goal

A drive runs a hub or mid-drive motor of an e-bike. Besides the motor, the
drive reads an E-Rider T9N torque and cadence sensor in the bottom bracket
and talks to a KT-LCD8 display. The assist is computed on the drive itself: pedal torque at low speed, pedal power at higher
speed, with a gain that rises exponentially with the pedal power. The base
assist level comes from the display.

## 2. Decisions

- **One firmware, one extra layer.** The drive (FOC, startup, settings,
  Cyphal) stays the motor node. The e-bike layer runs in Zephyr threads and
  hands its torque setpoint to the outer loop in process, through the same
  setpoint path the Cyphal setpoint uses. No Cyphal round trip to the own MCU.
  Cyphal stays for telemetry, registers and cymon.
- **Portable logic in `lib/ebike/`** (no Zephyr, ztest coverage), drivers in
  the platform (`ebike_io.cpp`), fake pedal and display inputs for `native_sim`.
  `lib/motor_control` is not changed.
- **Nothing in the hot path.** The crank torque is read by the thread side
  (on Jacobi a spare ADC input such as `AIN_ACC`, PC0, would take it); the
  cadence pulse is captured by a normal-priority GPIO interrupt with a
  timestamp, like the Hall edges.
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

Wiring: pin 1 +12 V (the sensor accepts 5 to 15 V), the cadence pulse (pull-up to 3V3), the torque (through a
2/3 divider, so 3.5 V at the sensor is 2.3 V at the ADC), ground. `analog_scaling.hpp` undoes the divider and
converts to torque above the nominal 1.5 V rest voltage. The offset of the individual sensor is trimmed by the
observer.

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
  30 s after 1 s of standstill. The setting `unimoc.ebike.torque_offset` is
  the initial offset; the drive stores the learned one (2026-11-08,
  `EbikeControl::OffsetToStore()`): at standstill, while the drive is not
  requested and the power stage is off, once it moved 0.1 N m from the stored
  value and at most once a minute (flash wear). It saves to flash only when no
  other register change is unsaved; otherwise the offset waits in RAM for the
  next `STORE_PERSISTENT_STATES`.

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

**Hardware:** the display needs a UART (9600 8N1). Without a free UART receive pin, `EdgeUartDecoder`
rebuilds the bytes from timestamped edges of a GPIO in a thread. Not verified: the logic levels of the display
(5 V on the receive line?), the interrupt latencies.

**Switch.** `CONFIG_UNIMOC_EBIKE` is off by default, because the assist parameters are untuned and the control
engages the power stage by itself. The simulation app keeps it off, so that the pytest integration tests can
still engage the drive over the bus; the e-bike tests of `sim_cyphal_drive` attach the control themselves, and
the pytest scenario `unimoc.app.drive.cyphal_integration.ebike` builds the simulation with it: the simulated rider and
display are the volatile registers `unimoc.sim.pedal_cadence`, `pedal_torque`, `display_on`, `display_level`,
`display_max_speed`, `display_light` and `display_frames` (README, "E-bike"), for pytest and yakut
(`tests/integration/cyphal_sim/test_ebike_sim.py`, marker `ebike`).

## 8. Steps

1. Torque scaling and zero offset. **Done** (scaling and the input divider fixed, offset trim in the observer).
2. `PedalObserver` with tests. **Done.**
3. Assist law with tests. **Done.**
4. Kunteng frame codec with tests. **Done.**
5. Drivers (removed with the G4 board), the drive hook, the registers and the native_sim rider. **Done**, except
   the open points below.

Open: tuning on the bench and on the road; the display on a real KT-LCD8; a board with the e-bike inputs and its
`ebike_io.cpp`; the logic levels; brake, light and throttle inputs; the display's current limit code. Done
2026-11-08: the `unimoc.sim.pedal_*` and `unimoc.sim.display_*` registers with the pytest of the simulated rider,
the learned torque offset stored in `unimoc.ebike.torque_offset`.
