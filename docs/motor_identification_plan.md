# Motor Identification Plan

Status: plan (2026-10-29), nothing implemented. The open decisions are in
section 9.

UNIMOC needs the electrical and mechanical parameters of the connected
motor: the current loop, the observers, the field weakening and the speed
loop are computed from them. Today they are entered by hand (registers
`unimoc.motor.*`, defaults of a hoverboard motor). The old firmware
([unimoc_old_software](https://github.com/Tecnologic/unimoc_old_software),
`src/measurement.cpp`, `docs/*.m`) measured R, L and psi; this plan carries
those algorithms over, fills the gaps and runs them as an assisted flow like
the hardware startup aid (README "Hardware Bring-Up Guide"), driven from
cypilot.

## 1. Scope

- PMSM first (the motor type the runtime runs today). ASM and EESM follow
  with their runtime paths (TODO.md, "Motor types"); section 4.9 names what
  they add.
- Commissioning, not online adaptation: the drive measures once, the
  operator reviews the results and applies and stores them.
- The flow starts after the startup aid has run (calibrated current and
  DC-link sense, a found ADC trigger) on a drive that is not engaged.

## 2. What the drive needs

| Parameter | Register | Used by | Measured by |
|---|---|---|---|
| Stator resistance R_s | `unimoc.motor.stator.R` | current PI (ki), decoupling, flux observer | M1 |
| Dead-time voltage | `unimoc.control.dtc.dead_time` (as a time) | dead-time compensation | M1 (intercept) |
| L_d, L_q (and the mean `stator.L`) | `unimoc.motor.pmsm.L_d`, `L_q`, `unimoc.motor.stator.L` | current PI (kp), decoupling, MTPA, HFI, flux observer | M2 |
| Flux linkage psi_PM | `unimoc.motor.pmsm.flux_pm` | torque constant, flux observer, field weakening, mechanical observer | M3 |
| Pole pairs p | `unimoc.motor.pole_pairs` | everything mechanical | entered, checked by M4 |
| Hall offset, Hall order | `unimoc.observer.hall.offset` | Hall fusion | M5 |
| Inertia J, friction | `unimoc.motor.mechanics.J` (friction: new, optional) | mechanical observer, speed PI | M6 |
| Current and speed PI gains | `unimoc.control.current.kp_d` ..., `unimoc.control.speed.kp`, `.ki` | the loops | derived (M7) |

`unimoc.motor.balance.gain_a/b/c` (`phase_balance_*`, "identified by
CMD_MEASURE_BALANCE") are unused; the startup aid now measures the gain of
each phase (`unimoc.startup.gain_a/b/c`, applied in the control cycle). See
decision 2.

## 3. The old firmware

`management::thread` ran the measurements as states of its 1 ms thread
(`MEASURE_RS`, `MEASURE_LS`, `MEASURE_PSI`, flag `measure::all`), each a
small state machine in `measurement.cpp`:

- **R** (`measurement::r`): open-loop voltage on the d-axis at the angles 0,
  +120 and -120 degrees, raised by 100 mV every 25 ms until i_d reached 75 %
  of the current limit (or u > 50 % of the battery voltage); R = mean(u/i)
  over the three angles. Then kp = L_d / (2 T_f) and T_n = L_q / R_s.
  `docs/linear_reg_resistance.m` shows the better estimate: a regression of
  several (i, u) points, R from the slope and the dead-time voltage from the
  intercept (`dt = 2 a / U_dc`). The single point u/i of the firmware carries
  the dead-time voltage in R.
- **L** (`measurement::l`): an open-loop voltage vector rotating at 800 Hz
  (u_q raised in 100 mV steps until |i| > 3 A), Goertzel on i_d at DC and at
  twice the frequency: L_d = u / (w (I_0 + I_2)), L_q = u / (w (I_0 - I_2))
  (`docs/l_measure_goerzel.m` simulates it). Marked "TODO needs rework: phase
  lag of current vector is motor dependent": R is neglected and the phase of
  the 2f component is not used, so the axis assignment is a guess.
- **psi** (`measurement::psi`): current control with i_q = 50 % of the limit
  at an open-loop angle whose speed rose by 1 rad/s every 100 ms (I/f) until
  the voltage exceeded 25 % of the battery or the speed 50 % of its limit;
  psi = |u - R i - j w L i| / w at that one point; then a slow ramp down.
- The Hall flag `measure::hall` existed but was never implemented; inertia
  was not measured.

What carries over: the sequence (R, then L, then psi), the three angles of
R, the Goertzel evaluation, the I/f spin-up, the stop handling (every
measurement ends in a clean state on a stop request). What changes: regression
instead of single points, closed-loop current where the old code ramped open-
loop voltage, the evaluation in a thread instead of the 1 ms loop, results
reviewed before they are applied.

## 4. Measurements

Each measurement has preconditions the operator confirms (section 6), a
limit on current, voltage and speed, plausibility checks, and a simulation
test against the plant truth (`lib/sim`, section 7).

### 4.1 M1: stator resistance and dead-time voltage (standstill)

- Current control at a forced angle (section 5.1), i_q = 0, i_d at five
  levels from 10 % to 50 % of `motor.limits.i_max` (both signs), each held
  until the PI has settled (about 5 tau = 5 L/R), then the mean of u_d and
  i_d over 50 ms.
- At the angles 0, 120 and 240 degrees, as the old code: each phase carries
  the full current once, so a bad connection or a broken strand shows as one
  outlier (the check: the three slopes within 10 %).
- Regression u_d = R_s i_d + U_dt sign(i_d): R_s from the slope, the
  dead-time voltage U_dt from the step at zero (`linear_reg_resistance.m`);
  t_dead = U_dt / U_dc times the PWM period, written to the dead-time
  compensation.
- The rotor aligns with the d current at the first level (a d current
  produces no torque once aligned); a PMSM needs no shaft lock.

### 4.2 M2: L_d and L_q (standstill)

- Rotor held aligned by a DC d current (20 % of the limit) at a forced angle.
- A sinusoidal voltage of f_inj (default 500 Hz: 32 control cycles per period) on the
  d-axis, then on the q-axis, amplitude raised until the AC current reaches
  10 % of the limit. The q-axis injection averages to zero torque; the d bias
  holds the rotor.
- Goertzel at f_inj on the recorded u and i of the axis (section 5.2): the
  complex impedance Z = U / I gives R from the real and w L from the imaginary
  part, so neither R nor the phase lag of the old method is assumed.
  L_d from the d injection, L_q from the q injection; `stator.L` is their mean.
- Optional: L_d at three d bias levels (saturation), reported, not stored.
- Check: R from M2 within 20 % of M1 (both measure the same winding).

### 4.3 M3: flux linkage (shaft free)

- I/f spin-up: current control at a forced angle whose speed rises along a
  ramp, i_q = 30 % of the limit, up to three speeds (20, 35 and 50 % of the
  speed limit, at most 25 % of the DC-link voltage as back-EMF).
- At each speed, held steady: the back-EMF in the forced frame
  e = u - R_s i - j w L i (L_d = L_q assumed for the vector; the error of a
  salient motor is small at the low load angle of I/f) and psi = |e| / w.
  The regression over the three speeds (|e| over w) cancels a constant error.
- Alternative with the phase-voltage sense (decision 4): spin up, switch
  the outputs off and measure the line-to-line back-EMF directly from `va`,
  `vb`, `vc` while the shaft coasts; no R, L or current involved.
- Then the rotor is slowed with the ramp and the outputs stop at standstill.
- Check: the loaded speed follows the forced speed (a stall shows as a back-
  EMF far below the expectation and aborts).

### 4.4 M4: pole pairs (shaft free, operator assisted)

- The pole pairs are entered (data sheet or counted magnets). The check: a
  slow forced rotation of exactly p electrical revolutions at a d current;
  the operator confirms in cypilot that the shaft turned exactly once (a mark
  on the shaft). With Hall sensors the drive also counts 6 p Hall edges.

### 4.5 M5: Hall offset and order (shaft free)

- A slow forced rotation (2 electrical revolutions per second), d current at
  30 % of the limit, forward then backward, two revolutions each way.
- The Hall events with their timestamps against the forced angle: the six
  edge angles of each direction; the mean of both directions cancels the lag
  of the rotor behind the forced angle. `hall.offset` follows; the order of
  the codes checks the wiring (a swapped pair of Hall lines or of phases
  shows as the reversed sequence and is reported, not stored: the code table
  of `HallDecoder` is fixed).
- Checks: every edge seen, each within 10 degrees of the 60 degree grid.

### 4.6 M6: inertia and friction (shaft free)

- With the observer running (psi known from M3, or Hall from M5) and the
  speed loop off: torque steps of +T, 0 and -T (T from 20 % of the current
  limit) between two speeds, and a coast-down with zero torque.
- Least squares on w(t): J dw/dt = T - B w - T_c sign(w); the coast-down gives
  B and T_c, the steps J. The speed stays below 30 % of its limit.
- Check: J of the acceleration and of the deceleration within 15 %.

### 4.7 M7: derived settings

From the measured values, as the defaults of `nvm_settings.hpp` already do
it from the hoverboard motor:

- current PI: kp = L w_c, ki = R w_c per axis with the current bandwidth w_c
  (today the constant `kHoverboardCurrentBandwidth`, 2000 rad/s, behind the
  defaults; a new register `unimoc.control.current.bandwidth`);
- speed PI from J and the speed bandwidth;
- the torque constant follows from psi and p (no register).

They are results of the run like the measured values and are applied with
them (section 5.4).

### 4.8 Not measured

Temperature dependence of R and psi, magnetic saturation beyond the M2
option, cogging, the encoder (none on the BCC yet).

### 4.9 ASM and EESM (later)

ASM: R_s as M1, the leakage inductance from M2 (at standstill the rotor
short-circuits it), R_r and L_m from a no-load run and a locked-rotor
injection. EESM: the field winding (R_f, L_f from a step of the fourth half
bridge), L_m from the back-EMF at a known field current. Both wait for the
runtime paths.

## 5. Firmware

### 5.1 Forced angle

The measurements need current control at an angle the FSM gives instead of
the observer's: a fixed angle (M1, M2), a ramp (I/f: M3 to M5). The angles
of the control cycle come from `SlowUpdate` (the observer interrupt, not zero
latency, section "Observer" of `docs/hotpath_plan.md`), so the forced angle is
a mode of `SlowUpdate`: it writes theta and omega of the forced angle into the
angle buffers instead of the mechanical observer's. The zero-latency control
cycle does not change. The observer keeps running in the background (M6 needs
it, and its estimate can be compared with the forced angle).

### 5.2 Recording

The estimators need per-cycle data (16 kHz): means of u and i (M1, M3), the
Goertzel terms of u and i at f_inj (M2), w(t) (M6). A buffer of 2048 cycles
of i_dq, u_dq, angle and DC link is 48 KB, too much next to the cymon buffer
in the 128 KB of the STM32G473. So `SlowUpdate` accumulates them online (a few
multiply-adds per cycle in the observer interrupt, not at zero latency) into a
small result block the FSM reads in the Cyphal thread once the window is
complete; M6 decimates w(t) to 1 kHz (2 s in 8 KB). cymon traces of the
same signals show the run in cypilot's scope.

M2's injection is the one change of the control cycle: the HFI injection
already adds a voltage per duty entry (`hfi.GetInjectionVoltage()` in
`OnControlCycle()`); M2 adds a sine of the FSM's amplitude, axis and
frequency on the same path, from a table of `RotorAngle` (no libm), and stays
within the zero-latency rules. With four duty entries per cycle a sine of up
to f_pwm / 8 is possible; 500 Hz leaves 32 cycles per period for Goertzel.

### 5.3 The identification FSM

`src/identification/` (`MotorIdentification`), in the style of `HwStartup`:
steps, results, one sample per FSM run in the Cyphal thread, the outputs
released by the FSM and never while engaged, the overcurrent limit of the run,
abort on run = 0, on the E-stop and on a trip. The estimators (regression,
Goertzel, the least squares of M6) are portable code in
`lib/motor_control/src/identification/` with ztests.

Registers (`unimoc.identify.*`, not persistent):

| Register | Type | Meaning |
|---|---|---|
| `run` | natural8 | 1 runs, 0 aborts (as the startup aid) |
| `select` | natural8 (bit mask) | M1 .. M6 to run (default all that the preconditions allow) |
| `step` | natural8 | the step; write 1 continues (the operator confirmations of section 6) |
| `results` | natural8[] | per step: 0 not run, 1 passed, 2 failed a check |
| `result.R`, `result.L_d`, `result.L_q`, `result.flux_pm`, `result.J`, `result.friction`, `result.hall_offset`, `result.dead_time` | real32 | the measured values, base units |
| `result.current_kp` ... | real32 | the derived gains (M7) |
| `apply` | natural8 | write 1: the results to the settings (validated, one transaction); then Store |
| `pole_pairs_turned` | natural8 | M4: the operator's confirmation |

The ExecuteCommand codes `CMD_MEASURE_RS/LD_LQ/PSI/BALANCE` (0x0100 ..
0x0103) and the result subjects 208 .. 211 (`uavcan.pub.meas_*`) of
`lib/cyphal/cyphal_interface.hpp` are an older design that nothing
implements; decision 1.

### 5.4 Results and apply

The run writes nothing to the settings by itself (unlike the startup aid at
DONE): the operator compares the results with the present settings in
cypilot and applies them (`apply` = 1), then stores them. Applying while
engaged waits for the disengage, as every motor setting does.

## 6. The flow in cypilot

A "Motor identification" card next to the startup aid (the Control view, or
a "Commissioning" view holding both), on the registers of section 5.3, with
the same safety rules: the operator's control lease, stop all, the E-stop and
a lost lease abort the run, the abort is open to every client, the
preconditions are confirmed per step. `cypilot-ctl identify NODE [run|abort|
status|apply]` is the same at the terminal.

1. **Before:** the startup aid has run on this drive (its calibration is
   stored; cypilot reads `unimoc.startup.results` and the calibration
   registers), the motor type is PMSM, the nameplate values are entered:
   pole pairs, current limit, speed limit (the measurement limits follow).
2. **Standstill (M1, M2):** "The motor may jerk once while it aligns. The
   shaft need not be locked." Live plot of i_d and u_d; the regression of M1
   as a chart.
3. **Free shaft (M3 to M6):** "The motor will spin up to <speed>. Remove the
   load, keep the area clear." A separate confirmation before the first
   rotation, like CONNECT_MOTOR of the startup aid. M4 asks "did the shaft turn
   exactly once?" with Yes, No and Repeat.
4. **Review:** a table of each value with the present setting, the
   difference and the checks; the derived gains. Apply, then Store, and
   "save as profile" (the profile of a motor type, `cypilot` profiles).
5. **Several drives:** identify one drive per motor type, copy the profile to
   the others (the profile copy of cypilot), run M5 (Hall) on each.

## 7. Tests

- `tests/lib/motor_control`: the estimators on synthetic data (regression with
  noise and a dead-time step, Goertzel of a known impedance, least squares of
  a known J and friction).
- `tests/sim/identification`: each measurement against `MotorSimulation` with
  known R, L_d != L_q, psi, J, friction and Hall offset: within 2 % (R, L,
  psi), 5 % (J), 2 degrees (Hall). The inverter model of `lib/sim` has no dead
  time yet; M1's dead-time estimate needs it (a dead-time voltage in
  `InverterModel`).
- `tests/sim/cyphal_drive` and the pycyphal integration test: the registers,
  run, abort, apply; cypilot: unit tests on the demo device, a sim test on the
  real simulation (as the startup aid).

## 8. Milestones

| | Content | Needs |
|---|---|---|
| I0 | Forced angle in `SlowUpdate`, recording buffer, FSM skeleton with run/step/abort/results, registers, cypilot card and CLI with the confirmations | - |
| I1 | M1 R_s and dead-time voltage; dead time in the sim inverter | I0 |
| I2 | M2 L_d, L_q | I0 |
| I3 | M3 psi (I/f), M4 pole-pair check | I1, I2 |
| I4 | M5 Hall offset | I3 (the I/f rotation) |
| I5 | M6 J and friction | I3 |
| I6 | M7 derived gains, apply, review in cypilot, profile | I1 .. I5 |
| I7 | Bench: the hoverboard motor (data sheet values) and the 4850 motor; compare with an LCR meter (L), a 4-wire measurement (R), a coast-down on the scope (psi) | board access |

## 9. Decisions

1. **Registers instead of ExecuteCommand.** Recommended: the register FSM of
   section 5.3, like the startup aid (one flow, steps with confirmations,
   readable state for several clients); remove `CMD_MEASURE_*` and the
   `uavcan.pub.meas_*` ports 208 .. 211 from `cyphal_interface.hpp`.
2. **`phase_balance_*`:** remove the registers (the startup aid's per-phase
   gains replace them), or keep them as a second, motor-side balance.
   Recommended: remove.
3. **Pole pairs:** entered and checked (M4, recommended), or counted with
   the Hall sensors alone.
4. **Phase-voltage sense for psi:** use `va`, `vb`, `vc` of the BCC for a
   coast-down back-EMF (more exact, needs the voltage channels in the
   platform interface and their calibration) or stay with the I/f estimate.
   The 4850 board's voltage sense is open (TODO.md, cart system).
5. **Apply:** reviewed in cypilot and applied by the operator (recommended),
   or written at the end of the run like the startup aid.
