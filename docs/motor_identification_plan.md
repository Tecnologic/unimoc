# Motor Identification Plan

Status: plan (2026-10-29), nothing implemented. Decisions 1 to 5 of
section 9 were taken on 2026-10-29; decision 6 (the startup aid on the same
bus contract) is open.

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

The phase-balance registers `unimoc.motor.balance.gain_a/b/c` were never
applied and are removed (decision 2): the startup aid measures the gain of
each phase (`unimoc.startup.gain_a/b/c`, applied in the control cycle).

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

### 4.3 M3: flux linkage by coast-down (shaft free)

The fixed-angle (I/f) estimate of the old firmware was inaccurate on the
bench: it rests on R and L and on the load angle of a rotor that lags the
forced angle (decision 4). M3 measures the back-EMF itself instead:

- Spin-up: I/f (current control at a forced angle with a speed ramp,
  i_q = 30 % of the limit) to the speed where the line-to-line back-EMF is
  about 25 % of the DC link, at most 50 % of the speed limit.
- Coast: the outputs off (all switches open, `PowerStageInterface::Disable()`;
  on the BCC MOE = 0 drives the idle levels OIS = 0 of active-high outputs, so
  both gates of each leg are low: check the gate-driver polarity on the bench,
  a driven low side would short the motor instead of letting it coast).
  The motor terminals then carry the back-EMF alone; the phase-voltage sense
  (`va`, `vb`, `vc` of the BCC, section 5.2) samples them at the PWM rate as
  before (the timer runs on, only the outputs are off).
- Per electrical period: the frequency from the zero crossings of the line-to-
  line voltages (no pole pairs, no speed estimate needed) and the amplitude of
  the line-to-line fundamental (Goertzel at the measured frequency over whole
  periods). psi = U_ll,peak / (sqrt(3) w_e), from each period while the shaft
  slows; the mean over the periods above 10 % of the start speed is the result,
  their spread the check (within 3 %).
- The speed decay of the same coast is the friction data of M6.
- Below the threshold speed the run waits for standstill (the outputs stay off).
- Check: no current flows while coasting (the body diodes conduct only above
  the DC link: the spin-up speed keeps the back-EMF at 25 % of it).

### 4.4 M4: pole pairs, measured or entered and checked (shaft free, operator assisted)

Both ways (decision 3); the operator chooses in cypilot:

- **Measured:** the operator marks the shaft. The drive turns a slow forced
  angle (one electrical revolution per second, d current at 30 % of the
  limit) and counts the electrical revolutions; the operator confirms when the
  mark is back (cypilot sends the confirmation, section 5.3), and p is the
  count rounded. With Hall sensors the drive also counts the Hall edges
  (6 p per mechanical revolution) and reports a mismatch.
- **Entered and checked:** the operator enters p (data sheet, counted
  magnets); the drive turns exactly p electrical revolutions and stops; the
  operator confirms that the shaft turned exactly once (Yes, No, Repeat).

Both end with p in the results; apply writes `unimoc.motor.pole_pairs`.

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

- Friction from the coast-down of M3: the speed from the back-EMF frequency,
  least squares on J dw/dt = -B w - T_c gives B / J and T_c / J.
- J from torque steps: with the observer running (psi known from M3, or Hall
  from M5) and the speed loop off, +T and -T (T from 20 % of the current limit)
  between two speeds; J dw/dt = T - B w - T_c sign(w) with B / J and T_c / J
  from the coast gives J, then B and T_c. The speed stays below 30 % of its
  limit.
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

### 5.2 Recording and the phase-voltage sense

The estimators need per-cycle data (16 kHz): means of u and i (M1, M3), the
Goertzel terms of u and i at f_inj (M2), w(t) (M6). A buffer of 2048 cycles
of i_dq, u_dq, angle and DC link is 48 KB, too much next to the cymon buffer
in the 128 KB of the STM32G473. So `SlowUpdate` accumulates them online (a few
multiply-adds per cycle in the observer interrupt, not at zero latency) into a
small result block the FSM reads in the Cyphal thread once the window is
complete; M6 decimates w(t) to 1 kHz (2 s in 8 KB). cymon traces of the
same signals show the run in cypilot's scope.

**Phase voltages (M3).** The BCC samples `va`, `vb`, `vc` with every
conversion (`foc_stm32g4::RawSamples`), but the platform interface does not
pass them on. M3 adds them to `ControlCycleSamples` (scaled like the DC link,
33k/1k) and `SlowUpdate` evaluates them during the coast (zero crossings,
Goertzel per period). Their gain is calibrated by the startup aid: in
DUTY_FORCE_LOW/MID/HIGH the phase voltage must be the duty times the
calibrated DC link, which also turns the manual scope check of those steps
into a measured one (`unimoc.startup.gain_va/vb/vc`, new persistent
registers). The simulation needs the terminal voltages of a motor with open
outputs (`lib/sim`: back-EMF plus the star point, the inverter without
conduction while the back-EMF stays below the DC link). The 4850 board's
voltage sense is open (TODO.md, cart system).

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
abort on the abort command, on the E-stop and on a trip. The estimators
(regression, Goertzel, zero crossings, the least squares of M6) are portable
code in `lib/motor_control/src/identification/` with ztests.

### 5.4 Bus interface: commands, a status message and registers

How does a tool know that a step has measured, waits for the operator, has
finished or has failed? Registers alone (as the startup aid until C0) give the
state only when polled, cannot tell "measuring" from "waiting for the
operator", and answer a refused write with the unchanged value, which looks
like an accepted one (the startup aid's former `step` register answered with
the step, not with the written 1). The `CMD_MEASURE_*` commands of the older
design answered "started" and published one result message at the end, but
had no progress, no operator input and nothing for a client that joins late.
So each of the three Cyphal mechanisms does what it is good at (decision 1):

**Actions: `uavcan.node.ExecuteCommand`** (vendor codes 0x0200 .. 0x02FF,
replacing `CMD_MEASURE_RS/LD_LQ/PSI` 0x0100 .. 0x0102). The answer says at
once whether the action was taken, with the reason in `output` (46 bytes):
SUCCESS; BAD_STATE (engaged, another procedure runs, a precondition is
missing); BAD_PARAMETER (a reading out of range, an unknown confirmation);
FAILURE (the power stage was not released).

| Code | Command | Parameter |
|---|---|---|
| 0x0200 | START_STARTUP_AID | - (decision 6, done in C0) |
| 0x0201 | START_IDENTIFICATION | the measurements, e.g. `M1,M2,M3,M4,M5,M6`; `p=15` for M4 "entered" |
| 0x0202 | CONTINUE | the confirmation the step waits for: `motor-connected`, `shaft-free`, `one-turn`, `not-one-turn` |
| 0x0203 | SET_READING | `vdc=48.1`, `current=0.85` (the meter readings of the startup aid) |
| 0x0204 | ABORT | - (also from a client without the control lease) |
| 0x0205 | APPLY_IDENTIFICATION | the results to apply, e.g. `R,L,psi,p,hall,J,gains`; validated, one transaction |

**Progress and completion: a status message** `unimoc.drive.CommissioningStatus.0.1`
(new DSDL type, one CAN FD frame, subject `uavcan.pub.commissioning.id`),
published on every change of state, at 2 Hz while a procedure runs and at
0.5 Hz otherwise, so a tool reacts without polling and a client that joins
late has the state within 0.5 s during a run and 2 s otherwise (default subject
215; the DSDL in `dsdl/unimoc/drive/CommissioningStatus.0.1.dsdl` is the
reference):

```
uint8 procedure        # NONE, STARTUP_AID, IDENTIFICATION
uint8 step             # the step of the procedure
uint8 state            # IDLE, MEASURING, WAITING (for CONTINUE), NEEDS_READING, DONE, FAILED, ABORTED
uint8 needs            # NONE, DC_LINK_VOLTAGE, PHASE_CURRENT, MOTOR_CONNECTED, SHAFT_FREE, ONE_TURN
uint8 progress         # 0 .. 100 % of the step
uint32 passed          # one bit per step
uint32 failed          # one bit per step: a check did not hold
uint8 fault            # NONE, ABORTED, EMERGENCY_STOP, OVERCURRENT, TRIP, ENABLE_LOST, APPLY_FAILED
uint32 counter         # electrical revolutions of M4, events of the step
```

**Values: registers** (`unimoc.identify.result.*`, real32 in base units, read
only, not persistent: `R`, `L_d`, `L_q`, `flux_pm`, `pole_pairs`,
`hall_offset`, `J`, `friction_viscous`, `friction_coulomb`, `dead_time`, the
derived gains), readable at any time with units and descriptions from the
register description file, so a review, a profile or a script does not depend
on having seen a message.

The `uavcan.pub.meas_*` subjects 208 .. 210 are removed with the old codes.

### 5.5 Results and apply

The run writes nothing to the settings by itself (decision 5): the cypilot
operator reviews the results, compares them with the present settings and
approves them; cypilot sends APPLY_IDENTIFICATION with the approved values,
then STORE. Applying while engaged waits for the disengage, as every motor
setting does.

## 6. The flow in cypilot

A "Motor identification" card next to the startup aid (the Control view, or
a "Commissioning" view holding both), on the interface of section 5.4 (actions as commands, the status message
for progress and completion, the result registers for the review), with
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
   rotation, like CONNECT_MOTOR of the startup aid. M4: "Measure" (mark the
   shaft, press Stop when the mark is back; cypilot shows the counted
   electrical revolutions) or "Enter and check" (enter p, then "did the shaft
   turn exactly once?" with Yes, No and Repeat). M3 shows the coast-down: the
   back-EMF amplitude over the frequency and psi per period.
4. **Review:** a table of each value with the present setting, the
   difference and the checks; the derived gains. The operator approves the
   values one by one or all; Apply, then Store, and "save as profile" (the
   profile of a motor type, `cypilot` profiles).
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
  `InverterModel`). M3 needs the terminal voltages of the motor with open
  outputs in the plant (section 5.2).
- `tests/sim/cyphal_drive` and the pycyphal integration test: the commands
  with their refusals, the status message (every change, 2 Hz, a late
  subscriber), the result registers, abort and apply; cypilot: unit tests on
  the demo device, a sim test on the real simulation (as the startup aid).

## 8. Milestones

| | Content | Needs |
|---|---|---|
| C0 | The bus contract of section 5.4: the command codes, `unimoc.drive.CommissioningStatus.0.1` and its publisher; the startup aid on it (decision 6); cypilot event-driven on the status message | decision 6 |
| I0 | Forced angle in `SlowUpdate`, the online accumulation, the identification FSM skeleton on C0, the result registers, the cypilot card and `cypilot-ctl identify` | C0 |
| I1 | M1 R_s and dead-time voltage; dead time in the sim inverter | I0 |
| I2 | M2 L_d, L_q | I0 |
| I3 | Phase voltages in `ControlCycleSamples` (BCC and sim plant with open outputs), their calibration in the startup aid's DUTY_FORCE steps; M3 psi by coast-down | I0 |
| I4 | M4 pole pairs (measured, entered and checked); M5 Hall offset | I0 (the forced rotation) |
| I5 | M6 J and friction (the coast of M3, the torque steps) | I3 |
| I6 | M7 derived gains; review, approval and apply in cypilot; profile | I1 .. I5 |
| I7 | Bench: the hoverboard motor (data sheet values) and the 4850 motor; compare with an LCR meter (L), a 4-wire measurement (R), a coast-down on the scope (psi) | board access |

## 9. Decisions

Taken on 2026-10-29:

1. **Bus interface:** actions as ExecuteCommand with their status codes,
   progress and completion as the status message, values as registers
   (section 5.4). The old `CMD_MEASURE_*` codes and the `meas_*` subjects go.
2. **`phase_balance_*`:** removed (done 2026-10-29, with CMD_MEASURE_BALANCE
   and subject 211).
3. **Pole pairs:** both, measured or entered and checked (M4).
4. **psi:** coast-down on the phase-voltage sense (M3); the fixed-angle
   estimate was inaccurate.
5. **Apply:** reviewed, compared and approved by the cypilot operator
   (section 5.5).
6. **The startup aid on the same contract** (milestone C0, done in the
   firmware 2026-10-29): its run, step, ext_current_A and ext_vdc_V registers
   became the START_STARTUP_AID, CONTINUE (`motor-connected` at
   CONNECT_MOTOR), SET_READING and ABORT commands and the status message; the
   results (`unimoc.startup.results`) and the calibration stay registers. A
   CONTINUE is refused while the step measures or waits for its reading, so a
   queued click cannot cross the CONNECT_MOTOR barrier. cypilot follows with a
   UNIMOC release.
