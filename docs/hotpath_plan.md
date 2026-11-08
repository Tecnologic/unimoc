# Plan: Control Cycle, Duty DMA and Hall Sensor Fusion

> **2026-11-03:** the battery case controller (BCC, STM32G473) and its
> platform `hardware/zephyr_bcc` are removed. BCC references below are
> history; the STM32H7 platform (`hardware/zephyr_stm32h7`, Jacobi and the
> NUCLEO-H743ZI) carries the same design.

> **2026-11-03 (control divider):** the control step runs every
> `control_divider` PWM half periods and always writes the CCR preload.
> Divider 1: after every peak and valley sample, an update at every boundary
> (RCR = 0), the fastest loop. Divider 2, 4, 8: after every first, second or
> fourth valley sample (the middle of the high-side pulses), an update at the
> peak only (RCR = 1): every pulse stays symmetric and the sample is free of
> ripple. Divider 0 takes the smallest the processor runs, so 32 kHz PWM gets
> a step every half period (64 kHz) and 64 kHz PWM a step every period. The
> observer runs after a power of two of steps (at most 16 kHz).
> `pwm_frequency` 0 selects the frequency from the inductance. Before that
> the PI ran once per four samples with a duty DMA (git history). HFI is
> gone: Hall sensors cover low speed.

Status: 2026-11-03. The times in section 4 are measured on the NUCLEO-H743ZI
(400 MHz, power stage off, no motor), the rest are estimates.

Timing base at 32 kHz PWM and 400 MHz (STM32H743): one PWM period T is
31.25 µs = 12500 cycles, half a period T_h 6250 cycles. The settings resolve
the timing (`lib/settings/control_timing.hpp`, `settings::ResolveControlTiming()`):

| Setting / board limit | Meaning |
|---|---|
| `unimoc.control.pwm_frequency` | 16, 20, 24, 28, 32, 40, 48 or 64 kHz; 0: the lowest of them from 20 kHz on whose ripple stays within 30 % of `motor_i_max` |
| `unimoc.control.divider` | PWM half periods per control step: 1, 2, 4 or 8; 0: the smallest the board runs |
| `max_pwm_frequency` (board) | Highest PWM frequency of the power stage; Jacobi and the sim: 64 kHz |
| `max_control_frequency` (board) | Highest control-step rate of the processor; STM32H743 from ITCM: 64 kHz |
| steps per observer cycle | The smallest of 1, 2, 4, 8 that keeps the observer at or below 16 kHz |

| PWM | Divider (automatic on the H743) | Control step | Observer | Delay T_d |
|---|---|---|---|---|
| 64 kHz | 2 (auto), every period | 64 kHz | 16 kHz (4 steps) | 15.6 µs |
| 64 kHz | 4, every second period | 32 kHz | 16 kHz (2 steps) | 23.4 µs |
| 40 kHz | 2 (auto) | 40 kHz | 10 kHz (4 steps) | 25 µs |
| 32 kHz (factory) | 1 (auto), every half period | 64 kHz | 16 kHz (4 steps) | 23.4 µs |
| 32 kHz | 2, every period | 32 kHz | 16 kHz (2 steps) | 31.25 µs |
| 20 kHz | 1 (auto) | 40 kHz | 10 kHz (4 steps) | 37.5 µs |

The delay is T_d = T_h + divider · T_h / 2 (`ControlTiming::OutputDelaySteps()`).

The register `unimoc.timing.pwm` reports the running timing,
`unimoc.timing.pwm_suggestion` the frequency for the inductance of the
running settings with the ripple there and at the running frequency.

**Ripple.** The worst peak-to-peak phase-current ripple of the space vector
modulation is ΔI_pp ≈ 0.15 V_dc / (L f_pwm) (`settings::PwmRipple()`, with the
smaller of L_d, L_q and `dc_link_regen_derate_start` as the highest operating
voltage). At 48 V and 50 A (15 A allowed): 10 µH needs 48 kHz (the choice is
64 kHz), 20 µH 28 kHz, 30 µH and more the 20 kHz floor. Above the board's
maximum the choice is that maximum, with the ripple above the target.

## 1. Goal

| Context | Rate | Content |
|---|---|---|
| Sample interrupt (ADC1 JEOS, zero latency) | 2 × f_pwm | Overcurrent and overvoltage check on raw counts at every peak and valley; after a valley the control step |
| Control step (inside the valley interrupt, zero latency) | f_pwm / divider | Calibration, angle step, Park, PI (dt = one step), inverse Park, dead-time compensation, SVM, duties into the CCR preload |
| Observer (software-pended normal interrupt) | at most 16 kHz | Flux observer, Kalman, Hall fusion, angle reference of the next cycle |
| Control thread (Zephyr, normal priority) | observer / 8 | Speed controller, position controller, MTPA, field weakening, setpoint limiting |
| Hall edges (EXTI, normal priority) | on edge | Only capture timestamp and Hall code |

The sample interrupt with its control step is the hot path. The observer is
not zero-latency: it is a normal Zephyr interrupt at the highest normal
priority, so the project rule (only the hot path is zero-latency) holds.

Assumption for the angle extrapolation: speed is constant between two
observer runs (62.5 µs at 16 kHz, longer when the observer is late).
Acceleration is not modeled.

## 2. Timing of One Control Step

Peaks P and valleys V of the center-aligned counter; the high-side pulses
(PWM mode 1) are centered on the valleys. Divider 2 (a step every period):

```
        V(n)            P               V(n+1)          P
--------|---------------|---------------|---------------|------
duty    |  w(n-1)                       |  w(n)  ...
        ^ sample n (trigger lead before V(n))
           ^ JEOS, control step n writes w(n) into the preload
                        ^ w(n) takes effect (update event at the peak only)
                                        ^ the pulse of w(n) is centered here
```

Divider 1 (a step every half period, B = every peak and valley):

```
        B(n-1)          B(n)            B(n+1)          B(n+2)
--------|---------------|---------------|---------------|------
duty    |    w(n-2)     |    w(n-1)     |     w(n)      |
                     ^ sample n (trigger lead before B(n))
                        ^ JEOS, control step n writes w(n) into the preload
                                        ^ w(n) takes effect (update at every boundary)
```

- **Sampling.** The ADC injected sequence is triggered `adc_trigger_offset`
  ticks before every boundary (TRGO2 = OC4REF before the peak, OC6REF before the
  valley). The JEOS interrupt reads ia, ib, ic and the DC link and compares
  them with the raw window (`SetOvercurrentWindow`, from `max_phase_current`)
  and the raw overvoltage level; a violation sets the break (`EGR = BG`) at once.
  At divider 2 and above a peak sample ends there; its trip is kept for the
  next control step (`Sample()`/`GetSample()` report the trips of the unread
  samples once).
- **Control step.** After every sample (divider 1) or every valley sample the
  interrupt calls `SampleHandler()` (`CurrentControlIsr::OnSample()`). Every
  `CallsPerStep()`-th call (1 at divider 1 and 2, then divider / 2) writes
  the duties into CCR1..3 (preload); the other calls return at once.
  Every `steps_per_cycle` steps end a control cycle and pend the observer.
- **Update.** Divider 1: RCR = 0, an update at every peak and valley.
  Divider 2 and above: RCR = 1 gives one update event per PWM period. The
  repetition counter raises it when it is 0 and counts every overflow and
  underflow; `InitTimer()` loads it with 0 (UG with RCR = 0, then RCR = 1),
  so the first and every following update is a peak. Measured on the
  NUCLEO-H743ZI (2026-11-03, UIF read in the sample interrupt): with the
  counter loaded with 1 every update fell on the valley, with 0 every update
  falls on the peak.
- **Deadline.** The duties must be in the preload before the next update:
  half a period after the sample, minus the end of the ADC sequence (JEOS
  about 1.7 µs after the boundary on the Nucleo). With the valley sampling a
  write before the valley (long trigger lead) also waits for the peak; at
  divider 1 the end of the sequence must come after the boundary of its
  trigger (lead shorter than the sequence), or the duties take effect half a
  period early. `SetDutyTicks()` counts a write after its update as an
  overrun (`DutyOverruns()`); it takes effect at the following one.
- **Delay.** w(n) takes effect half a period after sample n and holds for
  `divider` half periods: its middle is T_d = T_h + divider · T_h / 2 after
  the sample (`output_delay_steps`, `ControlTiming::OutputDelaySteps()`), 1.5
  half periods at divider 1, one period at divider 2.
- **Angles (D4).** The observer hands over one angle reference per cycle
  (`AngleReference`): the angle at the first sample of the next cycle, the
  rotation of one control step (ω·dt) and the rotation of the output delay
  (ω·T_d). Each control step rotates its angle by one step (four
  multiplications, `system::Rotated()`, renormalized) and takes the new
  reference when the observer has published one, rotating it forward by the
  steps it is late. A late observer means a longer extrapolation, not stale
  angles.
- **Phase of a sample.** The JEOS of a peak sample comes in the upper half of
  the counter (before or after the peak), the one of a valley in the lower
  half; the sample interrupt reads `CNT` to know the boundary type
  (`SampleAtPeak()`), as long as lead and sequence stay below a quarter period.

### Low-side shunts

A board with a shunt in the source of each low-side switch (foc0
`current-sense = "low-side"`, `settings::CurrentSense::kLowSide` in its
profile) measures a phase only while its low side conducts. The timing above
stays the same with three changes:

- **PWM mode 2.** CH1..3 run in PWM mode 2 (high side active while CNT >
  CCR), so the low-side pulses are centered on the valley and the valley
  sample is their middle. `SetDutyTicks()` writes ARR minus the duty ticks;
  `AppliedTicks()` keeps the duty ticks. The update stays at the peak: the
  period from peak to peak is symmetric about its valley, as with mode 1.
- **No step every half period.** At the peak every high side is on and the
  shunts carry no current, so divider 1 is not runnable
  (`ResolveControlTiming()` skips it, `Initialize()` refuses it). The
  automatic divider is 2 at 32 kHz as well: 2.7 kHz bandwidth in place of
  3.6 kHz. The peak sample still runs the DC-link check; its current check
  sees nothing.
- **Duty cap.** Each low side must stay on for `min_low_side_time` per PWM
  period (profile: twice dead time, settling and trigger lead), so
  `ControlTiming::duty_max` = 1 − f_pwm · `min_low_side_time` caps
  `svm_duty_max` (also for forced duties). The STM32H7 estimate with 1 µs
  dead time is 6 µs: 0.81 at 32 kHz, 0.62 at 64 kHz. A cap at or below 50 %
  is not runnable. `unimoc.timing.pwm` reports the cap as its fifth value.
  The phase with the highest duty switches closest to the valley; within the
  guard it is reconstructed from the other two, as with inline sensors.

Open: no board has low-side shunts yet (build check `unimoc.app.build.low_side`
on Jacobi with an overlay, the sim scenario `unimoc.sim.closed_loop.low_side`).
The cap costs voltage; it could apply to the middle phase only, with the
highest phase reconstructed (as the ST motor control SDK does), which needs
the limit in the SVM. Measure `min_low_side_time` and the amplifier sign on the
first such board.

## 3. Duty Range and Current Bandwidth

The duties take effect through the preload at a boundary, so the ISR timing
does not limit them. What remains:

- **Sampling window.** At a valley the phase with the lowest duty switches
  closest to the sample, at a peak the one with the highest. When that edge
  lies within `trigger lead + 1 µs` of the boundary, the STM32H7 layer drops
  the phase and reconstructs it from the other two (three inline sensors,
  ia + ib + ic = 0). The compare values active during the sample
  (`AppliedTicks()`) are the last ones written with the valley sampling, the
  ones written two samples before at divider 1.
- **Pulse symmetry.** At divider 1 the duties change at every boundary, so a
  pulse may be asymmetric about its valley and a sample is not exactly the
  mean of the ripple; with the valley sampling every pulse is symmetric.
- **Minimum pulse.** 3 % of a PWM period (31.25 µs) is 0.94 µs. Factory
  `svm_duty_min/max` = 0.03 / 0.97 in the sim profile, 0.04 / 0.96 on the
  STM32H7. To be checked on the scope with the bootstrap supply.
- **Dead time.** The voltage error is V_dc · t_d · f_pwm: 1.5 V at 48 V,
  1 µs and 32 kHz, 0.9 V at 300 ns and 64 kHz. The 4850 power board runs
  1 µs (devicetree `dead-time-ns`), which is why the factory frequency stays
  at 32 kHz; 64 kHz needs the dead time at or below about 300 ns. The
  dead-time compensation runs on every step from the current of its sample.
  Both power stages (Jacobi, and the legacy 4850 power board) are designed for
  200 ns (2026-11-08): 0.31 V at 32 kHz and 0.61 V at 64 kHz at 48 V. Not set
  yet: `dead-time-ns` stays 1000 until it is checked on the scope.

**Current bandwidth.** With pole-zero cancellation (kp = L·ω_c, ki = R·ω_c)
the open loop is an integrator with the delay T_d from a sample to the middle
of its duties; the phase margin is 90° − ω_c·T_d.

| Timing | T_d | ω_c for 60° margin |
|---|---|---|
| 64 kHz PWM, divider 2 (a step every period, auto) | 15.6 µs | 33.5 krad/s (5.3 kHz) |
| 32 kHz PWM, divider 1 (a step every half period, factory auto) | 23.4 µs | 22.3 krad/s (3.6 kHz) |
| 64 kHz PWM, divider 4 | 23.4 µs | 22.3 krad/s (3.6 kHz) |
| 40 kHz PWM, divider 2 (auto) | 25 µs | 20.9 krad/s (3.3 kHz) |
| 32 kHz PWM, divider 2 | 31.25 µs | 16.8 krad/s (2.7 kHz) |

At the same PWM frequency the step every half period has the shorter delay
(1.5 half periods against one period) and needs twice the control steps; the
step every period keeps the pulses symmetric and the sample at the mean of
the ripple. The automatic divider takes the half period step whenever the
processor sustains it. `sim_closed_loop` runs a 10 A step on a locked rotor
(100 µH, 50 mΩ) with the 60° tuning at the factory 32 kHz (divider 1) and at
64 kHz (divider 2), each within 0.5 A after 125 µs, and the same step at
divider 4. The factory gains are still the conservative 2000 rad/s of the
hoverboard default; raise them per motor (`unimoc.control.current.*`) with
ω_c = (π / 6) / T_d.

## 4. Data Flow

`AngleBuffer` (observer → control steps, double buffer, flip with
release/acquire) and a pair of `CycleRecord`s (control steps → observer):

| Field | Writer | Reader |
|---|---|---|
| `AngleReference` (sample, angle, step, output rotation) | observer (inactive buffer, then flip) | every control step |
| `CycleRecord` `sc[k]`, `i_ab_samples[k]` (up to 8 steps) | control step k of the cycle | observer, identification windows |
| `CycleRecord` `i_dq_mean`, `u_dq` (means of the cycle), `timestamp`, `dc_link` | last control step of the cycle | observer |

The last step of a cycle hands its record over (`observer_pending`) and the
next cycle fills the other one. A cycle that ends while the observer still
processes the previous one counts `observer_overruns`. A cycle with a step
that did not run the controller (trip, forced duties) is not handed over.

The control thread writes setpoints into a mailbox that every control step
reads. No kernel functions are called from the zero-latency contexts; the
observer interrupt may call them (for example to wake the control thread
every eighth cycle).

### Control step

1. The sample from the hardware (scaled, a disturbed phase reconstructed),
   then the calibration of the startup aid. Repeat of the overcurrent and
   overvoltage checks on the calibrated sample, latch, neutral duties.
2. Angle of the sample: the new reference or one rotation step on.
3. Clarke and Park with that angle; sums for the record.
4. PI with decoupling (dt = one control step).
5. Inverse Park with the angle at the middle of the interval the duties apply to,
   dead-time compensation from this sample's current, SVM (one division by
   V_dc), duties into the preload.

### Observer

1. Flux observer with the mean current and voltage of the cycle.
2. Mechanical prediction (torque from i_d, i_q).
3. Hall edges and sector.
4. Angle reference of the next cycle, flip.

### Measured on the NUCLEO-H743ZI (M0)

First with the PI on every half period (the design before the divider),
2026-11-03, `CONFIG_UNIMOC_HOT_PATH_TIMING=y`, 400 MHz, code in flash with the
caches on, power stage off (MOE = 0), no motor. The trip latched by the
floating ADC inputs was cleared and the window opened over the debugger, so
the full control path ran (PI, circle limiter, SVM). The flux observer was
idle (outputs off), Hall off. Cycles of the 400 MHz core, 6250 per half period:

| Section | `-Os` (default) min / mean / max | `-O2` min / mean / max |
|---|---|---|
| Sample interrupt (incl. control step) | 1631 / 1774 / 2276 | 1304 / 1506 / 2001 |
| Control step alone | 1405 / 1537 / 2025 | 1122 / 1317 / 1821 |
| Observer (16 kHz) | 1787 / 1965 / 5109 | 1456 / 1648 / 2901 |

64 300 samples and 16 070 observer runs per second, no duty overrun and no
observer overrun. The worst sample interrupt takes 36 % of a half period; the
mean load of hot path and observer is about 36 % of the CPU. Before the
change the same board measured 2311 cycles (mean) for the four-entry control
cycle, with the loop held in its trip path (the full path did not run then).

Hot path in ITCM (since 2026-11-03, `hardware/zephyr_stm32h7/hot_path_itcm.ld`,
data still in AXI SRAM through the D-cache), same bench:

| Section | ITCM `-Os` (default) min / mean / max | ITCM `-O2` min / mean / max |
|---|---|---|
| Sample interrupt (incl. control step) | 1618 / 1727 / 1908 | 1318 / 1426 / 1679 |
| Control step alone | 1385 / 1491 / 1671 | 1142 / 1247 / 1497 |
| Observer (16 kHz) | 1764 / 1867 / 3001 (one 4837) | 1398 / 1490 / 2438 |

ITCM saves only about 3 % of the mean (the M7 caches hid the flash), but
16 to 18 % of the worst sample interrupt: the jitter of a cache miss is gone.
The rest is the code itself (`-O2` saves another 17 %) and the peripheral
reads. 12.6 KB of the 64 KB ITCM are used.

With the control divider (ITCM, `-Os`, the same bench, a temporary probe of
the update flag in the sample interrupt):

| PWM, divider | Sample interrupt with step min / mean / max | Control step | Observer | Steps/s | Overruns |
|---|---|---|---|---|---|
| 32 kHz, 2 (every period) | 1788 / 1896 / 2059 | 1473 / 1577 / 1723 | 1764 / 1901 / 3014 (16 kHz) | 32 200 | 0 / 0 |
| 32 kHz, 1 (every half period) | 1768 / 1837 / 2043 | 1453 / 1522 / 1716 | 1768 / 1873 / 4898 (16 kHz) | 64 300 | 0 / 0 |
| 64 kHz, 2 (every period) | 1754 / 1828 / 2069 | 1438 / 1511 / 1723 | 1769 / 2112 / 4850 (16 kHz) | 64 400 | 0 / 0 |

With the valley sampling the update flag was set before every peak
interrupt and never before a valley one: every update falls on the peak. At
divider 1 it was set before every interrupt. The valley interrupt starts about
1.7 µs after the valley (the end of the ADC sequence); at 64 kHz its worst
case ends about 0.9 µs before the peak. That margin is thin at `-Os`: build
the hot path with `-O2` (17 % less) before running 64 kHz on the board, and
keep the trace capture (`Tick()`) in mind. The peak interrupts (trip checks
only) are not in the probe.

Open: the observer with the flux observer and Hall fusion running, and the
hot-path data in DTCM.

### Why the observer is a normal interrupt

| | Second zero-latency level | Normal interrupt (chosen) | Zephyr thread |
|---|---|---|---|
| Masked by kernel `irq_lock()` | never | yes, for the few µs of a kernel critical section | yes, plus scheduler |
| Jitter | only from the sample interrupt | a few µs | tens of µs with USB/CAN/Ethernet load |
| Kernel API allowed | no | yes (wake the control thread) | yes |
| 16 kHz overhead | interrupt entry only | interrupt entry only | context switch per cycle (5–8 % CPU) |
| CAN, USB, Ethernet | delayed up to the observer time every 62.5 µs | delayed up to the observer time (they have lower priority) | could delay the observer past its deadline |

Communication interrupts are served in all variants, as long as the total CPU
load leaves room: their peripherals buffer frames (FDCAN FIFO, USB endpoint
buffers, Ethernet DMA descriptors), and a delay of 10 to 20 µs is far below
their deadlines. A thread is too slow and too jittery for a 62.5 µs deadline.

## 5. Hall Sensor Fusion

Hardware: three Hall signals on PC13, PC14, PC15 (RC-filtered 3k3/2n2, time constant about
7 µs). These are not timer inputs, so the timer's Hall interface is not used: GPIO with EXTI
(lines 13 to 15) and timestamps from the DWT cycle counter.

**Capture.** The EXTI handler (normal priority, zero-latency IRQs preempt it) stores the Hall code
and timestamp in a small buffer. Invalid codes (000, 111) are counted as errors. Polling once per
control cycle would be too coarse: at 333 Hz electrical, 62.5 µs is about 7.5° of error.

**Measurement model** (both from the same Hall code):
- *Edge:* the angle of the edge is known from a table of six angles (rising/falling per Hall).
  The observer converts the timestamp back: ε = wrap(θ_edge + ω̂·age − θ̂). The age includes
  the filter delay. R comes from the alignment tolerance (for example ±3° electrical).
- *Sector:* without an edge only "the angle lies in the 60° sector" is known. The innovation term
  is 0 as long as θ̂ lies in the sector, otherwise the distance to the nearest sector boundary. This
  prevents the sector center from pulling the observer and keeps it in the right sector at standstill
  without drift. Inside the sector the update is skipped, not fused with zero innovation: a
  zero-innovation update in every cycle would shrink the angle covariance far below the real ±30°,
  and the next edge would then correct speed and load torque instead of the angle (overshoot and
  oscillation on a ramp). The first valid code after a restart sets θ̂ to the sector center (at most
  30° off); after a Hall failure the estimate is kept while it lies in the returning sector. The
  position home is the estimate at the start, so with Hall only it lies within half a sector of the
  shaft.

**Fusion with the flux observer.** The flux observer always runs. At low speed it provides no signal
(back-EMF about 0), so its R is scaled with speed (R_flux = R0 + k/ω²); it is not switched off. Hall
stays active at high speed and damps the drift of the flux observer. At standstill and near zero, Hall
leads. A connected encoder enters as a third measurement source with a small R.

**Table of edge angles.** Nominally 60° steps with an offset. The exact values are determined once with an
open-loop run (V/f or a predefined angle) and logging of the edges, and stored as a setting (flash backend
still pending, until then RAM and shell).

**Position.** From the fused θ and the pole-pair count, `PositionTracker`/`RotorAngle` gives the mechanical
position. The absolute reference remains an application question (homing).

**Simulation.** The plant of `lib/sim` (`MotorSimulation`) generates currents and Hall codes; the sim
hardware layer does not expose the Hall signals yet (M5).

## 6. Library Decisions (unchanged)

- **D2** `MechanicalObserver`: separate covariance prediction and measurement update
  (`PredictCovariance(dt)`, `UpdateAngle(ε, R)`); `PmsmFluxObserver::calculate()` returns the angle
  error; optional `SinCos` table in the observer. The flux observer runs always, also with an encoder.
  Each measurement carries its noise as a typed variance (`unit::AngleVariance` for an angle,
  `unit::AngularVelocityVariance` for the Hall speed bound R_sector/τ², checked against the unit of the
  innovation at compile time); the covariance P stays `float` (mixed-unit state vector). The Hall
  timing (tick period, edge age, time since the last change) is `unit::Time` (2026-10-29).
- **D3** Decoupling from the current of the sample (2026-11-03; before: the mean of the cycle).
- **D4** Dedicated angle for the output side (`AngleReference::out`, the rotation of the output
  delay), delay as a parameter.
- **Integer arithmetic: no.** On Cortex-M4F float add and multiply cost one cycle; the time goes into
  function pointers, `sqrtf` (libm, errno) and divisions.
- **STM32H7.** Advanced timer, TRGO, preload written by the control step (no DMA since 2026-11-03).
  Budget per half period 6250 cycles at 400 MHz (rev. Y). The hot path runs from ITCM
  (2026-11-03); DTCM and `-fsingle-precision-constant` are open options if the measurements
  need them.

## 7. Simulation (native_sim)

`hardware/zephyr_sim` closes the loop around `MotorSimulation`. A kernel timer in simulated time runs
four half periods (two PWM periods, from a valley) per expiry (exact mean rate, scheduled from the
elapsed time because 62.5 µs is not a whole number of 1 µs ticks). Each half period:

1. the plant with the duties in effect,
2. the sample at its end and the overcurrent and overvoltage checks (kept for the next read),
3. at an update (the peaks; every boundary at divider 1) the duties of the last control step take
   effect (the preload); after a valley (every sample at divider 1) the sample handler, and the
   observer when the step ended a control cycle.

The sim checks the control logic and this timing contract, not the time budget, ADC noise,
quantization or dead time (the inverter model has none). Tests: `sim_irq_cadence` (cadence, the
half periods the duties of a step apply to, the trips of unread samples), `sim_overcurrent`
(per-sample trip), `sim_closed_loop` (current steps on a locked rotor through the real control
core, with the 60° tuning at 32 kHz divider 1 and 64 kHz divider 2, and at divider 4),
`unimoc.sim.closed_loop.low_side` (`CONFIG_UNIMOC_SIM_LOW_SIDE_CURRENT_SENSE`: the samples read
nothing at the peak and a phase at the valley only with at least 2 µs of low-side time, the profile
asks for 4 µs; divider 2 at 32 kHz, the duty cap, divider 1 refused, the 60° step),
`unimoc.lib.settings` (`control_timing`:
suggestion, divider, observer rate, validation, low-side shunts).

## 8. Milestones

Each milestone builds for the STM32H7 boards and native_sim and keeps the twister green.

| M | Content | Evidence |
|---|---|---|
| M0 | Instrumentation (`CONFIG_UNIMOC_HOT_PATH_TIMING`, `hardware/hot_path_timing.hpp`): DWT cycles around every sample interrupt, the control step and the observer, min/mean/max, `DutyOverruns()`, `observer_overruns`; read over the `unimoc.timing.*` registers. First numbers from the NUCLEO-H743ZI (2026-11-03, section 4, outputs off). Open: with the flux observer and Hall fusion running. | Numbers from the board |
| M1 | Done: CCR preload, pended observer, sim at the 16 kHz cycle. Since 2026-11-03 the control step runs every 1, 2, 4 or 8 half periods: after every sample with an update at every boundary (RCR = 0), or after the valley samples with the update at the peak only (RCR = 1); the duty DMA is gone. | Twister: `sim_irq_cadence`, `sim_closed_loop`, `control_timing`; Nucleo: 64.3 k steps/s at 32 kHz (divider 1), 32.2 k and 64.4 k at 32 and 64 kHz (divider 2), updates where expected, no overrun |
| M2 | Done in code: circle limiter without libm (`vsqrt.f32` through `-fno-math-errno`). The control step makes one division by V_dc for its SVM. | `svm_test`, call-graph check below |
| M3 | Done in code: hot-path hardware access through `hardware/hot_path_interface.hpp` (link-time bound, no function pointers); the platform calls `SampleHandler()`/`ObserverHandler()` of the application directly; `-fno-math-errno` for the Zephyr build. Hot path in ITCM since 2026-11-03 (`hot_path_itcm.ld`). Inlining across the translation units (LTO or header-only access) is open; `-O2` saves about 15 % (section 4). | Call-graph check below, cycles against the M0 baseline |
| M4 | Done in code: `MechanicalObserver` with `PredictCovariance()`, `UpdateAngle()`/`UpdateSpeed()` each with its own R, error wrap in `angle_wrap.hpp`; the flux observer as an angle measurement with R_flux = R0 + k/ω². | `mechanical_observer_test` (synthetic profiles) |
| M5 | Done in code: `HallDecoder` and `HallEventQueue` (`hall_capture.hpp`), BCC EXTI capture with timestamps (not run on the board), sim Hall events with the edge time from the plant. | `hall_decoder_test`, `hall_sensor_test` (generated edges) |
| M6 | Done in code: `HallFusion` with edge and sector measurement, speed bound from the sector time, standstill hold, sector center at the start. | `hall_fusion_test`, `sim_hall_fusion` (standstill without drift, ramps, Hall failure) |
| M7 | Done in code: control thread (`control_thread.hpp`), `OuterLoop` with `SpeedController`, `PositionController`, setpoint limiting and `Mailbox`; speed and position setpoints over Cyphal. | `sim_speed_control` (step in ω, current limit), `sim_cyphal_drive` |
| M8 | Board: preload timing on the scope, ADC alignment, Hall table, closed current loop with the 50 A limit, current step at the 3.6 kHz tuning. | Measurements, oscilloscope |

### Call-graph check of the zero-latency path (M2/M3)

2026-11-03 on the NUCLEO-H743ZI build (`-Os`), everything reachable from
`SampleIsr` was listed from the disassembly (`arm-zephyr-eabi-objdump -d`,
following `bl`/`b.w`/`b`): 37 functions, the control step (`OnSample`,
`AdvanceAngle`, `Rotated`, the PI, SVM, dead-time compensation, the sine
table of the identification), the STM32H7 glue and driver functions, the
trace sampling (`trace::OnControlCycle`, `cymon::Device::Tick`), the timing
probe, `memset` and `memmove`. No libm function, no indirect call (`blx rN`),
no kernel function and no `__cxa_guard_*`. Repeat the check after changes to
the hot path.

Since the ITCM placement the check also lists where each function lies:
calls from ITCM (address 0) into flash go through linker veneers
(`__<symbol>_veneer`, an indirect jump), so a function missing from
`hot_path_itcm.ld` shows up as a veneer and a function above 0x08000000.
2026-11-03 at `-Os`: 37 functions, all in ITCM except `memset` (cymon `Tick`,
one zeroed frame while a capture runs). `memset` has to stay in flash: the
startup code clears the BSS with it before it copies ITCM. The observer
interrupt reaches the kernel (`k_sem_give` of the control thread) in flash, as
intended.

## 9. Risks and Open Points

- **Not verified on the scope.** The control step runs on the Nucleo at 32 and 64 kHz without a
  late preload write and the updates fall on the peak (2026-11-03, update flag), but the PWM
  edges, the boundary type from `CNT` and the ADC alignment are not checked against the outputs
  yet (M8, first with the outputs off).
- **ADC1 JEOS as "all ADCs done".** The sample interrupt reads ADC2/3/4 as well. This assumes equal
  sample times and simultaneous triggers on all ADCs (as before).
- **Overcurrent only in software.** The board has no comparators on the current channels; the check is
  per sample in the zero-latency interrupt (reaction within about 1 µs plus the half period until the
  next sample). The analog watchdog of the ADCs is a possible addition. The DC-link overvoltage is
  checked the same way since 2026-10-31: the sample interrupt compares the raw DC-link conversion of
  every sample with the trip level in counts (before the calibration, `SetOvervoltageCounts()`) and
  breaks the timer; the control step repeats the check on the calibrated sample. It reads one
  more register of the phase C ADC per sample (the same assumption: its sequence is done at the
  ADC1 JEOS). Not run on the board with a power stage.
- **Observer overrun.** A late observer only lengthens the angle extrapolation of the control steps.
  If it does not finish within one cycle, the control steps start to refill the record it reads;
  counted (`observer_overruns`), none measured on the Nucleo.
- **FPU in zero-latency ISRs.** Lazy stacking costs cycles when a thread uses the FPU. Measure in M0.
- **Hot-path data in AXI SRAM.** The code runs from ITCM since 2026-11-03 (section 4; the
  sample interrupt varies between 1618 and 1908 cycles, before between 1631 and 2276 from
  flash). It no longer stalls on a flash write either. The state of the control step and the
  sample are still in AXI SRAM through the D-cache; DTCM would remove the remaining cache
  misses. `hot_path_itcm.ld` names the functions by their mangled names: a renamed hot-path
  function falls back to flash silently, so repeat the call-graph check (section 8).
- **Hall alignment.** Without a calibrated table, angle errors of up to about ±10° are possible.
- **Start-up.** At standstill the fusion only knows the sector (±30°): about 87 % torque (cos 30°).
  HFI was removed (2026-11-03): the Hall sensors are the low-speed angle source.
- **Motor data missing.** Maximum electrical frequency, inductance and resistance of the target motors are
  needed for R, the thresholds and the controller parameters.
