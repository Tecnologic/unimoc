# Plan: Hot Path, Quarter-Cycle IRQ and Hall Sensor Fusion

Status: 2026-10-18. All times and cycle counts in this document are estimates from
the code. None of them is measured on the board (milestone M0 changes that).

Timing base at 20 kHz PWM and 170 MHz: one half period (one JEOS interrupt) is
25 µs = 4250 cycles, one quarter cycle (four half periods) is 100 µs = 17000 cycles.

## 1. Goal

| Context | Rate | Content |
|---|---|---|
| Hot path (JEOS, zero latency) | 40 kHz | Only what depends on the current measurement: Park, PI, inverse Park, SVM, CCR |
| Quarter-cycle IRQ (TIM8 update, zero latency) | 10 kHz | Precompute everything for the next four half periods, observer, Hall fusion |
| Control thread (Zephyr, normal priority) | 1 to 2 kHz | Speed controller, later position controller, MTPA, field weakening, setpoint limiting |
| Hall edges (EXTI, normal priority) | on edge | Only capture timestamp and Hall code |

Assumption for the precomputation: speed is constant over the four half periods
(100 µs). Acceleration is not modeled.

**Scope note.** The project rule says only the hot path runs in zero-latency IRQs and
everything else runs in Zephyr threads (`docs/zephyr_port_plan.md`). This plan, and the
current code, keep the quarter-cycle IRQ (observers, Hall fusion, plan build) as a
zero-latency IRQ. That is an open decision, see section 7.

## 2. Current State in the Code

- `CurrentControlIsr::on_jeoc()` (980 bytes) does everything in one pass today:
  reads the ADC through function pointers, overcurrent check, duty range check (reads
  the CCR back through another function pointer), Clarke, Park, PI with decoupling from the
  **measured** current, inverse Park, HFI voltage, dead-time compensation, SVM, and writes the
  duties (not inline).
- The call to `std::sqrt` in the voltage limiter is compiled to the libm call
  `sqrtf` (errno handling), not to `vsqrt`.
- `SlowUpdate::run_once()` (928 bytes) runs today in the TIM8 update callback,
  polls `samples_ready`, re-parks the four samples, averages them, calls the flux observer,
  Kalman prediction, HFI, and computes `sc[0..3]` for the next cycle
  (double buffer with an `active` flip).
- `MechanicalObserver` is a Kalman filter with the states ω, θ and load torque, with
  one scalar angle measurement per call to `inject_angle_error()`. Each call **also** runs
  the covariance prediction. `PmsmFluxObserver::calculate()` calls
  `inject_angle_error()` itself at the end. The comments mention Hall sensors, but the code
  has nothing for them. The angle error is not wrapped to (-π, π]. Prediction and
  correction compute `std::sin/cos` (two calls per cycle).
- The phase between the core's quarter-cycle counter (`sub_step_`) and the TIM8 update is
  not synchronized. The update sits at the peak, and CCR1 to 3 have no preload.
- No Hall capture and no speed controller in the core (only `PositionController`
  in the library).

## 3. Decisions

**D1 Preload and synchronous quarter-cycle IRQ (recommendation: yes, awaiting confirmation).**
Today the hot path writes the CCR without preload into the running counter. With RCR=0
and CCR preload, the new values only take effect at the next boundary (peak or valley;
in center-aligned mode RCR=0 generates an update event at both). The hot path sets the
quarter-cycle IRQ pending itself in sub-step 3 via `NVIC_SetPendingIRQ`, so the phase is
deterministic and the polling of `samples_ready` goes away. This changes the earlier
requirement "update IRQ with RCR=4". See section 3.1 for the reasoning.

**D2 Library changes (approved).**
- `MechanicalObserver`: separate covariance prediction and measurement update
  (`PredictCovariance(dt)`, `UpdateAngle(ε, R)`). `inject_angle_error()` stays as the
  combination of the two.
- `PmsmFluxObserver::calculate()` returns the angle error (the old variant as a wrapper).
- Optional: replace `std::sin/cos` in the observer with the `SinCos` table.
- The flux observer runs **always**, also with an encoder connected. It is not switched off,
  only weighted through its R. Measurement sources for the Kalman filter: flux (always),
  Hall (if present), optional encoder (small R). With an encoder the flux observer also gives
  a plausibility check (flux ε against encoder ε). The drift of the voltage model at standstill
  must be handled in M6 (integrator hold or leakage term at low speed).

**D3 Decoupling (decided):** computed from the mean of the last cycle.

**D4 Dedicated angle table for the output side (follows from D1).** With preload, the voltage
takes effect on average 1.5 half periods after the measurement. The angle for that is
θ + ω·(k + 1.5)·dt, with the delay as a parameter, tuned on the board. It costs nothing in the
hot path.

**Hall (decided):** standard 120°. Pole pairs do not matter: the mechanical observer and Hall
both work in the electrical angle.

### 3.1 Background on D1

**Integer arithmetic: no.** On Cortex-M4F a float multiplication and addition each cost one
cycle (like their integer counterparts). Only multiply-accumulate (3), division and square
root (14) are more expensive. Q15/Q31 brings little in the computation cycles and costs
scaling, overflow checks and rewriting the float-based library (`units`). The time is spent
in the function pointers, the CCR read-back, `sqrtf` and the divisions. Fixing those fixes M3,
without integers. The angle is already fixed point (`RotorAngle`, sine table).

**Writing the CCR directly with the current measurement.** The earliest point for a write is
trigger plus conversion time (about 1.3 µs) plus ISR time, so about 2.5 µs after the
measurement. But a phase at 90 % duty switches already 2.5 µs after the peak
(25 µs · (1 − 0.9)), and one at 95 % after 1.25 µs. Then:
- Only part of the three channels takes effect in this half period, the rest in the next.
  That distorts the line voltages.
- The result depends on the ISR jitter (quarter-cycle IRQ before the hot path, flash wait states).
- The usable duty range shrinks to about 10 to 90 %.

The gain from writing directly: one half period less delay (25 µs at 20 kHz, 12.5 µs at 40 kHz).
With preload it is 1.5 half periods until the mean effect, directly about 0.5 plus ISR time.

**Effect on the target motors.** Hoverboard hub motors (36 V, phase inductance in the mH range):
the delay does not matter. Hobby outrunners (200 kV, inductance typically 10 to 50 µH, electrically
above 1 kHz): at 20 kHz, 37.5 µs at 1.5 kHz control bandwidth is about 20° of phase, which D4
compensates in the angle. At such a small inductance 20 kHz is tight anyway: the current ripple is
on the order of 10 A peak-to-peak (25 V, 20 µH). A higher PWM frequency will pay off, and then the
budget of one half period halves. That also argues for preload, because there the deadline is the
whole half period.

**STM32H7.** The budget of one half period rises from 4250 to about 12000 cycles (480 MHz) or
13750 (550 MHz on H723/H725/H73x). The Cortex-M7 has two pipelines and an FPU. On the H7 I would
place code and hot-path data in ITCM/DTCM (deterministic, no flash wait states) and make sure the
compiler does not generate double constants (`-fsingle-precision-constant`). The concept (advanced
timer with break, ADC injected trigger via TRGO) is the same. Preload is also useful there, but
less essential because of the budget.

## 4. Data Flow

The quarter-cycle IRQ fills one plan per cycle (double buffer like `SubStepBuffer`,
flip with release/acquire) for the next four half periods:

```
struct HotpathPlan {
  SinCos sc_in[4];          // Park of the measured current
  SinCos sc_out[4];         // inverse Park of the demanded voltage (D4)
  Rotor<Voltage> ff;        // decoupling, constant over the cycle (ω constant)
  Stator<Voltage> v_inj[4]; // HFI injection per sub-step
  Rotor<Current> i_ref;     // setpoint, latched here (no torn d/q pair)
  float inv_vdc;            // 1/V_dc, smoothed, for SVM
  float v_limit_sq;         // (v_max * V_dc)^2 for the circle limiter
  float ki_dt_d, ki_dt_q, kb_dt_d, kb_dt_q;  // integrator factors incl. dt
  float kp_d, kp_q;
};
```

The hot path only reads the plan and writes back: `i_ab[k]`, `u_dq_last`, duties. The control
thread writes setpoints into a mailbox, which the quarter-cycle IRQ reads when it builds the plan
(seqlock or double buffer). No kernel functions are called from the zero-latency contexts.

### Hot path (per half period, target 400 cycles, 2.4 µs)

1. Read two shunt channels, scale with offset and gain, derive the third phase from
   ia + ib + ic = 0.
2. Clarke and Park merged with `plan.sc_in[k]`.
3. Error, PI with `kp`, integrator with `ki_dt`, plus `plan.ff`.
4. Circle limiter via `v_limit_sq`, square root only when saturated (`__builtin_sqrtf`,
   build with `-fno-math-errno`).
5. Inverse Park with `plan.sc_out[k]`, plus `plan.v_inj[k]`.
6. Dead-time compensation (sign of the current, parameters from the plan), SVM with
   `plan.inv_vdc` instead of a division.
7. Clamp duties to 5 to 95 %, write the CCR directly. No read-back, no function pointers. The
   hardware access object is chosen via CMake and included inline.
8. The overcurrent check stays before everything else (60 A, `power_stage.Trip()`).
9. In sub-step 3: set the quarter-cycle IRQ pending.

### Quarter-cycle IRQ (every 100 µs, budget about 15 µs, target 2500 cycles)

1. Read the samples of the last cycle (re-park them with the `sc_in` of the cycle, average as today).
2. Mechanical prediction with the mean current (torque from i_d, i_q).
3. Covariance prediction once, then measurement updates: flux observer, Hall, optional HFI.
   Each with its own R and an error wrapped to (-π, π].
4. Build the plan for the next cycle: θ_k = θ + k·ω·dt, `sc_in`, `sc_out`, `ff`, `v_inj`,
   `inv_vdc`, `v_limit_sq`, integrator factors, `i_ref`.
5. Power-stage state monitoring (fault, blocked), counters for timing measurement.

### Control thread (1 to 2 kHz)

Speed controller (PI with anti-windup), later position controller (`PositionController`),
MTPA, field weakening, limiting of the current setpoint (magnitude to `motor_i_max`, 50 A).
Output: `i_ref` into the mailbox. Input: ω and θ from a snapshot of the observer.

## 5. Hall Sensor Fusion

Hardware: three Hall signals on PC13, PC14, PC15 (RC-filtered 3k3/2n2, time constant about
7 µs). These are not timer inputs, so the timer's Hall interface is not used: GPIO with EXTI
(lines 13 to 15) and timestamps from the DWT cycle counter.

**Capture.** The EXTI handler (normal priority, zero-latency IRQs preempt it) stores the Hall code
and timestamp in a small buffer. Invalid codes (000, 111) are counted as errors. A 10 kHz polling
in the quarter-cycle IRQ would be too coarse: at 333 Hz electrical, 100 µs is about 12° of error.

**Measurement model** (both from the same Hall code):
- *Edge:* the angle of the edge is known from a table of six angles (rising/falling per Hall).
  The quarter-cycle IRQ converts the timestamp back: ε = wrap(θ_edge + ω̂·age − θ̂). The age includes
  the filter delay. R comes from the alignment tolerance (for example ±3° electrical).
- *Sector:* without an edge only "the angle lies in the 60° sector" is known. The innovation term
  is 0 as long as θ̂ lies in the sector, otherwise the distance to the nearest sector boundary. This
  prevents the sector center from pulling the observer and keeps it in the right sector at standstill
  without drift.

**Fusion with the flux observer.** The flux observer always runs. At low speed it provides no signal
(back-EMF about 0), so its R is scaled with speed (R_flux = R0 + k/ω²); it is not switched off. Hall
stays active at high speed and damps the drift of the flux observer. At standstill and near zero, Hall
leads. A connected encoder enters as a third measurement source with a small R.

**Table of edge angles.** Nominally 60° steps with an offset. The exact values are determined once with an
open-loop run (V/f or a predefined angle) and logging of the edges, and stored as a setting (flash backend
still pending, until then RAM and shell).

**Position.** From the fused θ and the pole-pair count, `PositionTracker`/`RotorAngle` gives the mechanical
position. The absolute reference remains an application question (homing).

**Simulation.** For native_sim, a simple PMSM plant model (ω, θ from torque, currents from voltages) in the
sim hardware, which generates currents and Hall edges. This runs the hot path including fusion in the twister
in a closed loop.

## 6. Milestones

Each milestone builds for BCC and native_sim and keeps the twister green.

| M | Content | Evidence |
|---|---|---|
| M0 | Timing measurement: DWT cycles around the hot path and the quarter-cycle IRQ, min/mean/max in a ring buffer, output via log/shell. Baseline of today's code. | Numbers from the board |
| M1 | Implement D1: RCR=0, CCR preload, quarter-cycle IRQ pended by the hot path, `samples_ready` polling removed. The sim mirrors this. | Test for order and cadence |
| M2 | `HotpathPlan` and plan build in the quarter-cycle IRQ (`sc_in`, `sc_out`, `ff`, `v_inj`, `inv_vdc`, limiter). | Golden test: same inputs, duties as before within tolerance |
| M3 | Slim the hot path: merged Park, no function pointers, `-fno-math-errno`, saturation without a libm call. | Cycles against the M0 baseline |
| M4 | Observer API per D2, error wrap, separate R. | gtest with synthetic profiles (constant ω, ramp, noise) |
| M5 | Hall driver (EXTI, timestamp, plausibility) and sim Hall from the plant model. | ztest with generated edges |
| M6 | Fusion in the quarter-cycle IRQ: sector and edge measurement, speed gating. | Tests: standstill without drift, acceleration and deceleration, Hall failure, switch to flux-only |
| M7 | Control thread with speed controller, setpoint limiting and mailbox. | Sim test: step in ω, current limit takes effect |
| M8 | Board: ADC alignment, offset, Hall table via open-loop run, then closed current controller with the 50 A limit, measurement of the ISR times. | Measurements, oscilloscope |

Order: M0 and M1 first, because D1 decides how much time the hot path has. M2 to M3 then together.
M4 (library change approved). M5/M6 can run in parallel with M7 after M4.

## 7. Risks and Open Points

- **Scope rule (open, decision needed).** The project rule keeps only the hot path in zero-latency
  IRQs and everything else in Zephyr threads. This plan and the current code keep the quarter-cycle
  IRQ (observers, Hall fusion, plan build) as a zero-latency IRQ. Either it stays an IRQ and the rule
  gets an exception, or it becomes a thread with a deadline. The second option changes the double
  buffer and the one-quarter-cycle timing. Until a decision is made, the plan stands as written.
- **ISR time.** The estimate (hot path about 400 cycles, quarter-cycle IRQ about 2500) is not measured.
  If hot path plus quarter-cycle IRQ do not finish within one half period, the hot path start shifts
  and the control loses a half period. M0 clarifies this early. If it is not enough: lower the PWM
  frequency or spread the quarter-cycle work over two half periods.
- **Code and data in flash.** At 170 MHz the flash has wait states (the ART cache mitigates them). If the
  jitter is a problem: place the hot path and the plan in CCM SRAM (Zephyr `CODE_DATA_RELOCATION`). Only
  after measuring.
- **FPU in zero-latency ISRs.** Lazy stacking costs cycles when a thread uses the FPU. Measure it in M0.
- **Hall alignment.** Without a calibrated table, angle errors of up to about ±10° are possible. The edge R
  must match.
- **Start-up.** At standstill the fusion only knows the sector (±30°). That is enough for the start with
  about 87 % torque (cos 30°). The Kalman filter converges at the first edges.
- **Motor data missing.** Maximum electrical rotation frequency, inductance and resistance of the target
  motors are needed for R, the thresholds, the controller parameters and the PWM frequency (current ripple
  at small inductance).
