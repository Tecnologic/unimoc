# Plan: Control Cycle, Duty DMA and Hall Sensor Fusion

Status: 2026-10-20. All times and cycle counts in this document are estimates
from the code. None of them is measured on the board (milestone M0 changes that).

Timing base at 32 kHz PWM and 170 MHz: one half period T_h (one current sample)
is 15.625 µs = 2656 cycles. One control cycle is four half periods, 62.5 µs =
10625 cycles (16 kHz).

## 1. Goal

| Context | Rate | Content |
|---|---|---|
| Sample interrupt (ADC1 JEOS, zero latency) | 64 kHz | Copy the raw currents into the cycle array, overcurrent check on raw counts |
| Control cycle (inside every 4th sample interrupt, zero latency) | 16 kHz | Mean current, PI once, four duty entries (inverse Park, HFI step, dead-time compensation, SVM), duty plan to the hardware |
| Observer (software-pended normal interrupt) | 16 kHz | Flux observer, Kalman, HFI demodulation, later Hall fusion, angles of the next cycle |
| Control thread (Zephyr, normal priority) | about 2 kHz | Speed controller, position controller, MTPA, field weakening, setpoint limiting |
| Hall edges (EXTI, normal priority) | on edge | Only capture timestamp and Hall code |

The control cycle and the sample interrupt are the hot path. The observer is
not zero-latency: it is a normal Zephyr interrupt at the highest normal
priority, so the project rule (only the hot path is zero-latency) holds. This
resolves the open decision of the former quarter-cycle IRQ.

Assumption for the precomputation: speed is constant over one cycle (62.5 µs).
Acceleration is not modeled.

## 2. Timing of One Cycle

Boundaries B are the peaks and valleys of the center-aligned TIM8 counter.
B0 is the control boundary of cycle c.

```
samples     s0      s1      s2      s3 (B0)
            |       |       |       |
------------B-3-----B-2-----B-1-----B0------B1------B2------B3------B4------B5----
                                    |  ctrl  |
duty in effect          ... prev[2] | prev[3]| new[0] | new[1] | new[2] | new[3] |
                                    ^ DMA slot 0 burst (stale, overwritten)
                                             ^ new[0] from the CCR preload
                                                      ^ DMA slot 1, 2, 3 ...
```

- **Sampling.** The ADC injected sequence is triggered `adc_trigger_offset`
  ticks before every boundary (TRGO2 = OC4REF before the peak, OC6REF before the
  valley). The JEOS interrupt copies ia, ib, ic into the cycle array and
  compares them with a raw window (`SetOvercurrentWindow`, from
  `max_phase_current`); a violation sets the break (`EGR = BG`) at once.
- **Control.** The fourth sample interrupt of the cycle runs the control cycle
  (`CurrentControlIsr::OnControlCycle()`), then pends the observer.
- **Duties (variant B).** TIM8 runs with RCR = 0 and CCR1..3 preload. Every
  update event requests a DMA burst of three words (TIM8_DCR/DMAR) from a
  circular buffer of four slots (DMA1 channel 1, DMAMUX request TIM8_UP). The
  burst at B0 (slot 0) writes the value for [B1, B2] before the control cycle
  has run, so the control cycle writes entry 0 directly into the preload after
  that burst; entries 1..3 go into slots 1..3. Slot 0 gets entry 3 as a
  fallback for a late next cycle.
- **Deadline.** Entry 0 must be in the preload before B1: about 14 µs after the
  sample. `SetDutyTicks()` checks the DMA counter: one burst done = normal case,
  no burst done (long trigger lead, control boundary still ahead) = entry 0
  goes into slot 0, anything else counts as an overrun (`DutyOverruns()`).
- **Delay.** Entry 0 takes effect 1 T_h after the sample; the middle of the
  four entries is 3 T_h = 46.9 µs after it. All four entries via DMA (variant A)
  would add one T_h. A classic 16 kHz single-update loop has about 94 µs.
- **Angles (D4).** The observer predicts the Park angle of each sample of the
  next cycle (θ + k·ω·T_h) and the inverse-Park angle of each duty entry
  (θ + (3 + delay + k)·ω·T_h, `output_delay_half_periods` = 1.5).
- **Start.** The counter starts at ARR/2, between both trigger compares, so the
  start produces no spurious ADC trigger. The first boundary is then both the
  first sample and the first DMA burst, and the sample counter starts at the
  last index of a cycle.

## 3. Duty Range

The duties take effect through the preload at the boundaries, so the ISR
timing no longer limits them. What remains:

- **Sampling window.** At a peak the phase with the highest duty switches
  closest to the sample, at a valley the one with the lowest. When that edge
  lies within `trigger lead + 1 µs` of the boundary, the BCC layer drops the
  phase and reconstructs it from the other two (three inline sensors,
  ia + ib + ic = 0). With SVM only one phase switches next to each sample.
  The phase and parity come from the plan (`AppliedTicks()`, `SampleAtPeak()`).
- **Minimum pulse.** 3 % of a PWM period (31.25 µs) is 0.94 µs, above the
  400 ns dead time. Factory `svm_duty_min/max` = 0.03 / 0.97 on the BCC and in
  the sim profile. To be checked on the scope with the bootstrap supply.
- **Dead time.** At 32 kHz the 400 ns dead time is 2.6 % of a half period;
  dead-time compensation matters twice as much as at 16 kHz.

## 4. Data Flow

`SubStepBuffer` (double buffer, flip with release/acquire):

| Field | Writer | Reader |
|---|---|---|
| `sc[4]` Park angle per sample | observer (inactive buffer) | control cycle |
| `sc_out[4]` inverse-Park angle per duty entry | observer (inactive buffer) | control cycle |
| `i_ab_samples[4]`, `i_dq_mean`, `u_dq` | control cycle (active buffer) | observer |

`observer_pending` (set by the control cycle, cleared by the observer after the
flip) hands a cycle to the observer. A control cycle that finds it still set
counts `observer_overruns` and reuses the old angles.

**HFI.** One injection period (four steps +d, −d, +q, −q) is one cycle: entry k
carries step k. Sample k + 1 answers step k, so the observer feeds sample 0 as
step 3 (end of the previous period) and samples 1..3 as steps 0..2. The mean
current of the PI is free of the injection response, because the four samples
cover a whole injection period. The alignment of steps and samples is to be
verified in the sim once HFI is active in a test.

The control thread writes setpoints into a mailbox (seqlock or double buffer)
that the control cycle reads. No kernel functions are called from the
zero-latency contexts; the observer interrupt may call them (for example to
wake the control thread every eighth cycle).

### Control cycle (target 1500 cycles, 9 µs)

1. Samples of the cycle from the hardware (scaled, the disturbed phase
   reconstructed). Repeat of the overcurrent check, latch, neutral plan.
2. Clarke and Park per sample with `sc[k]`, mean.
3. PI with decoupling once per cycle (dt = 62.5 µs).
4. Four entries: inverse Park with `sc_out[k]`, plus HFI step k, dead-time
   compensation from the newest current, SVM.
5. `SetDutyPlan()`: entry 0 to the preload, 1..3 to the DMA slots.

### Observer (budget about 45 µs, target 3000 cycles)

1. Flux observer with mean current and voltage of the cycle.
2. Mechanical prediction (torque from i_d, i_q).
3. HFI demodulation over the four samples (if active), later Hall (M6).
4. Angles of the next cycle, flip.

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
A second zero-latency level is the fallback if M0 shows that kernel critical
sections push the observer past its deadline.

### CPU budget (estimate, per cycle of 10625 cycles)

Sample interrupts 4 × 60, control cycle 1500, observer 3000: about 4700 cycles,
45 % CPU. Everything else (Zephyr, Cyphal, USB, control thread) gets the rest.

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

**Simulation.** The plant of `lib/sim` (`MotorSimulation`) generates currents and Hall codes; the sim
hardware layer does not expose the Hall signals yet (M5).

## 6. Library Decisions (unchanged)

- **D2** `MechanicalObserver`: separate covariance prediction and measurement update
  (`PredictCovariance(dt)`, `UpdateAngle(ε, R)`); `PmsmFluxObserver::calculate()` returns the angle
  error; optional `SinCos` table in the observer. The flux observer runs always, also with an encoder.
- **D3** Decoupling from the mean current of the cycle.
- **D4** Dedicated angle table for the output side (`sc_out`), delay as a parameter.
- **Integer arithmetic: no.** On Cortex-M4F float add and multiply cost one cycle; the time goes into
  function pointers, `sqrtf` (libm, errno) and divisions.
- **STM32H7.** Same concept (advanced timer, TRGO, preload, DMA burst). Budget per half period about
  7500 cycles at 480 MHz; code and plan in ITCM/DTCM, `-fsingle-precision-constant`.

## 7. Simulation (native_sim)

`hardware/zephyr_sim` closes the loop around `MotorSimulation`. A kernel timer in simulated time runs
one control cycle per expiry (exact mean rate, scheduled from the elapsed time because 62.5 µs is not a
whole number of 1 µs ticks):

1. control callback and observer callback on the samples of the cycle that ended,
2. the plant over the next four half periods: the first with entry 3 of the previous plan, the next three
   with entries 0..2 of the new plan; after each half period a sample and the overcurrent check.

That is the timing of variant B. The sim checks the control logic and this timing contract, not the
time budget, ADC noise, quantization or dead time (the inverter model has none). Tests:
`sim_irq_cadence` (cadence and plan timing), `sim_overcurrent` (per-sample trip), `sim_closed_loop`
(current steps on a locked rotor through the real control core).

## 8. Milestones

Each milestone builds for BCC and native_sim and keeps the twister green.

| M | Content | Evidence |
|---|---|---|
| M0 | Timing measurement: DWT cycles around sample interrupt, control cycle and observer, min/mean/max, `DutyOverruns()`, `observer_overruns`; output via log/shell. | Numbers from the board |
| M1 | Done in code: RCR = 0, CCR preload, DMA burst, control cycle in the 4th sample interrupt, pended observer, sim at the 16 kHz cycle. | Twister: `sim_irq_cadence`, `sim_closed_loop` |
| M2 | Done in code: one reciprocal of V_dc per cycle for the four SVM entries (`Svm::CalculateWithInverseDcLinkVoltage`), circle limiter without libm (`vsqrt.f32` through `-fno-math-errno`). Precomputing `v_limit_sq` and `ki·dt` in the observer was dropped: the PI runs once per cycle, so it saves only a few multiplications. | `svm_test`, call-graph check below |
| M3 | Done in code: hot-path hardware access through `hardware/hot_path_interface.hpp` (link-time bound, no function pointers); the platform calls `ControlCycleHandler()`/`ObserverHandler()` of the application directly; `-fno-math-errno` for the Zephyr build. Inlining across the translation units (LTO or header-only access) is open until M0 shows it matters. | Call-graph check below, cycles against the M0 baseline |
| M4 | Observer API per D2, error wrap, separate R. | ztest with synthetic profiles |
| M5 | Hall driver (EXTI, timestamp, plausibility) and Hall signals of the sim plant in the sim layer. | ztest with generated edges |
| M6 | Fusion in the observer: sector and edge measurement, speed gating. | Sim tests: standstill without drift, ramps, Hall failure |
| M7 | Control thread with speed controller, setpoint limiting and mailbox. | Sim test: step in ω, current limit takes effect |
| M8 | Board: DMA and preload on the scope, ADC alignment, Hall table, closed current loop with the 50 A limit. | Measurements, oscilloscope |

### Call-graph check of the zero-latency path (M2/M3)

On the BCC build, everything reachable from `SampleIsr` was listed from the
disassembly (`arm-zephyr-eabi-objdump -d`, following `bl`/`b.w`): the core
(`on_control_cycle`, SVM, dead-time compensation), the BCC glue and driver
functions and `memset`. No libm function (`sqrtf` was there before
`-fno-math-errno`), no indirect call (`blx rN`), no kernel function and no
`__cxa_guard_*`. Repeat the check after changes to the hot path.

## 9. Risks and Open Points

- **Not run on hardware.** DMA burst, preload, the start at ARR/2 and the parity detection
  (`TIM8_CR1.DIR` right after the control boundary) are written from the reference manual and only
  build-tested. M0/M8 must verify them, first with the outputs off.
- **ADC1 JEOS as "all ADCs done".** The sample interrupt reads ADC2/3/4 as well. This assumes equal
  sample times and simultaneous triggers on all ADCs (as before).
- **Overcurrent only in software.** The board has no comparators on the current channels; the check is
  per sample in the zero-latency interrupt (reaction within about 1 µs plus the half period until the
  next sample). The analog watchdog of the ADCs is a possible addition.
- **Observer overrun.** If the observer does not finish within one cycle, the control cycle reuses the
  old angles (and the sample buffer can race); counted, to be measured in M0.
- **FPU in zero-latency ISRs.** Lazy stacking costs cycles when a thread uses the FPU. Measure in M0.
- **Code and data in flash.** At 170 MHz the flash has wait states (the ART cache mitigates them). If the
  jitter is a problem: hot path and buffers in CCM SRAM (Zephyr `CODE_DATA_RELOCATION`). Only after measuring.
- **Check: is the hot path in CCM SRAM on the G4?** Today it is not: the sample ISR, the control cycle and
  their buffers are linked to flash and SRAM1 (nothing uses `CODE_DATA_RELOCATION`). Check the map file
  after any change to the linking. Running from CCM SRAM (32 KB on the G473, I-bus at 0x10000000) would
  also keep the hot path free of the flash stall while a write runs in bank 1 (settings, see
  `docs/settings_plan.md`). The settings gate still holds the stage disabled during a save.
- **Hall alignment.** Without a calibrated table, angle errors of up to about ±10° are possible.
- **Start-up.** At standstill the fusion only knows the sector (±30°): about 87 % torque (cos 30°).
- **Motor data missing.** Maximum electrical frequency, inductance and resistance of the target motors are
  needed for R, the thresholds and the controller parameters.
