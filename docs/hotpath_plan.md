# Plan: Control Cycle, Duty DMA and Hall Sensor Fusion

> **2026-11-03:** the battery case controller (BCC, STM32G473) and its
> platform `hardware/zephyr_bcc` are removed. BCC references below are
> history; the STM32H7 platform (`hardware/zephyr_stm32h7`, Jacobi and the
> NUCLEO-H743ZI) carries the same design.

> **2026-11-03 (per-sample control):** the current PI runs on every sample
> (every half PWM period, 64 kHz at 32 kHz PWM) instead of once per four
> samples, and the control step writes the CCR preload directly. The duty DMA
> and the four-entry duty plan are gone, and so is HFI: Hall sensors cover
> low speed. Sections 1 to 4 describe this design. The earlier
> four-entry design is in the git history (before this change).

Status: 2026-11-03. The times in section 4 are measured on the NUCLEO-H743ZI
(400 MHz, power stage off, no motor), the rest are estimates.

Timing base at 32 kHz PWM and 400 MHz (STM32H743): one half period T_h (one
current sample, one control step) is 15.625 µs = 6250 cycles. One control
cycle (the observer period) is four half periods, 62.5 µs = 25000 cycles (16 kHz).

## 1. Goal

| Context | Rate | Content |
|---|---|---|
| Sample interrupt (ADC1 JEOS, zero latency) | 64 kHz | Raw copy, overcurrent and overvoltage check on raw counts, then the control step |
| Control step (inside every sample interrupt, zero latency) | 64 kHz | Calibration, angle step, Park, PI (dt = T_h), inverse Park, dead-time compensation, SVM, duties into the CCR preload |
| Observer (software-pended normal interrupt) | 16 kHz | Flux observer, Kalman, Hall fusion, angle reference of the next cycle |
| Control thread (Zephyr, normal priority) | about 2 kHz | Speed controller, position controller, MTPA, field weakening, setpoint limiting |
| Hall edges (EXTI, normal priority) | on edge | Only capture timestamp and Hall code |

The sample interrupt with its control step is the hot path. The observer is
not zero-latency: it is a normal Zephyr interrupt at the highest normal
priority, so the project rule (only the hot path is zero-latency) holds.

Assumption for the angle extrapolation: speed is constant between two
observer runs (62.5 µs, longer when the observer is late). Acceleration is not
modeled.

## 2. Timing of One Sample

Boundaries B are the peaks and valleys of the center-aligned counter.

```
            B(n-1)          B(n)            B(n+1)          B(n+2)
------------|---------------|---------------|---------------|------
duty        |    w(n-2)     |    w(n-1)     |     w(n)      |
                         ^ sample n (trigger lead before B(n))
                            ^ JEOS, control step n writes w(n) into the preload
                                            ^ w(n) takes effect
```

- **Sampling.** The ADC injected sequence is triggered `adc_trigger_offset`
  ticks before every boundary (TRGO2 = OC4REF before the peak, OC6REF before the
  valley). The JEOS interrupt reads ia, ib, ic and the DC link and compares
  them with the raw window (`SetOvercurrentWindow`, from `max_phase_current`)
  and the raw overvoltage level; a violation sets the break (`EGR = BG`) at once.
- **Control step.** The same interrupt then calls `SampleHandler()`
  (`CurrentControlIsr::OnSample()`): it writes the duties of the next half
  period into CCR1..3 (preload). Every fourth step ends a control cycle and
  pends the observer.
- **Deadline.** The duties must be in the preload before the next boundary:
  T_h minus the end of the ADC sequence after the boundary. `SetDutyTicks()`
  counts a write after that boundary as an overrun (`DutyOverruns()`): the
  counter is then in the other half and moves away from the next boundary.
  The end of the sequence must come after the boundary of its trigger (lead
  shorter than the sequence, about 2.4 µs with the sample times of the
  Nucleo); with a longer lead the duties take effect half a period early.
- **Delay.** w(n) takes effect one half period after sample n and its middle
  is 1.5 T_h = 23.4 µs after it (`output_delay_half_periods`). The four-entry
  design had 4.5 T_h (70.3 µs) from the middle of the averaged samples to the
  middle of the entries, and a classic 16 kHz single-update loop has about 94 µs.
- **Angles (D4).** The observer hands over one angle reference per cycle
  (`AngleReference`): the angle at the first sample of the next cycle, the
  rotation of one half period (ω·T_h) and the rotation of the output delay
  (ω·1.5 T_h). Each control step rotates its angle by one step (four
  multiplications, `system::Rotated()`, renormalized) and takes the new
  reference when the observer has published one, rotating it forward by the
  samples it is late. A late observer means a longer extrapolation, not stale
  angles.
- **Phase of a sample.** The JEOS of a peak sample comes in the upper half of
  the counter (before or after the peak), the one of a valley in the lower
  half; the sample interrupt reads `CNT` to know the boundary type
  (`SampleAtPeak()`), as long as lead and sequence stay below a quarter period.

## 3. Duty Range and Current Bandwidth

The duties take effect through the preload at the boundaries, so the ISR
timing does not limit them. What remains:

- **Sampling window.** At a peak the phase with the highest duty switches
  closest to the sample, at a valley the one with the lowest. When that edge
  lies within `trigger lead + 1 µs` of the boundary, the STM32H7 layer drops
  the phase and reconstructs it from the other two (three inline sensors,
  ia + ib + ic = 0). The compare values active during the sample are the ones
  written two steps before (`AppliedTicks()`).
- **Minimum pulse.** 3 % of a PWM period (31.25 µs) is 0.94 µs. Factory
  `svm_duty_min/max` = 0.03 / 0.97 in the sim profile. To be checked on the
  scope with the bootstrap supply.
- **Dead time.** At 32 kHz the dead time is a larger part of a half period
  than at 16 kHz; dead-time compensation runs on every step from the
  current of its sample.

**Current bandwidth.** With pole-zero cancellation (kp = L·ω_c, ki = R·ω_c)
the open loop is an integrator with the delay T_d from a sample to the middle
of its duties; the phase margin is 90° − ω_c·T_d.

| Design | T_d at 32 kHz | ω_c for 60° margin | for 45° margin |
|---|---|---|---|
| PI every sample (now) | 1.5 T_h = 23.4 µs | 22.3 krad/s (3.6 kHz) | 33.5 krad/s (5.3 kHz) |
| PI every fourth sample (before) | 4.5 T_h = 70.3 µs | 7.4 krad/s (1.2 kHz) | 11.2 krad/s (1.8 kHz) |

In the discrete loop (dt = T_h, one step of delay) the 60° tuning,
kp = L·π / (9 T_h), gives poles with about 5 % overshoot. `sim_closed_loop`
runs a 10 A step with it on a locked rotor (100 µH, 50 mΩ): 60 % after one
control cycle (62.5 µs), within 0.5 A after 125 µs. The factory gains are
still the conservative 2000 rad/s of the hoverboard default; raise them per
motor (`unimoc.control.current.*`).

## 4. Data Flow

`AngleBuffer` (observer → control steps, double buffer, flip with
release/acquire) and a pair of `CycleRecord`s (control steps → observer):

| Field | Writer | Reader |
|---|---|---|
| `AngleReference` (sample, angle, step, output rotation) | observer (inactive buffer, then flip) | every control step |
| `CycleRecord` `sc[4]`, `i_ab_samples[4]` | control step k of the cycle | observer, identification windows |
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
4. PI with decoupling (dt = T_h).
5. Inverse Park with the angle at the middle of the next half period,
   dead-time compensation from this sample's current, SVM (one division by
   V_dc), duties into the preload.

### Observer

1. Flux observer with the mean current and voltage of the cycle.
2. Mechanical prediction (torque from i_d, i_q).
3. Hall edges and sector.
4. Angle reference of the next cycle, flip.

### Measured on the NUCLEO-H743ZI (M0)

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

Open: the observer with the flux observer and Hall fusion running, and the
hot path in ITCM (code and data in flash and AXI SRAM today).

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
  Budget per half period 6250 cycles at 400 MHz (rev. Y); ITCM/DTCM and
  `-fsingle-precision-constant` are open options if the measurements need them.

## 7. Simulation (native_sim)

`hardware/zephyr_sim` closes the loop around `MotorSimulation`. A kernel timer in simulated time runs
four half periods (one control cycle) per expiry (exact mean rate, scheduled from the elapsed time
because 62.5 µs is not a whole number of 1 µs ticks). Each half period:

1. the plant with the duties in effect,
2. the sample at its end and the per-sample overcurrent and overvoltage checks; at the boundary the
   duties of the last control step take effect (the preload),
3. the sample handler (control step), and the observer when the step ended a control cycle.

The sim checks the control logic and this timing contract, not the time budget, ADC noise,
quantization or dead time (the inverter model has none). Tests: `sim_irq_cadence` (cadence and the
half period the duties of a step apply to), `sim_overcurrent` (per-sample trip), `sim_closed_loop`
(current steps on a locked rotor through the real control core, one at the 3.6 kHz tuning).

## 8. Milestones

Each milestone builds for the STM32H7 boards and native_sim and keeps the twister green.

| M | Content | Evidence |
|---|---|---|
| M0 | Instrumentation (`CONFIG_UNIMOC_HOT_PATH_TIMING`, `hardware/hot_path_timing.hpp`): DWT cycles around every sample interrupt, the control step and the observer, min/mean/max, `DutyOverruns()`, `observer_overruns`; read over the `unimoc.timing.*` registers. First numbers from the NUCLEO-H743ZI (2026-11-03, section 4, outputs off). Open: with the flux observer and Hall fusion running. | Numbers from the board |
| M1 | Done: RCR = 0, CCR preload, pended observer, sim at the 16 kHz cycle. Since 2026-11-03 the control step runs in every sample interrupt and writes the preload; the duty DMA is gone. | Twister: `sim_irq_cadence`, `sim_closed_loop`; Nucleo: 64.3 k steps/s, no overrun |
| M2 | Done in code: circle limiter without libm (`vsqrt.f32` through `-fno-math-errno`). The control step makes one division by V_dc for its SVM. | `svm_test`, call-graph check below |
| M3 | Done in code: hot-path hardware access through `hardware/hot_path_interface.hpp` (link-time bound, no function pointers); the platform calls `SampleHandler()`/`ObserverHandler()` of the application directly; `-fno-math-errno` for the Zephyr build. Inlining across the translation units (LTO or header-only access) and ITCM placement are open; `-O2` saves about 15 % (section 4). | Call-graph check below, cycles against the M0 baseline |
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

## 9. Risks and Open Points

- **Not verified on the scope.** The control step runs on the Nucleo at 64 kHz without a late
  preload write (2026-11-03), but the PWM edges, the boundary type from `CNT` and the ADC
  alignment are not checked against the outputs yet (M8, first with the outputs off).
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
- **Code and data in flash.** The hot path runs from flash through the M7 caches (the
  sample interrupt varies between 1631 and 2276 cycles). ITCM (64 KB, zero wait states, Zephyr
  `CODE_DATA_RELOCATION`) would cut the time and the jitter and keep the hot path free of the
  flash stall of a settings write; the settings gate holds the stage disabled during a save.
- **Hall alignment.** Without a calibrated table, angle errors of up to about ±10° are possible.
- **Start-up.** At standstill the fusion only knows the sector (±30°): about 87 % torque (cos 30°).
  HFI was removed (2026-11-03): the Hall sensors are the low-speed angle source.
- **Motor data missing.** Maximum electrical frequency, inductance and resistance of the target motors are
  needed for R, the thresholds and the controller parameters.
