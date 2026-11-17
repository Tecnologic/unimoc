# Plan: Motor Temperature Estimate and Loss-Aware Battery Current Limit

Status: 2026-11-17. Plan; step T1 in progress. Nothing is verified on
hardware.

## 1. Goal

The drive estimates the winding temperature of its motor without a sensor
and uses it:

- to derate the motor current before the winding overheats,
- to follow the stator resistance R_s(T) where the control uses it
  (battery current estimate; later the flux observer),
- to report the motor temperature on the bus (`unimoc.cart.DriveStatus`,
  `unimoc.drive.Telemetry`) when no sensor is fitted.

The battery current limit (`control::DcLinkLimit`) gets the copper losses,
so that it also limits at standstill and low speed, where today the
mechanical power and with it the estimated battery current is zero.

## 2. Decisions

1. **Two thermal masses (Cauer network), driven by the copper losses.**
   Winding (`C_w`) and stator/housing (`C_s`), the winding to housing
   resistance `R_ws` and the housing to ambient resistance `R_sa`:

   ```text
   C_w dT_w/dt = P_cu − (T_w − T_s) / R_ws
   C_s dT_s/dt = (T_w − T_s) / R_ws − (T_s − T_amb) / R_sa
   P_cu        = 3/2 · R_s(T_w) · |i_dq|²         (amplitude-invariant Park)
   R_s(T)      = R_s,ref · (1 + α_Cu · (T − T_ref)),  α_Cu = 3.93e-3 /K
   ```

   The temperature dependence of R_s is part of the model: an i²t model with
   a constant resistance underestimates a hot winding by 20 to 30 %. Iron
   losses are left out (a `k_fe ω²` term can follow when a motor needs it).
   The step is an explicit Euler step of the control thread (2 kHz at a
   16 kHz control cycle); the smallest time constant is tens of seconds, so
   the step is far inside the stability bound. The model clamps the step at
   a fraction of the smallest time constant so a long gap (thread stall,
   test) cannot make it unstable.
2. **R_s is measured at a stated temperature.** New register
   `unimoc.motor.stator.R_temperature` [°C], default 25 °C: the winding
   temperature at which `unimoc.motor.stator.R` was measured. The
   identification (M1) writes it together with R when the platform measures
   a motor or ambient temperature; otherwise the user sets it.
3. **R_s-based correction at low speed.** The resistance seen by the
   current controller gives the winding temperature directly:
   `T_Rs = T_ref + (R_est / R_s,ref − 1) / α_Cu`. The estimate uses the
   power balance of the rotor frame per control cycle,

   ```text
   u_d i_d + u_q i_q = R_s |i|² + ω_el ψ_pm i_q + ω_el (L_d − L_q) i_d i_q
   ```

   as a least-squares fit through the origin over a window
   (`R_est = Σ(u·i − ω ψ i_q − ω ΔL i_d i_q) / Σ|i|²`), only from cycles with
   `|ω_el|` below `unimoc.motor.thermal.rs_speed_max` and `|i|` above
   `unimoc.motor.thermal.rs_current_min`. At low speed and high current the
   resistive voltage dominates; the residual inverter error (dead time after
   compensation, switch drops) is a voltage offset that the current threshold
   keeps small. A window with too few valid cycles is discarded. 1 % error
   in R_s is 2.5 K, so the estimate corrects the model slowly, not replaces
   it: `T_w += (T_Rs − T_w) · dt_window / τ_corr` (a first-order observer,
   `unimoc.motor.thermal.rs_correction_time`, 0 disables). The housing
   temperature is shifted by the same amount, so the correction keeps the
   modelled gradient.
4. **The sums come from the observer, the model runs in the control thread.**
   `SlowUpdate` (normal-priority interrupt, after each control cycle) adds
   `|i|²` and the regression terms of the cycle to running sums; the control
   thread takes the sums of the cycles since its last step. Nothing new
   runs in the zero-latency context.
5. **Start value.** Without a sensor the model starts at the ambient
   temperature (`unimoc.motor.thermal.ambient`, default 25 °C) and the first
   valid R_s window pulls it to the real winding temperature. A dedicated
   standstill R_s measurement when the drive is armed (a short d current, two
   levels, as M1) is step T5 and optional: it touches the arming sequence and
   is done only after the online estimate works in the simulation.
6. **A motor sensor wins.** When the platform measures the motor temperature
   (`TemperatureMeasurements::motor`, Jacobi PC5), it corrects the housing
   mass (most sensors sit on the end winding or the stator iron) with its own
   time constant; the R_s correction still acts on the winding. Without a
   sensor the bus reports the estimate.
7. **Thermal derating.** The motor current limit falls linearly from
   `unimoc.motor.thermal.derate_start` (default 100 °C) to zero at
   `unimoc.motor.thermal.limit` (default 130 °C; class F insulation is
   155 °C). The derated limit replaces `current_limit` in the outer loop, so
   the speed controller bounds and the torque limits of `DriveStatus` follow
   it. A new flag "thermally derated" goes into a later minor version of
   `unimoc.cart.DriveStatus`; until then the reduced torque limits carry it.
8. **Battery current with copper losses.** `DcLinkLimit` estimates the
   battery power as `P = k_t i_q ω + 3/2 R_s(T) |i|²` (inverter losses left
   out). The limit is then a quadratic in i_q; `Limit()` solves it in closed
   form for the largest i_q of the requested sign within the motoring and
   the charge window, `AllowedTorque()` gets the same bound for the speed
   controller. At standstill the drive limit now bounds the current to
   `sqrt(2 V_dc I_drive / (3 R_s))`. While braking the copper losses lower
   the charge current, so the charge limit stays on the safe side.
   `DriveStatus.dc_link_current` uses the same estimate.
9. **Battery limits stay with the BMS and `unimoc.cart.battery_share`**
   (user decision 2026-11-17). The VDC knows the share and gets the
   resulting torque limits through `DriveStatus`; `WheelCommand` is not
   changed.

## 3. Registers

| Register | Unit | Default | Meaning |
|---|---|---|---|
| `unimoc.motor.stator.R_temperature` | °C | 25 | Winding temperature at which `unimoc.motor.stator.R` was measured |
| `unimoc.motor.thermal.ambient` | °C | 25 | Ambient (coolant) temperature, start value without sensor |
| `unimoc.motor.thermal.C_winding` | J/K | 150 | Heat capacity of the winding |
| `unimoc.motor.thermal.C_stator` | J/K | 1500 | Heat capacity of stator and housing |
| `unimoc.motor.thermal.R_winding` | K/W | 0.4 | Thermal resistance winding to housing |
| `unimoc.motor.thermal.R_stator` | K/W | 0.8 | Thermal resistance housing to ambient |
| `unimoc.motor.thermal.derate_start` | °C | 100 | Winding temperature where the current derating starts |
| `unimoc.motor.thermal.limit` | °C | 130 | Winding temperature at zero current |
| `unimoc.motor.thermal.rs_speed_max` | rad/s (el.) | 100 | Largest electrical speed of an R_s window |
| `unimoc.motor.thermal.rs_current_min` | A | 3 | Smallest current magnitude of an R_s window |
| `unimoc.motor.thermal.rs_correction_time` | s | 30 | Time constant of the R_s correction; 0 off |

The defaults are a guess for the hoverboard hub motor (copper about
0.4 kg, stator and rim about 3 kg; 50 W copper loss at 15 A gives a
steady rise of about 60 K). They give a winding time constant of about 60 s
and a housing time constant of about 20 min. Real values come from a heat
run: constant current, the winding temperature from R_s over time.

## 4. Simulation

The plant (`lib/sim`) gets the same two-mass network with its own
parameters (`unimoc.sim.motor_thermal_*`, ambient) and a stator resistance
that follows the winding temperature. The plant's true winding temperature
is traced, so the tests compare the estimate with it, including a model
started too cold (the R_s correction must pull it in) and wrong thermal
parameters in the drive.

## 5. Steps

- **T1 – Thermal model (lib).** `lib/motor_control/src/motor_thermal_model.hpp`:
  parameters, state, `Step(i², dt)`, `StatorResistance()`, `Correct()`,
  `CurrentLimitScale()`. ztest in `tests/lib/motor_control`: steady state
  `T_w = T_amb + P (R_ws + R_sa)`, winding time constant, R_s(T), derating,
  correction, large-step robustness.
- **T2 – R_s estimator (lib).** `lib/motor_control/src/stator_resistance_estimator.hpp`:
  the window sums of decision 3 and the validity gates. ztest with
  synthetic cycles: exact R at standstill, speed and saliency terms
  removed, gates reject high speed and low current.
- **T3 – Plant temperature.** `lib/sim`: thermal network and R_s(T) in the
  PMSM model, sim registers, trace. ztest in `tests/lib/sim`.
- **T4 – Drive integration.** Settings and registers (section 3),
  `SlowUpdate` sums, model and correction in `OuterLoop::Step()`, derated
  current limit, estimate on the bus when no sensor is fitted, sim scenario
  (heat up at standstill, estimate within 5 K of the plant; cold start
  pulled in). Docs: `README.md`, `CONTEXT.md`.
- **T5 – Copper losses in `DcLinkLimit`.** Quadratic limit of decision 8,
  ztest in `dc_link_limit_test.cpp`, sim check of the battery current at
  standstill against the DC-link model.
- **T6 – Optional:** standstill R_s measurement when armed (decision 5),
  motor sensor fusion on Jacobi (decision 6), R_s(T) in the flux observer,
  the "thermally derated" flag in `DriveStatus`.

## 6. Hardware assumptions and open points

- α_Cu holds for copper windings; aluminium windings need 4.03e-3 /K
  (a register if such a motor appears).
- The R_s correction relies on the dead-time compensation and on the
  commanded voltage reaching the motor. On Jacobi the measured phase
  voltages can replace the commanded ones; to be checked on the bench.
- The thermal parameters of a real motor need a heat run on the bench.
- The ambient temperature is a register; the power stage temperature may be
  a better proxy for an enclosed drive (to decide on hardware).
