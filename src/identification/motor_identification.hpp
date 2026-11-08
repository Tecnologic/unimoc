/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file motor_identification.hpp
 *    @brief Motor identification FSM: stator resistance, dead time, L_d, L_q and psi_PM.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "current_control_isr.hpp"
#include "impedance_estimator.hpp"
#include "nvm_settings.hpp"
#include "resistance_regression.hpp"
#include "slow_update.hpp"
#include "units.hpp"

namespace unimoc::identification {

/// Steps of the identification; the step number of the commissioning status message.
enum class Step : uint8_t {
  kIdle = 0,         ///< Never started.
  kResistance = 1,   ///< M1: stator resistance and dead-time voltage (standstill).
  kInductanceD = 2,  ///< M2: L_d by a sine on the d axis (standstill).
  kInductanceQ = 3,  ///< M2: L_q by a sine on the q axis (standstill).
  kDone = 4,         ///< All requested measurements ended; the results are valid.
  kFault = 5,        ///< Ended early: aborted, an emergency stop, a trip or a lost enable.
  kFluxLinkage = 6,  ///< M3: psi_PM by a coast-down (shaft free; after DONE and FAULT, a later step).
};

/// The measurements a run takes (bits of the START_IDENTIFICATION parameter).
enum Measurement : uint8_t {
  kMeasureResistance = 1U << 0U,   ///< "M1"
  kMeasureInductance = 1U << 1U,   ///< "M2"
  kMeasureFluxLinkage = 1U << 2U,  ///< "M3" (spins the motor: only on request)
};

/// Why a run ended in Step::kFault (the fault of the commissioning status message).
enum class Fault : uint8_t {
  kNone = 0,
  kAborted = 1,
  kEmergencyStop = 2,
  kOvercurrent = 3,
  kTrip = 4,
  kEnableLost = 5,
};

/// Answer to a start request.
enum class StartResult : uint8_t {
  kStarted,        ///< The run began; the outputs are released.
  kRunning,        ///< A run is active already.
  kNotReleased,    ///< The power stage could not be released (external enable, fault).
  kNoMeasurement,  ///< No known measurement was requested.
};

/// Answer to a confirmation (CONTINUE).
enum class ConfirmResult : uint8_t {
  kAccepted,           ///< The step waited for it and goes on.
  kNotWaiting,         ///< No step waits for a confirmation.
  kWrongConfirmation,  ///< The step waits for another confirmation.
};

/// Results of the last run; valid flags per measurement.
struct Results {
  bool resistance_valid{false};     ///< M1 ended with a fit.
  unit::Resistance resistance;      ///< Stator phase resistance R_s.
  unit::Voltage dead_time_voltage;  ///< Dead-time voltage of the regression (alpha/beta amplitude).
  unit::Time dead_time;             ///< Dead time of the gate driver, as the dead-time compensation takes it.
  bool inductance_valid{false};     ///< M2 ended with both axes.
  unit::Inductance l_d;             ///< d-axis inductance.
  unit::Inductance l_q;             ///< q-axis inductance.
  unit::Inductance l_mean;          ///< Mean of L_d and L_q (`unimoc.motor.stator.L`).
  unit::Resistance resistance_ac;   ///< Mean real part of the M2 impedances (a check of R_s).
  bool flux_valid{false};           ///< M3 measured at least one period.
  unit::MagneticFlux flux_pm;       ///< Permanent-magnet flux linkage psi_PM (`unimoc.motor.pmsm.flux_pm`).
  unit::Ratio flux_spread;          ///< Largest relative deviation of a period's psi from the mean.
  unit::AngularVelocity coast_speed;  ///< Electrical speed at the start of the coast.
  uint32_t flux_periods{0U};          ///< Electrical periods in the result.
};

/// State of the run for the commissioning status message.
struct Status {
  Step step{Step::kIdle};
  bool active{false};
  uint8_t progress{0U};  ///< Of the present step [%].
  uint32_t passed{0U};   ///< One bit per step that passed its checks.
  uint32_t failed{0U};   ///< One bit per step that failed a check (the run goes on).
  Fault fault{Fault::kNone};
  uint32_t counter{0U};          ///< Operating points measured in the present step.
  bool needs_shaft_free{false};  ///< The step waits for the confirmation "shaft-free" (CONTINUE).
};

/**
 * @brief The motor identification (docs/motor_identification_plan.md, milestones I0 to I2).
 *
 * Runs in the Cyphal thread like the startup aid, one RunOnce() per spin.
 * Its clock is the control cycle: every wait is a measurement window of
 * SlowUpdate counted in control cycles, so a run takes the same virtual time
 * on the STM32H7, in the free-running and in the paused simulation.
 *
 * - M1 holds a d current at a forced angle (0, 120 and 240 degrees
 *   electrical, so each phase carries the full current once) at five levels
 *   of each sign, 10 % to 50 % of the motor current limit, without the
 *   dead-time compensation. Each level settles, then its mean voltage demand
 *   and current go into the regression u = R i + U_dt sign(i). The three
 *   slopes must agree within 10 %.
 * - M2 holds a d bias (20 % of the limit) at angle 0, freezes the current PI
 *   (the voltage is held) and adds a sine at f_pwm / 64 on the d axis, then
 *   on the q axis; its amplitude is scaled until the current answers with 10 %
 *   of the limit. The impedance U / I gives L from its imaginary part; its
 *   real part must agree with R_s of M1 within 20 %.
 *
 * - M3 (shaft free, only on request) waits for the operator's confirmation
 *   that the shaft turns freely, aligns the rotor with a q current of 30 %
 *   of the limit at a forced angle and spins it up by I/f (the forced angle
 *   ramps up at that current) until the back-EMF estimate |u - R i| reaches
 *   25 % of the DC link line to line, at most half the speed limit. Then it
 *   opens the outputs: the motor coasts and the phase-voltage sense measures
 *   its back-EMF (CoastDownMeter): psi = |e| / omega per electrical period,
 *   the frequency from the zero crossings. It passes with at least three
 *   periods within 3 % of their mean. It needs a platform with a
 *   phase-voltage sense.
 *
 * The outputs are released by the run and never while the drive is engaged;
 * a trip, a lost enable, the abort command or the emergency stop end it with
 * the outputs off. The run writes nothing to the settings: the operator
 * reviews and applies the results (plan section 5.5).
 */
class MotorIdentification {
 public:
  /// Levels of M1 per sign, as fractions of the motor current limit.
  static constexpr std::array<float, 5> kResistanceLevels{0.1F, 0.2F, 0.3F, 0.4F, 0.5F};
  /// Electrical angles of M1 [deg].
  static constexpr std::array<float, 3> kResistanceAngles{0.0F, 120.0F, 240.0F};
  /// d bias of M2, as a fraction of the motor current limit.
  static constexpr float kInductanceBias = 0.2F;
  /// Target amplitude of the injected current of M2, as a fraction of the limit.
  static constexpr float kInjectionCurrent = 0.1F;
  /// Accepted range of the injected current amplitude relative to the target.
  static constexpr float kInjectionTolerance = 0.25F;
  /// Most amplitude adaptations of one M2 axis.
  static constexpr uint8_t kMaxInjectionTries = 4U;
  /// Injection periods of the measurement window of an M1 level and of an M2 axis (50 ms at 500 Hz).
  static constexpr uint32_t kWindowPeriods = 25U;
  /// Shortest and longest settle time of a level [control cycles].
  static constexpr uint32_t kMinSettleCycles = 320U;
  static constexpr uint32_t kMaxSettleCycles = 8000U;
  /// Relative spread of the three M1 slopes that passes.
  static constexpr float kSlopeTolerance = 0.10F;
  /// Relative difference of the M2 resistance from M1 that passes.
  static constexpr float kAcResistanceTolerance = 0.20F;
  /// q current of the M3 spin-up, as a fraction of the motor current limit.
  static constexpr float kSpinUpCurrent = 0.3F;
  /// Line-to-line back-EMF peak at the end of the spin-up, as a fraction of the DC link.
  static constexpr float kSpinUpBackEmf = 0.25F;
  /// Highest spin-up speed, as a fraction of the forward speed limit.
  static constexpr float kSpinUpSpeedFraction = 0.5F;
  /// Time of the ramp from standstill to the highest spin-up speed [s].
  static constexpr float kSpinUpRampSeconds = 5.0F;
  /// Alignment before the ramp, and the hold at the spin-up speed [control cycles].
  static constexpr uint32_t kAlignCycles = 8000U;
  static constexpr uint32_t kHoldCycles = 3200U;
  /// Window of the spin-up checks [control cycles].
  static constexpr uint32_t kRampWindowCycles = 64U;
  /// Longest coast [control cycles] (60 s at 16 kHz).
  static constexpr uint32_t kCoastTimeoutCycles = 960000U;
  /// Fewest periods and their spread that pass M3.
  static constexpr uint32_t kMinFluxPeriods = 3U;
  static constexpr unit::Ratio kFluxSpreadTolerance = unit::operator""_ratio(0.03L);

  /**
   * @param isr Current-control ISR; must outlive the FSM.
   * @param slow Observer of the ISR (forced angle, measurement windows); must outlive the FSM.
   */
  MotorIdentification(current_control::CurrentControlIsr& isr, current_control::SlowUpdate& slow) noexcept
      : isr_{&isr}, slow_{&slow} {}

  /**
   * @brief Takes the limits and the prior motor data of the settings.
   *
   * The current levels follow `motor_i_max`; the settle time is five time
   * constants L/R of the present settings. Not during a run.
   */
  void Configure(const settings::NvmSettings& settings) noexcept;

  /**
   * @brief Starts a run of the requested measurements.
   * @param measurements Bits of Measurement.
   */
  [[nodiscard]] StartResult Start(uint8_t measurements) noexcept;

  /// Ends a running identification with the outputs off; no-op otherwise.
  void Abort(Fault reason = Fault::kAborted) noexcept;

  /**
   * @brief The operator's confirmation that the shaft turns freely (M3).
   * @param shaft_free True for the confirmation "shaft-free", false for any other.
   */
  [[nodiscard]] ConfirmResult Confirm(bool shaft_free) noexcept;

  /// True while a run is active.
  [[nodiscard]] bool IsActive() const noexcept {
    return step_ != Step::kIdle && step_ != Step::kDone && step_ != Step::kFault;
  }

  /// True once a run was started since the boot.
  [[nodiscard]] bool Started() const noexcept { return started_; }

  /// Advances the run; called once per Cyphal spin.
  void RunOnce() noexcept;

  /// The state of the run.
  [[nodiscard]] Status GetStatus() const noexcept;

  /// The results of the last run.
  [[nodiscard]] const Results& GetResults() const noexcept { return results_; }

 private:
  /// What the present measurement window is for.
  enum class Phase : uint8_t {
    kBias,           ///< M2: the PI sets up the d bias; the result is ignored.
    kSettle,         ///< Waits for the controller and the rotor; the result is ignored.
    kMeasure,        ///< The window is the measurement of the operating point.
    kWaitShaftFree,  ///< M3: outputs on at zero current until the operator confirms a free shaft.
    kAlign,          ///< M3: the q current at a fixed angle aligns the rotor.
    kRamp,           ///< M3: the forced angle ramps up at the q current.
    kHold,           ///< M3: the spin-up speed holds.
    kCoast,          ///< M3: the outputs are open, the coast-down meter runs.
  };

  void Finish(Step step, Fault fault) noexcept;
  void StartResistancePoint() noexcept;
  void StartInductanceAxis(Step step) noexcept;
  void OnResistanceWindow(const current_control::IdentificationWindowResult& window) noexcept;
  void OnInductanceWindow(const current_control::IdentificationWindowResult& window) noexcept;
  void FinishResistance() noexcept;
  void FinishInductance() noexcept;
  void StartFluxLinkage() noexcept;
  void RunFluxLinkage() noexcept;
  void OnSpinUpWindow(const current_control::IdentificationWindowResult& window) noexcept;
  void StartCoast() noexcept;
  void FinishFluxLinkage() noexcept;
  void DriveSpin(unit::Angle angle, unit::AngularAcceleration acceleration) noexcept;
  void Next() noexcept;
  void Drive(
      unit::Current d_current, unit::Angle angle, bool hold, unit::Voltage amplitude, uint8_t axis) noexcept;
  [[nodiscard]] unit::Current Level(uint32_t point) const noexcept;
  [[nodiscard]] uint32_t SettleCycles() const noexcept;
  /// Measurement window: kWindowPeriods injection periods [control cycles].
  [[nodiscard]] uint32_t WindowCycles() const noexcept;

  current_control::CurrentControlIsr* isr_;
  current_control::SlowUpdate* slow_;

  // Configuration from the settings.
  unit::Current current_max_;
  unit::Resistance resistance_prior_;
  unit::Inductance inductance_prior_;

  Step step_{Step::kIdle};
  Phase phase_{Phase::kSettle};
  uint8_t requested_{0U};
  unit::AngularVelocity speed_limit_;
  bool started_{false};
  Fault fault_{Fault::kNone};
  uint32_t passed_{0U};
  uint32_t failed_{0U};
  uint32_t point_{0U};  ///< Operating point of the step (M1: angle * 10 + level; M2: the tries).
  unit::Voltage amplitude_;
  unit::Voltage dc_link_;

  std::array<ResistanceRegression, kResistanceAngles.size()> per_angle_{};
  ResistanceRegression all_points_{};
  Results results_{};
};

}  // namespace unimoc::identification
