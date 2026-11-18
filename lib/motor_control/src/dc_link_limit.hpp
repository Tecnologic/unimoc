/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dc_link_limit.hpp
 *    @brief Battery current limits with copper losses and the DC-link overvoltage derating of regenerative
 * braking.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <limits>
#include "rotor_system.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::control control algorithms namespace
 */
namespace unimoc::control {

/**
 * @brief Keeps the battery current of a drive within its limits.
 *
 * The battery power is estimated from the mechanical power of the q current
 * and the copper losses of the winding (docs/motor_thermal_plan.md,
 * decision 8):
 *
 *     P = k_t i_q omega + 3/2 R_s (i_d^2 + i_q^2),   I_bat = P / V_dc
 *
 * with the torque constant k_t = 3/2 p psi_pm. The drive limit takes the
 * copper losses, so it also holds at standstill, where the mechanical power
 * is zero: the current is bounded to sqrt(2 V_dc I_drive / (3 R_s)). The
 * charge limit takes the mechanical power alone: every loss lowers the real
 * charge current below it, also when R_s or its temperature is estimated too
 * high (a loss-aware charge limit would then let too much current into the
 * battery). The inverter losses are left out; when motoring the real battery
 * current is somewhat higher than the estimate. A stator resistance of zero
 * gives the mechanical power alone.
 *
 * Regenerative braking is derated with the DC-link voltage: the charge limit
 * falls linearly from its full value at `regen_derate_start` to zero at
 * `trip`, where the control cycle trips the power stage (section 7.1 of
 * docs/system_plan.md). Only the q current is limited; the d current is
 * kept (it only adds losses).
 */
struct DcLinkLimit {
  /// Largest battery discharge (motoring) current.
  unit::Current drive_current_max;
  /// Largest battery charge (regenerative) current.
  unit::Current charge_current_max;
  /// DC-link voltage above which the charge limit falls.
  unit::Voltage regen_derate_start;
  /// DC-link voltage at which no charge current is allowed.
  unit::Voltage trip;
  /// Stator phase resistance of the copper losses (at the winding temperature); zero: none.
  unit::Resistance stator_resistance;

  /**
   * @brief Charge current allowed at a DC-link voltage.
   * @param dc_link DC-link voltage.
   * @return The full charge limit up to `regen_derate_start`, zero from `trip`.
   */
  [[nodiscard]] constexpr unit::Current AllowedChargeCurrent(const unit::Voltage dc_link) const noexcept {
    if (!(dc_link > regen_derate_start)) {
      return charge_current_max;
    }
    if (!(dc_link < trip)) {
      return unit::Current{};
    }
    return charge_current_max * ((trip - dc_link) / (trip - regen_derate_start));
  }

  /// Range of the q current.
  struct CurrentRange {
    unit::Current lower;  ///< Most negative q current.
    unit::Current upper;  ///< Most positive q current.
  };

  /// Range of the shaft torque.
  struct TorqueRange {
    unit::Torque lower;  ///< Most negative torque.
    unit::Torque upper;  ///< Most positive torque.
  };

  /**
   * @brief Battery current of an operating point (the estimate of the limits).
   * @param current Rotor-frame current.
   * @param torque_constant k_t = 3/2 p psi_pm (N m/A, the dimension of Wb).
   * @param speed Mechanical speed of the shaft.
   * @param dc_link DC-link voltage; not positive: zero.
   * @return Positive while the drive takes power from the battery.
   */
  [[nodiscard]] constexpr unit::Current BatteryCurrent(const system::Rotor<unit::Current>& current,
                                                       const unit::MagneticFlux torque_constant,
                                                       const unit::AngularVelocity speed,
                                                       const unit::Voltage dc_link) const noexcept {
    if (!(dc_link > unit::Voltage{})) {
      return unit::Current{};
    }
    return (((torque_constant * current.q) * speed) + CopperLoss(current)) / dc_link;
  }

  /**
   * @brief q current that keeps the battery current within the limits.
   *
   * The interval of i_q around zero in which the estimated battery power
   * stays within the drive limit and the mechanical power within the charge
   * limit, for a given d current.
   * Without a DC-link reading, or without a bound in a direction, the range
   * is unbounded there. When the d current alone exceeds the drive limit, the
   * range is the zero q current.
   *
   * @param torque_constant k_t = 3/2 p psi_pm (N m/A, the dimension of Wb).
   * @param speed Mechanical speed of the shaft.
   * @param dc_link DC-link voltage.
   * @param current_d The d current of the operating point.
   * @return The range; it always contains zero.
   */
  [[nodiscard]] CurrentRange AllowedCurrent(const unit::MagneticFlux torque_constant,
                                            const unit::AngularVelocity speed,
                                            const unit::Voltage dc_link,
                                            const unit::Current current_d) const noexcept {
    constexpr float kInfinity = std::numeric_limits<float>::infinity();
    CurrentRange range{unit::Current{-kInfinity}, unit::Current{kInfinity}};
    if (!(dc_link > unit::Voltage{}) || !unit::IsFinite(speed)) {
      return range;
    }
    // c i_q^2 + b i_q + c i_d^2 = P, with b the back-EMF per q current.
    const unit::Resistance kCopper = CopperResistance();
    const unit::Voltage kSlope = torque_constant * speed;
    const unit::Power kDrive = dc_link * drive_current_max;
    const unit::Power kCharge = dc_link * AllowedChargeCurrent(dc_link);
    const unit::Power kDLoss = CopperLoss({current_d, unit::Current{}});
    if (kDLoss > kDrive) {
      return {unit::Current{}, unit::Current{}};  // the d current alone takes the whole limit
    }
    if (!(kCopper > unit::Resistance{})) {
      // Linear: b i_q <= P_drive and b i_q >= -P_charge.
      if (kSlope > unit::Voltage{}) {
        return {-kCharge / kSlope, kDrive / kSlope};
      }
      if (kSlope < unit::Voltage{}) {
        return {kDrive / kSlope, -kCharge / kSlope};
      }
      return range;
    }
    // Drive limit: P <= P_drive between the two roots, one on each side of zero.
    const Roots kDriveRoots = SolveQuadratic(kCopper, kSlope, kDLoss - kDrive);
    if (kDriveRoots.real) {
      range = {kDriveRoots.lower, kDriveRoots.upper};
    }
    // Charge limit: the mechanical power alone, b i_q >= -P_charge.
    if (kSlope > unit::Voltage{}) {
      range.lower = unit::Max(range.lower, -kCharge / kSlope);
    } else if (kSlope < unit::Voltage{}) {
      range.upper = unit::Min(range.upper, -kCharge / kSlope);
    }
    return range;
  }

  /**
   * @brief Shaft torque that keeps the battery current within the limits at a speed.
   *
   * The q current range of AllowedCurrent() at zero d current, as torque: for
   * the output bounds of a speed controller, so its integral does not wind up
   * against the limit.
   *
   * @param speed Mechanical speed of the shaft.
   * @param dc_link DC-link voltage.
   * @param torque_constant k_t = 3/2 p psi_pm; not positive: unbounded.
   * @return The torque range; unbounded without a DC-link reading, and at
   *         standstill without copper losses.
   */
  [[nodiscard]] TorqueRange AllowedTorque(const unit::AngularVelocity speed,
                                          const unit::Voltage dc_link,
                                          const unit::MagneticFlux torque_constant) const noexcept {
    constexpr float kInfinity = std::numeric_limits<float>::infinity();
    if (!(torque_constant > unit::MagneticFlux{})) {
      return {unit::Torque{-kInfinity}, unit::Torque{kInfinity}};
    }
    const CurrentRange kRange = AllowedCurrent(torque_constant, speed, dc_link, unit::Current{});
    return {torque_constant * kRange.lower, torque_constant * kRange.upper};
  }

  /**
   * @brief Limits the q current so the estimated battery current stays within the limits.
   * @param current Rotor-frame current reference.
   * @param torque_constant k_t = 3/2 p psi_pm (N m/A, the dimension of Wb).
   * @param speed Mechanical speed of the shaft.
   * @param dc_link DC-link voltage.
   * @return The reference with the q current limited where needed; the d current unchanged.
   */
  [[nodiscard]] system::Rotor<unit::Current> Limit(const system::Rotor<unit::Current>& current,
                                                   const unit::MagneticFlux torque_constant,
                                                   const unit::AngularVelocity speed,
                                                   const unit::Voltage dc_link) const noexcept {
    const CurrentRange kRange = AllowedCurrent(torque_constant, speed, dc_link, current.d);
    return {current.d, current.q.Clamp(kRange.lower, kRange.upper)};
  }

 private:
  /// Real roots of a quadratic, ordered.
  struct Roots {
    bool real{false};
    unit::Current lower;
    unit::Current upper;
  };

  /// 3/2 R_s: the copper losses of the three phases per |i_dq|^2.
  [[nodiscard]] constexpr unit::Resistance CopperResistance() const noexcept {
    return stator_resistance * 1.5F;
  }

  [[nodiscard]] constexpr unit::Power CopperLoss(const system::Rotor<unit::Current>& current) const noexcept {
    return CopperResistance() * current.LengthSquared();
  }

  /**
   * @brief Roots of a x^2 + b x + c = 0 with a > 0, without cancellation.
   * @param quadratic Quadratic coefficient a (positive).
   * @param linear Linear coefficient b.
   * @param constant Constant c.
   */
  [[nodiscard]] static Roots SolveQuadratic(const unit::Resistance quadratic,
                                            const unit::Voltage linear,
                                            const unit::Power constant) noexcept {
    const auto kDiscriminant = (linear * linear) - ((quadratic * constant) * 4.0F);
    if (kDiscriminant < decltype(kDiscriminant){}) {
      return {};
    }
    const unit::Voltage kRoot = unit::Sqrt(kDiscriminant);
    // -(b + sign(b) sqrt(D)) / 2: no difference of nearly equal numbers.
    const unit::Voltage kHalfSum =
        (linear < unit::Voltage{}) ? (kRoot - linear) * 0.5F : -(linear + kRoot) * 0.5F;
    if (kHalfSum == unit::Voltage{}) {
      return {true, unit::Current{}, unit::Current{}};  // b = 0 and c = 0: a double root at zero
    }
    const unit::Current kFirst = kHalfSum / quadratic;
    const unit::Current kSecond = constant / kHalfSum;
    return {true, unit::Min(kFirst, kSecond), unit::Max(kFirst, kSecond)};
  }
};

}  // namespace unimoc::control
