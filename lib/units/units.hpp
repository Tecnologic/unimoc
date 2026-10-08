/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file units.hpp
 *	@brief Strongly typed SI units, unit arithmetic, literals, and type traits.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <numbers>  // For std::numbers::pi_v, C++20
#include <ratio>
#include <type_traits>  // For std::is_same_v, std::common_type_t

/**
 * @namespace unit Strongly typed SI units and unit operations.
 *
 * Unit values carry a representation type, a std::ratio period, and a tag
 * type.  The tags prevent incompatible physical quantities from being mixed
 * accidentally while the periods support explicit scaled-unit conversion.
 */
namespace unimoc::unit {
/// Tag identifying angles, whose base unit is the radian.
struct AngleTag;
/// Tag identifying angular velocities, whose base unit is radians per second.
struct AngularVelocityTag;
/// Tag identifying torque, whose base unit is the Newton-metre.
struct TorqueTag;
/// Tag identifying angular acceleration, whose base unit is radians per second
/// squared.
struct AngularAccelerationTag;
/// Tag identifying electric current, whose base unit is the ampere.
struct CurrentTag;
/// Tag identifying electric potential, whose base unit is the volt.
struct VoltageTag;
/// Tag identifying temperature readings in degrees Celsius.
struct TemperatureTag;
/// Tag identifying power, whose base unit is the watt.
struct PowerTag;
/// Tag identifying magnetic flux, whose base unit is the weber.
struct MagneticFluxTag;
/// Tag identifying resistance, whose base unit is the ohm.
struct ResistanceTag;
/// Tag identifying inductance, whose base unit is the henry.
struct InductanceTag;
/// Tag identifying rotational inertia, whose base unit is kg m^2.
struct InertiaTag;
/// Tag identifying time, whose base unit is the second.
struct TimeTag;
/// Tag identifying frequency, whose base unit is the hertz.
struct FrequencyTag;
/// Tag identifying inverse time, whose base unit is 1/s.
struct InverseTimeTag;
/// Tag identifying a dimensionless ratio, such as a sine or cosine result.
struct RatioTag;
/// Tag identifying voltage-per-current controller gains.
struct VoltagePerCurrentTag;
/// Tag identifying voltage-per-current-per-time controller gains.
struct VoltagePerCurrentTimeTag;
/// Tag identifying current-per-flux controller gains.
struct CurrentPerMagneticFluxTag;
/// Tag identifying current-per-flux-per-time controller gains.
struct CurrentPerMagneticFluxTimeTag;
/// Tag identifying current rate [A/s].
struct CurrentPerTimeTag;
/// Tag identifying current-per-voltage-per-time controller gains.
struct CurrentPerVoltageTimeTag;
/// Tag identifying angle-per-current gains.
struct AnglePerCurrentTag;
/// Tag identifying angular-velocity-per-angle controller gains.
struct AngularVelocityPerAngleTag;
/// Tag identifying magnetic-flux-per-current-per-time observer gains.
struct MagneticFluxPerCurrentTimeTag;

/**
 * @brief A generic class for representing a unit of measurement.
 * @tparam Rep The underlying representation type (e.g., float, double).
 * @tparam Period A std::ratio representing the scaling factor from the base
 * unit (e.g., std::milli, std::kilo).
 * @tparam Tag A tag struct to differentiate unit types (e.g., AngleTag,
 * CurrentTag).
 */
template <typename Rep, typename Period = std::ratio<1>, typename Tag = void>
class Unit {
 public:
  /// Representation type used to store the numeric value.
  using representation = Rep;
  /// Scale relative to the corresponding base unit.
  using period = Period;
  /// Tag that identifies the physical quantity represented by this unit.
  using tag = Tag;

  /// Constructs a zero-valued unit.
  constexpr Unit() : val_(0) {}
  /**
   * @brief Constructs a unit from a value expressed in this unit's period.
   * @param val Numeric value in the unit represented by this instance.
   */
  explicit constexpr Unit(Rep val) : val_(val) {}

  /**
   * @brief Converts a value with the same tag from another period.
   * @param other Unit value expressed in another period.
   */
  template <typename OtherPeriod>
  constexpr Unit(const Unit<Rep, OtherPeriod, Tag>& other)
      : val_(other.Value() * static_cast<Rep>(OtherPeriod::num) / OtherPeriod::den * static_cast<Rep>(Period::den) / Period::num) {}

  /**
   * @brief Assigns a value with the same tag from another period.
   * @param other Unit value expressed in another period.
   * @return This unit after conversion and assignment.
   */
  template <typename OtherPeriod>
  constexpr Unit& operator=(const Unit<Rep, OtherPeriod, Tag>& other) {
    val_ = other.Value() * static_cast<Rep>(OtherPeriod::num) / OtherPeriod::den * static_cast<Rep>(Period::den) / Period::num;
    return *this;
  }

  /**
   * @brief Returns the stored numeric value.
   * @return The value expressed in this unit's period.
   */
  [[nodiscard]] constexpr Rep Value() const { return val_; }

  /**
   * @brief Converts this unit to another unit of the same type but with a
   * different period.
   * @tparam OtherPeriod The period of the target unit.
   * @return A new Unit object with the converted value and OtherPeriod.
   *
   * @note The unit tag and representation type are preserved.
   */
  template <typename OtherPeriod>
  constexpr Unit<Rep, OtherPeriod, Tag> ConvertTo() const {
    // value_base = val_ * (period::num / period::den)
    // new_val * (OtherPeriod::num / OtherPeriod::den) = value_base
    // new_val = value_base * (OtherPeriod::den / OtherPeriod::num)
    // new_val = val_ * (period::num / period::den) * (OtherPeriod::den /
    // OtherPeriod::num)
    return Unit<Rep, OtherPeriod, Tag>(val_ * static_cast<Rep>(period::num) / period::den * OtherPeriod::den / OtherPeriod::num);
  }

  /**
   * @brief Clamps the unit's value between a minimum and maximum value.
   * @param min_val The minimum allowable value.
   * @param max_val The maximum allowable value.
   * @return A new Unit object with the clamped value.
   */
  constexpr Unit Clamp(const Rep& min_val, const Rep& max_val) const { return Unit(val_ < min_val ? min_val : (val_ > max_val ? max_val : val_)); }

  /**
   * @brief Clamps this unit between two values of the same unit.
   * @param min_val The minimum allowable value.
   * @param max_val The maximum allowable value.
   * @return A new Unit object with the clamped value.
   */
  constexpr Unit Clamp(const Unit& min_val, const Unit& max_val) const {
    return Unit(val_ < min_val.val_ ? min_val.val_ : (val_ > max_val.val_ ? max_val.val_ : val_));
  }

  /**
   * @brief Returns this unit unchanged.
   * @return A copy of this unit.
   */
  constexpr Unit operator+() const { return Unit(val_); }
  /**
   * @brief Negates this unit.
   * @return A unit with the negated numeric value.
   */
  constexpr Unit operator-() const { return Unit(-val_); }

  /**
   * @brief Adds another value with the same tag and period.
   * @param other Unit value to add.
   * @return This unit after the addition.
   * @pre `other` has the same tag and period as this unit.
   */
  constexpr Unit& operator+=(const Unit& other) {
    static_assert(std::is_same_v<tag, typename std::remove_cvref_t<decltype(other)>::tag>, "Cannot add units of different types.");
    static_assert(std::is_same_v<period, typename std::remove_cvref_t<decltype(other)>::period>,
                  "Implicit period conversion not allowed for "
                  "addition/subtraction. Convert "
                  "explicitly or ensure same period.");
    val_ += other.val_;
    return *this;
  }
  /**
   * @brief Subtracts another value with the same tag and period.
   * @param other Unit value to subtract.
   * @return This unit after the subtraction.
   * @pre `other` has the same tag and period as this unit.
   */
  constexpr Unit& operator-=(const Unit& other) {
    static_assert(std::is_same_v<tag, typename std::remove_cvref_t<decltype(other)>::tag>, "Cannot subtract units of different types.");
    static_assert(std::is_same_v<period, typename std::remove_cvref_t<decltype(other)>::period>,
                  "Implicit period conversion not allowed for "
                  "addition/subtraction. Convert "
                  "explicitly or ensure same period.");
    val_ -= other.val_;
    return *this;
  }
  /**
   * @brief Multiplies this unit by a scalar.
   * @param scalar Numeric scale factor.
   * @return This unit after scaling.
   */
  constexpr Unit& operator*=(const Rep& scalar) {
    val_ *= scalar;
    return *this;
  }
  /**
   * @brief Divides this unit by a scalar.
   * @param scalar Numeric divisor.
   * @return This unit after scaling.
   */
  constexpr Unit& operator/=(const Rep& scalar) {
    val_ /= scalar;
    return *this;
  }

 private:
  Rep val_;
};

/**
 * @brief Identifies specializations of the UNIMOC unit type.
 * @tparam T Type to inspect.
 */
template <typename T>
struct IsUnit : std::false_type {};

template <typename Rep, typename Period, typename Tag>
struct IsUnit<Unit<Rep, Period, Tag>> : std::true_type {};

/**
 * @brief Constrains a type to a UNIMOC unit specialization.
 * @tparam T Type to constrain.
 */
template <typename T>
concept UnitLike = IsUnit<T>::value;

// SI unit types (base type float, period std::ratio<1> for base SI units)
/// Angle in radians (rad).
using Angle = Unit<float, std::ratio<1>, AngleTag>;
/// Angular velocity in radians per second (rad/s).
using AngularVelocity = Unit<float, std::ratio<1>, AngularVelocityTag>;
/// Torque in Newton-metres (N m).
using Torque = Unit<float, std::ratio<1>, TorqueTag>;
/// Angular acceleration in radians per second squared (rad/s^2).
using AngularAcceleration = Unit<float, std::ratio<1>, AngularAccelerationTag>;
/// Electric current in amperes (A).
using Current = Unit<float, std::ratio<1>, CurrentTag>;
/// Electric potential in volts (V).
using Voltage = Unit<float, std::ratio<1>, VoltageTag>;
/// Temperature in degrees Celsius.
using Temperature = Unit<float, std::ratio<1>, TemperatureTag>;
/// Power in watts (W).
using Power = Unit<float, std::ratio<1>, PowerTag>;
/// Magnetic flux in webers (Wb).
using MagneticFlux = Unit<float, std::ratio<1>, MagneticFluxTag>;
/// Resistance in ohms.
using Resistance = Unit<float, std::ratio<1>, ResistanceTag>;
/// Inductance in henrys (H).
using Inductance = Unit<float, std::ratio<1>, InductanceTag>;
/// Rotational inertia in kilogram square metres (kg m^2).
using Inertia = Unit<float, std::ratio<1>, InertiaTag>;

// Helper units
/// Time in seconds (s).
using Time = Unit<float, std::ratio<1>, TimeTag>;
/// Frequency in hertz (Hz).
using Frequency = Unit<float, std::ratio<1>, FrequencyTag>;
/// Inverse time in reciprocal seconds (1/s).
using InverseTime = Unit<float, std::ratio<1>, InverseTimeTag>;
/// Dimensionless ratio, such as a sine or cosine result.
using Ratio = Unit<float, std::ratio<1>, RatioTag>;
/// Voltage per current gain [V/A].
using VoltagePerCurrent = Unit<float, std::ratio<1>, VoltagePerCurrentTag>;
/// Voltage per current per time gain [V/(A s)].
using VoltagePerCurrentTime = Unit<float, std::ratio<1>, VoltagePerCurrentTimeTag>;
/// Current per magnetic flux gain [A/Wb].
using CurrentPerMagneticFlux = Unit<float, std::ratio<1>, CurrentPerMagneticFluxTag>;
/// Current per magnetic flux per time gain [A/(Wb s)].
using CurrentPerMagneticFluxTime = Unit<float, std::ratio<1>, CurrentPerMagneticFluxTimeTag>;
/// Current rate [A/s].
using CurrentPerTime = Unit<float, std::ratio<1>, CurrentPerTimeTag>;
/// Current per voltage per time gain [A/(V s)].
using CurrentPerVoltageTime = Unit<float, std::ratio<1>, CurrentPerVoltageTimeTag>;
/// Angle per current gain [rad/A].
using AnglePerCurrent = Unit<float, std::ratio<1>, AnglePerCurrentTag>;
/// Angular velocity per angle gain [(rad/s)/rad].
using AngularVelocityPerAngle = Unit<float, std::ratio<1>, AngularVelocityPerAngleTag>;
/// Magnetic flux per current per time observer gain [Wb/(A s)].
using MagneticFluxPerCurrentTime = Unit<float, std::ratio<1>, MagneticFluxPerCurrentTimeTag>;

// --- Operators for units ---

/**
 * @brief Adds two unit values with the same tag and period.
 * @return The sum with the same unit type as the operands.
 */
template <typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag> operator+(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return Unit<Rep, Period, Tag>(lhs.Value() + rhs.Value());
}

/**
 * @brief Subtracts two unit values with the same tag and period.
 * @return The difference with the same unit type as the operands.
 */
template <typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag> operator-(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return Unit<Rep, Period, Tag>(lhs.Value() - rhs.Value());
}

/**
 * @brief Multiplies a unit value by a scalar.
 * @return A unit with the same tag and period as `lhs`.
 */
template <typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag> operator*(const Unit<Rep, Period, Tag>& lhs, const Rep& scalar) {
  return Unit<Rep, Period, Tag>(lhs.Value() * scalar);
}

/**
 * @brief Multiplies a scalar by a unit value.
 * @return A unit with the same tag and period as `rhs`.
 */
template <typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag> operator*(const Rep& scalar, const Unit<Rep, Period, Tag>& rhs) {
  return Unit<Rep, Period, Tag>(scalar * rhs.Value());
}

/**
 * @brief Divides a unit value by a scalar.
 * @return A unit with the same tag and period as `lhs`.
 */
template <typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag> operator/(const Unit<Rep, Period, Tag>& lhs, const Rep& scalar) {
  return Unit<Rep, Period, Tag>(lhs.Value() / scalar);
}

/**
 * @brief Divides two unit values with the same tag.
 * @return The dimensionless quotient with the quotient period.
 */
template <typename Rep, typename P1, typename P2, typename Tag>
constexpr auto operator/(const Unit<Rep, P1, Tag>& lhs, const Unit<Rep, P2, Tag>& rhs) {
  return Unit<Rep, std::ratio_divide<P1, P2>, RatioTag>(lhs.Value() / rhs.Value());
}

/**
 * @brief Multiplies a unit by a dimensionless ratio.
 * @return A unit with the original tag and period.
 */
template <typename Rep, typename P, typename Tag>
constexpr auto operator*(const Unit<Rep, P, Tag>& unit, const Ratio& ratio) {
  return Unit<Rep, P, Tag>(unit.Value() * ratio.Value());
}

/**
 * @brief Multiplies a dimensionless ratio by a unit.
 * @return A unit with the original unit's tag and period.
 */
template <typename Rep, typename P, typename Tag>
constexpr auto operator*(const Ratio& ratio, const Unit<Rep, P, Tag>& unit) {
  return unit * ratio;  // Commutative
}

/**
 * @brief Divides a unit by a dimensionless ratio.
 * @return A unit with the original tag and period.
 */
template <typename Rep, typename P, typename Tag>
constexpr auto operator/(const Unit<Rep, P, Tag>& unit, const Ratio& ratio) {
  return Unit<Rep, P, Tag>(unit.Value() / ratio.Value());
}

/**
 * @brief Divides an angle by time to obtain angular velocity.
 * @return Angular velocity with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, AngleTag>& angle, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_divide<P1, P2>, AngularVelocityTag>(angle.Value() / time.Value());
}

/**
 * @brief Multiplies angular velocity by time to obtain an angle.
 * @return Angle with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, AngleTag>(angVel.Value() * time.Value());
}

/**
 * @brief Multiplies time by angular velocity to obtain an angle.
 * @return Angle with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, AngularVelocityTag>& angVel) {
  return angVel * time;
}

/**
 * @brief Divides an angle by angular velocity to obtain time.
 * @return Time with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, AngleTag>& angle, const Unit<Rep, P2, AngularVelocityTag>& angVel) {
  return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(angle.Value() / angVel.Value());
}

/**
 * @brief Divides angular velocity by time to obtain angular acceleration.
 * @return Angular acceleration with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_divide<P1, P2>, AngularAccelerationTag>(angVel.Value() / time.Value());
}

/**
 * @brief Multiplies angular acceleration by time to obtain angular velocity.
 * @return Angular velocity with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, AngularAccelerationTag>& angAccel, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, AngularVelocityTag>(angAccel.Value() * time.Value());
}

/**
 * @brief Multiplies time by angular acceleration to obtain angular velocity.
 * @return Angular velocity with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, AngularAccelerationTag>& angAccel) {
  return angAccel * time;
}

/**
 * @brief Divides angular velocity by angular acceleration to obtain time.
 * @return Time with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, AngularAccelerationTag>& angAccel) {
  return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(angVel.Value() / angAccel.Value());
}

/**
 * @brief Multiplies voltage by current to obtain power.
 * @return Power with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, CurrentTag>& current) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, PowerTag>(voltage.Value() * current.Value());
}

/**
 * @brief Multiplies current by voltage to obtain power.
 * @return Power with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, CurrentTag>& current, const Unit<Rep, P2, VoltageTag>& voltage) {
  return voltage * current;
}

/**
 * @brief Divides power by current to obtain voltage.
 * @return Voltage with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, CurrentTag>& current) {
  return Unit<Rep, std::ratio_divide<P1, P2>, VoltageTag>(power.Value() / current.Value());
}

/**
 * @brief Divides power by voltage to obtain current.
 * @return Current with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, VoltageTag>& voltage) {
  return Unit<Rep, std::ratio_divide<P1, P2>, CurrentTag>(power.Value() / voltage.Value());
}

/**
 * @brief Divides magnetic flux by time to obtain voltage.
 * @return Voltage with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_divide<P1, P2>, VoltageTag>(flux.Value() / time.Value());
}

/**
 * @brief Multiplies voltage by time to obtain magnetic flux.
 * @return Magnetic flux with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, MagneticFluxTag>(voltage.Value() * time.Value());
}

/**
 * @brief Multiplies time by voltage to obtain magnetic flux.
 * @return Magnetic flux with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, VoltageTag>& voltage) {
  return voltage * time;
}

/**
 * @brief Multiplies inductance by current to obtain magnetic flux.
 * @return Magnetic flux with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, InductanceTag>& inductance, const Unit<Rep, P2, CurrentTag>& current) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, MagneticFluxTag>(inductance.Value() * current.Value());
}

/**
 * @brief Multiplies current by inductance to obtain magnetic flux.
 * @return Magnetic flux with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, CurrentTag>& current, const Unit<Rep, P2, InductanceTag>& inductance) {
  return inductance * current;
}

/**
 * @brief Divides inductance by resistance to obtain time.
 * @return Time with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, InductanceTag>& inductance, const Unit<Rep, P2, ResistanceTag>& resistance) {
  return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(inductance.Value() / resistance.Value());
}

/**
 * @brief Multiplies a current-per-flux gain by magnetic flux to obtain current.
 * @return Current with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, CurrentPerMagneticFluxTag>& gain, const Unit<Rep, P2, MagneticFluxTag>& flux) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, CurrentTag>(gain.Value() * flux.Value());
}

/**
 * @brief Multiplies magnetic flux by a current-per-flux gain to obtain current.
 * @return Current with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, CurrentPerMagneticFluxTag>& gain) {
  return gain * flux;
}

/**
 * @brief Multiplies a current-per-flux-per-time gain by magnetic flux to obtain current rate.
 * @return Current rate with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, CurrentPerMagneticFluxTimeTag>& gain, const Unit<Rep, P2, MagneticFluxTag>& flux) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, CurrentPerTimeTag>(gain.Value() * flux.Value());
}

/**
 * @brief Multiplies magnetic flux by a current-per-flux-per-time gain to obtain current rate.
 * @return Current rate with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, CurrentPerMagneticFluxTimeTag>& gain) {
  return gain * flux;
}

/**
 * @brief Multiplies current rate by time to obtain current.
 * @return Current with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, CurrentPerTimeTag>& current_rate, const Unit<Rep, P2, TimeTag>& time) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, CurrentTag>(current_rate.Value() * time.Value());
}

/**
 * @brief Multiplies time by current rate to obtain current.
 * @return Current with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, CurrentPerTimeTag>& current_rate) {
  return current_rate * time;
}

/**
 * @brief Divides magnetic flux by voltage to obtain time.
 * @return Time with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, VoltageTag>& voltage) {
  return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(flux.Value() / voltage.Value());
}

/**
 * @brief Multiplies inverse time by magnetic flux to obtain voltage.
 * @return Voltage with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, InverseTimeTag>& inverseTime, const Unit<Rep, P2, MagneticFluxTag>& flux) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, VoltageTag>(inverseTime.Value() * flux.Value());
}

/**
 * @brief Multiplies magnetic flux by inverse time to obtain voltage.
 * @return Voltage with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, InverseTimeTag>& inverseTime) {
  return inverseTime * flux;
}

/**
 * @brief Multiplies current by resistance to obtain voltage.
 * @return Voltage with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, CurrentTag>& current, const Unit<Rep, P2, ResistanceTag>& resistance) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, VoltageTag>(current.Value() * resistance.Value());
}

/**
 * @brief Multiplies resistance by current to obtain voltage.
 * @return Voltage with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, ResistanceTag>& resistance, const Unit<Rep, P2, CurrentTag>& current) {
  return current * resistance;
}

/**
 * @brief Divides voltage by resistance to obtain current.
 * @return Current with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, ResistanceTag>& resistance) {
  return Unit<Rep, std::ratio_divide<P1, P2>, CurrentTag>(voltage.Value() / resistance.Value());
}

/**
 * @brief Divides voltage by current to obtain resistance.
 * @return Resistance with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, CurrentTag>& current) {
  return Unit<Rep, std::ratio_divide<P1, P2>, ResistanceTag>(voltage.Value() / current.Value());
}

/**
 * @brief Multiplies inductance by angular velocity to obtain voltage.
 * @return Voltage with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, InductanceTag>& inductance, const Unit<Rep, P2, AngularVelocityTag>& angVel) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, VoltageTag>(inductance.Value() * angVel.Value());
}

/**
 * @brief Multiplies angular velocity by inductance to obtain voltage.
 * @return Voltage with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, InductanceTag>& inductance) {
  return inductance * angVel;
}

/**
 * @brief Divides voltage by inductance to obtain current.
 * @return Current with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, InductanceTag>& inductance) {
  return Unit<Rep, std::ratio_divide<P1, P2>, CurrentTag>(voltage.Value() / inductance.Value());
}

/**
 * @brief Multiplies torque by angular velocity to obtain power.
 * @return Power with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, TorqueTag>& torque, const Unit<Rep, P2, AngularVelocityTag>& angVel) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, PowerTag>(torque.Value() * angVel.Value());
}

/**
 * @brief Multiplies angular velocity by torque to obtain power.
 * @return Power with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, TorqueTag>& torque) {
  return torque * angVel;
}

/**
 * @brief Divides power by angular velocity to obtain torque.
 * @return Torque with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, AngularVelocityTag>& angVel) {
  return Unit<Rep, std::ratio_divide<P1, P2>, TorqueTag>(power.Value() / angVel.Value());
}

/**
 * @brief Divides power by torque to obtain angular velocity.
 * @return Angular velocity with the quotient period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, TorqueTag>& torque) {
  return Unit<Rep, std::ratio_divide<P1, P2>, AngularVelocityTag>(power.Value() / torque.Value());
}

/**
 * @brief Divides a scalar by time to obtain frequency.
 * @return Frequency with the reciprocal period.
 */
template <typename Rep, typename Period>
constexpr auto operator/(const Rep& scalar, const Unit<Rep, Period, TimeTag>& time) {
  return Unit<Rep, std::ratio_divide<std::ratio<1>, Period>, FrequencyTag>(scalar / time.Value());
}

/**
 * @brief Divides a scalar by frequency to obtain time.
 * @return Time with the reciprocal period.
 */
template <typename Rep, typename Period>
constexpr auto operator/(const Rep& scalar, const Unit<Rep, Period, FrequencyTag>& freq) {
  return Unit<Rep, std::ratio_divide<std::ratio<1>, Period>, TimeTag>(scalar / freq.Value());
}

/**
 * @brief Multiplies angle by frequency to obtain angular velocity.
 * @return Angular velocity with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, AngleTag>& angle, const Unit<Rep, P2, FrequencyTag>& freq) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, AngularVelocityTag>(angle.Value() * freq.Value());
}

/**
 * @brief Multiplies frequency by angle to obtain angular velocity.
 * @return Angular velocity with the product period.
 */
template <typename Rep, typename P1, typename P2>
constexpr auto operator*(const Unit<Rep, P1, FrequencyTag>& freq, const Unit<Rep, P2, AngleTag>& angle) {
  return angle * freq;
}

/**
 * @brief Compares two unit values for equality.
 * @return `true` when both values have the same numeric value.
 *
 * Both operands must have the same tag and period. Convert explicitly before
 * comparing values expressed in different periods.
 */
template <typename Rep, typename Period, typename Tag>
constexpr bool operator==(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return lhs.Value() == rhs.Value();
}

/**
 * @brief Compares two unit values for inequality.
 * @return `true` when the numeric values differ.
 */
template <typename Rep, typename Period, typename Tag>
constexpr bool operator!=(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Tests whether one unit value is less than another.
 * @return `true` when `lhs` has the smaller numeric value.
 */
template <typename Rep, typename Period, typename Tag>
constexpr bool operator<(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return lhs.Value() < rhs.Value();
}

/**
 * @brief Tests whether one unit value is less than or equal to another.
 * @return `true` when `lhs` is no greater than `rhs`.
 */
template <typename Rep, typename Period, typename Tag>
constexpr bool operator<=(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return lhs.Value() <= rhs.Value();
}

/**
 * @brief Tests whether one unit value is greater than another.
 * @return `true` when `lhs` has the greater numeric value.
 */
template <typename Rep, typename Period, typename Tag>
constexpr bool operator>(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return lhs.Value() > rhs.Value();
}

/**
 * @brief Tests whether one unit value is greater than or equal to another.
 * @return `true` when `lhs` is no less than `rhs`.
 */
template <typename Rep, typename Period, typename Tag>
constexpr bool operator>=(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs) {
  return lhs.Value() >= rhs.Value();
}

/**
 * @brief Creates an angle from a radian literal.
 * @param val Literal value in radians.
 * @return An angle in radians.
 */
constexpr Angle operator""_rad(long double val) { return Angle(static_cast<float>(val)); }
constexpr Angle operator""_deg(long double val) { return Angle(static_cast<float>(val * std::numbers::pi_v<long double> / 180.0L)); }

/**
 * @brief Creates an angle from a radian integer literal.
 * @param val Literal value in radians.
 * @return An angle in radians.
 */
constexpr Angle operator""_rad(unsigned long long val) {
  /**
   * @brief Creates an angle from a degree literal.
   * @param val Literal value in degrees.
   * @return An angle converted to radians.
   */
  return Angle(static_cast<float>(val));
}
constexpr Angle operator""_deg(unsigned long long val) {
  /**
   * @brief Creates an angle from a degree integer literal.
   * @param val Literal value in degrees.
   * @return An angle converted to radians.
   */
  return Angle(static_cast<float>(static_cast<long double>(val) * std::numbers::pi_v<long double> / 180.0L));
}

/**
 * @brief Creates a time value from a seconds literal.
 * @param val Literal value in seconds.
 * @return Time in seconds.
 */
constexpr Time operator""_s(long double val) { return Time(static_cast<float>(val)); }

/**
 * @brief Creates a time value from a millisecond literal.
 * @param val Literal value in milliseconds.
 * @return Time with a millisecond period.
 */
constexpr Unit<float, std::milli, TimeTag> operator""_ms(long double val) { return Unit<float, std::milli, TimeTag>(static_cast<float>(val)); }

/**
 * @brief Creates a time value from a seconds integer literal.
 * @param val Literal value in seconds.
 * @return Time in seconds.
 */
constexpr Time operator""_s(unsigned long long val) { return Time(static_cast<float>(val)); }

/**
 * @brief Creates a time value from a millisecond integer literal.
 * @param val Literal value in milliseconds.
 * @return Time with a millisecond period.
 */
constexpr Unit<float, std::milli, TimeTag> operator""_ms(unsigned long long val) { return Unit<float, std::milli, TimeTag>(static_cast<float>(val)); }

/**
 * @brief Creates a current value from an ampere literal.
 * @param val Literal value in amperes.
 * @return Current in amperes.
 */
constexpr Current operator""_A(long double val) { return Current(static_cast<float>(val)); }

/**
 * @brief Creates a current value from a milliampere literal.
 * @param val Literal value in milliamperes.
 * @return Current with a milliampere period.
 */
constexpr Unit<float, std::milli, CurrentTag> operator""_mA(long double val) { return Unit<float, std::milli, CurrentTag>(static_cast<float>(val)); }

/**
 * @brief Creates a current value from an ampere integer literal.
 * @param val Literal value in amperes.
 * @return Current in amperes.
 */
constexpr Current operator""_A(unsigned long long val) { return Current(static_cast<float>(val)); }

/**
 * @brief Creates a current value from a milliampere integer literal.
 * @param val Literal value in milliamperes.
 * @return Current with a milliampere period.
 */
constexpr Unit<float, std::milli, CurrentTag> operator""_mA(unsigned long long val) {
  return Unit<float, std::milli, CurrentTag>(static_cast<float>(val));
}

/**
 * @brief Creates a voltage value from a volt literal.
 * @param val Literal value in volts.
 * @return Voltage in volts.
 */
constexpr Voltage operator""_V(long double val) { return Voltage(static_cast<float>(val)); }

/**
 * @brief Creates a voltage value from a millivolt literal.
 * @param val Literal value in millivolts.
 * @return Voltage with a millivolt period.
 */
constexpr Unit<float, std::milli, VoltageTag> operator""_mV(long double val) { return Unit<float, std::milli, VoltageTag>(static_cast<float>(val)); }

/**
 * @brief Creates a voltage value from a kilovolt literal.
 * @param val Literal value in kilovolts.
 * @return Voltage with a kilovolt period.
 */
constexpr Unit<float, std::kilo, VoltageTag> operator""_kV(long double val) { return Unit<float, std::kilo, VoltageTag>(static_cast<float>(val)); }

/**
 * @brief Creates a voltage value from a volt integer literal.
 * @param val Literal value in volts.
 * @return Voltage in volts.
 */
constexpr Voltage operator""_V(unsigned long long val) { return Voltage(static_cast<float>(val)); }

/**
 * @brief Creates a voltage value from a millivolt integer literal.
 * @param val Literal value in millivolts.
 * @return Voltage with a millivolt period.
 */
constexpr Unit<float, std::milli, VoltageTag> operator""_mV(unsigned long long val) {
  return Unit<float, std::milli, VoltageTag>(static_cast<float>(val));
}

/**
 * @brief Creates a voltage value from a kilovolt integer literal.
 * @param val Literal value in kilovolts.
 * @return Voltage with a kilovolt period.
 */
constexpr Unit<float, std::kilo, VoltageTag> operator""_kV(unsigned long long val) {
  return Unit<float, std::kilo, VoltageTag>(static_cast<float>(val));
}

/**
 * @brief Creates a frequency value from a hertz literal.
 * @param val Literal value in hertz.
 * @return Frequency in hertz.
 */
constexpr Frequency operator""_Hz(long double val) { return Frequency(static_cast<float>(val)); }

/**
 * @brief Creates a frequency value from a kilohertz literal.
 * @param val Literal value in kilohertz.
 * @return Frequency with a kilohertz period.
 */
constexpr Unit<float, std::kilo, FrequencyTag> operator""_kHz(long double val) {
  return Unit<float, std::kilo, FrequencyTag>(static_cast<float>(val));
}

/**
 * @brief Creates a frequency value from a hertz integer literal.
 * @param val Literal value in hertz.
 * @return Frequency in hertz.
 */
constexpr Frequency operator""_Hz(unsigned long long val) { return Frequency(static_cast<float>(val)); }

/**
 * @brief Creates a frequency value from a kilohertz integer literal.
 * @param val Literal value in kilohertz.
 * @return Frequency with a kilohertz period.
 */
constexpr Unit<float, std::kilo, FrequencyTag> operator""_kHz(unsigned long long val) {
  return Unit<float, std::kilo, FrequencyTag>(static_cast<float>(val));
}

/**
 * @brief Creates a torque value from a Newton-metre literal.
 * @param val Literal value in Newton-metres.
 * @return Torque in Newton-metres.
 */
constexpr Torque operator""_Nm(long double val) { return Torque(static_cast<float>(val)); }

/**
 * @brief Creates a torque value from a Newton-metre integer literal.
 * @param val Literal value in Newton-metres.
 * @return Torque in Newton-metres.
 */
constexpr Torque operator""_Nm(unsigned long long val) { return Torque(static_cast<float>(val)); }

/**
 * @brief Creates a power value from a watt literal.
 * @param val Literal value in watts.
 * @return Power in watts.
 */
constexpr Power operator""_W(long double val) { return Power(static_cast<float>(val)); }

/**
 * @brief Creates a power value from a kilowatt literal.
 * @param val Literal value in kilowatts.
 * @return Power with a kilowatt period.
 */
constexpr Unit<float, std::kilo, PowerTag> operator""_kW(long double val) { return Unit<float, std::kilo, PowerTag>(static_cast<float>(val)); }

/**
 * @brief Creates a power value from a watt integer literal.
 * @param val Literal value in watts.
 * @return Power in watts.
 */
constexpr Power operator""_W(unsigned long long val) { return Power(static_cast<float>(val)); }

/**
 * @brief Creates a power value from a kilowatt integer literal.
 * @param val Literal value in kilowatts.
 * @return Power with a kilowatt period.
 */
constexpr Unit<float, std::kilo, PowerTag> operator""_kW(unsigned long long val) { return Unit<float, std::kilo, PowerTag>(static_cast<float>(val)); }

/**
 * @brief Creates a magnetic-flux value from a weber literal.
 * @param val Literal value in webers.
 * @return Magnetic flux in webers.
 */
constexpr MagneticFlux operator""_Wb(long double val) { return MagneticFlux(static_cast<float>(val)); }

/**
 * @brief Creates a magnetic-flux value from a milliweber literal.
 * @param val Literal value in milliwebers.
 * @return Magnetic flux with a milliweber period.
 */
constexpr Unit<float, std::milli, MagneticFluxTag> operator""_mWb(long double val) {
  return Unit<float, std::milli, MagneticFluxTag>(static_cast<float>(val));
}

/**
 * @brief Creates a magnetic-flux value from a weber integer literal.
 * @param val Literal value in webers.
 * @return Magnetic flux in webers.
 */
constexpr MagneticFlux operator""_Wb(unsigned long long val) { return MagneticFlux(static_cast<float>(val)); }

/**
 * @brief Creates a magnetic-flux value from a milliweber integer literal.
 * @param val Literal value in milliwebers.
 * @return Magnetic flux with a milliweber period.
 */
constexpr Unit<float, std::milli, MagneticFluxTag> operator""_mWb(unsigned long long val) {
  return Unit<float, std::milli, MagneticFluxTag>(static_cast<float>(val));
}

/**
 * @brief Creates a magnetic-flux value from a microweber literal.
 * @param val Literal value in microwebers.
 * @return Magnetic flux with a microweber period.
 */
constexpr Unit<float, std::micro, MagneticFluxTag> operator""_uWb(long double val) {
  return Unit<float, std::micro, MagneticFluxTag>(static_cast<float>(val));
}

constexpr Unit<float, std::micro, MagneticFluxTag> operator""_uWb(unsigned long long val) {
  return Unit<float, std::micro, MagneticFluxTag>(static_cast<float>(val));
}

/**
 * @brief Creates a resistance value from an ohm literal.
 * @param val Literal value in ohms.
 * @return Resistance in ohms.
 */
constexpr Resistance operator""_Ohm(long double val) { return Resistance(static_cast<float>(val)); }

/**
 * @brief Creates a resistance value from a kilo-ohm literal.
 * @param val Literal value in kilo-ohms.
 * @return Resistance with a kilo-ohm period.
 */
constexpr Unit<float, std::kilo, ResistanceTag> operator""_kOhm(long double val) {
  return Unit<float, std::kilo, ResistanceTag>(static_cast<float>(val));
}

/**
 * @brief Creates a resistance value from an ohm integer literal.
 * @param val Literal value in ohms.
 * @return Resistance in ohms.
 */
constexpr Resistance operator""_Ohm(unsigned long long val) { return Resistance(static_cast<float>(val)); }

/**
 * @brief Creates a resistance value from a kilo-ohm integer literal.
 * @param val Literal value in kilo-ohms.
 * @return Resistance with a kilo-ohm period.
 */
constexpr Unit<float, std::kilo, ResistanceTag> operator""_kOhm(unsigned long long val) {
  return Unit<float, std::kilo, ResistanceTag>(static_cast<float>(val));
}

/**
 * @brief Creates an inductance value from a henry literal.
 * @param val Literal value in henrys.
 * @return Inductance in henrys.
 */
constexpr Inductance operator""_H(long double val) { return Inductance(static_cast<float>(val)); }

/**
 * @brief Creates an inductance value from a millihenry literal.
 * @param val Literal value in millihenrys.
 * @return Inductance with a millihenry period.
 */
constexpr Unit<float, std::milli, InductanceTag> operator""_mH(long double val) {
  return Unit<float, std::milli, InductanceTag>(static_cast<float>(val));
}

/**
 * @brief Creates an inductance value from a henry integer literal.
 * @param val Literal value in henrys.
 * @return Inductance in henrys.
 */
constexpr Inductance operator""_H(unsigned long long val) { return Inductance(static_cast<float>(val)); }

/**
 * @brief Creates an inductance value from a millihenry integer literal.
 * @param val Literal value in millihenrys.
 * @return Inductance with a millihenry period.
 */
constexpr Unit<float, std::milli, InductanceTag> operator""_mH(unsigned long long val) {
  return Unit<float, std::milli, InductanceTag>(static_cast<float>(val));
}

/**
 * @brief Creates a time value from a microsecond literal.
 * @param val Literal value in microseconds.
 * @return Time with a microsecond period.
 */
constexpr Unit<float, std::micro, TimeTag> operator""_us(long double val) { return Unit<float, std::micro, TimeTag>(static_cast<float>(val)); }

constexpr Unit<float, std::micro, TimeTag> operator""_us(unsigned long long val) { return Unit<float, std::micro, TimeTag>(static_cast<float>(val)); }

/**
 * @brief Creates an angular velocity from a radians-per-second literal.
 * @param val Literal value in radians per second.
 * @return Angular velocity in radians per second.
 */
constexpr AngularVelocity operator""_rad_per_s(long double val) { return AngularVelocity(static_cast<float>(val)); }
constexpr AngularVelocity operator""_rad_per_s(unsigned long long val) { return AngularVelocity(static_cast<float>(val)); }

/**
 * @brief Creates an angular acceleration from a radians-per-second-squared
 * literal.
 * @param val Literal value in radians per second squared.
 * @return Angular acceleration in radians per second squared.
 */
constexpr AngularAcceleration operator""_rad_per_s2(long double val) { return AngularAcceleration(static_cast<float>(val)); }
constexpr AngularAcceleration operator""_rad_per_s2(unsigned long long val) { return AngularAcceleration(static_cast<float>(val)); }

/**
 * @brief Creates an inverse-time value from a reciprocal-second literal.
 * @param val Literal value in reciprocal seconds.
 * @return Inverse time in reciprocal seconds.
 */
constexpr InverseTime operator""_per_s(long double val) { return InverseTime(static_cast<float>(val)); }
constexpr InverseTime operator""_per_s(unsigned long long val) { return InverseTime(static_cast<float>(val)); }

/**
 * @brief Creates a dimensionless ratio literal.
 * @param val Ratio value.
 * @return A dimensionless ratio.
 */
constexpr Ratio operator""_ratio(long double val) { return Ratio(static_cast<float>(val)); }
constexpr Ratio operator""_ratio(unsigned long long val) { return Ratio(static_cast<float>(val)); }

/**
 * @brief Creates a rotational inertia from a kilogram-square-metre literal.
 * @param val Literal value in kg m^2.
 * @return Rotational inertia in kg m^2.
 */
constexpr Inertia operator""_kg_m2(long double val) { return Inertia(static_cast<float>(val)); }
constexpr Inertia operator""_kg_m2(unsigned long long val) { return Inertia(static_cast<float>(val)); }

/**
 * @brief Creates a voltage-per-current gain literal.
 * @param val Gain value in V/A.
 * @return A voltage-per-current gain.
 */
constexpr VoltagePerCurrent operator""_V_per_A(long double val) { return VoltagePerCurrent(static_cast<float>(val)); }
constexpr VoltagePerCurrent operator""_V_per_A(unsigned long long val) { return VoltagePerCurrent(static_cast<float>(val)); }

/**
 * @brief Creates a voltage-per-current-per-time gain literal.
 * @param val Gain value in V/(A s).
 * @return A voltage-per-current-per-time gain.
 */
constexpr VoltagePerCurrentTime operator""_V_per_A_s(long double val) { return VoltagePerCurrentTime(static_cast<float>(val)); }
constexpr VoltagePerCurrentTime operator""_V_per_A_s(unsigned long long val) { return VoltagePerCurrentTime(static_cast<float>(val)); }

/**
 * @brief Creates a current-per-flux gain literal.
 * @param val Gain value in A/Wb.
 * @return A current-per-flux gain.
 */
constexpr CurrentPerMagneticFlux operator""_A_per_Wb(long double val) { return CurrentPerMagneticFlux(static_cast<float>(val)); }
constexpr CurrentPerMagneticFlux operator""_A_per_Wb(unsigned long long val) { return CurrentPerMagneticFlux(static_cast<float>(val)); }

/**
 * @brief Creates a current-per-flux-per-time gain literal.
 * @param val Gain value in A/(Wb s).
 * @return A current-per-flux-per-time gain.
 */
constexpr CurrentPerMagneticFluxTime operator""_A_per_Wb_s(long double val) { return CurrentPerMagneticFluxTime(static_cast<float>(val)); }
constexpr CurrentPerMagneticFluxTime operator""_A_per_Wb_s(unsigned long long val) { return CurrentPerMagneticFluxTime(static_cast<float>(val)); }

/**
 * @brief Creates a current-rate literal.
 * @param val Rate value in A/s.
 * @return A current rate.
 */
constexpr CurrentPerTime operator""_A_per_s(long double val) { return CurrentPerTime(static_cast<float>(val)); }
constexpr CurrentPerTime operator""_A_per_s(unsigned long long val) { return CurrentPerTime(static_cast<float>(val)); }

/**
 * @brief Creates a current-per-voltage-per-time gain literal.
 * @param val Gain value in A/(V s).
 * @return A current-per-voltage-per-time gain.
 */
constexpr CurrentPerVoltageTime operator""_A_per_V_s(long double val) { return CurrentPerVoltageTime(static_cast<float>(val)); }
constexpr CurrentPerVoltageTime operator""_A_per_V_s(unsigned long long val) { return CurrentPerVoltageTime(static_cast<float>(val)); }

/**
 * @brief Creates an angle-per-current gain literal.
 * @param val Gain value in rad/A.
 * @return An angle-per-current gain.
 */
constexpr AnglePerCurrent operator""_rad_per_A(long double val) { return AnglePerCurrent(static_cast<float>(val)); }
constexpr AnglePerCurrent operator""_rad_per_A(unsigned long long val) { return AnglePerCurrent(static_cast<float>(val)); }

/**
 * @brief Creates an angular-velocity-per-angle gain literal.
 * @param val Gain value in (rad/s)/rad.
 * @return An angular-velocity-per-angle gain.
 */
constexpr AngularVelocityPerAngle operator""_rad_per_s_per_rad(long double val) { return AngularVelocityPerAngle(static_cast<float>(val)); }
constexpr AngularVelocityPerAngle operator""_rad_per_s_per_rad(unsigned long long val) { return AngularVelocityPerAngle(static_cast<float>(val)); }

/**
 * @brief Creates a magnetic-flux-per-current-per-time gain literal.
 * @param val Gain value in Wb/(A s).
 * @return A magnetic-flux-per-current-per-time gain.
 */
constexpr MagneticFluxPerCurrentTime operator""_Wb_per_A_s(long double val) { return MagneticFluxPerCurrentTime(static_cast<float>(val)); }
constexpr MagneticFluxPerCurrentTime operator""_Wb_per_A_s(unsigned long long val) { return MagneticFluxPerCurrentTime(static_cast<float>(val)); }

/**
 * @brief Creates a temperature value from a degrees-Celsius literal.
 * @param val Literal value in degrees Celsius.
 * @return Temperature in degrees Celsius.
 */
constexpr Temperature operator""_degC(long double val) { return Temperature(static_cast<float>(val)); }
constexpr Temperature operator""_degC(unsigned long long val) { return Temperature(static_cast<float>(val)); }

/**
 * @brief Tests whether a type carries the angle tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsAngle : std::is_base_of<AngleTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the angular-velocity tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsAngularVelocity : std::is_base_of<AngularVelocityTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the torque tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsTorque : std::is_base_of<TorqueTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the angular-acceleration tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsAngularAcceleration : std::is_base_of<AngularAccelerationTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the current tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsCurrent : std::is_base_of<CurrentTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the voltage tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsVoltage : std::is_base_of<VoltageTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the power tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsPower : std::is_base_of<PowerTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the magnetic-flux tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsMagneticFlux : std::is_base_of<MagneticFluxTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the resistance tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsResistance : std::is_base_of<ResistanceTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the inductance tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsInductance : std::is_base_of<InductanceTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the time tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsTime : std::is_base_of<TimeTag, typename T::tag> {};

/**
 * @brief Tests whether a type carries the frequency tag.
 * @tparam T Unit type whose `tag` member is inspected.
 */
template <typename T>
struct IsFrequency : std::is_base_of<FrequencyTag, typename T::tag> {};

/**
 * @brief Computes a common unit type for two units with the same tag.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type with the same tag as T1.
 *
 * The resulting representation and period are the common types of the two
 * operands. The tag must match exactly.
 */
template <typename T1, typename T2>
struct CommonUnitType {
  static_assert(std::is_same_v<typename T1::tag, typename T2::tag>, "Units must have the same tag to find common type.");
  /// Common unit type derived from the two input representations and periods.
  using type = Unit<std::common_type_t<typename T1::representation, typename T2::representation>,
                    std::common_type_t<typename T1::period, typename T2::period>,
                    typename T1::tag>;
};

/**
 * @brief Shorthand for the common unit type of two same-tag units.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type.
 */
template <typename T1, typename T2>
using common_unit_type_t = CommonUnitType<T1, T2>::type;

/**
 * @brief Computes a common type for two units with different tags.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type.
 *
 * The result uses a base-unit period and `void` as its tag because the mixed
 * operands do not identify one physical quantity.
 */
template <typename T1, typename T2>
struct CommonUnitTypeDifferentTags {
  /// Common base-period type with no physical-unit tag.
  using type = Unit<std::common_type_t<typename T1::representation, typename T2::representation>, std::ratio<1>, void>;  // Use void tag for mixed
                                                                                                                         // types
};

/**
 * @brief Shorthand for the common type of two different-tag units.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type.
 */
template <typename T1, typename T2>
using common_unit_type_different_tags_t = CommonUnitTypeDifferentTags<T1, T2>::type;

/**
 * @brief Computes a common unit type for matching or different tags.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type.
 *
 * Matching tags use CommonUnitType; different tags use
 * CommonUnitTypeDifferentTags.
 */
template <typename T1, typename T2>
struct CommonUnitTypeMixed {
  /// Common type selected according to whether the input tags match.
  using type =
      std::conditional_t<std::is_same_v<typename T1::tag, typename T2::tag>, common_unit_type_t<T1, T2>, common_unit_type_different_tags_t<T1, T2>>;
};

/**
 * @brief Shorthand for the common unit type of any two unit types.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type.
 */
template <typename T1, typename T2>
using common_unit_type_mixed_t = CommonUnitTypeMixed<T1, T2>::type;

}  // namespace unimoc::unit