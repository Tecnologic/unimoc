/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file units.hpp
 *	@brief Strongly typed SI units with dimension exponents, unit arithmetic, literals, and type traits.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>  // For std::numbers::pi_v, C++20
#include <ratio>
#include <type_traits>  // For std::is_same_v, std::common_type_t

/**
 * @namespace unit Strongly typed SI units and unit operations.
 *
 * Unit values carry a representation type, a std::ratio period, and a
 * dimension. The dimension holds the exponents of the SI base dimensions used
 * by the drive (mass, length, time, current, temperature) plus the radian as a
 * dimension of its own, so that rad/s and Hz stay distinct types. Products and
 * quotients of units derive their dimension at compile time, so a formula needs
 * no hand-written operator and incompatible quantities cannot be mixed. The
 * periods support explicit scaled-unit conversion. A unit has the layout of its
 * representation type; the dimension costs nothing at run time.
 */
namespace unimoc::unit {

/**
 * @brief Exponents of the base dimensions of a physical quantity.
 * @tparam MassExp Exponent of the mass (kg).
 * @tparam LengthExp Exponent of the length (m).
 * @tparam TimeExp Exponent of the time (s).
 * @tparam CurrentExp Exponent of the electric current (A).
 * @tparam TemperatureExp Exponent of the temperature (K, readings in degrees Celsius).
 * @tparam AngleExp Exponent of the plane angle (rad).
 *
 * Every combination of exponents is one type, so two quantities have the same
 * dimension exactly when their dimension types are the same.
 */
template <int MassExp, int LengthExp, int TimeExp, int CurrentExp, int TemperatureExp, int AngleExp>
struct Dimension {
  /// Exponent of the mass.
  static constexpr int kMass = MassExp;
  /// Exponent of the length.
  static constexpr int kLength = LengthExp;
  /// Exponent of the time.
  static constexpr int kTime = TimeExp;
  /// Exponent of the electric current.
  static constexpr int kCurrent = CurrentExp;
  /// Exponent of the temperature.
  static constexpr int kTemperature = TemperatureExp;
  /// Exponent of the plane angle.
  static constexpr int kAngle = AngleExp;
};

/**
 * @brief Dimension of the product of two quantities: the exponents add.
 * @tparam D1 Dimension of the first factor.
 * @tparam D2 Dimension of the second factor.
 */
template <typename D1, typename D2>
using DimensionProduct = Dimension<D1::kMass + D2::kMass,
                                   D1::kLength + D2::kLength,
                                   D1::kTime + D2::kTime,
                                   D1::kCurrent + D2::kCurrent,
                                   D1::kTemperature + D2::kTemperature,
                                   D1::kAngle + D2::kAngle>;

/**
 * @brief Dimension of the quotient of two quantities: the exponents subtract.
 * @tparam D1 Dimension of the numerator.
 * @tparam D2 Dimension of the denominator.
 */
template <typename D1, typename D2>
using DimensionQuotient = Dimension<D1::kMass - D2::kMass,
                                    D1::kLength - D2::kLength,
                                    D1::kTime - D2::kTime,
                                    D1::kCurrent - D2::kCurrent,
                                    D1::kTemperature - D2::kTemperature,
                                    D1::kAngle - D2::kAngle>;

/**
 * @brief Dimension of the square root of a quantity: the exponents halve.
 * @tparam D Dimension whose exponents are all even.
 */
template <typename D>
struct DimensionRoot {
  static_assert(D::kMass % 2 == 0 && D::kLength % 2 == 0 && D::kTime % 2 == 0 && D::kCurrent % 2 == 0 &&
                    D::kTemperature % 2 == 0 && D::kAngle % 2 == 0,
                "The square root needs a squared quantity (all dimension exponents even).");
  /// Dimension with the halved exponents.
  using type = Dimension<D::kMass / 2,
                         D::kLength / 2,
                         D::kTime / 2,
                         D::kCurrent / 2,
                         D::kTemperature / 2,
                         D::kAngle / 2>;
};

/**
 * @namespace dim Dimensions of the quantities used by the drive.
 *
 * The mechanical rotation quantities carry the radian: torque is energy per
 * radian (J/rad = N m/rad), so torque times angular velocity is power, and the
 * rotational inertia is kg m^2/rad^2, so torque over inertia is rad/s^2. The
 * magnetic flux is the plain weber (V s), so flux over time is voltage and
 * inductance times current is flux.
 */
namespace dim {
/// No dimension: ratios, duties, sine and cosine results.
using Dimensionless = Dimension<0, 0, 0, 0, 0, 0>;
/// Mass [kg].
using Mass = Dimension<1, 0, 0, 0, 0, 0>;
/// Length [m].
using Length = Dimension<0, 1, 0, 0, 0, 0>;
/// Time [s].
using Time = Dimension<0, 0, 1, 0, 0, 0>;
/// Electric current [A].
using Current = Dimension<0, 0, 0, 1, 0, 0>;
/// Temperature [K]; the temperature readings are in degrees Celsius.
using Temperature = Dimension<0, 0, 0, 0, 1, 0>;
/// Plane angle [rad].
using Angle = Dimension<0, 0, 0, 0, 0, 1>;

/// Frequency and inverse time [1/s = Hz].
using Frequency = DimensionQuotient<Dimensionless, Time>;
/// Angular velocity [rad/s].
using AngularVelocity = DimensionQuotient<Angle, Time>;
/// Angular acceleration [rad/s^2].
using AngularAcceleration = DimensionQuotient<AngularVelocity, Time>;
/// Energy [J = kg m^2/s^2].
using Energy = Dimension<1, 2, -2, 0, 0, 0>;
/// Power [W = J/s].
using Power = DimensionQuotient<Energy, Time>;
/// Torque [N m/rad = J/rad].
using Torque = DimensionQuotient<Energy, Angle>;
/// Rotational inertia [kg m^2/rad^2].
using Inertia = DimensionQuotient<Torque, AngularAcceleration>;
/// Electric potential [V = W/A].
using Voltage = DimensionQuotient<Power, Current>;
/// Resistance [Ohm = V/A].
using Resistance = DimensionQuotient<Voltage, Current>;
/// Magnetic flux [Wb = V s].
using MagneticFlux = DimensionProduct<Voltage, Time>;
/// Inductance [H = Wb/A].
using Inductance = DimensionQuotient<MagneticFlux, Current>;
}  // namespace dim

/**
 * @brief Dimension of the product of two quantities, including the
 * electromechanical coupling.
 * @tparam D1 Dimension of the first factor.
 * @tparam D2 Dimension of the second factor.
 *
 * In general the exponents add (DimensionProduct). The specializations below
 * are the electromechanical coupling of a machine: the flux linkage changes by
 * its amplitude per electrical radian, so one radian drops out where the
 * electrical and the mechanical side meet. Angular velocity times flux is the
 * back-EMF in volts and flux times current is the torque in N m/rad; the
 * products agree with each other: (omega * psi) * i and omega * (psi * i) are
 * both power.
 *
 * A rule exists only where the plain result has no meaning of its own. There
 * is no reactance rule (omega * L): write omega * (L * i). Voltage over flux
 * stays 1/s (e.g. R/L or the inverse of a flux time constant), so the speed of
 * a back-EMF is (e / psi) * 1.0_rad.
 */
template <typename D1, typename D2>
struct ProductDimension {
  /// Dimension of the product.
  using type = DimensionProduct<D1, D2>;
};

/// Back-EMF: angular velocity times magnetic flux is a voltage.
template <>
struct ProductDimension<dim::AngularVelocity, dim::MagneticFlux> {
  /// Voltage.
  using type = dim::Voltage;
};

/// Back-EMF: magnetic flux times angular velocity is a voltage.
template <>
struct ProductDimension<dim::MagneticFlux, dim::AngularVelocity> {
  /// Voltage.
  using type = dim::Voltage;
};

/// Torque of one flux/current pair: magnetic flux times current.
template <>
struct ProductDimension<dim::MagneticFlux, dim::Current> {
  /// Torque.
  using type = dim::Torque;
};

/// Torque of one flux/current pair: current times magnetic flux.
template <>
struct ProductDimension<dim::Current, dim::MagneticFlux> {
  /// Torque.
  using type = dim::Torque;
};

/**
 * @brief Dimension of the quotient of two quantities, including the
 * electromechanical coupling.
 * @tparam D1 Dimension of the numerator.
 * @tparam D2 Dimension of the denominator.
 *
 * In general the exponents subtract (DimensionQuotient). The specializations
 * invert the coupling rules of ProductDimension where the plain quotient has
 * no meaning of its own (see ProductDimension).
 */
template <typename D1, typename D2>
struct QuotientDimension {
  /// Dimension of the quotient.
  using type = DimensionQuotient<D1, D2>;
};

/// Torque over a torque constant (flux) is the current that produces it.
template <>
struct QuotientDimension<dim::Torque, dim::MagneticFlux> {
  /// Current.
  using type = dim::Current;
};

/// Torque over current is the torque constant, a magnetic flux (N m/A = Wb).
template <>
struct QuotientDimension<dim::Torque, dim::Current> {
  /// Magnetic flux.
  using type = dim::MagneticFlux;
};

/// Back-EMF over angular velocity is the flux linkage.
template <>
struct QuotientDimension<dim::Voltage, dim::AngularVelocity> {
  /// Magnetic flux.
  using type = dim::MagneticFlux;
};

/**
 * @brief A generic class for representing a unit of measurement.
 * @tparam Rep The underlying representation type (e.g., float, double).
 * @tparam Period A std::ratio representing the scaling factor from the base
 * unit (e.g., std::milli, std::kilo).
 * @tparam Dim The Dimension of the quantity (e.g., dim::Current).
 */
template <typename Rep, typename Period = std::ratio<1>, typename Dim = dim::Dimensionless>
class Unit {
 public:
  /// Representation type used to store the numeric value.
  using representation = Rep;
  /// Scale relative to the corresponding base unit.
  using period = Period;
  /// Dimension of the physical quantity represented by this unit.
  using dimension = Dim;

  /// Constructs a zero-valued unit.
  constexpr Unit() : val_(0) {}
  /**
   * @brief Constructs a unit from a value expressed in this unit's period.
   * @param val Numeric value in the unit represented by this instance.
   */
  explicit constexpr Unit(Rep val) : val_(val) {}

  /**
   * @brief Converts a value with the same dimension from another period.
   * @param other Unit value expressed in another period.
   */
  template <typename OtherPeriod>
  constexpr Unit(const Unit<Rep, OtherPeriod, Dim>& other)
      : val_(other.Value() * static_cast<Rep>(OtherPeriod::num) / OtherPeriod::den *
             static_cast<Rep>(Period::den) / Period::num) {}

  /**
   * @brief Assigns a value with the same dimension from another period.
   * @param other Unit value expressed in another period.
   * @return This unit after conversion and assignment.
   */
  template <typename OtherPeriod>
  constexpr Unit& operator=(const Unit<Rep, OtherPeriod, Dim>& other) {
    val_ = other.Value() * static_cast<Rep>(OtherPeriod::num) / OtherPeriod::den *
           static_cast<Rep>(Period::den) / Period::num;
    return *this;
  }

  /**
   * @brief Returns the stored numeric value.
   * @return The value expressed in this unit's period.
   */
  [[nodiscard]] constexpr Rep Value() const { return val_; }

  /**
   * @brief Converts this unit to another unit of the same dimension but with a
   * different period.
   * @tparam OtherPeriod The period of the target unit.
   * @return A new Unit object with the converted value and OtherPeriod.
   *
   * @note The dimension and representation type are preserved.
   */
  template <typename OtherPeriod>
  constexpr Unit<Rep, OtherPeriod, Dim> ConvertTo() const {
    // value_base = val_ * (period::num / period::den)
    // new_val * (OtherPeriod::num / OtherPeriod::den) = value_base
    // new_val = value_base * (OtherPeriod::den / OtherPeriod::num)
    // new_val = val_ * (period::num / period::den) * (OtherPeriod::den /
    // OtherPeriod::num)
    return Unit<Rep, OtherPeriod, Dim>(val_ * static_cast<Rep>(period::num) / period::den * OtherPeriod::den /
                                       OtherPeriod::num);
  }

  /**
   * @brief Clamps this unit between two values of the same unit.
   * @param min_val The minimum allowable value.
   * @param max_val The maximum allowable value.
   * @return A new Unit object with the clamped value.
   */
  [[nodiscard]] constexpr Unit Clamp(const Unit& min_val, const Unit& max_val) const {
    return Unit(std::min(std::max(val_, min_val.val_), max_val.val_));
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
   * @brief Adds another value with the same dimension and period.
   * @param other Unit value to add.
   * @return This unit after the addition.
   */
  constexpr Unit& operator+=(const Unit& other) {
    val_ += other.val_;
    return *this;
  }
  /**
   * @brief Subtracts another value with the same dimension and period.
   * @param other Unit value to subtract.
   * @return This unit after the subtraction.
   */
  constexpr Unit& operator-=(const Unit& other) {
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

template <typename Rep, typename Period, typename Dim>
struct IsUnit<Unit<Rep, Period, Dim>> : std::true_type {};

/**
 * @brief Constrains a type to a UNIMOC unit specialization.
 * @tparam T Type to constrain.
 */
template <typename T>
concept UnitLike = IsUnit<T>::value;

// SI unit types (base type float, period std::ratio<1> for base SI units)
/// Angle in radians (rad).
using Angle = Unit<float, std::ratio<1>, dim::Angle>;
/// Angular velocity in radians per second (rad/s).
using AngularVelocity = Unit<float, std::ratio<1>, dim::AngularVelocity>;
/// Torque in Newton-metres per radian (N m/rad).
using Torque = Unit<float, std::ratio<1>, dim::Torque>;
/// Angular acceleration in radians per second squared (rad/s^2).
using AngularAcceleration = Unit<float, std::ratio<1>, dim::AngularAcceleration>;
/// Electric current in amperes (A).
using Current = Unit<float, std::ratio<1>, dim::Current>;
/// Electric potential in volts (V).
using Voltage = Unit<float, std::ratio<1>, dim::Voltage>;
/// Temperature in degrees Celsius.
using Temperature = Unit<float, std::ratio<1>, dim::Temperature>;
/// Power in watts (W).
using Power = Unit<float, std::ratio<1>, dim::Power>;
/// Magnetic flux in webers (Wb).
using MagneticFlux = Unit<float, std::ratio<1>, dim::MagneticFlux>;
/// Resistance in ohms.
using Resistance = Unit<float, std::ratio<1>, dim::Resistance>;
/// Inductance in henrys (H).
using Inductance = Unit<float, std::ratio<1>, dim::Inductance>;
/// Rotational inertia in kilogram square metres per square radian (kg m^2/rad^2).
using Inertia = Unit<float, std::ratio<1>, dim::Inertia>;

// Helper units. Aliases of the same dimension are the same type: Frequency,
// InverseTime and AngularVelocityPerAngle are 1/s; Resistance,
// VoltagePerCurrent and MagneticFluxPerCurrentTime are V/A;
// CurrentPerMagneticFlux and CurrentPerVoltageTime are A/Wb.
/// Time in seconds (s).
using Time = Unit<float, std::ratio<1>, dim::Time>;
/// Frequency in hertz (Hz).
using Frequency = Unit<float, std::ratio<1>, dim::Frequency>;
/// Inverse time in reciprocal seconds (1/s).
using InverseTime = Unit<float, std::ratio<1>, dim::Frequency>;
/// Dimensionless ratio, such as a sine or cosine result.
using Ratio = Unit<float, std::ratio<1>, dim::Dimensionless>;
/// Voltage per current gain [V/A].
using VoltagePerCurrent = Unit<float, std::ratio<1>, dim::Resistance>;
/// Voltage per current per time gain [V/(A s)].
using VoltagePerCurrentTime = Unit<float, std::ratio<1>, DimensionQuotient<dim::Resistance, dim::Time>>;
/// Current per magnetic flux gain [A/Wb].
using CurrentPerMagneticFlux = Unit<float, std::ratio<1>, DimensionQuotient<dim::Current, dim::MagneticFlux>>;
/// Current per magnetic flux per time gain [A/(Wb s)].
using CurrentPerMagneticFluxTime =
    Unit<float,
         std::ratio<1>,
         DimensionQuotient<DimensionQuotient<dim::Current, dim::MagneticFlux>, dim::Time>>;
/// Current rate [A/s].
using CurrentPerTime = Unit<float, std::ratio<1>, DimensionQuotient<dim::Current, dim::Time>>;
/// Current per voltage per time gain [A/(V s)].
using CurrentPerVoltageTime = Unit<float, std::ratio<1>, DimensionQuotient<dim::Current, dim::MagneticFlux>>;
/// Angle per current gain [rad/A].
using AnglePerCurrent = Unit<float, std::ratio<1>, DimensionQuotient<dim::Angle, dim::Current>>;
/// Angular velocity per angle gain [(rad/s)/rad = 1/s].
using AngularVelocityPerAngle = Unit<float, std::ratio<1>, dim::Frequency>;
/// Magnetic flux per current per time observer gain [Wb/(A s) = V/A].
using MagneticFluxPerCurrentTime = Unit<float, std::ratio<1>, dim::Resistance>;
/// Viscous rotational damping, torque per angular velocity [N m s/rad^2].
using TorquePerAngularVelocity =
    Unit<float, std::ratio<1>, DimensionQuotient<dim::Torque, dim::AngularVelocity>>;

// --- Operators for units ---

/**
 * @brief Adds two unit values with the same dimension and period.
 * @return The sum with the same unit type as the operands.
 */
template <typename Rep, typename Period, typename Dim>
constexpr Unit<Rep, Period, Dim> operator+(const Unit<Rep, Period, Dim>& lhs,
                                           const Unit<Rep, Period, Dim>& rhs) {
  return Unit<Rep, Period, Dim>(lhs.Value() + rhs.Value());
}

/**
 * @brief Subtracts two unit values with the same dimension and period.
 * @return The difference with the same unit type as the operands.
 */
template <typename Rep, typename Period, typename Dim>
constexpr Unit<Rep, Period, Dim> operator-(const Unit<Rep, Period, Dim>& lhs,
                                           const Unit<Rep, Period, Dim>& rhs) {
  return Unit<Rep, Period, Dim>(lhs.Value() - rhs.Value());
}

/**
 * @brief Multiplies a unit value by a scalar.
 * @return A unit with the same dimension and period as `lhs`.
 */
template <typename Rep, typename Period, typename Dim>
constexpr Unit<Rep, Period, Dim> operator*(const Unit<Rep, Period, Dim>& lhs, const Rep& scalar) {
  return Unit<Rep, Period, Dim>(lhs.Value() * scalar);
}

/**
 * @brief Multiplies a scalar by a unit value.
 * @return A unit with the same dimension and period as `rhs`.
 */
template <typename Rep, typename Period, typename Dim>
constexpr Unit<Rep, Period, Dim> operator*(const Rep& scalar, const Unit<Rep, Period, Dim>& rhs) {
  return Unit<Rep, Period, Dim>(scalar * rhs.Value());
}

/**
 * @brief Divides a unit value by a scalar.
 * @return A unit with the same dimension and period as `lhs`.
 */
template <typename Rep, typename Period, typename Dim>
constexpr Unit<Rep, Period, Dim> operator/(const Unit<Rep, Period, Dim>& lhs, const Rep& scalar) {
  return Unit<Rep, Period, Dim>(lhs.Value() / scalar);
}

/**
 * @brief Divides a scalar by a unit value, e.g. 1 / Time to obtain a frequency.
 * @return A unit with the inverse dimension and the reciprocal period.
 */
template <typename Rep, typename Period, typename Dim>
constexpr auto operator/(const Rep& scalar, const Unit<Rep, Period, Dim>& rhs) {
  return Unit<Rep, std::ratio_divide<std::ratio<1>, Period>, DimensionQuotient<dim::Dimensionless, Dim>>(
      scalar / rhs.Value());
}

/**
 * @brief Multiplies two unit values.
 * @return A unit with the product dimension (ProductDimension) and the product period.
 */
template <typename Rep, typename P1, typename D1, typename P2, typename D2>
constexpr auto operator*(const Unit<Rep, P1, D1>& lhs, const Unit<Rep, P2, D2>& rhs) {
  return Unit<Rep, std::ratio_multiply<P1, P2>, typename ProductDimension<D1, D2>::type>(lhs.Value() *
                                                                                         rhs.Value());
}

/**
 * @brief Divides two unit values.
 * @return A unit with the quotient dimension (QuotientDimension) and the quotient period; a
 * dimensionless ratio for operands of the same dimension.
 */
template <typename Rep, typename P1, typename D1, typename P2, typename D2>
constexpr auto operator/(const Unit<Rep, P1, D1>& lhs, const Unit<Rep, P2, D2>& rhs) {
  return Unit<Rep, std::ratio_divide<P1, P2>, typename QuotientDimension<D1, D2>::type>(lhs.Value() /
                                                                                        rhs.Value());
}

/**
 * @brief Compares two unit values for equality.
 * @return `true` when both values have the same numeric value.
 *
 * Both operands must have the same dimension and period. Convert explicitly
 * before comparing values expressed in different periods.
 */
template <typename Rep, typename Period, typename Dim>
constexpr bool operator==(const Unit<Rep, Period, Dim>& lhs, const Unit<Rep, Period, Dim>& rhs) {
  return lhs.Value() == rhs.Value();
}

/**
 * @brief Compares two unit values for inequality.
 * @return `true` when the numeric values differ.
 */
template <typename Rep, typename Period, typename Dim>
constexpr bool operator!=(const Unit<Rep, Period, Dim>& lhs, const Unit<Rep, Period, Dim>& rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Tests whether one unit value is less than another.
 * @return `true` when `lhs` has the smaller numeric value.
 */
template <typename Rep, typename Period, typename Dim>
constexpr bool operator<(const Unit<Rep, Period, Dim>& lhs, const Unit<Rep, Period, Dim>& rhs) {
  return lhs.Value() < rhs.Value();
}

/**
 * @brief Tests whether one unit value is less than or equal to another.
 * @return `true` when `lhs` is no greater than `rhs`.
 */
template <typename Rep, typename Period, typename Dim>
constexpr bool operator<=(const Unit<Rep, Period, Dim>& lhs, const Unit<Rep, Period, Dim>& rhs) {
  return lhs.Value() <= rhs.Value();
}

/**
 * @brief Tests whether one unit value is greater than another.
 * @return `true` when `lhs` has the greater numeric value.
 */
template <typename Rep, typename Period, typename Dim>
constexpr bool operator>(const Unit<Rep, Period, Dim>& lhs, const Unit<Rep, Period, Dim>& rhs) {
  return lhs.Value() > rhs.Value();
}

/**
 * @brief Tests whether one unit value is greater than or equal to another.
 * @return `true` when `lhs` is no less than `rhs`.
 */
template <typename Rep, typename Period, typename Dim>
constexpr bool operator>=(const Unit<Rep, Period, Dim>& lhs, const Unit<Rep, Period, Dim>& rhs) {
  return lhs.Value() >= rhs.Value();
}

// =========================================================================
// Math on units: no escape through Value() for the common operations
// =========================================================================

/**
 * @brief Absolute value of a unit.
 * @return The magnitude with the same dimension and period.
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] constexpr Unit<Rep, Period, Dim> Abs(const Unit<Rep, Period, Dim>& value) noexcept {
  return (value.Value() < Rep{0}) ? -value : value;
}

/**
 * @brief Smaller of two values of the same unit.
 * @return `lhs` when the values are equal.
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] constexpr Unit<Rep, Period, Dim> Min(const Unit<Rep, Period, Dim>& lhs,
                                                   const Unit<Rep, Period, Dim>& rhs) noexcept {
  return (rhs < lhs) ? rhs : lhs;
}

/**
 * @brief Larger of two values of the same unit.
 * @return `lhs` when the values are equal.
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] constexpr Unit<Rep, Period, Dim> Max(const Unit<Rep, Period, Dim>& lhs,
                                                   const Unit<Rep, Period, Dim>& rhs) noexcept {
  return (lhs < rhs) ? rhs : lhs;
}

/**
 * @brief True when the value is neither infinite nor NaN.
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] constexpr bool IsFinite(const Unit<Rep, Period, Dim>& value) noexcept {
  return std::isfinite(value.Value());
}

/**
 * @brief Magnitude of the vector (first, second) of two values of one unit.
 *
 * sqrt(first^2 + second^2) in the unit of the components, e.g. the length of a current
 * vector. A plain square root (`vsqrt.f32` on the Cortex-M4 with
 * -fno-math-errno), so usable in the hot path; no overflow protection beyond
 * float range.
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] Unit<Rep, Period, Dim> Hypot(const Unit<Rep, Period, Dim>& first,
                                           const Unit<Rep, Period, Dim>& second) noexcept {
  return Unit<Rep, Period, Dim>(
      std::sqrt((first.Value() * first.Value()) + (second.Value() * second.Value())));
}

/**
 * @brief Square root of a squared quantity, e.g. the current of a squared current.
 * @return The root in the base period with the halved dimension exponents.
 *
 * Does not compile for a quantity with an odd dimension exponent. The value is
 * converted to the base period first, so the period of the result is
 * std::ratio<1>. A plain square root (`vsqrt.f32` on the Cortex-M4 with
 * -fno-math-errno), so usable in the hot path.
 */
template <typename Rep, typename Period, typename Dim>
[[nodiscard]] Unit<Rep, std::ratio<1>, typename DimensionRoot<Dim>::type> Sqrt(
    const Unit<Rep, Period, Dim>& value) noexcept {
  const Unit<Rep, std::ratio<1>, Dim> kBase = value;
  return Unit<Rep, std::ratio<1>, typename DimensionRoot<Dim>::type>(std::sqrt(kBase.Value()));
}

/**
 * @brief Creates an angle from a radian literal.
 * @param val Literal value in radians.
 * @return An angle in radians.
 */
constexpr Angle operator""_rad(long double val) { return Angle(static_cast<float>(val)); }
constexpr Angle operator""_deg(long double val) {
  return Angle(static_cast<float>(val * std::numbers::pi_v<long double> / 180.0L));
}

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
constexpr Unit<float, std::milli, dim::Time> operator""_ms(long double val) {
  return Unit<float, std::milli, dim::Time>(static_cast<float>(val));
}

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
constexpr Unit<float, std::milli, dim::Time> operator""_ms(unsigned long long val) {
  return Unit<float, std::milli, dim::Time>(static_cast<float>(val));
}

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
constexpr Unit<float, std::milli, dim::Current> operator""_mA(long double val) {
  return Unit<float, std::milli, dim::Current>(static_cast<float>(val));
}

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
constexpr Unit<float, std::milli, dim::Current> operator""_mA(unsigned long long val) {
  return Unit<float, std::milli, dim::Current>(static_cast<float>(val));
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
constexpr Unit<float, std::milli, dim::Voltage> operator""_mV(long double val) {
  return Unit<float, std::milli, dim::Voltage>(static_cast<float>(val));
}

/**
 * @brief Creates a voltage value from a kilovolt literal.
 * @param val Literal value in kilovolts.
 * @return Voltage with a kilovolt period.
 */
constexpr Unit<float, std::kilo, dim::Voltage> operator""_kV(long double val) {
  return Unit<float, std::kilo, dim::Voltage>(static_cast<float>(val));
}

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
constexpr Unit<float, std::milli, dim::Voltage> operator""_mV(unsigned long long val) {
  return Unit<float, std::milli, dim::Voltage>(static_cast<float>(val));
}

/**
 * @brief Creates a voltage value from a kilovolt integer literal.
 * @param val Literal value in kilovolts.
 * @return Voltage with a kilovolt period.
 */
constexpr Unit<float, std::kilo, dim::Voltage> operator""_kV(unsigned long long val) {
  return Unit<float, std::kilo, dim::Voltage>(static_cast<float>(val));
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
constexpr Unit<float, std::kilo, dim::Frequency> operator""_kHz(long double val) {
  return Unit<float, std::kilo, dim::Frequency>(static_cast<float>(val));
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
constexpr Unit<float, std::kilo, dim::Frequency> operator""_kHz(unsigned long long val) {
  return Unit<float, std::kilo, dim::Frequency>(static_cast<float>(val));
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
constexpr Unit<float, std::kilo, dim::Power> operator""_kW(long double val) {
  return Unit<float, std::kilo, dim::Power>(static_cast<float>(val));
}

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
constexpr Unit<float, std::kilo, dim::Power> operator""_kW(unsigned long long val) {
  return Unit<float, std::kilo, dim::Power>(static_cast<float>(val));
}

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
constexpr Unit<float, std::milli, dim::MagneticFlux> operator""_mWb(long double val) {
  return Unit<float, std::milli, dim::MagneticFlux>(static_cast<float>(val));
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
constexpr Unit<float, std::milli, dim::MagneticFlux> operator""_mWb(unsigned long long val) {
  return Unit<float, std::milli, dim::MagneticFlux>(static_cast<float>(val));
}

/**
 * @brief Creates a magnetic-flux value from a microweber literal.
 * @param val Literal value in microwebers.
 * @return Magnetic flux with a microweber period.
 */
constexpr Unit<float, std::micro, dim::MagneticFlux> operator""_uWb(long double val) {
  return Unit<float, std::micro, dim::MagneticFlux>(static_cast<float>(val));
}

constexpr Unit<float, std::micro, dim::MagneticFlux> operator""_uWb(unsigned long long val) {
  return Unit<float, std::micro, dim::MagneticFlux>(static_cast<float>(val));
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
constexpr Unit<float, std::kilo, dim::Resistance> operator""_kOhm(long double val) {
  return Unit<float, std::kilo, dim::Resistance>(static_cast<float>(val));
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
constexpr Unit<float, std::kilo, dim::Resistance> operator""_kOhm(unsigned long long val) {
  return Unit<float, std::kilo, dim::Resistance>(static_cast<float>(val));
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
constexpr Unit<float, std::milli, dim::Inductance> operator""_mH(long double val) {
  return Unit<float, std::milli, dim::Inductance>(static_cast<float>(val));
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
constexpr Unit<float, std::milli, dim::Inductance> operator""_mH(unsigned long long val) {
  return Unit<float, std::milli, dim::Inductance>(static_cast<float>(val));
}

/**
 * @brief Creates a time value from a microsecond literal.
 * @param val Literal value in microseconds.
 * @return Time with a microsecond period.
 */
constexpr Unit<float, std::micro, dim::Time> operator""_us(long double val) {
  return Unit<float, std::micro, dim::Time>(static_cast<float>(val));
}

constexpr Unit<float, std::micro, dim::Time> operator""_us(unsigned long long val) {
  return Unit<float, std::micro, dim::Time>(static_cast<float>(val));
}

/**
 * @brief Creates an angular velocity from a radians-per-second literal.
 * @param val Literal value in radians per second.
 * @return Angular velocity in radians per second.
 */
constexpr AngularVelocity operator""_rad_per_s(long double val) {
  return AngularVelocity(static_cast<float>(val));
}
constexpr AngularVelocity operator""_rad_per_s(unsigned long long val) {
  return AngularVelocity(static_cast<float>(val));
}

/**
 * @brief Creates an angular acceleration from a radians-per-second-squared
 * literal.
 * @param val Literal value in radians per second squared.
 * @return Angular acceleration in radians per second squared.
 */
constexpr AngularAcceleration operator""_rad_per_s2(long double val) {
  return AngularAcceleration(static_cast<float>(val));
}
constexpr AngularAcceleration operator""_rad_per_s2(unsigned long long val) {
  return AngularAcceleration(static_cast<float>(val));
}

/**
 * @brief Creates an inverse-time value from a reciprocal-second literal.
 * @param val Literal value in reciprocal seconds.
 * @return Inverse time in reciprocal seconds.
 */
constexpr InverseTime operator""_per_s(long double val) { return InverseTime(static_cast<float>(val)); }
constexpr InverseTime operator""_per_s(unsigned long long val) {
  return InverseTime(static_cast<float>(val));
}

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
constexpr VoltagePerCurrent operator""_V_per_A(long double val) {
  return VoltagePerCurrent(static_cast<float>(val));
}
constexpr VoltagePerCurrent operator""_V_per_A(unsigned long long val) {
  return VoltagePerCurrent(static_cast<float>(val));
}

/**
 * @brief Creates a voltage-per-current-per-time gain literal.
 * @param val Gain value in V/(A s).
 * @return A voltage-per-current-per-time gain.
 */
constexpr VoltagePerCurrentTime operator""_V_per_A_s(long double val) {
  return VoltagePerCurrentTime(static_cast<float>(val));
}
constexpr VoltagePerCurrentTime operator""_V_per_A_s(unsigned long long val) {
  return VoltagePerCurrentTime(static_cast<float>(val));
}

/**
 * @brief Creates a current-per-flux gain literal.
 * @param val Gain value in A/Wb.
 * @return A current-per-flux gain.
 */
constexpr CurrentPerMagneticFlux operator""_A_per_Wb(long double val) {
  return CurrentPerMagneticFlux(static_cast<float>(val));
}
constexpr CurrentPerMagneticFlux operator""_A_per_Wb(unsigned long long val) {
  return CurrentPerMagneticFlux(static_cast<float>(val));
}

/**
 * @brief Creates a current-per-flux-per-time gain literal.
 * @param val Gain value in A/(Wb s).
 * @return A current-per-flux-per-time gain.
 */
constexpr CurrentPerMagneticFluxTime operator""_A_per_Wb_s(long double val) {
  return CurrentPerMagneticFluxTime(static_cast<float>(val));
}
constexpr CurrentPerMagneticFluxTime operator""_A_per_Wb_s(unsigned long long val) {
  return CurrentPerMagneticFluxTime(static_cast<float>(val));
}

/**
 * @brief Creates a current-rate literal.
 * @param val Rate value in A/s.
 * @return A current rate.
 */
constexpr CurrentPerTime operator""_A_per_s(long double val) {
  return CurrentPerTime(static_cast<float>(val));
}
constexpr CurrentPerTime operator""_A_per_s(unsigned long long val) {
  return CurrentPerTime(static_cast<float>(val));
}

/**
 * @brief Creates a current-per-voltage-per-time gain literal.
 * @param val Gain value in A/(V s).
 * @return A current-per-voltage-per-time gain.
 */
constexpr CurrentPerVoltageTime operator""_A_per_V_s(long double val) {
  return CurrentPerVoltageTime(static_cast<float>(val));
}
constexpr CurrentPerVoltageTime operator""_A_per_V_s(unsigned long long val) {
  return CurrentPerVoltageTime(static_cast<float>(val));
}

/**
 * @brief Creates an angle-per-current gain literal.
 * @param val Gain value in rad/A.
 * @return An angle-per-current gain.
 */
constexpr AnglePerCurrent operator""_rad_per_A(long double val) {
  return AnglePerCurrent(static_cast<float>(val));
}
constexpr AnglePerCurrent operator""_rad_per_A(unsigned long long val) {
  return AnglePerCurrent(static_cast<float>(val));
}

/**
 * @brief Creates an angular-velocity-per-angle gain literal.
 * @param val Gain value in (rad/s)/rad.
 * @return An angular-velocity-per-angle gain.
 */
constexpr AngularVelocityPerAngle operator""_rad_per_s_per_rad(long double val) {
  return AngularVelocityPerAngle(static_cast<float>(val));
}
constexpr AngularVelocityPerAngle operator""_rad_per_s_per_rad(unsigned long long val) {
  return AngularVelocityPerAngle(static_cast<float>(val));
}

/**
 * @brief Creates a magnetic-flux-per-current-per-time gain literal.
 * @param val Gain value in Wb/(A s).
 * @return A magnetic-flux-per-current-per-time gain.
 */
constexpr MagneticFluxPerCurrentTime operator""_Wb_per_A_s(long double val) {
  return MagneticFluxPerCurrentTime(static_cast<float>(val));
}
constexpr MagneticFluxPerCurrentTime operator""_Wb_per_A_s(unsigned long long val) {
  return MagneticFluxPerCurrentTime(static_cast<float>(val));
}

/**
 * @brief Creates a viscous rotational damping literal.
 * @param val Damping value in N m s/rad.
 * @return A torque-per-angular-velocity value.
 */
constexpr TorquePerAngularVelocity operator""_Nm_s_per_rad(long double val) {
  return TorquePerAngularVelocity(static_cast<float>(val));
}
constexpr TorquePerAngularVelocity operator""_Nm_s_per_rad(unsigned long long val) {
  return TorquePerAngularVelocity(static_cast<float>(val));
}

/**
 * @brief Creates a temperature value from a degrees-Celsius literal.
 * @param val Literal value in degrees Celsius.
 * @return Temperature in degrees Celsius.
 */
constexpr Temperature operator""_degC(long double val) { return Temperature(static_cast<float>(val)); }
constexpr Temperature operator""_degC(unsigned long long val) { return Temperature(static_cast<float>(val)); }

/**
 * @brief Tests whether a unit type has the given dimension, in any period.
 * @tparam T Unit type whose `dimension` member is inspected.
 * @tparam Dim Dimension to compare with.
 */
template <typename T, typename Dim>
struct HasDimension : std::is_same<typename T::dimension, Dim> {};

/**
 * @brief Tests whether a type has the angle dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsAngle : HasDimension<T, dim::Angle> {};

/**
 * @brief Tests whether a type has the angular-velocity dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsAngularVelocity : HasDimension<T, dim::AngularVelocity> {};

/**
 * @brief Tests whether a type has the torque dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsTorque : HasDimension<T, dim::Torque> {};

/**
 * @brief Tests whether a type has the angular-acceleration dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsAngularAcceleration : HasDimension<T, dim::AngularAcceleration> {};

/**
 * @brief Tests whether a type has the current dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsCurrent : HasDimension<T, dim::Current> {};

/**
 * @brief Tests whether a type has the voltage dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsVoltage : HasDimension<T, dim::Voltage> {};

/**
 * @brief Tests whether a type has the power dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsPower : HasDimension<T, dim::Power> {};

/**
 * @brief Tests whether a type has the magnetic-flux dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsMagneticFlux : HasDimension<T, dim::MagneticFlux> {};

/**
 * @brief Tests whether a type has the resistance dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsResistance : HasDimension<T, dim::Resistance> {};

/**
 * @brief Tests whether a type has the inductance dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsInductance : HasDimension<T, dim::Inductance> {};

/**
 * @brief Tests whether a type has the time dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsTime : HasDimension<T, dim::Time> {};

/**
 * @brief Tests whether a type has the frequency dimension.
 * @tparam T Unit type whose `dimension` member is inspected.
 */
template <typename T>
struct IsFrequency : HasDimension<T, dim::Frequency> {};

/**
 * @brief Computes a common unit type for two units with the same dimension.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type with the same dimension as T1.
 *
 * The resulting representation and period are the common types of the two
 * operands. The dimension must match exactly.
 */
template <typename T1, typename T2>
struct CommonUnitType {
  static_assert(std::is_same_v<typename T1::dimension, typename T2::dimension>,
                "Units must have the same dimension to find common type.");
  /// Common unit type derived from the two input representations and periods.
  using type = Unit<std::common_type_t<typename T1::representation, typename T2::representation>,
                    std::common_type_t<typename T1::period, typename T2::period>,
                    typename T1::dimension>;
};

/**
 * @brief Shorthand for the common unit type of two same-dimension units.
 * @tparam T1 First unit type.
 * @tparam T2 Second unit type.
 */
template <typename T1, typename T2>
using common_unit_type_t = CommonUnitType<T1, T2>::type;

}  // namespace unimoc::unit
