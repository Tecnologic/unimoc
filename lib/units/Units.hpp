/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifndef UNIMOC_UNITLS_H_
#define UNIMOC_UNITLS_H_

#include <numbers>  // For std::numbers::pi_v, C++20
#include <ratio>
#include <type_traits>  // For std::is_same_v, std::common_type_t

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace unit unit namespace for SI units.
 */
namespace unit
{
// Forward declaration for unit tags
struct AngleTag;
struct AngularVelocityTag;
struct TorqueTag;
struct AngularAccelerationTag;
struct CurrentTag;
struct VoltageTag;
struct PowerTag;
struct MagneticFluxTag;
struct ResistanceTag;
struct InductanceTag;
struct TimeTag;
struct FrequencyTag;
struct DimensionlessRatioTag;

/**
 * @brief A generic class for representing a unit of measurement.
 * @tparam Rep The underlying representation type (e.g., float, double).
 * @tparam Period A std::ratio representing the scaling factor from the base unit (e.g., std::milli,
 * std::kilo).
 * @tparam Tag A tag struct to differentiate unit types (e.g., AngleTag, CurrentTag).
 */
template<typename Rep, typename Period = std::ratio<1>, typename Tag = void>
class Unit
{
public:
	using representation = Rep;
	using period = Period;
	using tag = Tag;

	constexpr Unit() : val_(0) {}
	explicit constexpr Unit(Rep val) : val_(val) {}

	constexpr Rep
	value() const
	{
		return val_;
	}

	/**
	 * @brief Converts this unit to another unit of the same type but with a different period.
	 * @tparam OtherPeriod The period of the target unit.
	 * @return A new Unit object with the converted value and OtherPeriod.
	 */
	template<typename OtherPeriod>
	constexpr Unit<Rep, OtherPeriod, Tag>
	convert_to() const
	{
		// value_base = val_ * (period::num / period::den)
		// new_val * (OtherPeriod::num / OtherPeriod::den) = value_base
		// new_val = value_base * (OtherPeriod::den / OtherPeriod::num)
		// new_val = val_ * (period::num / period::den) * (OtherPeriod::den / OtherPeriod::num)
		return Unit<Rep, OtherPeriod, Tag>(val_ * static_cast<Rep>(period::num) / period::den *
										   OtherPeriod::den / OtherPeriod::num);
	}

	// Unary operators
	constexpr Unit
	operator+() const
	{
		return Unit(val_);
	}
	constexpr Unit
	operator-() const
	{
		return Unit(-val_);
	}

	// Compound assignment operators
	constexpr Unit&
	operator+=(const Unit& other)
	{
		static_assert(std::is_same_v<tag, typename decltype(other)::tag>,
					  "Cannot add units of different types.");
		static_assert(std::is_same_v<period, typename decltype(other)::period>,
					  "Implicit period conversion not allowed for addition/subtraction. Convert "
					  "explicitly or ensure same period.");
		val_ += other.val_;
		return *this;
	}
	constexpr Unit&
	operator-=(const Unit& other)
	{
		static_assert(std::is_same_v<tag, typename decltype(other)::tag>,
					  "Cannot subtract units of different types.");
		static_assert(std::is_same_v<period, typename decltype(other)::period>,
					  "Implicit period conversion not allowed for addition/subtraction. Convert "
					  "explicitly or ensure same period.");
		val_ -= other.val_;
		return *this;
	}
	constexpr Unit&
	operator*=(const Rep& scalar)
	{
		val_ *= scalar;
		return *this;
	}
	constexpr Unit&
	operator/=(const Rep& scalar)
	{
		val_ /= scalar;
		return *this;
	}

private:
	Rep val_;
};

// SI Unit types (base type float, period std::ratio<1> for base SI units)
using Angle = Unit<float, std::ratio<1>, AngleTag>;  // radians (rad)
using AngularVelocity =
	Unit<float, std::ratio<1>, AngularVelocityTag>;    // radians per second (rad/s)
using Torque = Unit<float, std::ratio<1>, TorqueTag>;  // Newton-meters (Nm)
using AngularAcceleration =
	Unit<float, std::ratio<1>, AngularAccelerationTag>;  // radians per second squared (rad/s^2)
using Current = Unit<float, std::ratio<1>, CurrentTag>;  // Amperes (A)
using Voltage = Unit<float, std::ratio<1>, VoltageTag>;  // Volts (V)
using Power = Unit<float, std::ratio<1>, PowerTag>;      // Watts (W)
using MagneticFlux = Unit<float, std::ratio<1>, MagneticFluxTag>;  // Webers (Wb)
using Resistance = Unit<float, std::ratio<1>, ResistanceTag>;      // Ohms (Ω)
using Inductance = Unit<float, std::ratio<1>, InductanceTag>;      // Henrys (H)

// Helper units
using Time = Unit<float, std::ratio<1>, TimeTag>;            // seconds (s)
using Frequency = Unit<float, std::ratio<1>, FrequencyTag>;  // Hertz (Hz)
using DimensionlessRatio =
	Unit<float, std::ratio<1>, DimensionlessRatioTag>;  // New type alias for sin/cos results

// --- Operators for units ---

// Binary arithmetic for same unit type and period
template<typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag>
operator+(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return Unit<Rep, Period, Tag>(lhs.value() + rhs.value());
}

template<typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag>
operator-(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return Unit<Rep, Period, Tag>(lhs.value() - rhs.value());
}

// Scalar multiplication and division
template<typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag>
operator*(const Unit<Rep, Period, Tag>& lhs, const Rep& scalar)
{
	return Unit<Rep, Period, Tag>(lhs.value() * scalar);
}

template<typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag>
operator*(const Rep& scalar, const Unit<Rep, Period, Tag>& rhs)
{
	return Unit<Rep, Period, Tag>(scalar * rhs.value());
}

template<typename Rep, typename Period, typename Tag>
constexpr Unit<Rep, Period, Tag>
operator/(const Unit<Rep, Period, Tag>& lhs, const Rep& scalar)
{
	return Unit<Rep, Period, Tag>(lhs.value() / scalar);
}

// Division of same units (same tag and period) results in a scalar
template<typename Rep, typename Period, typename Tag>
constexpr Rep
operator/(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return lhs.value() / rhs.value();
}

// Operations with DimensionlessRatio
// DimensionlessRatio is a special case where it can be multiplied/divided with any unit
// Unit * DimensionlessRatio -> Unit (maintains original unit's tag and period)
template<typename Rep, typename P, typename Tag>
constexpr auto
operator*(const Unit<Rep, P, Tag>& unit, const DimensionlessRatio& ratio)
{
	return Unit<Rep, P, Tag>(unit.value() * ratio.value());
}

template<typename Rep, typename P, typename Tag>
constexpr auto
operator*(const DimensionlessRatio& ratio, const Unit<Rep, P, Tag>& unit)
{
	return unit * ratio;  // Commutative
}

// Unit / DimensionlessRatio -> Unit (maintains original unit's tag and period)
template<typename Rep, typename P, typename Tag>
constexpr auto
operator/(const Unit<Rep, P, Tag>& unit, const DimensionlessRatio& ratio)
{
	return Unit<Rep, P, Tag>(unit.value() / ratio.value());
}

// --- Mixed unit operations ---

// Angle, Time, AngularVelocity, AngularAcceleration
// AngularVelocity = Angle / Time
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, AngleTag>& angle, const Unit<Rep, P2, TimeTag>& time)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, AngularVelocityTag>(angle.value() / time.value());
}
// Angle = AngularVelocity * Time
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, TimeTag>& time)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, AngleTag>(angVel.value() * time.value());
}
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, AngularVelocityTag>& angVel)
{
	return angVel * time;
}
// Time = Angle / AngularVelocity
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, AngleTag>& angle, const Unit<Rep, P2, AngularVelocityTag>& angVel)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(angle.value() / angVel.value());
}

// AngularAcceleration = AngularVelocity / Time
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, TimeTag>& time)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, AngularAccelerationTag>(angVel.value() /
																		time.value());
}
// AngularVelocity = AngularAcceleration * Time
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, AngularAccelerationTag>& angAccel, const Unit<Rep, P2, TimeTag>& time)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, AngularVelocityTag>(angAccel.value() *
																	  time.value());
}
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, AngularAccelerationTag>& angAccel)
{
	return angAccel * time;
}
// Time = AngularVelocity / AngularAcceleration
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, AngularVelocityTag>& angVel,
		  const Unit<Rep, P2, AngularAccelerationTag>& angAccel)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(angVel.value() / angAccel.value());
}

// Voltage, Current, Power
// Power = Voltage * Current
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, CurrentTag>& current)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, PowerTag>(voltage.value() * current.value());
}
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, CurrentTag>& current, const Unit<Rep, P2, VoltageTag>& voltage)
{
	return voltage * current;
}
// Voltage = Power / Current
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, CurrentTag>& current)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, VoltageTag>(power.value() / current.value());
}
// Current = Power / Voltage
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, VoltageTag>& voltage)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, CurrentTag>(power.value() / voltage.value());
}

// MagneticFlux, Time, Voltage (Faraday's Law: V = d(Flux)/dt)
// Voltage = MagneticFlux / Time
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, TimeTag>& time)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, VoltageTag>(flux.value() / time.value());
}
// MagneticFlux = Voltage * Time
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, TimeTag>& time)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, MagneticFluxTag>(voltage.value() * time.value());
}
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, TimeTag>& time, const Unit<Rep, P2, VoltageTag>& voltage)
{
	return voltage * time;
}
// Time = MagneticFlux / Voltage
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, MagneticFluxTag>& flux, const Unit<Rep, P2, VoltageTag>& voltage)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, TimeTag>(flux.value() / voltage.value());
}

// Resistance, Voltage, Current (Ohm's Law: V = I * R)
// Voltage = Current * Resistance
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, CurrentTag>& current, const Unit<Rep, P2, ResistanceTag>& resistance)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, VoltageTag>(current.value() * resistance.value());
}

template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, ResistanceTag>& resistance, const Unit<Rep, P2, CurrentTag>& current)
{
	return current * resistance;
}

// Current = Voltage / Resistance
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, ResistanceTag>& resistance)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, CurrentTag>(voltage.value() / resistance.value());
}

// Resistance = Voltage / Current
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, CurrentTag>& current)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, ResistanceTag>(voltage.value() / current.value());
}

// Inductance, Current, Voltage (Faraday's Law for inductors: V = L * dI/dt)
// Voltage = Inductance * d(Current)/dt
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, InductanceTag>& inductance,
		  const Unit<Rep, P2, AngularVelocityTag>& angVel)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, VoltageTag>(inductance.value() * angVel.value());
}

template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, AngularVelocityTag>& angVel,
		  const Unit<Rep, P2, InductanceTag>& inductance)
{
	return inductance * angVel;
}

// Current = Voltage / Inductance
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, VoltageTag>& voltage, const Unit<Rep, P2, InductanceTag>& inductance)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, CurrentTag>(voltage.value() / inductance.value());
}

// Torque, AngularVelocity, Power (Power = Torque * AngularVelocity)
// Power = Torque * AngularVelocity
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, TorqueTag>& torque, const Unit<Rep, P2, AngularVelocityTag>& angVel)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, PowerTag>(torque.value() * angVel.value());
}
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, AngularVelocityTag>& angVel, const Unit<Rep, P2, TorqueTag>& torque)
{
	return torque * angVel;
}
// Torque = Power / AngularVelocity
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, AngularVelocityTag>& angVel)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, TorqueTag>(power.value() / angVel.value());
}
// AngularVelocity = Power / Torque
template<typename Rep, typename P1, typename P2>
constexpr auto
operator/(const Unit<Rep, P1, PowerTag>& power, const Unit<Rep, P2, TorqueTag>& torque)
{
	return Unit<Rep, std::ratio_divide<P1, P2>, AngularVelocityTag>(power.value() / torque.value());
}

// Frequency and Time (Frequency = scalar / Time)
template<typename Rep, typename Period>
constexpr auto
operator/(const Rep& scalar, const Unit<Rep, Period, TimeTag>& time)
{
	return Unit<Rep, std::ratio_divide<std::ratio<1>, Period>, FrequencyTag>(scalar / time.value());
}
template<typename Rep, typename Period>
constexpr auto
operator/(const Rep& scalar, const Unit<Rep, Period, FrequencyTag>& freq)
{
	return Unit<Rep, std::ratio_divide<std::ratio<1>, Period>, TimeTag>(scalar / freq.value());
}

// AngularVelocity = Angle * Frequency
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, AngleTag>& angle, const Unit<Rep, P2, FrequencyTag>& freq)
{
	return Unit<Rep, std::ratio_multiply<P1, P2>, AngularVelocityTag>(angle.value() * freq.value());
}
template<typename Rep, typename P1, typename P2>
constexpr auto
operator*(const Unit<Rep, P1, FrequencyTag>& freq, const Unit<Rep, P2, AngleTag>& angle)
{
	return angle * freq;
}

// Comparison operators (only for same unit type and period)
// For comparisons between different periods, convert one unit explicitly first.
template<typename Rep, typename Period, typename Tag>
constexpr bool
operator==(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return lhs.value() == rhs.value();
}
template<typename Rep, typename Period, typename Tag>
constexpr bool
operator!=(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return !(lhs == rhs);
}
template<typename Rep, typename Period, typename Tag>
constexpr bool
operator<(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return lhs.value() < rhs.value();
}
template<typename Rep, typename Period, typename Tag>
constexpr bool
operator<=(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return lhs.value() <= rhs.value();
}
template<typename Rep, typename Period, typename Tag>
constexpr bool
operator>(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return lhs.value() > rhs.value();
}
template<typename Rep, typename Period, typename Tag>
constexpr bool
operator>=(const Unit<Rep, Period, Tag>& lhs, const Unit<Rep, Period, Tag>& rhs)
{
	return lhs.value() >= rhs.value();
}

// --- User-defined literals ---
// Angle literals
constexpr Angle
operator"" _rad(long double val)
{
	return Angle(static_cast<float>(val));
}
constexpr Angle
operator"" _deg(long double val)
{
	return Angle(static_cast<float>(val * std::numbers::pi_v<long double> / 180.0L));
}
constexpr Angle
operator"" _rad(unsigned long long val)
{
	return Angle(static_cast<float>(val));
}
constexpr Angle
operator"" _deg(unsigned long long val)
{
	return Angle(static_cast<float>(static_cast<long double>(val) *
									std::numbers::pi_v<long double> / 180.0L));
}

// Time literals
constexpr Time
operator"" _s(long double val)
{
	return Time(static_cast<float>(val));
}
constexpr Unit<float, std::milli, TimeTag> operator"" _ms(long double val)
{
	return Unit<float, std::milli, TimeTag>(static_cast<float>(val));
}
constexpr Time
operator"" _s(unsigned long long val)
{
	return Time(static_cast<float>(val));
}
constexpr Unit<float, std::milli, TimeTag> operator"" _ms(unsigned long long val)
{
	return Unit<float, std::milli, TimeTag>(static_cast<float>(val));
}

// Current literals
constexpr Current
operator"" _A(long double val)
{
	return Current(static_cast<float>(val));
}
constexpr Unit<float, std::milli, CurrentTag> operator"" _mA(long double val)
{
	return Unit<float, std::milli, CurrentTag>(static_cast<float>(val));
}
constexpr Current
operator"" _A(unsigned long long val)
{
	return Current(static_cast<float>(val));
}
constexpr Unit<float, std::milli, CurrentTag> operator"" _mA(unsigned long long val)
{
	return Unit<float, std::milli, CurrentTag>(static_cast<float>(val));
}

// Voltage literals
constexpr Voltage
operator"" _V(long double val)
{
	return Voltage(static_cast<float>(val));
}
constexpr Unit<float, std::milli, VoltageTag> operator"" _mV(long double val)
{
	return Unit<float, std::milli, VoltageTag>(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, VoltageTag> operator"" _kV(long double val)
{
	return Unit<float, std::kilo, VoltageTag>(static_cast<float>(val));
}
constexpr Voltage
operator"" _V(unsigned long long val)
{
	return Voltage(static_cast<float>(val));
}
constexpr Unit<float, std::milli, VoltageTag> operator"" _mV(unsigned long long val)
{
	return Unit<float, std::milli, VoltageTag>(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, VoltageTag> operator"" _kV(unsigned long long val)
{
	return Unit<float, std::kilo, VoltageTag>(static_cast<float>(val));
}

// Frequency Literals
constexpr Frequency
operator"" _Hz(long double val)
{
	return Frequency(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, FrequencyTag> operator"" _kHz(long double val)
{
	return Unit<float, std::kilo, FrequencyTag>(static_cast<float>(val));
}
constexpr Frequency
operator"" _Hz(unsigned long long val)
{
	return Frequency(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, FrequencyTag> operator"" _kHz(unsigned long long val)
{
	return Unit<float, std::kilo, FrequencyTag>(static_cast<float>(val));
}

// Torque Literals
constexpr Torque
operator"" _Nm(long double val)
{
	return Torque(static_cast<float>(val));
}
constexpr Torque
operator"" _Nm(unsigned long long val)
{
	return Torque(static_cast<float>(val));
}

// Power Literals
constexpr Power
operator"" _W(long double val)
{
	return Power(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, PowerTag> operator"" _kW(long double val)
{
	return Unit<float, std::kilo, PowerTag>(static_cast<float>(val));
}
constexpr Power
operator"" _W(unsigned long long val)
{
	return Power(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, PowerTag> operator"" _kW(unsigned long long val)
{
	return Unit<float, std::kilo, PowerTag>(static_cast<float>(val));
}

// Magnetic Flux Literals
constexpr MagneticFlux
operator"" _Wb(long double val)
{
	return MagneticFlux(static_cast<float>(val));
}
constexpr Unit<float, std::milli, MagneticFluxTag> operator"" _mWb(long double val)
{
	return Unit<float, std::milli, MagneticFluxTag>(static_cast<float>(val));
}
constexpr MagneticFlux
operator"" _Wb(unsigned long long val)
{
	return MagneticFlux(static_cast<float>(val));
}
constexpr Unit<float, std::milli, MagneticFluxTag> operator"" _mWb(unsigned long long val)
{
	return Unit<float, std::milli, MagneticFluxTag>(static_cast<float>(val));
}

// Resistance Literals
constexpr Resistance
operator"" _Ohm(long double val)
{
	return Resistance(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, ResistanceTag> operator"" _kOhm(long double val)
{
	return Unit<float, std::kilo, ResistanceTag>(static_cast<float>(val));
}

constexpr Resistance
operator"" _Ohm(unsigned long long val)
{
	return Resistance(static_cast<float>(val));
}
constexpr Unit<float, std::kilo, ResistanceTag> operator"" _kOhm(unsigned long long val)
{
	return Unit<float, std::kilo, ResistanceTag>(static_cast<float>(val));
}

// Inductance Literals
constexpr Inductance
operator"" _H(long double val)
{
	return Inductance(static_cast<float>(val));
}
constexpr Unit<float, std::milli, InductanceTag> operator"" _mH(long double val)
{
	return Unit<float, std::milli, InductanceTag>(static_cast<float>(val));
}

constexpr Inductance
operator"" _H(unsigned long long val)
{
	return Inductance(static_cast<float>(val));
}
constexpr Unit<float, std::milli, InductanceTag> operator"" _mH(unsigned long long val)
{
	return Unit<float, std::milli, InductanceTag>(static_cast<float>(val));
}

// --- Type traits for unit tags ---
template<typename T>
struct is_angle : std::is_base_of<AngleTag, typename T::tag>
{};

template<typename T>
struct is_angular_velocity : std::is_base_of<AngularVelocityTag, typename T::tag>
{};

template<typename T>
struct is_torque : std::is_base_of<TorqueTag, typename T::tag>
{};

template<typename T>
struct is_angular_acceleration : std::is_base_of<AngularAccelerationTag, typename T::tag>
{};

template<typename T>
struct is_current : std::is_base_of<CurrentTag, typename T::tag>
{};

template<typename T>
struct is_voltage : std::is_base_of<VoltageTag, typename T::tag>
{};

template<typename T>
struct is_power : std::is_base_of<PowerTag, typename T::tag>
{};

template<typename T>
struct is_magnetic_flux : std::is_base_of<MagneticFluxTag, typename T::tag>
{};

template<typename T>
struct is_resistance : std::is_base_of<ResistanceTag, typename T::tag>
{};

template<typename T>
struct is_inductance : std::is_base_of<InductanceTag, typename T::tag>
{};

template<typename T>
struct is_time : std::is_base_of<TimeTag, typename T::tag>
{};

template<typename T>
struct is_frequency : std::is_base_of<FrequencyTag, typename T::tag>
{};

// Common type for two units with the same tag
template<typename T1, typename T2>
struct common_unit_type
{
	static_assert(std::is_same_v<typename T1::tag, typename T2::tag>,
				  "Units must have the same tag to find common type.");
	using type = Unit<
		typename std::common_type<typename T1::representation, typename T2::representation>::type,
		typename std::common_type<typename T1::period, typename T2::period>::type,
		typename T1::tag>;
};

template<typename T1, typename T2>
using common_unit_type_t = typename common_unit_type<T1, T2>::type;

// Common type for two units with different tags
template<typename T1, typename T2>
struct common_unit_type_different_tags
{
	using type = Unit<
		typename std::common_type<typename T1::representation, typename T2::representation>::type,
		std::ratio<1>,
		void>;  // Use void tag for mixed types
};

template<typename T1, typename T2>
using common_unit_type_different_tags_t = typename common_unit_type_different_tags<T1, T2>::type;

// Common type for two units, handling both same and different tags
template<typename T1, typename T2>
struct common_unit_type_mixed
{
	using type =
		std::conditional_t<std::is_same_v<typename T1::tag, typename T2::tag>,
						   common_unit_type_t<T1, T2>, common_unit_type_different_tags_t<T1, T2>>;
};

template<typename T1, typename T2>
using common_unit_type_mixed_t = typename common_unit_type_mixed<T1, T2>::type;

}  // namespace unit
}  // namespace unimoc

#endif /* UNIMOC_UNITLS_H_ */