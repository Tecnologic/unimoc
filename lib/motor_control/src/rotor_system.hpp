/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file rotor_system.hpp
 *	@brief Rotating d/q reference frame and transformations.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>

#include "rotor_angle.hpp"
#include "stator_system.hpp"
#include "units.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system
{

/**
 * @brief Stores a unit-typed vector in the rotating d/q reference frame.
 *
 * The d axis is aligned with the rotor flux and the q axis is orthogonal to it. Both
 * components use the same unit type, which prevents the frame from being instantiated
 * with an untyped scalar or with mixed quantities.
 *
 * @tparam T UNIMOC unit type used for the d and q components.
 */
template <unimoc::unit::UnitLike T = unimoc::unit::Ratio>
struct Rotor
{
	/// Unit representation type used by each component.
	using Representation = T::representation;

	/// Direct-axis component.
	T d;
	/// Quadrature-axis component.
	T q;

	/** @brief Constructs a zero-valued rotor vector. */
	constexpr Rotor() = default;

	/**
	 * @brief Constructs a rotor vector from unit values.
	 * @param d_in Direct-axis component.
	 * @param q_in Quadrature-axis component.
	 */
	constexpr Rotor(T d_in, T q_in) : d(d_in), q(q_in) {}

	/**
	 * @brief Constructs a rotor vector from base representation values.
	 * @param d_in Direct-axis representation value.
	 * @param q_in Quadrature-axis representation value.
	 */
	constexpr Rotor(Representation d_in, Representation q_in)
		: d(T{d_in}), q(T{q_in})
	{}

	/**
	 * @brief Compares two rotor vectors for equality.
	 * @param other Vector to compare.
	 * @return `true` when both components are equal.
	 */
	constexpr bool
	operator==(const Rotor &other) const
	{
		return (d == other.d && q == other.q);
	}

	/**
	 * @brief Compares two rotor vectors for inequality.
	 * @param other Vector to compare.
	 * @return `true` when at least one component differs.
	 */
	constexpr bool
	operator!=(const Rotor &other) const
	{
		return !(*this == other);
	}

	/**
	 * @brief Adds two rotor vectors component-wise.
	 * @param other Vector to add.
	 * @return The component-wise sum.
	 */
	constexpr Rotor
	operator+(const Rotor &other) const
	{
		return Rotor(d + other.d, q + other.q);
	}

	/**
	 * @brief Subtracts two rotor vectors component-wise.
	 * @param other Vector to subtract.
	 * @return The component-wise difference.
	 */
	constexpr Rotor
	operator-(const Rotor &other) const
	{
		return Rotor(d - other.d, q - other.q);
	}

	/**
	 * @brief Returns this vector unchanged.
	 * @return A copy of this vector.
	 */
	constexpr Rotor
	operator+() const
	{
		return *this;
	}

	/**
	 * @brief Negates both components.
	 * @return The negated vector.
	 */
	constexpr Rotor
	operator-() const
	{
		return Rotor(-d, -q);
	}

	/**
	 * @brief Scales both components.
	 * @param scalar Numeric scale factor.
	 * @return The scaled vector.
	 */
	constexpr Rotor
	operator*(Representation scalar) const
	{
		return Rotor(d * scalar, q * scalar);
	}

	/**
	 * @brief Divides both components by a scalar.
	 * @param scalar Numeric divisor.
	 * @return The scaled vector.
	 */
	constexpr Rotor
	operator/(Representation scalar) const
	{
		return Rotor(d / scalar, q / scalar);
	}

	/**
	 * @brief Returns the d and q values in that order.
	 * @return An array containing d and q.
	 */
	[[nodiscard]] constexpr auto
	ToArray() const noexcept -> std::array<T, 2>
	{
		return {d, q};
	}

	/**
	 * @brief Returns the Euclidean length of the rotor vector.
	 * @return The vector length in the component unit.
	 */
	[[nodiscard]] constexpr T
	Length() const noexcept
	{
		return T{std::sqrt((d.Value() * d.Value()) + (q.Value() * q.Value()))};
	}

	/**
	 * @brief Returns the squared Euclidean length of the rotor vector.
	 * @return The squared length in the component representation type.
	 *
	 * @note Prefer this for limit and threshold-based derating comparisons. It
	 * avoids the multi-cycle floating-point square-root instruction; compare it
	 * with a limit squared. The result has squared component units, so it is
	 * returned as the representation rather than as `T`.
	 */
	[[nodiscard]] constexpr Representation
	LengthSquared() const noexcept
	{
		const Representation kDValue = d.Value();
		const Representation kQValue = q.Value();
		return (kDValue * kDValue) + (kQValue * kQValue);
	}

	/**
	 * @brief Applies the inverse Park transform to a rotor vector.
	 * @param angle Electrical rotor angle supplying sine and cosine.
	 * @return The vector in the stationary alpha/beta reference frame.
	 */
	[[nodiscard]] constexpr Stator<T>
	ToStator(const RotorAngle &angle) const noexcept
	{
		const Representation kSin = angle.Sin().Value();
		const Representation kCos = angle.Cos().Value();

		return Stator<T>(T{(d.Value() * kCos) - (q.Value() * kSin)},
				 T{(d.Value() * kSin) + (q.Value() * kCos)});
	}

	/**
	 * @brief Applies the inverse Park transform using a precomputed sine/cosine pair.
	 * @param sin_cos Precomputed sine and cosine of the electrical angle.
	 * @return The vector in the stationary alpha/beta reference frame.
	 */
	[[nodiscard]] constexpr Stator<T>
	ToStator(const SinCos<unit::Ratio> &sin_cos) const noexcept
	{
		const Representation kSin = sin_cos.sin.Value();
		const Representation kCos = sin_cos.cos.Value();

		return Stator<T>(T{(d.Value() * kCos) - (q.Value() * kSin)},
					 T{(d.Value() * kSin) + (q.Value() * kCos)});
	}
};

}  // namespace unimoc::system
