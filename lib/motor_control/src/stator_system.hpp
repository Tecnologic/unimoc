/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file stator_system.hpp
 *	@brief Stationary alpha/beta reference frame and transformations.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>

#include "rotor_angle.hpp"
#include "units.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system
{

/** Forward declaration of the rotating reference frame. */
template <unimoc::unit::UnitLike T>
struct Rotor;

/** Forward declaration of the three-phase reference frame. */
template <unimoc::unit::UnitLike T>
struct ThreePhase;

/**
 * @brief Stores a unit-typed vector in the stationary alpha/beta reference frame.
 *
 * The alpha axis is aligned with the stator reference axis and the beta axis is
 * orthogonal to it. Both components use the same unit type, which prevents the frame
 * from being instantiated with an untyped scalar or with mixed quantities.
 *
 * @tparam T UNIMOC unit type used for the alpha and beta components.
 */
template <unimoc::unit::UnitLike T = unimoc::unit::Ratio>
struct Stator
{
	/// Unit representation type used by each component.
	using Representation = T::representation;

	/// Alpha-axis component.
	T alpha;
	/// Beta-axis component.
	T beta;

	/** @brief Constructs a zero-valued stator vector. */
	constexpr Stator() = default;

	/**
	 * @brief Constructs a stator vector from unit values.
	 * @param alpha_in Alpha-axis component.
	 * @param beta_in Beta-axis component.
	 */
	constexpr Stator(T alpha_in, T beta_in) : alpha(alpha_in), beta(beta_in) {}

	/**
	 * @brief Constructs a stator vector from base representation values.
	 * @param alpha_in Alpha-axis representation value.
	 * @param beta_in Beta-axis representation value.
	 */
	constexpr Stator(Representation alpha_in, Representation beta_in)
		: alpha(T{alpha_in}), beta(T{beta_in})
	{}

	/**
	 * @brief Compares two stator vectors for equality.
	 * @param other Vector to compare.
	 * @return `true` when both components are equal.
	 */
	constexpr bool
	operator==(const Stator &other) const
	{
		return (alpha == other.alpha && beta == other.beta);
	}

	/**
	 * @brief Compares two stator vectors for inequality.
	 * @param other Vector to compare.
	 * @return `true` when at least one component differs.
	 */
	constexpr bool
	operator!=(const Stator &other) const
	{
		return !(*this == other);
	}

	/**
	 * @brief Adds two stator vectors component-wise.
	 * @param other Vector to add.
	 * @return The component-wise sum.
	 */
	constexpr Stator
	operator+(const Stator &other) const
	{
		return Stator(alpha + other.alpha, beta + other.beta);
	}

	/**
	 * @brief Subtracts two stator vectors component-wise.
	 * @param other Vector to subtract.
	 * @return The component-wise difference.
	 */
	constexpr Stator
	operator-(const Stator &other) const
	{
		return Stator(alpha - other.alpha, beta - other.beta);
	}

	/**
	 * @brief Returns this vector unchanged.
	 * @return A copy of this vector.
	 */
	constexpr Stator
	operator+() const
	{
		return *this;
	}

	/**
	 * @brief Negates both components.
	 * @return The negated vector.
	 */
	constexpr Stator
	operator-() const
	{
		return Stator(-alpha, -beta);
	}

	/**
	 * @brief Scales both components.
	 * @param scalar Numeric scale factor.
	 * @return The scaled vector.
	 */
	constexpr Stator
	operator*(Representation scalar) const
	{
		return Stator(alpha * scalar, beta * scalar);
	}

	/**
	 * @brief Divides both components by a scalar.
	 * @param scalar Numeric divisor.
	 * @return The scaled vector.
	 */
	constexpr Stator
	operator/(Representation scalar) const
	{
		return Stator(alpha / scalar, beta / scalar);
	}

	/**
	 * @brief Returns the alpha and beta values in that order.
	 * @return An array containing alpha and beta.
	 */
	[[nodiscard]] constexpr auto
	ToArray() const noexcept -> std::array<T, 2>
	{
		return {alpha, beta};
	}

	/**
	 * @brief Applies the inverse Clarke transform to a stator vector.
	 * @return The corresponding phase values in A, B, C order.
	 *
	 * The transform is amplitude-invariant and preserves the unit type of the
	 * stator components. A zero-sequence component is not added.
	 */
	[[nodiscard]] constexpr ThreePhase<T>
	ToThreePhase() const noexcept;

	/**
	 * @brief Returns the Euclidean length of the stator vector.
	 * @return The vector length in the component unit.
	 */
	[[nodiscard]] constexpr T
	Length() const noexcept
	{
		return T{std::sqrt((alpha.Value() * alpha.Value()) + (beta.Value() * beta.Value()))};
	}

	/**
	 * @brief Returns the squared Euclidean length of the stator vector.
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
		const Representation kAlphaValue = alpha.Value();
		const Representation kBetaValue = beta.Value();
		return (kAlphaValue * kAlphaValue) + (kBetaValue * kBetaValue);
	}

	/**
	 * @brief Applies the Park transform to a stator vector.
	 * @param angle Electrical rotor angle supplying sine and cosine.
	 * @return The vector in the rotating d/q reference frame.
	 */
	[[nodiscard]] constexpr Rotor<T>
	ToRotor(const RotorAngle &angle) const noexcept
	{
		const Representation kSin = angle.Sin().Value();
		const Representation kCos = angle.Cos().Value();

		return Rotor<T>(T{(alpha.Value() * kCos) + (beta.Value() * kSin)},
					 T{(beta.Value() * kCos) - (alpha.Value() * kSin)});
	}

	/**
	 * @brief Applies the Park transform using a precomputed sine/cosine pair.
	 * @param sin_cos Precomputed sine and cosine of the electrical angle.
	 * @return The vector in the rotating d/q reference frame.
	 */
	[[nodiscard]] constexpr Rotor<T>
	ToRotor(const SinCos<unit::Ratio> &sin_cos) const noexcept
	{
		const Representation kSin = sin_cos.sin.Value();
		const Representation kCos = sin_cos.cos.Value();

		return Rotor<T>(T{(alpha.Value() * kCos) + (beta.Value() * kSin)},
					 T{(beta.Value() * kCos) - (alpha.Value() * kSin)});
	}

};

}  // namespace unimoc::system
