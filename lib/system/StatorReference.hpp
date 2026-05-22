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

#ifndef UNIMOC_SYSTEM_STATOR_REFERENCE_H_
#define UNIMOC_SYSTEM_STATOR_REFERENCE_H_

#include <array>
#include <cmath>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
    /**
     * @namespace coordinate systems.
     */
    namespace system
    {
        // forward declaration of rotating system
        template <typename T>
        struct RotorReference;
        // forward declaration of sin_cos
        template <typename T>
        struct sin_cos;

        ///< rotating dq system.
        template <typename T = float>
        struct StatorReference
        {
            T alpha;
            T beta;

            constexpr StatorReference() = default;
            constexpr StatorReference(T alpha, T beta) : alpha(alpha), beta(beta) {}

            // copy constructor
            constexpr StatorReference(const StatorReference &other) : alpha(other.alpha), beta(other.beta) {}
            // move constructor
            constexpr StatorReference(StatorReference &&other) noexcept : alpha(other.alpha), beta(other.beta) {}
            // copy assignment operator
            constexpr StatorReference &operator=(const StatorReference &other)
            {
                if (this != &other)
                {
                    alpha = other.alpha;
                    beta = other.beta;
                }
                return *this;
            }

            // move assignment operator
            constexpr StatorReference &operator=(StatorReference &&other) noexcept
            {
                if (this != &other)
                {
                    alpha = other.alpha;
                    beta = other.beta;
                }
                return *this;
            }

            // equality operator
            constexpr bool operator==(const StatorReference &other) const
            {
                return (alpha == other.alpha && beta == other.beta);
            }

            // inequality operator
            constexpr bool operator!=(const StatorReference &other) const
            {
                return !(*this == other);
            }

            // addition operator
            constexpr StatorReference operator+(const StatorReference &other) const
            {
                return StatorReference(alpha + other.alpha, beta + other.beta);
            }

            // subtraction operator
            constexpr StatorReference operator-(const StatorReference &other) const
            {
                return StatorReference(alpha - other.alpha, beta - other.beta);
            }

            // multiplication operator
            constexpr RotorReference<T> operator*(const RotorReference<T> &other) const
            {
                return RotorReference<T>(alpha * other.d - beta * other.q, alpha * other.q + beta * other.d);
            }

            // division operator
            constexpr StatorReference operator/(const StatorReference &other) const
            {
                return StatorReference(alpha / other.alpha, beta / other.beta);
            }

            // transform to array
            constexpr auto to_array() const noexcept -> std::array<T, 2>
            {
                return {alpha, beta};
            }

            // length of the vector
            constexpr auto length() const noexcept -> T
            {
                return std::sqrt(alpha * alpha + beta * beta);
            }

            // transform alpha beta vector to dq vector.
            constexpr RotorReference<T> park(const sin_cos<T> &angle) const noexcept
            {
                return RotorReference<T>{
                    .d = alpha * angle.cos + beta * angle.sin,
                    .q = -alpha * angle.sin + beta * angle.cos,
                };
            }

        };
    } // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_STATOR_REFERENCE_H_ */