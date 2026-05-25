/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

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

#ifndef UNIMOC_SYSTEM_ROTOR_REFERENCE_H_
#define UNIMOC_SYSTEM_ROTOR_REFERENCE_H_

#include <array>
#include "SinCos.hpp"

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
        // forward declaration of stationary system
        template <typename T>
        struct StatorReference;

        ///< rotating dq system.
        template <typename T = float>
        struct RotorReference
        {
            T d;
            T q;

            constexpr RotorReference() = default;
            constexpr RotorReference(T d_in, T q_in) : d(d_in), q(q_in) {}

            // copy constructor
            constexpr RotorReference(const RotorReference &other) : d(other.d), q(other.q) {}
            // move constructor
            constexpr RotorReference(RotorReference &&other) noexcept : d(other.d), q(other.q) {}
            // copy assignment operator
            constexpr RotorReference &operator=(const RotorReference &other)
            {
                if (this != &other)
                {
                    d = other.d;
                    q = other.q;
                }
                return *this;
            }

            // move assignment operator
            constexpr RotorReference &operator=(RotorReference &&other) noexcept
            {
                if (this != &other)
                {
                    d = other.d;
                    q = other.q;
                }
                return *this;
            }

            // equality operator
            constexpr bool operator==(const RotorReference &other) const
            {
                return (d == other.d && q == other.q);
            }

            // inequality operator
            constexpr bool operator!=(const RotorReference &other) const
            {
                return !(*this == other);
            }

            // addition operator
            constexpr RotorReference operator+(const RotorReference &other) const
            {
                return RotorReference(d + other.d, q + other.q);
            }

            // subtraction operator
            constexpr RotorReference operator-(const RotorReference &other) const
            {
                return RotorReference(d - other.d, q - other.q);
            }

            // multiplication operator
            constexpr RotorReference operator*(const RotorReference &other) const
            {
                return RotorReference(d * other.d - q * other.q, d * other.q + q * other.d);
            }

            // division operator
            constexpr RotorReference operator/(const RotorReference &other) const
            {
                return RotorReference((d * other.d + q * other.q) / (other.d * other.d + other.q * other.q),
                          (q * other.d - d * other.q) / (other.d * other.d + other.q * other.q));
            }

            // unary minus operator
            constexpr RotorReference operator-() const
            {
                return RotorReference(-d, -q);
            }

            // unary plus operator
            constexpr RotorReference operator+() const
            {
                return *this;
            }

            // scalar multiplication operator
            constexpr RotorReference operator*(const T &scalar) const
            {
                return RotorReference(d * scalar, q * scalar);
            }

            // scalar division operator
            constexpr RotorReference operator/(const T &scalar) const
            {
                return RotorReference(d / scalar, q / scalar);
            }


            // transform to array
            constexpr auto to_array() const noexcept -> std::array<T, 2>
            {
                return {d, q};
            }

            // length of the vector
            constexpr auto length() const noexcept -> T
            {
                return std::sqrt(d * d + q * q);
            }

            // transform dq vector to alpha beta vector.
            constexpr auto inverse_park(const SinCos<T> &angle) const noexcept -> StatorReference<T>
            {
                return StatorReference<T>(
                    d * angle.cos - q * angle.sin,
                    d * angle.sin + q * angle.cos
                );
            }
        };
    } // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_ROTOR_REFERENCE_H_ */