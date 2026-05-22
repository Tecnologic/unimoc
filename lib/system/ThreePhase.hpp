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

#ifndef UNIMOC_SYSTEM_THREE_PHASE_H_
#define UNIMOC_SYSTEM_THREE_PHASE_H_

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
        // forward declaration of stationary system
        template <typename T>
        struct StatorReference;
        // forward declaration of sin_cos
        template <typename T>
        struct sin_cos;

        ///< rotating dq system.
        template <typename T = float>
        struct ThreePhase
        {
            T a;
            T b;
            T c;
            constexpr ThreePhase() = default;
            constexpr ThreePhase(T a, T b, T c) : a(a), b(b), c(c) {}

            // copy constructor
            constexpr ThreePhase(const ThreePhase &other) : a(other.a), b(other.b), c(other.c) {}
            // move constructor
            constexpr ThreePhase(ThreePhase &&other) noexcept : a(other.a), b(other.b), c(other.c) {}
            // copy assignment operator
            constexpr ThreePhase &operator=(const ThreePhase &other)
            {
                if (this != &other)
                {
                    a = other.a;
                    b = other.b;
                    c = other.c;
                }
                return *this;
            }

            // move assignment operator
            constexpr ThreePhase &operator=(ThreePhase &&other) noexcept
            {
                if (this != &other)
                {
                    a = other.a;
                    b = other.b;
                    c = other.c;
                }
                return *this;
            }
            // equality operator
            constexpr bool operator==(const ThreePhase &other) const
            {
                return (a == other.a && b == other.b && c == other.c);
            }
            // inequality operator
            constexpr bool operator!=(const ThreePhase &other) const
            {
                return !(*this == other);
            }
            // addition operator
            constexpr ThreePhase operator+(const ThreePhase &other) const
            {
                return ThreePhase(a + other.a, b + other.b, c + other.c);
            }
            // subtraction operator
            constexpr ThreePhase operator-(const ThreePhase &other) const
            {
                return ThreePhase(a - other.a, b - other.b, c - other.c);
            }

            // transform to array
            constexpr auto to_array() const noexcept -> std::array<T, 3>
            {
                return {a, b, c};
            }


            // clarke transformation
            // transform abc 3 phase vector to alpha beta vector.
            constexpr StatorReference<T> clark() const noexcept
            {
                constexpr T sqrt3by2 = std::sqrt(3.0f) / 2.0f;
                constexpr T _2by3 = 2.0f / 3.0f;

                return StatorReference<T>{
                    .alpha = _2by3 * (a - (0.5f * b) - (0.5f * c)),
                    .beta = _2by3 * ((sqrt3by2 * b) - (sqrt3by2 * c)),
                };
            }
        };

    } // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_THREE_PHASE_H_ */