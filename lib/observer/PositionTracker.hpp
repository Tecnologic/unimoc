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

#ifndef UNIMOC_OBSERVER_POSITION_TRACKER_H_
#define UNIMOC_OBSERVER_POSITION_TRACKER_H_

#include <cmath>
#include <concepts>
#include <cstdint>
#include <numbers>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace observer observer algorithms namespace
 */
namespace observer
{

/**
 * @brief Absolute multi-turn position tracker.
 *
 * Overview
 * --------
 * The MechanicalObserver provides only a wrapped electrical angle
 * θ ∈ (−π, π].  PositionTracker consumes that wrapped angle every cycle,
 * detects 2π crossings to maintain a turn counter, and computes an
 * unwrapped absolute mechanical shaft position supporting ±4096 mechanical
 * revolutions (the int32_t turn counter can handle far more).
 *
 * Coordinate conventions
 * ----------------------
 *   theta_electrical  — electrical angle in radians, wrapped to (−π, π].
 *   pole_pairs        — number of electrical cycles per mechanical revolution.
 *   position_rad      — absolute mechanical shaft angle [rad], unwrapped,
 *                       referenced to the home position (0 after set_home()).
 *   position_rev      — same, expressed in mechanical revolutions.
 *
 * Absolute position formula
 * -------------------------
 *   raw_rad  = (turns * 2π + theta_electrical) / pole_pairs
 *   position_rad = raw_rad − home_offset_rad
 *
 * Homing
 * ------
 * Call set_home() to latch the current raw position as the application zero.
 * Afterwards position_rad == 0 at that shaft location.  The is_homed flag is
 * set and remains set until reset() is called.
 *
 * Cyphal interface
 * ----------------
 * A homing-trigger Cyphal service call should invoke set_home() (or set
 * home_offset_rad directly).  The is_homed flag and position_rad / position_rev
 * are published as Cyphal subjects at a reduced rate by the application layer.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct PositionTracker
{
    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /// Signed turn counter (electrical turns).
    /// int32_t supports ±2 147 483 648 electrical turns — far beyond any
    /// mechanical requirement; at 7 pole-pairs ±4096 mechanical revolutions
    /// requires only ±28 672 electrical turns.
    int32_t turns{0};

    /// Previous electrical angle [rad], used for wrap detection.
    T theta_prev{static_cast<T>(0)};

    // -------------------------------------------------------------------------
    // Homing
    // -------------------------------------------------------------------------

    /// Absolute raw position captured at the last set_home() call [rad].
    T home_offset_rad{static_cast<T>(0)};

    /// True once set_home() has been called at least once since the last
    /// reset().  Position values are only meaningful when this is true.
    bool is_homed{false};

    // -------------------------------------------------------------------------
    // Outputs (updated by update())
    // -------------------------------------------------------------------------

    /// Absolute mechanical shaft position referenced to home [rad].
    /// Positive direction is the positive electrical rotation direction.
    T position_rad{static_cast<T>(0)};

    /// Absolute mechanical shaft position referenced to home [revolutions].
    T position_rev{static_cast<T>(0)};

    /**
     * @brief Update the position tracker.
     *
     * Call once per control cycle, passing the latest wrapped electrical angle
     * from MechanicalObserver::theta.
     *
     * @param theta_electrical  Wrapped electrical angle [rad] ∈ (−π, π].
     * @param pole_pairs        Motor pole-pair count (positive integer).
     */
    constexpr void
    update(const T theta_electrical, const int pole_pairs) noexcept
    {
        constexpr T pi     = std::numbers::pi_v<T>;
        constexpr T two_pi = static_cast<T>(2) * pi;

        // Detect wrap crossings: a jump larger than π means the angle wrapped.
        const T delta = theta_electrical - theta_prev;

        if (delta > pi)
        {
            // Wrapped from +π to −π: shaft moved in the negative direction
            --turns;
        }
        else if (delta < -pi)
        {
            // Wrapped from −π to +π: shaft moved in the positive direction
            ++turns;
        }

        theta_prev = theta_electrical;

        // Compute raw absolute mechanical position
        const T raw_rad = (static_cast<T>(turns) * two_pi + theta_electrical)
                          / static_cast<T>(pole_pairs);

        position_rad = raw_rad - home_offset_rad;
        position_rev = position_rad / two_pi;
    }

    /**
     * @brief Capture the current position as the application zero (home).
     *
     * After this call position_rad == 0 at the current shaft location.
     * Sets is_homed = true.
     *
     * @param pole_pairs  Motor pole-pair count (must match the value used in
     *                    update()).
     */
    constexpr void
    set_home(const int pole_pairs) noexcept
    {
        constexpr T two_pi = static_cast<T>(2) * std::numbers::pi_v<T>;

        home_offset_rad = (static_cast<T>(turns) * two_pi + theta_prev)
                          / static_cast<T>(pole_pairs);
        position_rad = static_cast<T>(0);
        position_rev = static_cast<T>(0);
        is_homed     = true;
    }

    /**
     * @brief Reset all state.
     *
     * Clears the turn counter, home offset, and homed flag.  Call on drive
     * enable or after a fault that may have caused position loss.
     */
    constexpr void
    reset() noexcept
    {
        turns           = 0;
        theta_prev      = static_cast<T>(0);
        home_offset_rad = static_cast<T>(0);
        is_homed        = false;
        position_rad    = static_cast<T>(0);
        position_rev    = static_cast<T>(0);
    }
};

}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_OBSERVER_POSITION_TRACKER_H_ */
