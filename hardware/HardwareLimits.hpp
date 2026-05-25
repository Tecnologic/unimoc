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

#ifndef UNIMOC_HARDWARE_LIMITS_H_
#define UNIMOC_HARDWARE_LIMITS_H_

/**
 * @file HardwareLimits.hpp
 * @brief Compile-time hardware electrical safety limits.
 *
 * Each hardware target must define the following macros (typically via a
 * target-specific CMakeLists.txt `target_compile_definitions`) before any
 * translation unit that includes this header is compiled:
 *
 *   HARDWARE_MAX_PHASE_CURRENT_A
 *       Absolute maximum phase current the power-stage inverter can carry [A].
 *       Exceeding this will destroy the FETs.
 *
 *   HARDWARE_MAX_MOTOR_CURRENT_A
 *       Absolute maximum resultant stator current vector magnitude [A].
 *       Typically set to the motor's peak rating or the drive's continuous
 *       rating, whichever is lower.  Must be <= HARDWARE_MAX_PHASE_CURRENT_A.
 *
 *   HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A
 *       Absolute maximum battery discharge current [A].
 *       Set by the battery / wiring / fuse rating for the specific build.
 *
 *   HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A
 *       Absolute maximum battery charge (regenerative braking) current [A].
 *       Set to 0 for systems that do not support regenerative braking.
 *
 * If a target does not define a macro, a conservative default is used and a
 * compile-time diagnostic is emitted so the omission is visible.
 *
 * Runtime-configurable limits (from NvmSettings) must never exceed these
 * compile-time values.  Use hardware::Limits::clamp() to enforce this.
 */

#ifndef HARDWARE_MAX_PHASE_CURRENT_A
#  pragma message("HARDWARE_MAX_PHASE_CURRENT_A not defined — using default 100 A")
#  define HARDWARE_MAX_PHASE_CURRENT_A 100.0f
#endif

#ifndef HARDWARE_MAX_MOTOR_CURRENT_A
#  pragma message("HARDWARE_MAX_MOTOR_CURRENT_A not defined — using default 40 A")
#  define HARDWARE_MAX_MOTOR_CURRENT_A 40.0f
#endif

#ifndef HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A
#  pragma message("HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A not defined — using default 15 A")
#  define HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A 15.0f
#endif

#ifndef HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A
#  pragma message("HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A not defined — using default 5 A")
#  define HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A 5.0f
#endif

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace hardware hardware namespace
 */
namespace hardware
{

/**
 * @brief Compile-time hardware electrical safety limits.
 *
 * All constants are set by the hardware target at compile time via preprocessor
 * macros.  They represent absolute maximums that must never be exceeded to
 * avoid hardware damage.  Runtime-configurable operating limits (stored in
 * NvmSettings) must always be at or below these values.
 *
 * ### Usage in firmware
 * @code
 * #include "HardwareLimits.hpp"
 *
 * // Clamp a runtime limit before use
 * float safe_i = hardware::Limits::clamp_motor_current(nvm.motor_i_max);
 * @endcode
 */
struct Limits
{
    // =========================================================================
    // Compile-time constants
    // =========================================================================

    /// Absolute maximum phase current the inverter power stage can handle [A].
    static constexpr float max_phase_current_A
        = HARDWARE_MAX_PHASE_CURRENT_A;

    /// Absolute maximum motor (resultant stator vector) current [A].
    static constexpr float max_motor_current_A
        = HARDWARE_MAX_MOTOR_CURRENT_A;

    /// Absolute maximum battery discharge (drive) current [A].
    static constexpr float max_battery_drive_current_A
        = HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A;

    /// Absolute maximum battery charge (regenerative) current [A].
    static constexpr float max_battery_charge_current_A
        = HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A;

    // =========================================================================
    // Compile-time sanity checks
    // =========================================================================

    static_assert(max_phase_current_A > 0.0f,
        "HARDWARE_MAX_PHASE_CURRENT_A must be > 0");
    static_assert(max_motor_current_A > 0.0f,
        "HARDWARE_MAX_MOTOR_CURRENT_A must be > 0");
    static_assert(max_motor_current_A <= max_phase_current_A,
        "HARDWARE_MAX_MOTOR_CURRENT_A must not exceed HARDWARE_MAX_PHASE_CURRENT_A");
    static_assert(max_battery_drive_current_A > 0.0f,
        "HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A must be > 0");
    static_assert(max_battery_charge_current_A >= 0.0f,
        "HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A must be >= 0");

    // =========================================================================
    // Runtime clamping helpers
    // =========================================================================

    /**
     * @brief Clamp a requested motor current to the hardware maximum.
     *
     * @param requested  Requested motor current limit [A].
     * @return           Value in [0, max_motor_current_A].
     */
    [[nodiscard]] static constexpr float
    clamp_motor_current(const float requested) noexcept
    {
        if (requested < 0.0f)
            return 0.0f;
        if (requested > max_motor_current_A)
            return max_motor_current_A;
        return requested;
    }

    /**
     * @brief Clamp a requested phase current to the hardware maximum.
     *
     * @param requested  Requested phase current limit [A].
     * @return           Value in [0, max_phase_current_A].
     */
    [[nodiscard]] static constexpr float
    clamp_phase_current(const float requested) noexcept
    {
        if (requested < 0.0f)
            return 0.0f;
        if (requested > max_phase_current_A)
            return max_phase_current_A;
        return requested;
    }

    /**
     * @brief Clamp a requested battery discharge current to the hardware maximum.
     *
     * @param requested  Requested battery drive current limit [A].
     * @return           Value in [0, max_battery_drive_current_A].
     */
    [[nodiscard]] static constexpr float
    clamp_battery_drive_current(const float requested) noexcept
    {
        if (requested < 0.0f)
            return 0.0f;
        if (requested > max_battery_drive_current_A)
            return max_battery_drive_current_A;
        return requested;
    }

    /**
     * @brief Clamp a requested battery charge current to the hardware maximum.
     *
     * @param requested  Requested battery charge current limit [A].
     * @return           Value in [0, max_battery_charge_current_A].
     */
    [[nodiscard]] static constexpr float
    clamp_battery_charge_current(const float requested) noexcept
    {
        if (requested < 0.0f)
            return 0.0f;
        if (requested > max_battery_charge_current_A)
            return max_battery_charge_current_A;
        return requested;
    }
};

}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_LIMITS_H_ */
