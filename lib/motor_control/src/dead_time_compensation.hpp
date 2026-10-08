/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file dead_time_compensation.hpp
 *    @brief Unit-typed inverter dead-time compensation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cmath>
#include <concepts>
#include "nvm_settings.hpp"
#include "stator_system.hpp"
#include "three_phase_system.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace control control algorithms namespace
 */
namespace control
{

/**
 * @brief Dead-time compensation for a three-phase voltage-source inverter.
 *
 * Gate-driver dead time prevents both switches of a half-bridge from
 * conducting simultaneously, but introduces an unwanted voltage error that
 * depends on the sign of the phase current.  For each phase the error is:
 *
 *   ΔV_phase = sign(i_phase) · V_dc · t_dead · f_pwm
 *
 * This class computes the compensation voltage in the stationary α/β frame
 * so that it can be added to the voltage reference before it is passed to
 * the SVM modulator.
 *
 * A configurable threshold @p i_threshold is used around zero current to
 * apply a soft sign function, smoothing out chattering at phase-current
 * zero crossings.
 *
 * @tparam T  Floating-point type (float by default).
 */
template <std::floating_point T = float>
struct DeadTimeCompensation
{
    /// Dead time of the gate driver [s].
    unit::Time dead_time{};

    /// PWM switching frequency [Hz].
    unit::Frequency f_pwm{};

    /**
     * @brief Threshold current [A] for the soft sign function.
     *
     * Currents with magnitude below this value produce a linearly-interpolated
     * sign rather than a hard ±1, which avoids large voltage spikes and chattering
     * near zero-crossings.
     */
    unit::Current i_threshold{};

    /**
     * @brief Load dead-time compensation parameters from NVM settings.
     *
     * @param settings Validated NVM settings.
     */
    constexpr void
    init(const settings::NvmSettings& settings) noexcept
    {
        dead_time   = settings.dtc_dead_time;
        f_pwm       = settings.dtc_f_pwm;
        i_threshold = settings.dtc_i_threshold;
    }

    /**
     * @brief Compute the α/β compensation voltage to add to the modulator input.
     *
     * The stationary-frame currents (i_alpha, i_beta) are used to reconstruct
     * the three phase currents via the inverse Clarke transform, the dead-time
     * voltage error is estimated for each phase, and the result is transformed
     * back to α/β via the (amplitude-invariant) Clarke transform.
     *
     * @param i_ab  Measured stator current in the stationary α/β frame [A].
     * @return      Compensation voltage vector (normalised by V_dc) to add to
     *              the α/β voltage reference.
     */
    [[nodiscard]] constexpr system::Stator<unit::Ratio>
    calculate(const system::Stator<unit::Current>& i_ab) const noexcept
    {
        // --- Reconstruct three-phase currents from α/β ---
        constexpr T k = static_cast<T>(0.8660254037844386);  // √3 / 2

        const auto phase_currents = i_ab.ToThreePhase();
        const T ia = phase_currents.a.Value();
        const T ib = phase_currents.b.Value();
        const T ic = phase_currents.c.Value();

        // --- Soft sign function: clamp(i / threshold, -1, +1) ---
        // This provides linear interpolation through zero, preventing chattering.
        const T threshold = i_threshold.Value();
        T sign_a = unit::Ratio{ia / threshold}.Clamp(static_cast<T>(-1), static_cast<T>(1)).Value();
        T sign_b = unit::Ratio{ib / threshold}.Clamp(static_cast<T>(-1), static_cast<T>(1)).Value();
        T sign_c = unit::Ratio{ic / threshold}.Clamp(static_cast<T>(-1), static_cast<T>(1)).Value();

        // Normalised per-phase voltage error: sign · t_dead · f_pwm
        T dt_norm = dead_time.Value() * f_pwm.Value();
        T dva     = sign_a * dt_norm;
        T dvb     = sign_b * dt_norm;
        T dvc     = sign_c * dt_norm;

        // --- Clarke transform (amplitude-invariant) back to α/β ---
        constexpr T two_thirds = static_cast<T>(2.0 / 3.0);

        T d_alpha = two_thirds * (dva - static_cast<T>(0.5) * dvb - static_cast<T>(0.5) * dvc);
        T d_beta  = two_thirds * (k * dvb - k * dvc);

        return system::Stator<unit::Ratio>{d_alpha, d_beta};
    }
};

}  // namespace control
}  // namespace unimoc

