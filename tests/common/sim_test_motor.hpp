/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_test_motor.hpp
 *    @brief The test motor of the sim suites that start from the factory profile.
 *
 *    The factory profile describes the default motor (a hoverboard hub motor,
 *    nvm_settings.hpp). The suites that use the platform profile were written
 *    for a smaller motor without a permanent magnet; this profile keeps the
 *    platform values and replaces the motor, so the suites do not follow a
 *    change of the default motor.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "settings_profile.hpp"

namespace unimoc::test {

/**
 * @brief The platform profile with the test motor.
 *
 * One pole pair, 0.1 Ohm, 1 mH, no permanent magnet, 1e-4 kg m^2, +-2000 rad/s
 * electrical, the motor current at the capability of the power stage, unit
 * current gains and a soft speed loop.
 *
 * @param platform The profile of the platform (`kSettings.GetSettingsProfile()`).
 * @return The profile, built once on the first call.
 */
inline const settings::SettingsProfile& SimTestMotorProfile(const settings::SettingsProfile& platform) {
  using namespace unit;
  static const settings::SettingsProfile kProfile = [&platform] {
    settings::SettingsProfile profile = platform;
    settings::NvmSettings& motor = profile.factory_settings;
    motor.pole_pairs = 1U;
    motor.stator_r = 0.1_Ohm;
    motor.stator_l = 1.0_mH;
    motor.flux_pm = 0.0_Wb;
    motor.l_d = 1.0_mH;
    motor.l_q = 1.0_mH;
    motor.motor_j = 1.0e-4_kg_m2;
    motor.motor_i_max = profile.capabilities.max_motor_current;
    motor.motor_omega_max = 2000.0_rad_per_s;
    motor.motor_omega_min = -2000.0_rad_per_s;
    motor.current_kp_d = 1.0_V_per_A;
    motor.current_ki_d = 100.0_V_per_A_s;
    motor.current_kp_q = 1.0_V_per_A;
    motor.current_ki_q = 100.0_V_per_A_s;
    motor.speed_kp = 0.01_Nm_s_per_rad;
    motor.speed_ki = 10.0_per_s;
    return profile;
  }();
  return kProfile;
}

}  // namespace unimoc::test
