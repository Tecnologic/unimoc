/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file nvm_settings_test.cpp
 *    @brief Test entry point for typed persistent settings.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "nvm_settings.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

namespace unimoc::settings::test {

using namespace unit;

// --- Default node_id == 0 triggers PnP
ZTEST(nvm_settings, test_default_node_id_zero) {
  NvmSettings settings;
  zexpect_equal(settings.node_id, 0U);
}

// --- Default motor type is PMSM
ZTEST(nvm_settings, test_default_motor_type) {
  NvmSettings settings;
  zexpect_equal(settings.motor_type, system::MotorType::kPmsm);
}

// --- Default control mode is TORQUE
ZTEST(nvm_settings, test_default_control_mode) {
  NvmSettings settings;
  zexpect_equal(settings.control_mode, cyphal::ControlMode::kTorque);
}

// --- Identity defaults are preserved
// --- reset_to_defaults restores the defaults
ZTEST(nvm_settings, test_reset_to_defaults_restores_node_id) {
  NvmSettings settings;
  settings.node_id = 42U;
  settings.ResetToDefaults();
  zexpect_equal(settings.node_id, NvmSettings{}.node_id);
}

ZTEST(nvm_settings, test_identity_default_name) {
  NvmSettings settings;
  zexpect_equal(settings.identity.GetName(), std::string_view("unimoc"));
}

// --- Node ID can be updated to valid range
ZTEST(nvm_settings, test_node_id_can_be_set) {
  NvmSettings settings;
  settings.node_id = 42U;
  zexpect_equal(settings.node_id, 42U);
}

// --- Parameter spot-checks (verify fields exist + have sensible defaults)
ZTEST(nvm_settings, test_default_stator_r) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.stator_r, 0.15_Ohm);
}

// --- The default motor is a 6.5" hoverboard hub motor: 15 pole pairs, 0.52 N m per A
ZTEST(nvm_settings, test_default_motor_is_hoverboard) {
  NvmSettings settings;
  zexpect_equal(settings.pole_pairs, 15U);
  const auto kPolePairs = static_cast<float>(settings.pole_pairs);
  UNIMOC_EXPECT_NEAR(settings.flux_pm * (1.5F * kPolePairs) * 1.0_A, 0.52_Nm, 0.01_Nm);  // torque at 1 A
  UNIMOC_EXPECT_FLOAT_EQ(settings.l_d, settings.l_q);
}

// --- The current PI cancels the stator pole: kp / ki = L / R
ZTEST(nvm_settings, test_default_current_gains_cancel_stator_pole) {
  NvmSettings settings;
  UNIMOC_EXPECT_NEAR(settings.current_kp_q / settings.current_ki_q, settings.l_q / settings.stator_r, 1e-6_s);
  UNIMOC_EXPECT_FLOAT_EQ(settings.current_kp_d, settings.current_kp_q);
  UNIMOC_EXPECT_FLOAT_EQ(settings.current_ki_d, settings.current_ki_q);
}

ZTEST(nvm_settings, test_default_fw_v_max) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.fw_v_max, 0.9_V);
}

ZTEST(nvm_settings, test_default_svm_duty_range) {
  NvmSettings settings;
  zexpect_true((settings.svm_duty_min) < (settings.svm_duty_max));
}

ZTEST(nvm_settings, test_default_pos_speed_limit) {
  NvmSettings settings;
  zexpect_true(settings.pos_speed_limit > AngularVelocity{});
}

ZTEST(nvm_settings, test_default_excitation_mode) {
  NvmSettings settings;
  zexpect_equal(settings.excitation_mode, 0U);  // CurrentMode
}

// --- Phase current balance defaults ---

ZTEST(nvm_settings, test_default_phase_balance_a) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_a, 1.0_ratio);
}

ZTEST(nvm_settings, test_default_phase_balance_b) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_b, 1.0_ratio);
}

ZTEST(nvm_settings, test_default_phase_balance_c) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_c, 1.0_ratio);
}

ZTEST(nvm_settings, test_phase_balance_can_be_modified) {
  NvmSettings settings;
  settings.phase_balance_a = 1.02_ratio;
  settings.phase_balance_b = 0.98_ratio;
  settings.phase_balance_c = 1.00_ratio;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_a, 1.02_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_b, 0.98_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_c, 1.00_ratio);
}

ZTEST(nvm_settings, test_reset_restores_phase_balance_defaults) {
  NvmSettings settings;
  settings.phase_balance_a = 1.05_ratio;
  settings.phase_balance_b = 0.95_ratio;
  settings.phase_balance_c = 0.99_ratio;
  settings.ResetToDefaults();
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_a, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_b, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_c, 1.0_ratio);
}

}  // namespace unimoc::settings::test

ZTEST_SUITE(nvm_settings, nullptr, nullptr, nullptr, nullptr, nullptr);
