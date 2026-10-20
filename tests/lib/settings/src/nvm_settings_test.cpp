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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "nvm_settings.hpp"
#include <zephyr/ztest.h>
#include "unimoc_test.hpp"

namespace unimoc::settings::test {

using namespace unit;

// --- Default construction is valid
ZTEST(nvm_settings, test_default_is_valid) {
  NvmSettings settings;
  zexpect_true(settings.IsValid());
}

ZTEST(nvm_settings, test_default_magic_and_version) {
  NvmSettings settings;
  zexpect_equal(settings.magic, kNvmMagic);
  zexpect_equal(settings.version, kNvmVersion);
}

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

// --- Corrupt magic invalidates the block
ZTEST(nvm_settings, test_corrupt_magic_invalid) {
  NvmSettings settings;
  settings.magic = 0xDEADBEEFU;
  zexpect_false(settings.IsValid());
}

// --- Corrupt version invalidates the block
ZTEST(nvm_settings, test_corrupt_version_invalid) {
  NvmSettings settings;
  settings.version = 0xFFFFU;
  zexpect_false(settings.IsValid());
}

// --- reset_to_defaults restores valid state
ZTEST(nvm_settings, test_reset_to_defaults_is_valid) {
  NvmSettings settings;
  settings.magic = 0U;
  settings.version = 0U;
  settings.ResetToDefaults();
  zexpect_true(settings.IsValid());
}

// --- Identity defaults are preserved
ZTEST(nvm_settings, test_identity_default_name) {
  NvmSettings settings;
  zexpect_equal(settings.identity.get_name(), std::string_view("unimoc"));
}

// --- Node ID can be updated to valid range
ZTEST(nvm_settings, test_node_id_can_be_set) {
  NvmSettings settings;
  settings.node_id = 42U;
  zexpect_equal(settings.node_id, 42U);
  zexpect_true(settings.IsValid());
}

// --- Parameter spot-checks (verify fields exist + have sensible defaults)
ZTEST(nvm_settings, test_default_stator_r) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.stator_r.Value(), 0.1F);
}

ZTEST(nvm_settings, test_default_fw_v_max) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.fw_v_max.Value(), 0.9F);
}

ZTEST(nvm_settings, test_default_svm_duty_range) {
  NvmSettings settings;
  zexpect_true((settings.svm_duty_min) < (settings.svm_duty_max));
}

ZTEST(nvm_settings, test_default_pos_speed_limit) {
  NvmSettings settings;
  zexpect_true((settings.pos_speed_limit.Value()) > (0.0F));
}

ZTEST(nvm_settings, test_default_excitation_mode) {
  NvmSettings settings;
  zexpect_equal(settings.excitation_mode, 0U);  // CurrentMode
}

// --- Phase current balance defaults ---

ZTEST(nvm_settings, test_default_phase_balance_a) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_a.Value(), 1.0F);
}

ZTEST(nvm_settings, test_default_phase_balance_b) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_b.Value(), 1.0F);
}

ZTEST(nvm_settings, test_default_phase_balance_c) {
  NvmSettings settings;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_c.Value(), 1.0F);
}

ZTEST(nvm_settings, test_phase_balance_can_be_modified) {
  NvmSettings settings;
  settings.phase_balance_a = 1.02_ratio;
  settings.phase_balance_b = 0.98_ratio;
  settings.phase_balance_c = 1.00_ratio;
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_a.Value(), 1.02F);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_b.Value(), 0.98F);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_c.Value(), 1.00F);
  zexpect_true(settings.IsValid());
}

ZTEST(nvm_settings, test_reset_restores_phase_balance_defaults) {
  NvmSettings settings;
  settings.phase_balance_a = 1.05_ratio;
  settings.phase_balance_b = 0.95_ratio;
  settings.phase_balance_c = 0.99_ratio;
  settings.ResetToDefaults();
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_a.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_b.Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ(settings.phase_balance_c.Value(), 1.0F);
}

// --- NVM version is 2 after layout change ---

ZTEST(nvm_settings, test_nvm_version_is_two) {
  zexpect_equal(kNvmVersion, 3U);
  NvmSettings settings;
  zexpect_equal(settings.version, 3U);
}

}  // namespace unimoc::settings::test

ZTEST_SUITE(nvm_settings, nullptr, nullptr, nullptr, nullptr, nullptr);
