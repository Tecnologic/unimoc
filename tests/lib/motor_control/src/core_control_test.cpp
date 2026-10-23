/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file core_control_test.cpp
 *    @brief ztest coverage of dead-time compensation, MTPA, field weakening and the current controller.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <cmath>
#include "current_controller.hpp"
#include "dead_time_compensation.hpp"
#include "field_weakening.hpp"
#include "mtpa.hpp"
#include "unimoc_test.hpp"

using DimensionlessStator = unimoc::system::Stator<unimoc::unit::Ratio>;
using CurrentStator = unimoc::system::Stator<unimoc::unit::Current>;
using CurrentRotor = unimoc::system::Rotor<unimoc::unit::Current>;
using VoltageRotor = unimoc::system::Rotor<unimoc::unit::Voltage>;
using namespace unimoc::unit;

ZTEST(dead_time_compensation, test_reconstructs_and_transforms_phase_signs) {
  unimoc::settings::NvmSettings settings;
  settings.dtc_dead_time = 2.0_us;
  settings.dtc_f_pwm = 10.0_kHz;
  settings.dtc_i_threshold = 1.0_A;

  unimoc::control::DeadTimeCompensation<float> compensation;
  compensation.Init(settings);

  const auto kResult = compensation.Calculate(CurrentStator{1.0F, 0.0F});

  UNIMOC_EXPECT_NEAR(kResult.alpha, 0.02_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kResult.beta, 0.0_ratio, 1.0e-6_ratio);
}

ZTEST(dead_time_compensation, test_soft_sign_interpolates_and_saturates) {
  unimoc::control::DeadTimeCompensation<float> compensation;
  compensation.dead_time = 2.0_us;
  compensation.f_pwm = 10.0_kHz;
  compensation.i_threshold = 1.0_A;

  const auto kInterpolated = compensation.Calculate(CurrentStator{0.5F, 0.0F});
  const auto kSaturated = compensation.Calculate(CurrentStator{2.0F, 0.0F});

  UNIMOC_EXPECT_NEAR(kInterpolated.alpha, 0.01_ratio, 1.0e-6_ratio);
  UNIMOC_EXPECT_NEAR(kSaturated.alpha, 0.0266667_ratio, 1.0e-6_ratio);
}

ZTEST(mtpa, test_zero_saliency_returns_zero) {
  unimoc::control::Mtpa<float> mtpa;
  mtpa.flux_pm = 0.1_Wb;
  mtpa.L_d = 1.0_mH;
  mtpa.L_q = 1.0_mH;

  UNIMOC_EXPECT_FLOAT_EQ(mtpa.Calculate(2.0_A), 0.0_A);
}

ZTEST(mtpa, test_calculates_and_limits_daxis_current) {
  unimoc::control::Mtpa<float> mtpa;
  mtpa.flux_pm = 0.0_Wb;
  mtpa.L_d = 2.0_mH;
  mtpa.L_q = 1.0_mH;
  UNIMOC_EXPECT_FLOAT_EQ(mtpa.Calculate(2.0_A), -1.0_A);

  mtpa.flux_pm = 0.01_Wb;
  mtpa.L_d = 1.0_mH;
  mtpa.L_q = 2.0_mH;
  UNIMOC_EXPECT_FLOAT_EQ(mtpa.Calculate(1.0_A), -1.0_A);
}

ZTEST(field_weakening, test_integrates_only_negative_voltage_headroom) {
  unimoc::control::FieldWeakening<float> field_weakening;
  field_weakening.v_max = 0.9_V;
  field_weakening.ki = 10.0_A_per_V_s;
  field_weakening.i_d_min = -0.5_A;

  UNIMOC_EXPECT_FLOAT_EQ(field_weakening.Update(unimoc::system::Stator<unimoc::unit::Voltage>{0.5_V, 0.0_V},
                                                0.1_s),
                         0.0_A);
  UNIMOC_EXPECT_NEAR(field_weakening.Update(unimoc::system::Stator<unimoc::unit::Voltage>{1.0_V, 0.0_V},
                                            0.1_s),
                     -0.1_A,
                     1.0e-6_A);
  UNIMOC_EXPECT_FLOAT_EQ(field_weakening.Update(unimoc::system::Stator<unimoc::unit::Voltage>{10.0_V, 0.0_V},
                                                1.0_s),
                         -0.5_A);

  field_weakening.Reset();
  UNIMOC_EXPECT_FLOAT_EQ(field_weakening.i_d_fw, 0.0_A);
}

ZTEST(current_controller, test_unsaturated_pi_step_updates_integrator) {
  unimoc::control::CurrentController<float> controller;
  controller.kp_d = 2.0_V_per_A;
  controller.ki_d = 10.0_V_per_A_s;
  controller.v_max = 1.0_ratio;

  const VoltageRotor kOutput =
      controller.Update(CurrentRotor{1.0F, 0.0F}, CurrentRotor{0.0F, 0.0F}, 0.0_rad_per_s, 0.1_s, 10.0_V);

  UNIMOC_EXPECT_FLOAT_EQ(kOutput.d, 2.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(kOutput.q, 0.0_V);
  UNIMOC_EXPECT_FLOAT_EQ(controller.integrator_d, 1.0_V);
}

ZTEST(current_controller, test_circular_limit_scales_both_axes) {
  unimoc::control::CurrentController<float> controller;
  controller.kp_d = 2.0_V_per_A;
  controller.kp_q = 2.0_V_per_A;
  controller.ki_d = 0.0_V_per_A_s;
  controller.ki_q = 0.0_V_per_A_s;
  controller.kb_d = 0.0_per_s;
  controller.kb_q = 0.0_per_s;
  controller.v_max = 0.1_ratio;

  const VoltageRotor kOutput =
      controller.Update(CurrentRotor{1.0F, 1.0F}, CurrentRotor{0.0F, 0.0F}, 0.0_rad_per_s, 0.1_s, 10.0_V);
  UNIMOC_EXPECT_NEAR(kOutput.Length(), 1.0_V, 1.0e-6_V);
  UNIMOC_EXPECT_NEAR(kOutput.d, kOutput.q, 1.0e-6_V);
}

ZTEST_SUITE(dead_time_compensation, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(mtpa, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(field_weakening, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(current_controller, nullptr, nullptr, nullptr, nullptr, nullptr);
