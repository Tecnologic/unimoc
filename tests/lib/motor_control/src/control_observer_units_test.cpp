/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_observer_units_test.cpp
 *    @brief Tests for unit-typed control and observer configuration.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <zephyr/ztest.h>
#include <type_traits>
#include "asm_flux_controller.hpp"
#include "asm_flux_observer.hpp"
#include "dead_time_compensation.hpp"
#include "field_weakening.hpp"
#include "hfi.hpp"
#include "mechanical_observer.hpp"
#include "mtpa.hpp"
#include "nvm_settings.hpp"
#include "svm.hpp"
#include "unimoc_test.hpp"

namespace unimoc::test {

using namespace unit;

ZTEST(control_observer_units, test_ratio_per_time_is_no_angular_velocity) {
  static_assert(std::is_same_v<decltype(unit::Ratio{} / unit::Time{}), unit::InverseTime>);
  static_assert(!std::is_same_v<decltype(unit::Ratio{} / unit::Time{}), unit::AngularVelocity>);
  static_assert(
      std::is_same_v<decltype((unit::Ratio{} * unit::Angle{}) / unit::Time{}), unit::AngularVelocity>);
}

ZTEST(control_observer_units, test_mechanical_observer_uses_unit_types) {
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::psi), unit::MagneticFlux>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::J), unit::Inertia>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::omega), unit::AngularVelocity>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::theta), unit::Angle>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::m_l), unit::Torque>);
}

ZTEST(control_observer_units, test_motor_control_physical_state_uses_unit_types) {
  static_assert(std::is_same_v<decltype(control::FieldWeakening<float>::v_max), unit::Voltage>);
  static_assert(std::is_same_v<decltype(control::FieldWeakening<float>::i_d_fw), unit::Current>);
  static_assert(std::is_same_v<decltype(control::Mtpa<float>::flux_pm), unit::MagneticFlux>);
  static_assert(std::is_same_v<decltype(control::Mtpa<float>::L_d), unit::Inductance>);
  static_assert(std::is_same_v<decltype(control::AsmFluxController::R_r), unit::Resistance>);
  static_assert(std::is_same_v<decltype(unit::MagneticFlux{} - unit::MagneticFlux{}), unit::MagneticFlux>);
  static_assert(std::is_same_v<decltype(unit::Inductance{} / unit::Resistance{}), unit::Time>);
  static_assert(
      std::is_same_v<decltype(unit::CurrentPerMagneticFlux{} * unit::MagneticFlux{}), unit::Current>);
  static_assert(
      std::is_same_v<decltype(unit::CurrentPerMagneticFluxTime{} * unit::MagneticFlux{} * unit::Time{}),
                     unit::Current>);
  static_assert(std::is_same_v<decltype(((unit::Inductance{} * unit::Current{}) / unit::MagneticFlux{} *
                                         unit::Angle{}) /
                                        unit::Time{}),
                               unit::AngularVelocity>);
  static_assert(
      std::is_same_v<decltype(observer::AsmFluxObserver<float>::flux_magnitude), unit::MagneticFlux>);
  static_assert(std::is_same_v<decltype(observer::AsmFluxObserver<float>::flux_angle), unit::Angle>);
  static_assert(std::is_same_v<decltype(observer::Hfi<float>::i_d_step0), unit::Current>);
}

ZTEST(control_observer_units, test_asm_and_hfi_apis_accept_physical_units) {
  observer::MechanicalObserver<float> mechanical_observer;
  mechanical_observer.J = 1.0_kg_m2;
  mechanical_observer.omega_min = -100.0_rad_per_s;
  mechanical_observer.omega_max = 100.0_rad_per_s;

  observer::AsmFluxObserver<float> asm_observer;
  asm_observer.Update(system::Stator<unit::Voltage>{1.0_V, 0.0_V},
                      system::Stator<unit::Current>{0.0_A, 0.0_A},
                      10_us,
                      mechanical_observer);
  zexpect_true(IsFinite(asm_observer.flux_magnitude));
  for (int step = 0; step < 100; ++step) {
    asm_observer.Update(system::Stator<unit::Voltage>{1.0_V, 0.5_V},
                        system::Stator<unit::Current>{0.2_A, 0.1_A},
                        10_us,
                        mechanical_observer);
  }
  // sin/cos of the flux angle come from the flux vector and agree with the angle.
  zexpect_true(asm_observer.flux_magnitude > unit::MagneticFlux{});
  const system::SinCos<unit::Ratio> kFluxSinCos{asm_observer.flux_angle};
  UNIMOC_EXPECT_NEAR(asm_observer.sin_flux, kFluxSinCos.sin, 1e-5_ratio);
  UNIMOC_EXPECT_NEAR(asm_observer.cos_flux, kFluxSinCos.cos, 1e-5_ratio);

  control::AsmFluxController asm_controller;
  const unit::Current kCurrentReference = asm_controller.Update(0.05_Wb, 0.01_Wb, 1.0_A, 100_us);
  zexpect_true(kCurrentReference >= asm_controller.i_d_min);

  observer::Hfi<float> hfi;
  hfi.v_inject = 2.0_V;
  const auto kInjection = hfi.GetInjectionVoltage(0.0_ratio, 1.0_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kInjection.alpha, 2.0_V);
  hfi.Update(system::Stator<unit::Current>{0.0F, 0.0F}, 0.0_ratio, 1.0_ratio, 10_us, mechanical_observer);
}

ZTEST(control_observer_units, test_algorithms_load_initial_settings_from_nvm) {
  settings::NvmSettings settings{};
  settings.flux_pm = 0.012_Wb;
  settings.l_d = 2.0_mH;
  settings.l_q = 3.0_mH;
  settings.motor_j = 0.004_kg_m2;
  settings.motor_omega_max = 123.0_rad_per_s;
  settings.motor_omega_min = -45.0_rad_per_s;
  settings.mech_obs_q = 0.005F;
  settings.mech_obs_r = 0.006_rad2;
  settings.dtc_dead_time = 2_us;
  settings.dtc_f_pwm = 25.0_kHz;
  settings.dtc_i_threshold = 0.7_A;
  settings.hfi_v_inject = 2.5_V;
  settings.hfi_error_gain = 3.0_rad_per_A;
  settings.svm_duty_min = 0.1_ratio;
  settings.svm_duty_max = 0.9_ratio;

  observer::MechanicalObserver<float> mechanical_observer;
  mechanical_observer.Init(settings);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.psi, 0.012_Wb);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.L_d, 0.002_H);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.L_q, 0.003_H);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.J, 0.004_kg_m2);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.omega_max, 123.0_rad_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.omega_min, -45.0_rad_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.Q, 0.005F);
  UNIMOC_EXPECT_FLOAT_EQ(mechanical_observer.R, 0.006_rad2);

  control::DeadTimeCompensation<float> dead_time_compensation;
  dead_time_compensation.Init(settings);
  UNIMOC_EXPECT_FLOAT_EQ(dead_time_compensation.dead_time, 0.000002_s);
  UNIMOC_EXPECT_FLOAT_EQ(dead_time_compensation.f_pwm, 25000.0_Hz);
  UNIMOC_EXPECT_FLOAT_EQ(dead_time_compensation.i_threshold, 0.7_A);

  observer::Hfi<float> hfi;
  hfi.Init(settings);
  UNIMOC_EXPECT_FLOAT_EQ(hfi.v_inject, 2.5_V);
  UNIMOC_EXPECT_FLOAT_EQ(hfi.error_gain, 3.0_rad_per_A);

  control::Svm svm;
  svm.Init(settings);
  UNIMOC_EXPECT_FLOAT_EQ(svm.duty_min, 0.1_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(svm.duty_max, 0.9_ratio);
}

}  // namespace unimoc::test

ZTEST_SUITE(control_observer_units, nullptr, nullptr, nullptr, nullptr, nullptr);
