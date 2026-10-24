/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file excitation_controller_test.cpp
 *    @brief ztest coverage of the EESM excitation controller (current and flux mode).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "excitation_controller.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

// Shared helpers of the excitation_controller suite.
namespace {

using Ctrl = ExcitationController<float>;

Ctrl MakeCurrentMode() {
  Ctrl c;
  c.mode = ExcitationMode::kCurrentMode;
  c.kp = 5.0_ratio;
  c.ki = 50.0_per_s;
  c.i_f_min = 0.0_A;
  c.i_f_max = 10.0_A;
  c.L_m = 47.0_mH;
  c.current_setpoint = unimoc::unit::Current{};
  return c;
}

Ctrl MakeFluxMode() {
  Ctrl c = MakeCurrentMode();
  c.mode = ExcitationMode::kFluxMode;
  // ψ_f* = 2.35 Wb  →  I_f* = 2.35 / 0.047 = 50 A (clamped to 10 A)
  c.flux_setpoint = 0.47_Wb;  // ψ_f* [Wb] → I_f* = 0.47/0.047 = 10 A
  return c;
}

}  // namespace

// --- CurrentMode: zero setpoint, zero measurement → zero output
ZTEST(excitation_controller, test_current_mode_zero_error) {
  auto c = MakeCurrentMode();
  const Current kOut = c.Update(Current{}, 100_us);
  UNIMOC_EXPECT_FLOAT_EQ(kOut, 0.0_A);
}

// --- CurrentMode: positive setpoint drives positive output
ZTEST(excitation_controller, test_current_mode_positive_setpoint) {
  auto c = MakeCurrentMode();
  c.current_setpoint = 5.0_A;
  const Current kOut = c.Update(Current{}, 100_us);
  zexpect_true(kOut > 0.0_A);
  zexpect_true(kOut <= c.i_f_max);
}

// --- CurrentMode: integrator saturates at i_f_max
ZTEST(excitation_controller, test_current_mode_integrator_clamp) {
  auto c = MakeCurrentMode();
  c.current_setpoint = 100.0_A;  // Large error
  for (int i = 0; i < 1000; ++i) {
    c.Update(unimoc::unit::Current{}, 1_ms);
  }
  UNIMOC_EXPECT_FLOAT_EQ(c.i_f_ref, c.i_f_max);
  zexpect_true(c.integrator <= c.i_f_max);
}

// --- CurrentMode: output never goes below i_f_min
ZTEST(excitation_controller, test_current_mode_min_clamp) {
  auto c = MakeCurrentMode();
  c.current_setpoint = -100.0_A;  // drive negative
  for (int i = 0; i < 1000; ++i) {
    c.Update(unimoc::unit::Current{}, 1_ms);
  }
  UNIMOC_EXPECT_FLOAT_EQ(c.i_f_ref, c.i_f_min);
}

// --- FluxMode: ψ_f* / L_m = correct current reference
ZTEST(excitation_controller, test_flux_mode_conversion) {
  auto c = MakeFluxMode();
  // setpoint = 0.47 Wb, L_m = 0.047 H → I_f* = 10 A (exactly at max)
  for (int i = 0; i < 5000; ++i) {
    c.Update(unimoc::unit::Current{}, 100_us);
  }
  UNIMOC_EXPECT_FLOAT_EQ(c.i_f_ref, c.i_f_max);
}

// --- reset() clears integrator and output
ZTEST(excitation_controller, test_reset_clears_state) {
  auto c = MakeCurrentMode();
  c.current_setpoint = 5.0_A;
  for (int i = 0; i < 100; ++i) {
    c.Update(unimoc::unit::Current{}, 1_ms);
  }
  c.Reset();
  UNIMOC_EXPECT_FLOAT_EQ(c.integrator, 0.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(c.i_f_ref, 0.0_A);
}

// --- Output converges to setpoint when feedback equals reference
ZTEST(excitation_controller, test_current_mode_converges) {
  auto c = MakeCurrentMode();
  c.current_setpoint = 3.0_A;
  Current meas;

  for (int i = 0; i < 5000; ++i) {
    const Current kRef = c.Update(meas, 100_us);
    // Simulate first-order plant: meas tracks ref slowly
    meas += 0.01F * (kRef - meas);
  }
  UNIMOC_EXPECT_NEAR(meas, 3.0_A, 0.1_A);
}

}  // namespace unimoc::control::test

ZTEST_SUITE(excitation_controller, nullptr, nullptr, nullptr, nullptr, nullptr);
