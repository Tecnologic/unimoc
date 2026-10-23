/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file excitation_observer_test.cpp
 *    @brief ztest coverage of the EESM excitation observer.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "excitation_observer.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include "unimoc_test.hpp"

namespace unimoc::observer::test {

using namespace unit;

// Shared helpers of the excitation_observer suite.
namespace {

using Obs = ExcitationObserver<float>;

}  // namespace

// --- Default-constructed observer has zero outputs
ZTEST(excitation_observer, test_default_state_is_zero) {
  Obs observer;
  UNIMOC_EXPECT_FLOAT_EQ(observer.i_f_hat, 0.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(observer.psi_f_hat, 0.0_Wb);
}

// --- Single step moves toward measurement
ZTEST(excitation_observer, test_single_step_converges_direction) {
  Obs observer;
  observer.tau = 5.0_ms;
  observer.L_m = 47.0_mH;
  observer.Update(10.0_A, 100_us);  // dt = 0.1 ms
  zexpect_true(observer.i_f_hat > 0.0_A);
  zexpect_true(observer.i_f_hat < 10.0_A);
}

// --- After many steps the output converges to the measurement
ZTEST(excitation_observer, test_converges_after_many_steps) {
  Obs observer;
  observer.tau = 1.0_ms;
  observer.L_m = 47.0_mH;
  for (int i = 0; i < 10000; ++i) {
    observer.Update(8.0_A, 10_us);  // total time >> tau
  }
  UNIMOC_EXPECT_NEAR(observer.i_f_hat, 8.0_A, 1.0e-3_A);
  UNIMOC_EXPECT_NEAR(observer.psi_f_hat, (8.0F * 47e-3F) * 1.0_Wb, 1.0e-4_Wb);
}

// --- psi_f_hat = L_m * i_f_hat at every step
ZTEST(excitation_observer, test_psi_equals_lm_times_i) {
  Obs observer;
  observer.tau = 2.0_ms;
  observer.L_m = 50.0_mH;
  for (int i = 0; i < 20; ++i) {
    observer.Update(5.0_A, 100_us);
    UNIMOC_EXPECT_NEAR(observer.psi_f_hat, observer.L_m * observer.i_f_hat, 1.0e-7_Wb);
  }
}

// --- Zero time constant passes measurement through immediately
ZTEST(excitation_observer, test_zero_tau_pass_through) {
  Obs observer;
  observer.tau = unimoc::unit::Time{};
  observer.L_m = 47.0_mH;
  observer.Update(7.5_A, 100_us);
  UNIMOC_EXPECT_FLOAT_EQ(observer.i_f_hat, 7.5_A);
}

// --- reset() with init value
ZTEST(excitation_observer, test_reset_with_init_value) {
  Obs observer;
  observer.tau = 2.0_ms;
  observer.L_m = 47.0_mH;
  for (int i = 0; i < 100; ++i) {
    observer.Update(5.0_A, 100_us);
  }
  observer.Reset(3.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(observer.i_f_hat, 3.0_A);
  UNIMOC_EXPECT_NEAR(observer.psi_f_hat, (3.0F * 47.0e-3F) * 1.0_Wb, 1.0e-7_Wb);
}

}  // namespace unimoc::observer::test

ZTEST_SUITE(excitation_observer, nullptr, nullptr, nullptr, nullptr, nullptr);
