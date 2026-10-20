/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file position_controller_test.cpp
 *    @brief ztest coverage of the cascaded position and speed controller.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "position_controller.hpp"
#include <zephyr/ztest.h>
#include <cmath>
#include "position_tracker.hpp"
#include "unimoc_test.hpp"

namespace unimoc::control::test {

using namespace unit;

// Shared helpers of the position_controller suite.
namespace {

using Ctrl = PositionController<float>;
using Tracker = unimoc::observer::PositionTracker;
using Angle = unimoc::unit::Angle;
using AngularVelocity = unimoc::unit::AngularVelocity;
using AngularAcceleration = unimoc::unit::AngularAcceleration;
using Current = unimoc::unit::Current;
using Time = unimoc::unit::Time;

Ctrl MakeDefault() {
  Ctrl c;
  c.kp_pos = 10.0_rad_per_s_per_rad;
  c.kp_speed = 5.0_ratio;
  c.ki_speed = 20.0_per_s;
  c.speed_limit = 50.0_rad_per_s;
  c.accel_limit = 500.0_rad_per_s2;
  c.position_tolerance = 0.01_rad;
  c.speed_tolerance = 0.5_rad_per_s;
  c.homing_speed = 3.0_rad_per_s;
  c.pos_ref_rad = Angle{};
  return c;
}

}  // namespace

constexpr void SetHomeAdapter(void* ctx) { static_cast<unimoc::observer::PositionTracker*>(ctx)->SetHome(); }

// --- At rest, zero setpoint → zero output
ZTEST(position_controller, test_zero_setpoint_zero_output) {
  auto c = MakeDefault();
  const float kOut = c.Update(Angle{}, AngularVelocity{}, 100_us).Value();
  UNIMOC_EXPECT_FLOAT_EQ(kOut, 0.0F);
}

// --- Positive position error drives positive omega_ref
ZTEST(position_controller, test_positive_error_drives_positive_output) {
  auto c = MakeDefault();
  c.pos_ref_rad = 1.0_rad;
  const float kOut = c.Update(Angle{}, AngularVelocity{}, 100_us).Value();
  zexpect_true((kOut) > (0.0F));
}

ZTEST(position_controller, test_large_jump_uses_trapezoid_planning) {
  auto c = MakeDefault();
  c.trapezoid_jump_threshold = 0.2_rad;
  c.pos_ref_rad = 10.0_rad;
  c.Update(Angle{}, AngularVelocity{}, 1_ms);
  zexpect_true((c.pos_ref_limited) < (c.pos_ref_rad));
}

// --- Output never exceeds speed_limit
ZTEST(position_controller, test_output_clamped_to_speed_limit) {
  auto c = MakeDefault();
  c.pos_ref_rad = 1000.0_rad;  // huge error
  for (int i = 0; i < 1000; ++i) {
    c.Update(Angle{}, AngularVelocity{}, 1_ms);
  }
  zexpect_true((std::abs(c.omega_ref.Value())) <= (c.speed_limit.Value()));
}

// --- in_position flag set when close to target
ZTEST(position_controller, test_in_position_flag_set) {
  auto c = MakeDefault();
  c.pos_ref_rad = Angle{};
  c.Update(Angle{}, AngularVelocity{}, 100_us);
  zexpect_true(c.in_position);
}

// --- in_position not set when position error is large
ZTEST(position_controller, test_in_position_flag_clear_when_far) {
  auto c = MakeDefault();
  c.pos_ref_rad = 10.0_rad;
  c.Update(Angle{}, AngularVelocity{}, 100_us);
  zexpect_false(c.in_position);
}

// --- Homing: SEARCHING state outputs homing_speed
ZTEST(position_controller, test_homing_searching_outputs_homing_speed) {
  auto c = MakeDefault();
  c.StartHoming();
  zexpect_equal(c.homing_state, HomingState::kSearching);
  const float kOut = c.Update(Angle{}, AngularVelocity{}, 100_us).Value();
  UNIMOC_EXPECT_FLOAT_EQ(kOut, c.homing_speed.Value());
}

// --- Homing: TriggerZeroing advances to ZEROING then DONE
ZTEST(position_controller, test_homing_trigger_zeroing_advances_to_done) {
  auto c = MakeDefault();
  Tracker tracker;
  tracker.Update(unimoc::system::RotorAngle::FromAngle(0.3_rad), 2);
  c.SetHomeCallback(&SetHomeAdapter, &tracker);
  c.StartHoming();
  c.Update(Angle{}, AngularVelocity{}, 100_us);  // SEARCHING step
  c.TriggerZeroing();
  zexpect_equal(c.homing_state, HomingState::kZeroing);
  c.Update(Angle{}, AngularVelocity{}, 100_us);  // ZEROING step → transitions to DONE
  zexpect_equal(c.homing_state, HomingState::kDone);
  zexpect_true(tracker.IsHomed());
}

ZTEST(position_controller, test_homing_auto_zeroing_on_current_threshold) {
  auto c = MakeDefault();
  c.homing_block_current_threshold = 5.0_A;
  c.StartHoming();
  zexpect_equal(c.homing_state, HomingState::kSearching);
  c.Update(Angle{}, AngularVelocity{}, 100_us, 5.1_A);
  zexpect_equal(c.homing_state, HomingState::kDone);
}

// --- TriggerZeroing ignored when not SEARCHING
ZTEST(position_controller, test_trigger_zeroing_ignored_when_not_searching) {
  auto c = MakeDefault();
  c.TriggerZeroing();  // in IDLE
  zexpect_equal(c.homing_state, HomingState::kIdle);
}

// --- Fault() transitions to FAULT and zeroes output
ZTEST(position_controller, test_fault_transition_and_zero_output) {
  auto c = MakeDefault();
  c.StartHoming();
  c.Fault();
  zexpect_equal(c.homing_state, HomingState::kFault);
  UNIMOC_EXPECT_FLOAT_EQ(c.omega_ref.Value(), 0.0F);
}

// --- Reset() returns to IDLE
ZTEST(position_controller, test_reset_returns_to_idle) {
  auto c = MakeDefault();
  c.StartHoming();
  c.Fault();
  c.Reset();
  zexpect_equal(c.homing_state, HomingState::kIdle);
  UNIMOC_EXPECT_FLOAT_EQ(c.speed_integrator.Value(), 0.0F);
  UNIMOC_EXPECT_FLOAT_EQ(c.omega_ref.Value(), 0.0F);
}

// --- Closed-loop convergence: shaft reaches target
ZTEST(position_controller, test_converges_to_setpoint) {
  auto c = MakeDefault();
  float pos = 0.0F;
  float vel = 0.0F;

  c.pos_ref_rad = 2.0_rad;  // 2 rad target

  for (int i = 0; i < 50000; ++i) {
    const float kOmegaCmd = c.Update(Angle{pos}, AngularVelocity{vel}, 100_us).Value();
    // Simple first-order motor model
    vel += 0.01F * (kOmegaCmd - vel);
    pos += vel * 1e-4F;
  }
  UNIMOC_EXPECT_NEAR(pos, 2.0F, 0.1F);
}

}  // namespace unimoc::control::test

ZTEST_SUITE(position_controller, nullptr, nullptr, nullptr, nullptr, nullptr);
