#pragma once

#ifndef UNIMOC_TEST_POSITION_CONTROLLER_H_
#define UNIMOC_TEST_POSITION_CONTROLLER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "position_controller.hpp"
#include "position_tracker.hpp"

namespace unimoc {
namespace control {
namespace test {

using namespace unit;

class PositionControllerTest : public ::testing::Test {
 protected:
  using Ctrl = PositionController<float>;
  using Tracker = unimoc::observer::PositionTracker;
  using Angle = unimoc::unit::Angle;
  using AngularVelocity = unimoc::unit::AngularVelocity;
  using AngularAcceleration = unimoc::unit::AngularAcceleration;
  using Current = unimoc::unit::Current;
  using Time = unimoc::unit::Time;

  Ctrl make_default() {
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
};

constexpr void SetHomeAdapter(void* ctx) { static_cast<unimoc::observer::PositionTracker*>(ctx)->SetHome(); }

// --- At rest, zero setpoint → zero output
TEST_F(PositionControllerTest, ZeroSetpointZeroOutput) {
  auto c = make_default();
  const float out = c.update(Angle{}, AngularVelocity{}, 100_us).Value();
  EXPECT_FLOAT_EQ(out, 0.0f);
}

// --- Positive position error drives positive omega_ref
TEST_F(PositionControllerTest, PositiveErrorDrivesPositiveOutput) {
  auto c = make_default();
  c.pos_ref_rad = 1.0_rad;
  const float out = c.update(Angle{}, AngularVelocity{}, 100_us).Value();
  EXPECT_GT(out, 0.0f);
}

TEST_F(PositionControllerTest, LargeJumpUsesTrapezoidPlanning) {
  auto c = make_default();
  c.trapezoid_jump_threshold = 0.2_rad;
  c.pos_ref_rad = 10.0_rad;
  c.update(Angle{}, AngularVelocity{}, 1_ms);
  EXPECT_LT(c.pos_ref_limited, c.pos_ref_rad);
}

// --- Output never exceeds speed_limit
TEST_F(PositionControllerTest, OutputClampedToSpeedLimit) {
  auto c = make_default();
  c.pos_ref_rad = 1000.0_rad;  // huge error
  for (int i = 0; i < 1000; ++i) c.update(Angle{}, AngularVelocity{}, 1_ms);
  EXPECT_LE(std::abs(c.omega_ref.Value()), c.speed_limit.Value());
}

// --- in_position flag set when close to target
TEST_F(PositionControllerTest, InPositionFlagSet) {
  auto c = make_default();
  c.pos_ref_rad = Angle{};
  c.update(Angle{}, AngularVelocity{}, 100_us);
  EXPECT_TRUE(c.in_position);
}

// --- in_position not set when position error is large
TEST_F(PositionControllerTest, InPositionFlagClearWhenFar) {
  auto c = make_default();
  c.pos_ref_rad = 10.0_rad;
  c.update(Angle{}, AngularVelocity{}, 100_us);
  EXPECT_FALSE(c.in_position);
}

// --- Homing: SEARCHING state outputs homing_speed
TEST_F(PositionControllerTest, HomingSearchingOutputsHomingSpeed) {
  auto c = make_default();
  c.start_homing();
  EXPECT_EQ(c.homing_state, HomingState::SEARCHING);
  const float out = c.update(Angle{}, AngularVelocity{}, 100_us).Value();
  EXPECT_FLOAT_EQ(out, c.homing_speed.Value());
}

// --- Homing: trigger_zeroing advances to ZEROING then DONE
TEST_F(PositionControllerTest, HomingTriggerZeroingAdvancesToDone) {
  auto c = make_default();
  Tracker tracker;
  tracker.Update(unimoc::system::RotorAngle::FromAngle(0.3_rad), 2);
  c.set_home_callback(&SetHomeAdapter, &tracker);
  c.start_homing();
  c.update(Angle{}, AngularVelocity{}, 100_us);  // SEARCHING step
  c.trigger_zeroing();
  EXPECT_EQ(c.homing_state, HomingState::ZEROING);
  c.update(Angle{}, AngularVelocity{}, 100_us);  // ZEROING step → transitions to DONE
  EXPECT_EQ(c.homing_state, HomingState::DONE);
  EXPECT_TRUE(tracker.IsHomed());
}

TEST_F(PositionControllerTest, HomingAutoZeroingOnCurrentThreshold) {
  auto c = make_default();
  c.homing_block_current_threshold = 5.0_A;
  c.start_homing();
  EXPECT_EQ(c.homing_state, HomingState::SEARCHING);
  c.update(Angle{}, AngularVelocity{}, 100_us, 5.1_A);
  EXPECT_EQ(c.homing_state, HomingState::DONE);
}

// --- trigger_zeroing ignored when not SEARCHING
TEST_F(PositionControllerTest, TriggerZeroingIgnoredWhenNotSearching) {
  auto c = make_default();
  c.trigger_zeroing();  // in IDLE
  EXPECT_EQ(c.homing_state, HomingState::IDLE);
}

// --- fault() transitions to FAULT and zeroes output
TEST_F(PositionControllerTest, FaultTransitionAndZeroOutput) {
  auto c = make_default();
  c.start_homing();
  c.fault();
  EXPECT_EQ(c.homing_state, HomingState::FAULT);
  EXPECT_FLOAT_EQ(c.omega_ref.Value(), 0.0F);
}

// --- reset() returns to IDLE
TEST_F(PositionControllerTest, ResetReturnsToIdle) {
  auto c = make_default();
  c.start_homing();
  c.fault();
  c.reset();
  EXPECT_EQ(c.homing_state, HomingState::IDLE);
  EXPECT_FLOAT_EQ(c.speed_integrator.Value(), 0.0F);
  EXPECT_FLOAT_EQ(c.omega_ref.Value(), 0.0F);
}

// --- Closed-loop convergence: shaft reaches target
TEST_F(PositionControllerTest, ConvergesToSetpoint) {
  auto c = make_default();
  float pos = 0.0f;
  float vel = 0.0f;

  c.pos_ref_rad = 2.0_rad;  // 2 rad target

  for (int i = 0; i < 50000; ++i) {
    const float omega_cmd = c.update(Angle{pos}, AngularVelocity{vel}, 100_us).Value();
    // Simple first-order motor model
    vel += 0.01f * (omega_cmd - vel);
    pos += vel * 1e-4f;
  }
  EXPECT_NEAR(pos, 2.0f, 0.1f);
}

}  // namespace test
}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_TEST_POSITION_CONTROLLER_H_ */
