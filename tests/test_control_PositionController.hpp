#pragma once

#ifndef UNIMOC_TEST_POSITION_CONTROLLER_H_
#define UNIMOC_TEST_POSITION_CONTROLLER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "PositionController.hpp"
#include "PositionTracker.hpp"

namespace unimoc
{
namespace control
{
namespace test
{

class PositionControllerTest : public ::testing::Test
{
protected:
    using Ctrl = PositionController<float>;
    using Tracker = unimoc::observer::PositionTracker<float>;

    Ctrl make_default()
    {
        Ctrl c;
        c.kp_pos           = 10.0f;
        c.kp_speed         = 5.0f;
        c.ki_speed         = 20.0f;
        c.speed_limit      = 50.0f;
        c.accel_limit      = 500.0f;
        c.position_tolerance = 0.01f;
        c.speed_tolerance  = 0.5f;
        c.homing_speed     = 3.0f;
        c.pos_ref_rad      = 0.0f;
        return c;
    }
};

constexpr void set_home_adapter(void* ctx, int pole_pairs)
{
    static_cast<unimoc::observer::PositionTracker<float>*>(ctx)->set_home(pole_pairs);
}

// --- At rest, zero setpoint → zero output
TEST_F(PositionControllerTest, ZeroSetpointZeroOutput)
{
    auto c = make_default();
    const float out = c.update(0.0f, 0.0f, 1e-4f);
    EXPECT_FLOAT_EQ(out, 0.0f);
}

// --- Positive position error drives positive omega_ref
TEST_F(PositionControllerTest, PositiveErrorDrivesPositiveOutput)
{
    auto c = make_default();
    c.pos_ref_rad = 1.0f;
    const float out = c.update(0.0f, 0.0f, 1e-4f);
    EXPECT_GT(out, 0.0f);
}

TEST_F(PositionControllerTest, LargeJumpUsesTrapezoidPlanning)
{
    auto c = make_default();
    c.trapezoid_jump_threshold = 0.2f;
    c.pos_ref_rad = 10.0f;
    c.update(0.0f, 0.0f, 1e-3f);
    EXPECT_LT(c.pos_ref_limited, c.pos_ref_rad);
}

// --- Output never exceeds speed_limit
TEST_F(PositionControllerTest, OutputClampedToSpeedLimit)
{
    auto c = make_default();
    c.pos_ref_rad = 1000.0f;  // huge error
    for (int i = 0; i < 1000; ++i)
        c.update(0.0f, 0.0f, 1e-3f);
    EXPECT_LE(std::abs(c.omega_ref), c.speed_limit);
}

// --- in_position flag set when close to target
TEST_F(PositionControllerTest, InPositionFlagSet)
{
    auto c = make_default();
    c.pos_ref_rad = 0.0f;
    c.update(0.0f, 0.0f, 1e-4f);
    EXPECT_TRUE(c.in_position);
}

// --- in_position not set when position error is large
TEST_F(PositionControllerTest, InPositionFlagClearWhenFar)
{
    auto c = make_default();
    c.pos_ref_rad = 10.0f;
    c.update(0.0f, 0.0f, 1e-4f);
    EXPECT_FALSE(c.in_position);
}

// --- Homing: SEARCHING state outputs homing_speed
TEST_F(PositionControllerTest, HomingSearchingOutputsHomingSpeed)
{
    auto c = make_default();
    c.start_homing();
    EXPECT_EQ(c.homing_state, HomingState::SEARCHING);
    const float out = c.update(0.0f, 0.0f, 1e-4f);
    EXPECT_FLOAT_EQ(out, c.homing_speed);
}

// --- Homing: trigger_zeroing advances to ZEROING then DONE
TEST_F(PositionControllerTest, HomingTriggerZeroingAdvancesToDone)
{
    auto c = make_default();
    Tracker tracker;
    tracker.update(0.3f, 2);
    c.set_home_callback(&set_home_adapter, &tracker, 2);
    c.start_homing();
    c.update(0.0f, 0.0f, 1e-4f);  // SEARCHING step
    c.trigger_zeroing();
    EXPECT_EQ(c.homing_state, HomingState::ZEROING);
    c.update(0.0f, 0.0f, 1e-4f);  // ZEROING step → transitions to DONE
    EXPECT_EQ(c.homing_state, HomingState::DONE);
    EXPECT_TRUE(tracker.is_homed);
}

TEST_F(PositionControllerTest, HomingAutoZeroingOnCurrentThreshold)
{
    auto c = make_default();
    c.homing_block_current_threshold = 5.0f;
    c.start_homing();
    EXPECT_EQ(c.homing_state, HomingState::SEARCHING);
    c.update(0.0f, 0.0f, 1e-4f, 5.1f);
    EXPECT_EQ(c.homing_state, HomingState::DONE);
}

// --- trigger_zeroing ignored when not SEARCHING
TEST_F(PositionControllerTest, TriggerZeroingIgnoredWhenNotSearching)
{
    auto c = make_default();
    c.trigger_zeroing();  // in IDLE
    EXPECT_EQ(c.homing_state, HomingState::IDLE);
}

// --- fault() transitions to FAULT and zeroes output
TEST_F(PositionControllerTest, FaultTransitionAndZeroOutput)
{
    auto c = make_default();
    c.start_homing();
    c.fault();
    EXPECT_EQ(c.homing_state, HomingState::FAULT);
    EXPECT_FLOAT_EQ(c.omega_ref, 0.0f);
}

// --- reset() returns to IDLE
TEST_F(PositionControllerTest, ResetReturnsToIdle)
{
    auto c = make_default();
    c.start_homing();
    c.fault();
    c.reset();
    EXPECT_EQ(c.homing_state, HomingState::IDLE);
    EXPECT_FLOAT_EQ(c.speed_integrator, 0.0f);
    EXPECT_FLOAT_EQ(c.omega_ref, 0.0f);
}

// --- Closed-loop convergence: shaft reaches target
TEST_F(PositionControllerTest, ConvergesToSetpoint)
{
    auto  c   = make_default();
    float pos = 0.0f;
    float vel = 0.0f;

    c.pos_ref_rad = 2.0f;  // 2 rad target

    for (int i = 0; i < 50000; ++i)
    {
        const float omega_cmd = c.update(pos, vel, 1e-4f);
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
