#pragma once

#ifndef UNIMOC_TEST_POSITION_TRACKER_H_
#define UNIMOC_TEST_POSITION_TRACKER_H_

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include "PositionTracker.hpp"

namespace unimoc
{
namespace observer
{
namespace test
{

class PositionTrackerTest : public ::testing::Test
{
protected:
    using Tracker = PositionTracker<float>;

    static constexpr float pi     = std::numbers::pi_v<float>;
    static constexpr float two_pi = 2.0f * pi;
    static constexpr int   pp     = 4;  // pole pairs
};

// --- Default state
TEST_F(PositionTrackerTest, DefaultStateIsZero)
{
    Tracker t;
    EXPECT_EQ(t.turns,        0);
    EXPECT_FLOAT_EQ(t.position_rad, 0.0f);
    EXPECT_FLOAT_EQ(t.position_rev, 0.0f);
    EXPECT_FALSE(t.is_homed);
}

// --- No wrapping: small forward step stays at 0 turns
TEST_F(PositionTrackerTest, NoWrapSmallStep)
{
    Tracker t;
    t.update(0.5f, pp);
    EXPECT_EQ(t.turns, 0);
}

// --- Positive wrap detection: cross from just below +π to just above −π
TEST_F(PositionTrackerTest, PositiveWrapDetected)
{
    Tracker t;
    // Start near +π
    t.update(pi - 0.01f, pp);
    EXPECT_EQ(t.turns, 0);

    // Cross wrap boundary (jump as if angle wrapped to −π+ε)
    t.update(-pi + 0.01f, pp);
    EXPECT_EQ(t.turns, 1);
}

// --- Negative wrap detection
TEST_F(PositionTrackerTest, NegativeWrapDetected)
{
    Tracker t;
    t.update(-pi + 0.01f, pp);
    EXPECT_EQ(t.turns, 0);

    t.update(pi - 0.01f, pp);
    EXPECT_EQ(t.turns, -1);
}

// --- Multiple positive turns
TEST_F(PositionTrackerTest, MultiplePositiveTurns)
{
    Tracker t;
    float angle = 0.0f;
    const float step = 0.1f;
    int expected_turns = 0;

    for (int i = 0; i < 400; ++i)
    {
        angle += step;
        if (angle > pi)
        {
            angle -= two_pi;
            ++expected_turns;
        }
        t.update(angle, pp);
    }
    EXPECT_EQ(t.turns, expected_turns);
}

// --- Position formula: (turns * 2π + theta) / pole_pairs − home_offset
TEST_F(PositionTrackerTest, PositionFormula)
{
    Tracker t;
    t.update(pi / 2.0f, pp);  // 0 turns, θ = π/2
    // position_rad = (0 * 2π + π/2) / 4 = π/8
    EXPECT_NEAR(t.position_rad, pi / 8.0f, 1e-5f);
}

// --- set_home() zeroes position
TEST_F(PositionTrackerTest, SetHomeClearsPosition)
{
    Tracker t;
    t.update(pi / 2.0f, pp);
    t.set_home(pp);
    EXPECT_FLOAT_EQ(t.position_rad, 0.0f);
    EXPECT_FLOAT_EQ(t.position_rev, 0.0f);
    EXPECT_TRUE(t.is_homed);
}

// --- After homing, position is relative to home
TEST_F(PositionTrackerTest, PositionRelativeToHome)
{
    Tracker t;
    // Home at θ = 0
    t.update(0.0f, pp);
    t.set_home(pp);

    // Advance to θ = π/2 (no wrap)
    t.update(pi / 2.0f, pp);
    // expected: (0*2π + π/2)/4 − home_offset_rad
    // home_offset_rad = (0*2π + 0)/4 = 0
    EXPECT_NEAR(t.position_rad, pi / 8.0f, 1e-5f);
}

// --- 4096 mechanical revolutions with 1 pole-pair
TEST_F(PositionTrackerTest, SupportsFourThousandRevolutions)
{
    Tracker t;
    float angle = 0.0f;
    const float step = 0.01f;  // [rad]
    const int pole_pairs = 1;

    // Simulate forward until turns == 4096
    while (t.turns < 4096)
    {
        float next = angle + step;
        if (next > pi) next -= two_pi;
        t.update(next, pole_pairs);
        angle = next;
    }
    EXPECT_EQ(t.turns, 4096);
}

// --- reset() clears all state
TEST_F(PositionTrackerTest, ResetClearsAll)
{
    Tracker t;
    t.update(pi / 2.0f, pp);
    t.set_home(pp);
    t.update(1.0f, pp);
    t.reset();
    EXPECT_EQ(t.turns,        0);
    EXPECT_FLOAT_EQ(t.position_rad, 0.0f);
    EXPECT_FALSE(t.is_homed);
}

}  // namespace test
}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_TEST_POSITION_TRACKER_H_ */
