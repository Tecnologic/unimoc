#pragma once

#ifndef UNIMOC_TEST_EXCITATION_OBSERVER_H_
#define UNIMOC_TEST_EXCITATION_OBSERVER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "ExcitationObserver.hpp"

namespace unimoc
{
namespace observer
{
namespace test
{

class ExcitationObserverTest : public ::testing::Test
{
protected:
    using Obs = ExcitationObserver<float>;
};

// --- Default-constructed observer has zero outputs
TEST_F(ExcitationObserverTest, DefaultStateIsZero)
{
    Obs o;
    EXPECT_FLOAT_EQ(o.i_f_hat,   0.0f);
    EXPECT_FLOAT_EQ(o.psi_f_hat, 0.0f);
}

// --- Single step moves toward measurement
TEST_F(ExcitationObserverTest, SingleStepConvergesDirection)
{
    Obs o;
    o.tau = 5e-3f;  // 5 ms
    o.L_m = 47e-3f;
    o.update(10.0f, 1e-4f);  // dt = 0.1 ms
    EXPECT_GT(o.i_f_hat, 0.0f);
    EXPECT_LT(o.i_f_hat, 10.0f);
}

// --- After many steps the output converges to the measurement
TEST_F(ExcitationObserverTest, ConvergesAfterManySteps)
{
    Obs o;
    o.tau = 1e-3f;
    o.L_m = 47e-3f;
    for (int i = 0; i < 10000; ++i)
        o.update(8.0f, 1e-5f);  // total time >> tau
    EXPECT_NEAR(o.i_f_hat,   8.0f,            1e-3f);
    EXPECT_NEAR(o.psi_f_hat, 8.0f * 47e-3f,   1e-4f);
}

// --- psi_f_hat = L_m * i_f_hat at every step
TEST_F(ExcitationObserverTest, PsiEqualsLmTimesI)
{
    Obs o;
    o.tau = 2e-3f;
    o.L_m = 50e-3f;
    for (int i = 0; i < 20; ++i)
    {
        o.update(5.0f, 1e-4f);
        EXPECT_NEAR(o.psi_f_hat, o.L_m * o.i_f_hat, 1e-7f);
    }
}

// --- Zero time constant passes measurement through immediately
TEST_F(ExcitationObserverTest, ZeroTauPassThrough)
{
    Obs o;
    o.tau = 0.0f;
    o.L_m = 47e-3f;
    o.update(7.5f, 1e-4f);
    EXPECT_FLOAT_EQ(o.i_f_hat, 7.5f);
}

// --- reset() with init value
TEST_F(ExcitationObserverTest, ResetWithInitValue)
{
    Obs o;
    o.tau = 2e-3f;
    o.L_m = 47e-3f;
    for (int i = 0; i < 100; ++i)
        o.update(5.0f, 1e-4f);
    o.reset(3.0f);
    EXPECT_FLOAT_EQ(o.i_f_hat,   3.0f);
    EXPECT_NEAR(o.psi_f_hat, 3.0f * 47e-3f, 1e-7f);
}

}  // namespace test
}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_TEST_EXCITATION_OBSERVER_H_ */
