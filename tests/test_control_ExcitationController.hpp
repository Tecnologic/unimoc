#pragma once

#ifndef UNIMOC_TEST_EXCITATION_CONTROLLER_H_
#define UNIMOC_TEST_EXCITATION_CONTROLLER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "ExcitationController.hpp"

namespace unimoc
{
namespace control
{
namespace test
{

class ExcitationControllerTest : public ::testing::Test
{
protected:
    using Ctrl = ExcitationController<float>;

    Ctrl make_current_mode()
    {
        Ctrl c;
        c.mode     = ExcitationMode::CurrentMode;
        c.kp       = 5.0f;
        c.ki       = 50.0f;
        c.i_f_min  = 0.0f;
        c.i_f_max  = 10.0f;
        c.L_m      = 47e-3f;
        c.setpoint = 0.0f;
        return c;
    }

    Ctrl make_flux_mode()
    {
        Ctrl c = make_current_mode();
        c.mode     = ExcitationMode::FluxMode;
        // ψ_f* = 2.35 Wb  →  I_f* = 2.35 / 0.047 = 50 A (clamped to 10 A)
        c.setpoint = 0.47f;  // ψ_f* [Wb]  → I_f* = 0.47/0.047 = 10 A
        return c;
    }
};

// --- CurrentMode: zero setpoint, zero measurement → zero output
TEST_F(ExcitationControllerTest, CurrentModeZeroError)
{
    auto c = make_current_mode();
    const float out = c.update(0.0f, 1e-4f);
    EXPECT_FLOAT_EQ(out, 0.0f);
}

// --- CurrentMode: positive setpoint drives positive output
TEST_F(ExcitationControllerTest, CurrentModePositiveSetpoint)
{
    auto c     = make_current_mode();
    c.setpoint = 5.0f;  // 5 A
    const float out = c.update(0.0f, 1e-4f);
    EXPECT_GT(out, 0.0f);
    EXPECT_LE(out, c.i_f_max);
}

// --- CurrentMode: integrator saturates at i_f_max
TEST_F(ExcitationControllerTest, CurrentModeIntegratorClamp)
{
    auto c     = make_current_mode();
    c.setpoint = 100.0f;  // Large error
    for (int i = 0; i < 1000; ++i)
        c.update(0.0f, 1e-3f);
    EXPECT_FLOAT_EQ(c.i_f_ref, c.i_f_max);
    EXPECT_LE(c.integrator, c.i_f_max);
}

// --- CurrentMode: output never goes below i_f_min
TEST_F(ExcitationControllerTest, CurrentModeMinClamp)
{
    auto c     = make_current_mode();
    c.setpoint = -100.0f;  // drive negative
    for (int i = 0; i < 1000; ++i)
        c.update(0.0f, 1e-3f);
    EXPECT_FLOAT_EQ(c.i_f_ref, c.i_f_min);
}

// --- FluxMode: ψ_f* / L_m = correct current reference
TEST_F(ExcitationControllerTest, FluxModeConversion)
{
    auto c = make_flux_mode();
    // setpoint = 0.47 Wb, L_m = 0.047 H → I_f* = 10 A (exactly at max)
    for (int i = 0; i < 5000; ++i)
        c.update(0.0f, 1e-4f);
    EXPECT_FLOAT_EQ(c.i_f_ref, c.i_f_max);
}

// --- reset() clears integrator and output
TEST_F(ExcitationControllerTest, ResetClearsState)
{
    auto c     = make_current_mode();
    c.setpoint = 5.0f;
    for (int i = 0; i < 100; ++i)
        c.update(0.0f, 1e-3f);
    c.reset();
    EXPECT_FLOAT_EQ(c.integrator, 0.0f);
    EXPECT_FLOAT_EQ(c.i_f_ref, 0.0f);
}

// --- Output converges to setpoint when feedback equals reference
TEST_F(ExcitationControllerTest, CurrentModeConverges)
{
    auto c     = make_current_mode();
    c.setpoint = 3.0f;
    float meas = 0.0f;

    for (int i = 0; i < 5000; ++i)
    {
        const float ref = c.update(meas, 1e-4f);
        // Simulate first-order plant: meas tracks ref slowly
        meas += 0.01f * (ref - meas);
    }
    EXPECT_NEAR(meas, 3.0f, 0.1f);
}

}  // namespace test
}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_TEST_EXCITATION_CONTROLLER_H_ */
