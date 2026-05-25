#pragma once

#ifndef UNIMOC_TEST_NVM_SETTINGS_H_
#define UNIMOC_TEST_NVM_SETTINGS_H_

#include <gtest/gtest.h>
#include "NvmSettings.hpp"

namespace unimoc
{
namespace system
{
namespace test
{

class NvmSettingsTest : public ::testing::Test {};

// --- Default construction is valid
TEST_F(NvmSettingsTest, DefaultIsValid)
{
    NvmSettings s;
    EXPECT_TRUE(s.is_valid());
}

TEST_F(NvmSettingsTest, DefaultMagicAndVersion)
{
    NvmSettings s;
    EXPECT_EQ(s.magic,   NVM_MAGIC);
    EXPECT_EQ(s.version, NVM_VERSION);
}

// --- Default node_id == 0 triggers PnP
TEST_F(NvmSettingsTest, DefaultNodeIdZero)
{
    NvmSettings s;
    EXPECT_EQ(s.node_id, 0u);
}

// --- Default motor type is PMSM
TEST_F(NvmSettingsTest, DefaultMotorType)
{
    NvmSettings s;
    EXPECT_EQ(s.motor_type, MotorType::PMSM);
}

// --- Default control mode is TORQUE
TEST_F(NvmSettingsTest, DefaultControlMode)
{
    NvmSettings s;
    EXPECT_EQ(s.control_mode, ControlMode::TORQUE);
}

// --- Corrupt magic invalidates the block
TEST_F(NvmSettingsTest, CorruptMagicInvalid)
{
    NvmSettings s;
    s.magic = 0xDEADBEEFu;
    EXPECT_FALSE(s.is_valid());
}

// --- Corrupt version invalidates the block
TEST_F(NvmSettingsTest, CorruptVersionInvalid)
{
    NvmSettings s;
    s.version = 0xFFFFu;
    EXPECT_FALSE(s.is_valid());
}

// --- reset_to_defaults restores valid state
TEST_F(NvmSettingsTest, ResetToDefaultsIsValid)
{
    NvmSettings s;
    s.magic = 0u;
    s.version = 0u;
    s.reset_to_defaults();
    EXPECT_TRUE(s.is_valid());
}

// --- Identity defaults are preserved
TEST_F(NvmSettingsTest, IdentityDefaultName)
{
    NvmSettings s;
    EXPECT_EQ(s.identity.get_name(), std::string_view("unimoc"));
}

// --- Node ID can be updated to valid range
TEST_F(NvmSettingsTest, NodeIdCanBeSet)
{
    NvmSettings s;
    s.node_id = 42u;
    EXPECT_EQ(s.node_id, 42u);
    EXPECT_TRUE(s.is_valid());
}

// --- Parameter spot-checks (verify fields exist + have sensible defaults)
TEST_F(NvmSettingsTest, DefaultStatorR)
{
    NvmSettings s;
    EXPECT_FLOAT_EQ(s.stator_R, 0.1f);
}

TEST_F(NvmSettingsTest, DefaultFwVMax)
{
    NvmSettings s;
    EXPECT_FLOAT_EQ(s.fw_v_max, 0.9f);
}

TEST_F(NvmSettingsTest, DefaultSvmDutyRange)
{
    NvmSettings s;
    EXPECT_LT(s.svm_duty_min, s.svm_duty_max);
}

TEST_F(NvmSettingsTest, DefaultPosSpeedLimit)
{
    NvmSettings s;
    EXPECT_GT(s.pos_speed_limit, 0.0f);
}

TEST_F(NvmSettingsTest, DefaultExcitationMode)
{
    NvmSettings s;
    EXPECT_EQ(s.excitation_mode, 0u);  // CurrentMode
}

// --- Phase current balance defaults ---

TEST_F(NvmSettingsTest, DefaultPhaseBalanceA)
{
    NvmSettings s;
    EXPECT_FLOAT_EQ(s.phase_balance_a, 1.0f);
}

TEST_F(NvmSettingsTest, DefaultPhaseBalanceB)
{
    NvmSettings s;
    EXPECT_FLOAT_EQ(s.phase_balance_b, 1.0f);
}

TEST_F(NvmSettingsTest, DefaultPhaseBalanceC)
{
    NvmSettings s;
    EXPECT_FLOAT_EQ(s.phase_balance_c, 1.0f);
}

TEST_F(NvmSettingsTest, PhaseBalanceCanBeModified)
{
    NvmSettings s;
    s.phase_balance_a = 1.02f;
    s.phase_balance_b = 0.98f;
    s.phase_balance_c = 1.00f;
    EXPECT_FLOAT_EQ(s.phase_balance_a, 1.02f);
    EXPECT_FLOAT_EQ(s.phase_balance_b, 0.98f);
    EXPECT_FLOAT_EQ(s.phase_balance_c, 1.00f);
    EXPECT_TRUE(s.is_valid());
}

TEST_F(NvmSettingsTest, ResetRestoresPhaseBalanceDefaults)
{
    NvmSettings s;
    s.phase_balance_a = 1.05f;
    s.phase_balance_b = 0.95f;
    s.phase_balance_c = 0.99f;
    s.reset_to_defaults();
    EXPECT_FLOAT_EQ(s.phase_balance_a, 1.0f);
    EXPECT_FLOAT_EQ(s.phase_balance_b, 1.0f);
    EXPECT_FLOAT_EQ(s.phase_balance_c, 1.0f);
}

// --- NVM version is 2 after layout change ---

TEST_F(NvmSettingsTest, NvmVersionIsTwo)
{
    EXPECT_EQ(NVM_VERSION, 2u);
    NvmSettings s;
    EXPECT_EQ(s.version, 2u);
}

}  // namespace test
}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_TEST_NVM_SETTINGS_H_ */
