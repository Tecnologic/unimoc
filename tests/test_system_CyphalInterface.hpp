/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifndef UNIMOC_TEST_CYPHAL_INTERFACE_H_
#define UNIMOC_TEST_CYPHAL_INTERFACE_H_

#include <gtest/gtest.h>
#include "CyphalInterface.hpp"

namespace unimoc
{
namespace system
{
namespace cyphal
{
namespace test
{

class CyphalInterfaceTest : public ::testing::Test {};

// --- Standard command codes match Cyphal specification §5.3.4 ---

TEST_F(CyphalInterfaceTest, StandardCommandRestart)
{
    EXPECT_EQ(cmd::COMMAND_RESTART, 65535u);
}

TEST_F(CyphalInterfaceTest, StandardCommandPowerOff)
{
    EXPECT_EQ(cmd::COMMAND_POWER_OFF, 65534u);
}

TEST_F(CyphalInterfaceTest, StandardCommandBeginSoftwareUpdate)
{
    EXPECT_EQ(cmd::COMMAND_BEGIN_SOFTWARE_UPDATE, 65533u);
}

TEST_F(CyphalInterfaceTest, StandardCommandFactoryReset)
{
    EXPECT_EQ(cmd::COMMAND_FACTORY_RESET, 65532u);
}

TEST_F(CyphalInterfaceTest, StandardCommandEmergencyStop)
{
    EXPECT_EQ(cmd::COMMAND_EMERGENCY_STOP, 65531u);
}

TEST_F(CyphalInterfaceTest, StandardCommandStorePersistentStates)
{
    EXPECT_EQ(cmd::COMMAND_STORE_PERSISTENT_STATES, 65530u);
}

// --- Standard command codes are in the reserved range (≥ 0xFF00) ---

TEST_F(CyphalInterfaceTest, StandardCommandsInReservedRange)
{
    EXPECT_GE(cmd::COMMAND_RESTART,                 0xFF00u);
    EXPECT_GE(cmd::COMMAND_POWER_OFF,               0xFF00u);
    EXPECT_GE(cmd::COMMAND_BEGIN_SOFTWARE_UPDATE,   0xFF00u);
    EXPECT_GE(cmd::COMMAND_FACTORY_RESET,           0xFF00u);
    EXPECT_GE(cmd::COMMAND_EMERGENCY_STOP,          0xFF00u);
    EXPECT_GE(cmd::COMMAND_STORE_PERSISTENT_STATES, 0xFF00u);
}

// --- Vendor-specific measurement commands are in the vendor range (< 0x8000) ---

TEST_F(CyphalInterfaceTest, MeasurementCommandsInVendorRange)
{
    EXPECT_LT(cmd::CMD_GET_ROTOR_ANGLE,        0x8000u);
    EXPECT_LT(cmd::CMD_GET_ROTOR_SPEED,        0x8000u);
    EXPECT_LT(cmd::CMD_GET_SHAFT_POSITION,     0x8000u);
    EXPECT_LT(cmd::CMD_GET_IN_POSITION,        0x8000u);
    EXPECT_LT(cmd::CMD_GET_HOMING_STATE,       0x8000u);
    EXPECT_LT(cmd::CMD_GET_DC_VOLTAGE,         0x8000u);
    EXPECT_LT(cmd::CMD_GET_PHASE_CURRENT,      0x8000u);
    EXPECT_LT(cmd::CMD_GET_EXCITATION_CURRENT, 0x8000u);
}

// --- All vendor-specific measurement commands are non-zero ---

TEST_F(CyphalInterfaceTest, MeasurementCommandsNonZero)
{
    EXPECT_NE(cmd::CMD_GET_ROTOR_ANGLE,        0u);
    EXPECT_NE(cmd::CMD_GET_ROTOR_SPEED,        0u);
    EXPECT_NE(cmd::CMD_GET_SHAFT_POSITION,     0u);
    EXPECT_NE(cmd::CMD_GET_IN_POSITION,        0u);
    EXPECT_NE(cmd::CMD_GET_HOMING_STATE,       0u);
    EXPECT_NE(cmd::CMD_GET_DC_VOLTAGE,         0u);
    EXPECT_NE(cmd::CMD_GET_PHASE_CURRENT,      0u);
    EXPECT_NE(cmd::CMD_GET_EXCITATION_CURRENT, 0u);
}

// --- All vendor-specific measurement commands are unique ---

TEST_F(CyphalInterfaceTest, MeasurementCommandsUnique)
{
    const uint16_t codes[] = {
        cmd::CMD_GET_ROTOR_ANGLE,
        cmd::CMD_GET_ROTOR_SPEED,
        cmd::CMD_GET_SHAFT_POSITION,
        cmd::CMD_GET_IN_POSITION,
        cmd::CMD_GET_HOMING_STATE,
        cmd::CMD_GET_DC_VOLTAGE,
        cmd::CMD_GET_PHASE_CURRENT,
        cmd::CMD_GET_EXCITATION_CURRENT,
    };
    constexpr std::size_t N = sizeof(codes) / sizeof(codes[0]);
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = i + 1; j < N; ++j)
        {
            EXPECT_NE(codes[i], codes[j])
                << "Duplicate command code at indices " << i << " and " << j;
        }
    }
}

// --- Identification result publisher port IDs ---

TEST_F(CyphalInterfaceTest, MeasResultPortIdsAboveTelemetry)
{
    // All measurement result ports must be above the last telemetry port (207)
    EXPECT_GT(PUB_MEAS_RS,      PUB_EXCITATION_CURRENT);
    EXPECT_GT(PUB_MEAS_LD_LQ,   PUB_EXCITATION_CURRENT);
    EXPECT_GT(PUB_MEAS_PSI,     PUB_EXCITATION_CURRENT);
    EXPECT_GT(PUB_MEAS_BALANCE, PUB_EXCITATION_CURRENT);
}

TEST_F(CyphalInterfaceTest, MeasResultPortIdsUnique)
{
    const uint16_t ports[] = {
        PUB_MEAS_RS,
        PUB_MEAS_LD_LQ,
        PUB_MEAS_PSI,
        PUB_MEAS_BALANCE,
    };
    constexpr std::size_t N = sizeof(ports) / sizeof(ports[0]);
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = i + 1; j < N; ++j)
        {
            EXPECT_NE(ports[i], ports[j])
                << "Duplicate port ID at indices " << i << " and " << j;
        }
    }
}

// --- Motor identification command codes ---

TEST_F(CyphalInterfaceTest, IdentificationCommandsInVendorRange)
{
    EXPECT_LT(cmd::CMD_MEASURE_RS,       0x8000u);
    EXPECT_LT(cmd::CMD_MEASURE_LD_LQ,    0x8000u);
    EXPECT_LT(cmd::CMD_MEASURE_PSI,      0x8000u);
    EXPECT_LT(cmd::CMD_MEASURE_BALANCE,  0x8000u);
}

TEST_F(CyphalInterfaceTest, IdentificationCommandsInExpectedSubrange)
{
    // CMD_MEASURE_* commands live in 0x0100..0x01FF
    EXPECT_GE(cmd::CMD_MEASURE_RS,      0x0100u);
    EXPECT_GE(cmd::CMD_MEASURE_LD_LQ,   0x0100u);
    EXPECT_GE(cmd::CMD_MEASURE_PSI,     0x0100u);
    EXPECT_GE(cmd::CMD_MEASURE_BALANCE, 0x0100u);
    EXPECT_LE(cmd::CMD_MEASURE_RS,      0x01FFu);
    EXPECT_LE(cmd::CMD_MEASURE_LD_LQ,   0x01FFu);
    EXPECT_LE(cmd::CMD_MEASURE_PSI,     0x01FFu);
    EXPECT_LE(cmd::CMD_MEASURE_BALANCE, 0x01FFu);
}

TEST_F(CyphalInterfaceTest, IdentificationCommandsUnique)
{
    const uint16_t codes[] = {
        cmd::CMD_MEASURE_RS,
        cmd::CMD_MEASURE_LD_LQ,
        cmd::CMD_MEASURE_PSI,
        cmd::CMD_MEASURE_BALANCE,
    };
    constexpr std::size_t N = sizeof(codes) / sizeof(codes[0]);
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = i + 1; j < N; ++j)
        {
            EXPECT_NE(codes[i], codes[j])
                << "Duplicate command code at indices " << i << " and " << j;
        }
    }
}

TEST_F(CyphalInterfaceTest, IdentificationCommandsDontOverlapGetCommands)
{
    // No CMD_MEASURE_* should share a value with any CMD_GET_*
    const uint16_t get_codes[] = {
        cmd::CMD_GET_ROTOR_ANGLE,
        cmd::CMD_GET_ROTOR_SPEED,
        cmd::CMD_GET_SHAFT_POSITION,
        cmd::CMD_GET_IN_POSITION,
        cmd::CMD_GET_HOMING_STATE,
        cmd::CMD_GET_DC_VOLTAGE,
        cmd::CMD_GET_PHASE_CURRENT,
        cmd::CMD_GET_EXCITATION_CURRENT,
    };
    const uint16_t measure_codes[] = {
        cmd::CMD_MEASURE_RS,
        cmd::CMD_MEASURE_LD_LQ,
        cmd::CMD_MEASURE_PSI,
        cmd::CMD_MEASURE_BALANCE,
    };
    for (auto g : get_codes)
    {
        for (auto m : measure_codes)
        {
            EXPECT_NE(g, m);
        }
    }
}

// --- Parameter bitmask flags ---

TEST_F(CyphalInterfaceTest, ParamApplyToRamIsBit0)
{
    EXPECT_EQ(cmd::PARAM_APPLY_TO_RAM,   0x0001u);
}

TEST_F(CyphalInterfaceTest, ParamPersistToNvmIsBit1)
{
    EXPECT_EQ(cmd::PARAM_PERSIST_TO_NVM, 0x0002u);
}

TEST_F(CyphalInterfaceTest, ParamFlagsAreDistinctBits)
{
    // The two flags must not share any bit
    EXPECT_EQ(cmd::PARAM_APPLY_TO_RAM & cmd::PARAM_PERSIST_TO_NVM, 0u);
}

// --- Response status codes ---

TEST_F(CyphalInterfaceTest, StatusSuccess)
{
    EXPECT_EQ(cmd::STATUS_SUCCESS,        0u);
}

TEST_F(CyphalInterfaceTest, StatusCodesUnique)
{
    const uint8_t statuses[] = {
        cmd::STATUS_SUCCESS,
        cmd::STATUS_FAILURE,
        cmd::STATUS_NOT_AUTHORIZED,
        cmd::STATUS_BAD_STATE,
        cmd::STATUS_BAD_PARAMETER,
        cmd::STATUS_BAD_COMMAND,
    };
    constexpr std::size_t N = sizeof(statuses) / sizeof(statuses[0]);
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = i + 1; j < N; ++j)
        {
            EXPECT_NE(statuses[i], statuses[j])
                << "Duplicate status code at indices " << i << " and " << j;
        }
    }
}

}  // namespace test
}  // namespace cyphal
}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_TEST_CYPHAL_INTERFACE_H_ */
