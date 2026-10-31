/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_interface_test.cpp
 *    @brief ztest coverage for Cyphal identifiers and command ranges.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

/**
 * @file test_cyphal_interface.hpp
 * @brief Test fixture and cases for Cyphal interface identifiers.
 */
#include "cyphal/cyphal_interface.hpp"
#include <unimoc/drive/CommissioningStatus_0_1.h>
#include <zephyr/ztest.h>
#include <array>
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {

// --- Standard command codes match Cyphal specification §5.3.4 ---

ZTEST(cyphal_interface, test_standard_command_restart) { zexpect_equal(cmd::kCommandRestart, 65535U); }

ZTEST(cyphal_interface, test_standard_command_power_off) { zexpect_equal(cmd::kCommandPowerOff, 65534U); }

ZTEST(cyphal_interface, test_standard_command_begin_software_update) {
  zexpect_equal(cmd::kCommandBeginSoftwareUpdate, 65533U);
}

ZTEST(cyphal_interface, test_standard_command_factory_reset) {
  zexpect_equal(cmd::kCommandFactoryReset, 65532U);
}

ZTEST(cyphal_interface, test_standard_command_emergency_stop) {
  zexpect_equal(cmd::kCommandEmergencyStop, 65531U);
}

ZTEST(cyphal_interface, test_standard_command_store_persistent_states) {
  zexpect_equal(cmd::kCommandStorePersistentStates, 65530U);
}

// --- Standard command codes are in the reserved range (≥ 0xFF00) ---

ZTEST(cyphal_interface, test_standard_commands_in_reserved_range) {
  zexpect_true((cmd::kCommandRestart) >= (0xFF00U));
  zexpect_true((cmd::kCommandPowerOff) >= (0xFF00U));
  zexpect_true((cmd::kCommandBeginSoftwareUpdate) >= (0xFF00U));
  zexpect_true((cmd::kCommandFactoryReset) >= (0xFF00U));
  zexpect_true((cmd::kCommandEmergencyStop) >= (0xFF00U));
  zexpect_true((cmd::kCommandStorePersistentStates) >= (0xFF00U));
}

// --- Vendor-specific diagnostics commands ---

ZTEST(cyphal_interface, test_diagnostics_commands_in_their_range) {
  zexpect_equal(cmd::kCmdResetTiming, 0x0300U);  // above the retired codes and the commissioning
}

// --- Commissioning (startup aid, motor identification) ---

ZTEST(cyphal_interface, test_commissioning_port_after_the_drive_status) {
  zexpect_equal(kPubCommissioning, 215U);
  zexpect_true(kPubCommissioning > kPubDriveStatus);
}

ZTEST(cyphal_interface, test_commissioning_commands_in_their_range) {
  const std::array<uint16_t, 6> kCodes = {
      unimoc_drive_CommissioningStatus_0_1_COMMAND_START_STARTUP_AID,
      unimoc_drive_CommissioningStatus_0_1_COMMAND_START_IDENTIFICATION,
      unimoc_drive_CommissioningStatus_0_1_COMMAND_CONTINUE,
      unimoc_drive_CommissioningStatus_0_1_COMMAND_SET_READING,
      unimoc_drive_CommissioningStatus_0_1_COMMAND_ABORT,
      unimoc_drive_CommissioningStatus_0_1_COMMAND_APPLY_IDENTIFICATION,
  };
  for (std::size_t i = 0; i < kCodes.size(); ++i) {
    zexpect_true(kCodes[i] >= 0x0200U && kCodes[i] <= 0x02FFU);
    for (std::size_t j = i + 1; j < kCodes.size(); ++j) {
      zexpect_not_equal(kCodes[i], kCodes[j]);
    }
  }
}

// --- Response status codes ---

ZTEST(cyphal_interface, test_status_success) { zexpect_equal(cmd::kStatusSuccess, 0U); }

// Values of uavcan.node.ExecuteCommand.1.3 Response.STATUS_*.
ZTEST(cyphal_interface, test_status_codes_match_the_specification) {
  zexpect_equal(cmd::kStatusFailure, 1U);
  zexpect_equal(cmd::kStatusNotAuthorized, 2U);
  zexpect_equal(cmd::kStatusBadCommand, 3U);
  zexpect_equal(cmd::kStatusBadParameter, 4U);
  zexpect_equal(cmd::kStatusBadState, 5U);
  zexpect_equal(cmd::kStatusInternalError, 6U);
}

ZTEST(cyphal_interface, test_status_codes_unique) {
  const std::array<uint8_t, 6> kStatuses = {
      cmd::kStatusSuccess,
      cmd::kStatusFailure,
      cmd::kStatusNotAuthorized,
      cmd::kStatusBadState,
      cmd::kStatusBadParameter,
      cmd::kStatusBadCommand,
  };
  constexpr std::size_t kCount = kStatuses.size();
  for (std::size_t i = 0; i < kCount; ++i) {
    for (std::size_t j = i + 1; j < kCount; ++j) {
      zexpect_not_equal(kStatuses[i],
                        kStatuses[j],
                        "Duplicate status code at indices %d and %d",
                        static_cast<int>(i),
                        static_cast<int>(j));
    }
  }
}

}  // namespace unimoc::cyphal::test

ZTEST_SUITE(cyphal_interface, nullptr, nullptr, nullptr, nullptr, nullptr);
