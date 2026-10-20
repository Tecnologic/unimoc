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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

/**
 * @file test_cyphal_interface.hpp
 * @brief Test fixture and cases for Cyphal interface identifiers.
 */
#include "cyphal/cyphal_interface.hpp"
#include <zephyr/ztest.h>
#include <array>
#include "unimoc_test.hpp"

namespace unimoc::cyphal::test {

// --- Standard command codes match Cyphal specification §5.3.4 ---

ZTEST(cyphal_interface, test_standard_command_restart) { zexpect_equal(cmd::COMMAND_RESTART, 65535U); }

ZTEST(cyphal_interface, test_standard_command_power_off) { zexpect_equal(cmd::COMMAND_POWER_OFF, 65534U); }

ZTEST(cyphal_interface, test_standard_command_begin_software_update) {
  zexpect_equal(cmd::COMMAND_BEGIN_SOFTWARE_UPDATE, 65533U);
}

ZTEST(cyphal_interface, test_standard_command_factory_reset) {
  zexpect_equal(cmd::COMMAND_FACTORY_RESET, 65532U);
}

ZTEST(cyphal_interface, test_standard_command_emergency_stop) {
  zexpect_equal(cmd::COMMAND_EMERGENCY_STOP, 65531U);
}

ZTEST(cyphal_interface, test_standard_command_store_persistent_states) {
  zexpect_equal(cmd::COMMAND_STORE_PERSISTENT_STATES, 65530U);
}

// --- Standard command codes are in the reserved range (≥ 0xFF00) ---

ZTEST(cyphal_interface, test_standard_commands_in_reserved_range) {
  zexpect_true((cmd::COMMAND_RESTART) >= (0xFF00U));
  zexpect_true((cmd::COMMAND_POWER_OFF) >= (0xFF00U));
  zexpect_true((cmd::COMMAND_BEGIN_SOFTWARE_UPDATE) >= (0xFF00U));
  zexpect_true((cmd::COMMAND_FACTORY_RESET) >= (0xFF00U));
  zexpect_true((cmd::COMMAND_EMERGENCY_STOP) >= (0xFF00U));
  zexpect_true((cmd::COMMAND_STORE_PERSISTENT_STATES) >= (0xFF00U));
}

// --- Vendor-specific measurement commands are in the vendor range (< 0x8000) ---

ZTEST(cyphal_interface, test_measurement_commands_in_vendor_range) {
  zexpect_true((cmd::CMD_GET_ROTOR_ANGLE) < (0x8000U));
  zexpect_true((cmd::CMD_GET_ROTOR_SPEED) < (0x8000U));
  zexpect_true((cmd::CMD_GET_SHAFT_POSITION) < (0x8000U));
  zexpect_true((cmd::CMD_GET_IN_POSITION) < (0x8000U));
  zexpect_true((cmd::CMD_GET_HOMING_STATE) < (0x8000U));
  zexpect_true((cmd::CMD_GET_DC_VOLTAGE) < (0x8000U));
  zexpect_true((cmd::CMD_GET_PHASE_CURRENT) < (0x8000U));
  zexpect_true((cmd::CMD_GET_EXCITATION_CURRENT) < (0x8000U));
}

// --- All vendor-specific measurement commands are non-zero ---

ZTEST(cyphal_interface, test_measurement_commands_non_zero) {
  zexpect_not_equal(cmd::CMD_GET_ROTOR_ANGLE, 0U);
  zexpect_not_equal(cmd::CMD_GET_ROTOR_SPEED, 0U);
  zexpect_not_equal(cmd::CMD_GET_SHAFT_POSITION, 0U);
  zexpect_not_equal(cmd::CMD_GET_IN_POSITION, 0U);
  zexpect_not_equal(cmd::CMD_GET_HOMING_STATE, 0U);
  zexpect_not_equal(cmd::CMD_GET_DC_VOLTAGE, 0U);
  zexpect_not_equal(cmd::CMD_GET_PHASE_CURRENT, 0U);
  zexpect_not_equal(cmd::CMD_GET_EXCITATION_CURRENT, 0U);
}

// --- All vendor-specific measurement commands are unique ---

ZTEST(cyphal_interface, test_measurement_commands_unique) {
  const std::array<uint16_t, 8> kCodes = {
      cmd::CMD_GET_ROTOR_ANGLE,
      cmd::CMD_GET_ROTOR_SPEED,
      cmd::CMD_GET_SHAFT_POSITION,
      cmd::CMD_GET_IN_POSITION,
      cmd::CMD_GET_HOMING_STATE,
      cmd::CMD_GET_DC_VOLTAGE,
      cmd::CMD_GET_PHASE_CURRENT,
      cmd::CMD_GET_EXCITATION_CURRENT,
  };
  constexpr std::size_t kCount = kCodes.size();
  for (std::size_t i = 0; i < kCount; ++i) {
    for (std::size_t j = i + 1; j < kCount; ++j) {
      zexpect_not_equal(kCodes[i],
                        kCodes[j],
                        "Duplicate command code at indices %d and %d",
                        static_cast<int>(i),
                        static_cast<int>(j));
    }
  }
}

// --- Identification result publisher port IDs ---

ZTEST(cyphal_interface, test_meas_result_port_ids_above_telemetry) {
  // All measurement result ports must be above the last telemetry port (207)
  zexpect_true((PUB_MEAS_RS) > (PUB_EXCITATION_CURRENT));
  zexpect_true((PUB_MEAS_LD_LQ) > (PUB_EXCITATION_CURRENT));
  zexpect_true((PUB_MEAS_PSI) > (PUB_EXCITATION_CURRENT));
  zexpect_true((PUB_MEAS_BALANCE) > (PUB_EXCITATION_CURRENT));
}

ZTEST(cyphal_interface, test_meas_result_port_ids_unique) {
  const std::array<uint16_t, 4> kPorts = {
      PUB_MEAS_RS,
      PUB_MEAS_LD_LQ,
      PUB_MEAS_PSI,
      PUB_MEAS_BALANCE,
  };
  constexpr std::size_t kCount = kPorts.size();
  for (std::size_t i = 0; i < kCount; ++i) {
    for (std::size_t j = i + 1; j < kCount; ++j) {
      zexpect_not_equal(kPorts[i],
                        kPorts[j],
                        "Duplicate port ID at indices %d and %d",
                        static_cast<int>(i),
                        static_cast<int>(j));
    }
  }
}

// --- Motor identification command codes ---

ZTEST(cyphal_interface, test_identification_commands_in_vendor_range) {
  zexpect_true((cmd::CMD_MEASURE_RS) < (0x8000U));
  zexpect_true((cmd::CMD_MEASURE_LD_LQ) < (0x8000U));
  zexpect_true((cmd::CMD_MEASURE_PSI) < (0x8000U));
  zexpect_true((cmd::CMD_MEASURE_BALANCE) < (0x8000U));
}

ZTEST(cyphal_interface, test_identification_commands_in_expected_subrange) {
  // CMD_MEASURE_* commands live in 0x0100..0x01FF
  zexpect_true((cmd::CMD_MEASURE_RS) >= (0x0100U));
  zexpect_true((cmd::CMD_MEASURE_LD_LQ) >= (0x0100U));
  zexpect_true((cmd::CMD_MEASURE_PSI) >= (0x0100U));
  zexpect_true((cmd::CMD_MEASURE_BALANCE) >= (0x0100U));
  zexpect_true((cmd::CMD_MEASURE_RS) <= (0x01FFU));
  zexpect_true((cmd::CMD_MEASURE_LD_LQ) <= (0x01FFU));
  zexpect_true((cmd::CMD_MEASURE_PSI) <= (0x01FFU));
  zexpect_true((cmd::CMD_MEASURE_BALANCE) <= (0x01FFU));
}

ZTEST(cyphal_interface, test_identification_commands_unique) {
  const std::array<uint16_t, 4> kCodes = {
      cmd::CMD_MEASURE_RS,
      cmd::CMD_MEASURE_LD_LQ,
      cmd::CMD_MEASURE_PSI,
      cmd::CMD_MEASURE_BALANCE,
  };
  constexpr std::size_t kCount = kCodes.size();
  for (std::size_t i = 0; i < kCount; ++i) {
    for (std::size_t j = i + 1; j < kCount; ++j) {
      zexpect_not_equal(kCodes[i],
                        kCodes[j],
                        "Duplicate command code at indices %d and %d",
                        static_cast<int>(i),
                        static_cast<int>(j));
    }
  }
}

ZTEST(cyphal_interface, test_identification_commands_dont_overlap_get_commands) {
  // No CMD_MEASURE_* should share a value with any CMD_GET_*
  const std::array<uint16_t, 8> kGetCodes = {
      cmd::CMD_GET_ROTOR_ANGLE,
      cmd::CMD_GET_ROTOR_SPEED,
      cmd::CMD_GET_SHAFT_POSITION,
      cmd::CMD_GET_IN_POSITION,
      cmd::CMD_GET_HOMING_STATE,
      cmd::CMD_GET_DC_VOLTAGE,
      cmd::CMD_GET_PHASE_CURRENT,
      cmd::CMD_GET_EXCITATION_CURRENT,
  };
  const std::array<uint16_t, 4> kMeasureCodes = {
      cmd::CMD_MEASURE_RS,
      cmd::CMD_MEASURE_LD_LQ,
      cmd::CMD_MEASURE_PSI,
      cmd::CMD_MEASURE_BALANCE,
  };
  for (auto get_code : kGetCodes) {
    for (auto measure_code : kMeasureCodes) {
      zexpect_not_equal(get_code, measure_code);
    }
  }
}

// --- Parameter bitmask flags ---

ZTEST(cyphal_interface, test_param_apply_to_ram_is_bit0) { zexpect_equal(cmd::PARAM_APPLY_TO_RAM, 0x0001U); }

ZTEST(cyphal_interface, test_param_persist_to_nvm_is_bit1) {
  zexpect_equal(cmd::PARAM_PERSIST_TO_NVM, 0x0002U);
}

ZTEST(cyphal_interface, test_param_flags_are_distinct_bits) {
  // The two flags must not share any bit
  zexpect_equal(cmd::PARAM_APPLY_TO_RAM & cmd::PARAM_PERSIST_TO_NVM, 0U);
}

// --- Response status codes ---

ZTEST(cyphal_interface, test_status_success) { zexpect_equal(cmd::STATUS_SUCCESS, 0U); }

ZTEST(cyphal_interface, test_status_codes_unique) {
  const std::array<uint8_t, 6> kStatuses = {
      cmd::STATUS_SUCCESS,
      cmd::STATUS_FAILURE,
      cmd::STATUS_NOT_AUTHORIZED,
      cmd::STATUS_BAD_STATE,
      cmd::STATUS_BAD_PARAMETER,
      cmd::STATUS_BAD_COMMAND,
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
