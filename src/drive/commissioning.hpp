/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file commissioning.hpp
 *    @brief Commissioning over Cyphal: the commands, the status message and the results of the startup aid.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <unimoc/drive/CommissioningStatus_0_1.h>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include "cyphal/node.hpp"
#include "hw_startup.hpp"

namespace unimoc::drive {

/**
 * @brief The commissioning interface of the drive (docs/motor_identification_plan.md, section 5.4).
 *
 * - **Actions** are ExecuteCommand codes (COMMAND_* of
 *   unimoc.drive.CommissioningStatus.0.1): start the startup aid (refused
 *   while engaged or after an emergency stop), continue with the confirmation
 *   the step waits for, a meter reading ("vdc=48.1", "current=0.85"), abort.
 *   The answer says at once whether the action was taken, with the reason in
 *   the output text.
 * - **Progress and completion** are the status message, published on every
 *   change and at 2 Hz while a run is active, at 0.5 Hz otherwise.
 * - **Values** stay registers: `unimoc.startup.results` (natural8[14], read
 *   only, 1 per passed step) here, the calibration among the persistent
 *   settings (`unimoc.startup.adc_offset_*`, `gain_*`).
 *
 * The provider runs in the Cyphal thread, like the FSM it controls.
 */
class Commissioning final : public cyphal::RegisterProvider {
 public:
  /// Period of the status message while a run is active.
  static constexpr uint64_t kActivePeriodUs = 500'000U;
  /// Period of the status message otherwise.
  static constexpr uint64_t kIdlePeriodUs = 2'000'000U;

  /**
   * @param fsm Startup FSM; must outlive the provider.
   * @param engaged True while the drive is engaged; a start is refused then.
   * @param emergency_stop True while an emergency stop is latched; a start is refused then.
   */
  Commissioning(startup::HwStartup& fsm, const bool& engaged, const bool& emergency_stop) noexcept
      : fsm_{&fsm}, engaged_{&engaged}, emergency_stop_{&emergency_stop} {}

  /// True for the command codes this interface takes.
  [[nodiscard]] static bool Handles(uint16_t command) noexcept;

  /// Executes one commissioning command.
  [[nodiscard]] cyphal::CommandResult OnCommand(uint16_t command,
                                                std::span<const uint8_t> parameter) noexcept;

  /// The status message of the present state.
  [[nodiscard]] unimoc_drive_CommissioningStatus_0_1 Status() const noexcept;

  /**
   * @brief True when the status message is due: it changed, or its period passed.
   * @param now_us Present time.
   * @param status Receives the message when due.
   */
  [[nodiscard]] bool Due(uint64_t now_us, unimoc_drive_CommissioningStatus_0_1& status) noexcept;

  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;
  [[nodiscard]] cyphal::RegisterDescription Describe(std::size_t index) const noexcept override;

 private:
  [[nodiscard]] cyphal::CommandResult Start() noexcept;
  [[nodiscard]] cyphal::CommandResult Continue(std::string_view confirmation) noexcept;
  [[nodiscard]] cyphal::CommandResult SetReading(std::string_view reading) noexcept;

  startup::HwStartup* fsm_;
  const bool* engaged_;
  const bool* emergency_stop_;
  unimoc_drive_CommissioningStatus_0_1 last_{};
  bool published_{false};
  uint64_t next_us_{0U};
};

}  // namespace unimoc::drive
