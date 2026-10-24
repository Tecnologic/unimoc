/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_save_gate.hpp
 *    @brief Save gate that allows writing settings to flash only while the power stage is disabled.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "power_stage_interface.hpp"
#include "settings_store.hpp"

namespace unimoc::settings {

/**
 * @brief Gate that allows saving settings only while the power stage is disabled.
 *
 * `PowerStageState::kDisabled` only: with a latched fault or a missing
 * external enable the stage is off too, but saving then waits until the
 * state is cleared, like every other transition. While a save runs,
 * `PowerStageInterface::Enable()` is held off and fails.
 *
 * @param power_stage Power stage to watch; it must outlive the gate.
 * @return Gate for `SettingsStore`.
 */
[[nodiscard]] inline SettingsSaveGate PowerStageSaveGate(
    const hardware::PowerStageInterface& power_stage) noexcept {
  return SettingsSaveGate{const_cast<hardware::PowerStageInterface*>(&power_stage),
                          [](void* context) {
                            const auto& stage = *static_cast<const hardware::PowerStageInterface*>(context);
                            if (!stage.HoldOffEnable()) {
                              return false;
                            }
                            if (stage.GetState() != hardware::PowerStageState::kDisabled) {
                              stage.ReleaseEnable();
                              return false;
                            }
                            return true;
                          },
                          [](void* context) {
                            static_cast<const hardware::PowerStageInterface*>(context)->ReleaseEnable();
                          }};
}

}  // namespace unimoc::settings
