/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file startup_registers.hpp
 *    @brief Cyphal registers of the hardware startup aid (unimoc.startup.*).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstddef>
#include <string_view>
#include "cyphal/node.hpp"
#include "hw_startup.hpp"

namespace unimoc::drive {

/**
 * @brief Runs and steps the startup FSM (`HwStartup`) over uavcan.register.
 *
 * - `unimoc.startup.run` (natural8[1]): read 1 while the FSM runs; write 1
 *   starts it (refused while engaged), 0 aborts it.
 * - `unimoc.startup.step` (natural8[1]): read the FsmState; write 1 advances
 *   to the next step.
 * - `unimoc.startup.ext_current_A` (real32[1]): clamp-meter reading for
 *   CURRENT_SENSE_CALIBRATION.
 * - `unimoc.startup.ext_vdc_V` (real32[1]): multimeter reading for
 *   DC_LINK_VOLTAGE_CHECK.
 * - `unimoc.startup.results` (natural8[14], read only): 1 per passed step,
 *   indexed by FsmState.
 *
 * The calibration results (`unimoc.startup.adc_offset_*`, `gain_*`) are
 * persistent settings registers; the FSM writes them at DONE. None of these
 * registers is persistent. The provider runs in the Cyphal thread, like the
 * FSM it controls.
 */
class StartupRegisters final : public cyphal::RegisterProvider {
 public:
  /**
   * @param fsm Startup FSM; must outlive the provider.
   * @param engaged True while the drive is engaged; a run is refused then.
   */
  StartupRegisters(startup::HwStartup& fsm, const bool& engaged) noexcept : fsm_{&fsm}, engaged_{&engaged} {}

  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;

 private:
  startup::HwStartup* fsm_;
  const bool* engaged_;
  float ext_current_{0.0F};
  float ext_vdc_{0.0F};
};

}  // namespace unimoc::drive
