/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file startup_registers.cpp
 *    @brief Cyphal registers of the hardware startup aid (unimoc.startup.*).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "startup_registers.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/register_value.hpp"

namespace unimoc::drive {

namespace {

enum class StartupRegister : uint8_t { kRun, kStep, kExtCurrent, kExtVdc, kResults };

constexpr std::array<std::string_view, 5> kNames{
    cyphal::reg::kStartupRun,
    cyphal::reg::kStartupStep,
    cyphal::reg::kStartupExtCurrentA,
    cyphal::reg::kStartupExtVdcV,
    cyphal::reg::kStartupResults,
};

using cyphal::RegisterEffect;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr std::array<cyphal::RegisterDescription, kNames.size()> kDescriptions{{
    {{},
     0.0F,
     1.0F,
     RegisterEffect::kImmediate,
     "1 starts the startup aid (refused while engaged), 0 aborts it"},
    {{}, 1.0F, 1.0F, RegisterEffect::kImmediate, "Startup aid step; 1 continues with the next step"},
    {"A", -kInf, kInf, RegisterEffect::kImmediate, "Externally measured current of the running step"},
    {"V", -kInf, kInf, RegisterEffect::kImmediate, "Externally measured DC-link voltage of the running step"},
    {{},
     -kInf,
     kInf,
     RegisterEffect::kImmediate,
     "Startup aid result per step: 1 passed, 0 failed or not run"},
}};

}  // namespace

std::size_t StartupRegisters::Count() const noexcept { return kNames.size(); }

std::string_view StartupRegisters::Name(const std::size_t index) const noexcept {
  return index < kNames.size() ? kNames[index] : std::string_view{};
}

cyphal::RegisterDescription StartupRegisters::Describe(const std::size_t index) const noexcept {
  return index < kDescriptions.size() ? kDescriptions[index] : cyphal::RegisterDescription{};
}

cyphal::RegisterFlags StartupRegisters::Read(const std::size_t index,
                                             cyphal::RegisterValue& value) const noexcept {
  switch (static_cast<StartupRegister>(index)) {
    case StartupRegister::kRun:
      cyphal::SetNatural8(value, fsm_->IsActive() ? 1U : 0U);
      break;
    case StartupRegister::kStep:
      cyphal::SetNatural8(value, static_cast<uint8_t>(fsm_->State()));
      break;
    case StartupRegister::kExtCurrent:
      cyphal::SetReal32(value, ext_current_);
      break;
    case StartupRegister::kExtVdc:
      cyphal::SetReal32(value, ext_vdc_);
      break;
    case StartupRegister::kResults: {
      uavcan_register_Value_1_0_select_natural8_(&value);
      const auto& passed = fsm_->results.passed;
      for (std::size_t step = 0U; step < passed.size(); ++step) {
        value.natural8.value.elements[step] = passed[step] ? 1U : 0U;
      }
      value.natural8.value.count = passed.size();
      return {false, false};
    }
    default:
      cyphal::SetEmpty(value);
      return {false, false};
  }
  return {true, false};
}

cyphal::RegisterWriteResult StartupRegisters::Write(const std::size_t index,
                                                    const cyphal::RegisterValue& value) noexcept {
  if (index >= kNames.size() || static_cast<StartupRegister>(index) == StartupRegister::kResults) {
    return cyphal::RegisterWriteResult::kReadOnly;
  }
  const std::optional<float> kNumber = cyphal::GetNumber(value);
  if (!kNumber.has_value()) {
    return cyphal::RegisterWriteResult::kTypeMismatch;
  }
  const float kValue = *kNumber;
  if (!std::isfinite(kValue)) {
    return cyphal::RegisterWriteResult::kOutOfRange;
  }
  switch (static_cast<StartupRegister>(index)) {
    case StartupRegister::kRun:
      if (kValue == 0.0F) {
        fsm_->RequestAbort();
        return cyphal::RegisterWriteResult::kAccepted;
      }
      if (kValue != 1.0F || *engaged_) {
        return cyphal::RegisterWriteResult::kOutOfRange;  // the outputs belong to the setpoints
      }
      if (!fsm_->IsActive()) {
        ext_current_ = 0.0F;  // a new run starts without external measurements
        ext_vdc_ = 0.0F;
        fsm_->RequestRun();
      }
      return cyphal::RegisterWriteResult::kAccepted;
    case StartupRegister::kStep:
      if (kValue != 1.0F || !fsm_->IsActive()) {
        return cyphal::RegisterWriteResult::kOutOfRange;
      }
      fsm_->RequestNextStep();
      return cyphal::RegisterWriteResult::kAccepted;
    case StartupRegister::kExtCurrent:
      ext_current_ = kValue;
      fsm_->SetExtCurrent(kValue);
      return cyphal::RegisterWriteResult::kAccepted;
    case StartupRegister::kExtVdc:
      ext_vdc_ = kValue;
      fsm_->SetExtVdc(kValue);
      return cyphal::RegisterWriteResult::kAccepted;
    default:
      return cyphal::RegisterWriteResult::kReadOnly;
  }
}

}  // namespace unimoc::drive
