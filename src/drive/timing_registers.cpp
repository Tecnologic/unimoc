/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file timing_registers.cpp
 *    @brief The execution times of the interrupts as uavcan.register (M0).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "timing_registers.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include "current_control_isr.hpp"
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/register_value.hpp"
#include "hardware_interface_config.hpp"
#include "hot_path_timing.hpp"

namespace unimoc::drive {

namespace {

enum class TimingRegister : uint8_t { kSampleIsr, kControlCycle, kObserver, kOverruns, kClockHz, kReset };

constexpr std::array<std::string_view, 6> kNames{
    cyphal::reg::kTimingSampleIsr,
    cyphal::reg::kTimingControlCycle,
    cyphal::reg::kTimingObserver,
    cyphal::reg::kTimingOverruns,
    cyphal::reg::kTimingClockHz,
    cyphal::reg::kTimingReset,
};

void SetNatural32Array(cyphal::RegisterValue& value, const std::span<const uint32_t> numbers) noexcept {
  uavcan_register_Value_1_0_select_natural32_(&value);
  for (std::size_t i = 0U; i < numbers.size(); ++i) {
    value.natural32.value.elements[i] = numbers[i];
  }
  value.natural32.value.count = numbers.size();
}

void SetStats(cyphal::RegisterValue& value, const hardware::timing::Section section) noexcept {
  const hardware::timing::Stats kStats = hardware::timing::ProbeOf(section).Read();
  const uint32_t kMin = (kStats.count == 0U) ? 0U : kStats.min_cycles;
  const std::array<uint32_t, 4> kNumbers{kMin, kStats.MeanCycles(), kStats.max_cycles, kStats.count};
  SetNatural32Array(value, kNumbers);
}

}  // namespace

std::size_t TimingRegisters::Count() const noexcept { return kNames.size(); }

std::string_view TimingRegisters::Name(const std::size_t index) const noexcept {
  return index < kNames.size() ? kNames[index] : std::string_view{};
}

cyphal::RegisterFlags TimingRegisters::Read(const std::size_t index,
                                            cyphal::RegisterValue& value) const noexcept {
  switch (static_cast<TimingRegister>(index)) {
    case TimingRegister::kSampleIsr:
      SetStats(value, hardware::timing::Section::kSampleIsr);
      break;
    case TimingRegister::kControlCycle:
      SetStats(value, hardware::timing::Section::kControlCycle);
      break;
    case TimingRegister::kObserver:
      SetStats(value, hardware::timing::Section::kObserver);
      break;
    case TimingRegister::kOverruns: {
      const std::array<uint32_t, 2> kNumbers{
          hardware::timing::DutyOverruns(),
          current_control::CurrentControlIsr::Instance().state.observer_overruns};
      SetNatural32Array(value, kNumbers);
      break;
    }
    case TimingRegister::kClockHz:
      cyphal::SetNatural32(value, hardware::kTimestampFrequency);
      break;
    case TimingRegister::kReset:
      cyphal::SetNatural8(value, 0U);
      return {true, false};
    default:
      cyphal::SetEmpty(value);
      return {false, false};
  }
  return {false, false};
}

cyphal::RegisterWriteResult TimingRegisters::Write(const std::size_t index,
                                                   const cyphal::RegisterValue& value) noexcept {
  if (index >= kNames.size()) {
    return cyphal::RegisterWriteResult::kReadOnly;
  }
  if (static_cast<TimingRegister>(index) != TimingRegister::kReset) {
    return cyphal::RegisterWriteResult::kReadOnly;
  }
  const std::optional<int64_t> kNumber = cyphal::GetInteger(value);
  if (!kNumber.has_value()) {
    return cyphal::RegisterWriteResult::kTypeMismatch;
  }
  if (*kNumber != 1) {
    return cyphal::RegisterWriteResult::kOutOfRange;
  }
  hardware::timing::ProbeOf(hardware::timing::Section::kSampleIsr).Reset();
  hardware::timing::ProbeOf(hardware::timing::Section::kControlCycle).Reset();
  hardware::timing::ProbeOf(hardware::timing::Section::kObserver).Reset();
  return cyphal::RegisterWriteResult::kAccepted;
}

}  // namespace unimoc::drive
