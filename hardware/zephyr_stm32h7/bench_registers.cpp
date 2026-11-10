/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file bench_registers.cpp
 *    @brief Read-only bench registers of the STM32H7 platform (`unimoc.h7.*`).
 *
 *    The counts are the last injected conversion (12 bit, 0 to 4095 for 0 to
 *    3.3 V at the pin), the same values the sample interrupt reads; zero
 *    before the control started. The scaling to physical values stays in the
 *    test, so these registers show what the pin sees.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "bench_registers.hpp"
#include <array>
#include <cstddef>
#include <string_view>
#include "foc_stm32h7.hpp"

namespace unimoc::hardware {

namespace {

using cyphal::RegisterDescription;
using cyphal::RegisterFlags;
using cyphal::RegisterValue;
using cyphal::RegisterWriteResult;

/// The registers in order: the nine foc0 channels, then the enable and the Hall code.
enum class BenchRegister : uint8_t {
  kIa,
  kIb,
  kIc,
  kVa,
  kVb,
  kVc,
  kVdc,
  kBridgeTemperature,
  kMotorTemperature,
  kExternalEnable,
  kHallCode,
  kCount,
};

struct BenchEntry {
  std::string_view name;
  std::string_view help;
};

constexpr std::array<BenchEntry, static_cast<std::size_t>(BenchRegister::kCount)> kEntries{{
    {"unimoc.h7.adc.ia", "Raw count of the phase A current (12 bit, 2048 = 1.65 V = zero current)"},
    {"unimoc.h7.adc.ib", "Raw count of the phase B current (12 bit, 2048 = 1.65 V = zero current)"},
    {"unimoc.h7.adc.ic", "Raw count of the phase C current (12 bit, 2048 = 1.65 V = zero current)"},
    {"unimoc.h7.adc.va", "Raw count of the phase A voltage divider (12 bit, 4095 = 3.3 V)"},
    {"unimoc.h7.adc.vb", "Raw count of the phase B voltage divider (12 bit, 4095 = 3.3 V)"},
    {"unimoc.h7.adc.vc", "Raw count of the phase C voltage divider (12 bit, 4095 = 3.3 V)"},
    {"unimoc.h7.adc.vdc", "Raw count of the DC-link voltage divider (12 bit, 4095 = 3.3 V)"},
    {"unimoc.h7.adc.bridge_temperature", "Raw count of the bridge NTC (12 bit, 4095 = 3.3 V)"},
    {"unimoc.h7.adc.motor_temperature", "Raw count of the motor NTC (12 bit, 4095 = 3.3 V)"},
    {"unimoc.h7.external_enable", "1 while the external enable (break input) releases the outputs"},
    {"unimoc.h7.hall_code", "Present Hall code, bit 0 = A (0 and 7 are invalid)"},
}};

[[nodiscard]] uint32_t RawCount(const foc_stm32h7::RawSamples& raw, const BenchRegister which) noexcept {
  switch (which) {
    case BenchRegister::kIa:
      return raw.ia;
    case BenchRegister::kIb:
      return raw.ib;
    case BenchRegister::kIc:
      return raw.ic;
    case BenchRegister::kVa:
      return raw.va;
    case BenchRegister::kVb:
      return raw.vb;
    case BenchRegister::kVc:
      return raw.vc;
    case BenchRegister::kVdc:
      return raw.vdc;
    case BenchRegister::kBridgeTemperature:
      return raw.bridge_temperature;
    default:
      return raw.motor_temperature;
  }
}

class Registers final : public cyphal::RegisterProvider {
 public:
  [[nodiscard]] std::size_t Count() const noexcept override { return kEntries.size(); }

  [[nodiscard]] std::string_view Name(const std::size_t index) const noexcept override {
    return index < kEntries.size() ? kEntries[index].name : std::string_view{};
  }

  [[nodiscard]] RegisterDescription Describe(const std::size_t index) const noexcept override {
    RegisterDescription description{};
    if (index < kEntries.size()) {
      description.help = kEntries[index].help;
    }
    return description;
  }

  [[nodiscard]] RegisterFlags Read(const std::size_t index, RegisterValue& value) const noexcept override {
    const auto kWhich = static_cast<BenchRegister>(index);
    if (index >= kEntries.size()) {
      cyphal::SetEmpty(value);
    } else if (kWhich == BenchRegister::kExternalEnable) {
      cyphal::SetBit(value, foc_stm32h7::ExternalEnablePresent());
    } else if (kWhich == BenchRegister::kHallCode) {
      cyphal::SetNatural8(value, PresentHallCode());
    } else {
      cyphal::SetNatural16(value, static_cast<uint16_t>(RawCount(foc_stm32h7::ReadRaw(), kWhich)));
    }
    return {false, false};
  }

  [[nodiscard]] RegisterWriteResult Write(std::size_t /*index*/,
                                          const RegisterValue& /*value*/) noexcept override {
    return RegisterWriteResult::kReadOnly;
  }
};

Registers g_registers;

}  // namespace

cyphal::RegisterProvider& BenchRegisters() noexcept { return g_registers; }

}  // namespace unimoc::hardware
