/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file register_file.cpp
 *    @brief The register description file (registers.json) of the simulated drive.
 *
 *    native_sim links the host C library, so the file is written with stdio.
 *    The schema is documented in docs/settings_plan.md ("Register
 *    description file"); a change of it raises kSchemaVersion.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "register_file.hpp"
#include <app_version.h>
#include <array>
#include <charconv>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <string_view>
#include <type_traits>
#include "cyphal/register_value.hpp"

namespace unimoc::hardware::sim {

namespace {

/// Version of the file layout; tools accept the versions they know.
constexpr int kSchemaVersion = 1;

/// Writes @p text as a JSON string.
void PutString(std::FILE* file, const std::string_view text) noexcept {
  (void)std::fputc('"', file);
  for (const char kChar : text) {
    if (kChar == '"' || kChar == '\\') {
      (void)std::fputc('\\', file);
      (void)std::fputc(kChar, file);
    } else if (static_cast<unsigned char>(kChar) < 0x20U) {
      (void)std::fprintf(file, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(kChar)));
    } else {
      (void)std::fputc(kChar, file);
    }
  }
  (void)std::fputc('"', file);
}

/// Writes a float in its shortest exact form; JSON has no infinity or NaN, so those are null.
void PutNumber(std::FILE* file, const float number) noexcept {
  if (!std::isfinite(number)) {
    (void)std::fputs("null", file);
    return;
  }
  std::array<char, 32> text{};
  const std::to_chars_result kResult = std::to_chars(text.data(), text.data() + text.size(), number);
  (void)std::fwrite(text.data(), 1U, static_cast<std::size_t>(kResult.ptr - text.data()), file);
}

/// Writes the elements of a numeric array as a JSON array (integers exactly).
template <typename Array>
void PutElements(std::FILE* file, const Array& array) noexcept {
  (void)std::fputc('[', file);
  for (std::size_t i = 0U; i < array.count; ++i) {
    (void)std::fputs(i > 0U ? ", " : "", file);
    if constexpr (std::is_floating_point_v<std::remove_cvref_t<decltype(array.elements[0])>>) {
      PutNumber(file, array.elements[i]);
    } else {
      (void)std::fprintf(file, "%" PRIu64, static_cast<uint64_t>(array.elements[i]));
    }
  }
  (void)std::fputc(']', file);
}

/// Writes `"type": ..., "size": ..., "default": ...` of a register value.
void PutValue(std::FILE* file, const cyphal::RegisterValue& value) noexcept {
  const auto kField = [file](const char* type, const std::size_t size) {
    (void)std::fprintf(file, R"("type": "%s", "size": %zu, "default": )", type, size);
  };
  if (uavcan_register_Value_1_0_is_string_(&value)) {
    const auto& text = value._string.value;
    kField("string", text.count);
    PutString(file, std::string_view{reinterpret_cast<const char*>(text.elements), text.count});
  } else if (uavcan_register_Value_1_0_is_unstructured_(&value)) {
    const auto& bytes = value.unstructured.value;
    kField("unstructured", bytes.count);
    PutElements(file, bytes);
  } else if (uavcan_register_Value_1_0_is_bit_(&value)) {
    const auto& bits = value.bit.value;
    kField("bit", bits.count);
    (void)std::fputc('[', file);
    for (std::size_t i = 0U; i < bits.count; ++i) {
      (void)std::fputs(i > 0U ? ", " : "", file);
      (void)std::fputs(((bits.bitpacked[i / 8U] >> (i % 8U)) & 1U) != 0U ? "true" : "false", file);
    }
    (void)std::fputc(']', file);
  } else if (uavcan_register_Value_1_0_is_natural8_(&value)) {
    kField("natural8", value.natural8.value.count);
    PutElements(file, value.natural8.value);
  } else if (uavcan_register_Value_1_0_is_natural16_(&value)) {
    kField("natural16", value.natural16.value.count);
    PutElements(file, value.natural16.value);
  } else if (uavcan_register_Value_1_0_is_natural32_(&value)) {
    kField("natural32", value.natural32.value.count);
    PutElements(file, value.natural32.value);
  } else if (uavcan_register_Value_1_0_is_real32_(&value)) {
    kField("real32", value.real32.value.count);
    PutElements(file, value.real32.value);
  } else {
    kField("empty", 0U);
    (void)std::fputs("null", file);
  }
}

void PutRegister(std::FILE* file,
                 const std::string_view name,
                 const cyphal::RegisterFlags flags,
                 const cyphal::RegisterValue& value,
                 const cyphal::RegisterDescription& description) noexcept {
  (void)std::fputs("    {\"name\": ", file);
  PutString(file, name);
  (void)std::fputs(", ", file);
  PutValue(file, value);
  (void)std::fprintf(file,
                     R"(, "mutable": %s, "persistent": %s)",
                     flags.is_mutable ? "true" : "false",
                     flags.is_persistent ? "true" : "false");
  if (!description.unit.empty()) {
    (void)std::fputs(", \"unit\": ", file);
    PutString(file, description.unit);
  }
  if (std::isfinite(description.minimum)) {
    (void)std::fputs(", \"min\": ", file);
    PutNumber(file, description.minimum);
  }
  if (std::isfinite(description.maximum)) {
    (void)std::fputs(", \"max\": ", file);
    PutNumber(file, description.maximum);
  }
  if (flags.is_mutable) {
    (void)std::fputs(", \"applies\": ", file);
    PutString(file, cyphal::EffectName(description.effect));
  }
  (void)std::fputs(", \"help\": ", file);
  PutString(file, description.help);
  (void)std::fputc('}', file);
}

}  // namespace

bool WriteRegisterFile(const cyphal::CyphalNode& node,
                       const cyphal::NodeInfo& info,
                       const char* const path) noexcept {
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): a stdio file, closed at the end of this function.
  std::FILE* const kFile = std::fopen(path, "w");
  if (kFile == nullptr) {
    return false;
  }
  (void)std::fprintf(kFile,
                     "{\n  \"schema\": %d,\n  \"firmware\": {\"name\": \"unimoc\", \"version\": \"%s\", "
                     "\"vcs_revision\": \"%016" PRIx64
                     "\", \"target\": \"native_sim\"},\n  \"registers\": [\n",
                     kSchemaVersion,
                     APP_VERSION_STRING,
                     info.vcs_revision_id);
  bool first = true;
  for (const cyphal::RegisterProvider* const kProvider : node.RegisterProviders()) {
    for (std::size_t index = 0U; index < kProvider->Count(); ++index) {
      cyphal::RegisterValue value{};
      const cyphal::RegisterFlags kFlags = kProvider->Read(index, value);
      (void)std::fputs(first ? "" : ",\n", kFile);
      first = false;
      PutRegister(kFile, kProvider->Name(index), kFlags, value, kProvider->Describe(index));
    }
  }
  (void)std::fputs("\n  ]\n}\n", kFile);
  const bool kWritten = std::ferror(kFile) == 0;
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): the stdio file opened above.
  return (std::fclose(kFile) == 0) && kWritten;
}

}  // namespace unimoc::hardware::sim
