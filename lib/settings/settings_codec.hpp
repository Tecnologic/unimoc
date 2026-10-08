/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_codec.hpp
 *    @brief Explicit binary encoding for persistent settings.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include "nvm_settings.hpp"

namespace unimoc::settings {

/**
 * @brief Fixed size of the versioned settings image in bytes.
 *
 * The image contains fixed-width scalar values in little-endian order. Unit
 * wrappers are encoded through their numeric `Value()` representation, so the
 * image format is independent of the in-memory C++ layout.
 */
inline constexpr std::size_t kSettingsImageSize = 326U;

namespace settings_codec_internal {

class Writer {
 public:
  explicit constexpr Writer(std::span<std::byte> output) noexcept : output_{output} {}

  /**
   * @brief Writes an 8-bit unsigned integer to the output.
   * @param kValue The value to write.
   * @return True if the write was successful, false otherwise.
   */
  bool WriteUint8(const uint8_t kValue) noexcept {

    if (offset_ + sizeof(kValue) > output_.size()) {
      return false;
    }

    output_[offset_++] = static_cast<std::byte>(kValue);
    return true;
  }

  /**
   * @brief Writes a 16-bit unsigned integer to the output.
   * @param kValue The value to write.
   * @return True if the write was successful, false otherwise.
   */
  bool WriteUint16(const uint16_t kValue) noexcept {
    if (offset_ + sizeof(kValue) > output_.size()) {
      return false;
    }
    output_[offset_++] = static_cast<std::byte>(kValue & 0xFFU);
    output_[offset_++] = static_cast<std::byte>((kValue >> 8U) & 0xFFU);
    return true;
  }

  /**
   * @brief Writes a 32-bit unsigned integer to the output.
   * @param kValue The value to write.
   * @return True if the write was successful, false otherwise.
   */
  bool WriteUint32(const uint32_t kValue) noexcept {
    if (offset_ + sizeof(kValue) > output_.size()) {
      return false;
    }
    for (uint32_t shift = 0U; shift < 32U; shift += 8U) {
      output_[offset_++] = static_cast<std::byte>((kValue >> shift) & 0xFFU);
    }
    return true;
  }

  /**
   * @brief Writes a 32-bit floating-point value to the output.
   * @param kValue The value to write.
   * @return True if the write was successful, false otherwise.
   */
  bool WriteFloat(const float kValue) noexcept { return WriteUint32(std::bit_cast<uint32_t>(kValue)); }

  /**
   * @brief Writes a unit-wrapped numeric value to the output.
   * @tparam UnitType The unit type.
   * @param kValue The value to write.
   * @return True if the write was successful, false otherwise.
   */
  template <typename UnitType>
  bool WriteUnit(const UnitType kValue) noexcept {
    return WriteFloat(kValue.Value());
  }


  /**
   * @brief Writes a sequence of bytes to the output.
   * @param kBytes Pointer to the byte sequence.
   * @param kSize Number of bytes to write.
   * @return True if the write was successful, false otherwise.
   */
  bool WriteBytes(const char* const kBytes, const std::size_t kSize) noexcept {
    if (offset_ + kSize > output_.size()) {
      return false;
    }
    std::memcpy(output_.data() + offset_, kBytes, kSize);
    offset_ += kSize;
    return true;
  }

  /**
   * @brief Returns the current write position in the output.
   * @return The current offset in the output buffer.
   */
  [[nodiscard]] constexpr std::size_t Position() const noexcept { return offset_; }

 private:
  std::span<std::byte> output_;
  std::size_t offset_{0U};
};

class Reader {
 public:
  explicit constexpr Reader(std::span<const std::byte> input) noexcept : input_{input} {}

  /**
   * @brief Reads an 8-bit unsigned integer from the input.
   * @param value Reference to store the read value.
   * @return True if the read was successful, false otherwise.
   */
  bool ReadUint8(uint8_t& value) noexcept {
    if (offset_ + sizeof(value) > input_.size()) {
      return false;
    }

    value = std::to_integer<uint8_t>(input_[offset_++]);
    return true;
  }

  /**
   * @brief Reads a 16-bit unsigned integer from the input.
   * @param value Reference to store the read value.
   * @return True if the read was successful, false otherwise.
   */
  bool ReadUint16(uint16_t& value) noexcept {
    if (offset_ + sizeof(value) > input_.size()) {
      return false;
    }

    value = std::to_integer<uint16_t>(input_[offset_++]);
    value = static_cast<uint16_t>(value |
                                  static_cast<uint16_t>(std::to_integer<uint8_t>(input_[offset_++]) << 8U));
    return true;
  }

  /**
   * @brief Reads a 32-bit unsigned integer from the input.
   * @param value Reference to store the read value.
   * @return True if the read was successful, false otherwise.
   */
  bool ReadUint32(uint32_t& value) noexcept {
    if (offset_ + sizeof(value) > input_.size()) {
      return false;
    }

    value = 0U;
    for (uint32_t shift = 0U; shift < 32U; shift += 8U) {
      value |= static_cast<uint32_t>(std::to_integer<uint8_t>(input_[offset_++])) << shift;
    }
    return true;
  }

  /**
   * @brief Reads a 32-bit floating-point value from the input.
   * @param value Reference to store the read value.
   * @return True if the read was successful, false otherwise.
   */
  bool ReadFloat(float& value) noexcept {
    uint32_t bits = 0U;
    if (!ReadUint32(bits)) {
      return false;
    }

    value = std::bit_cast<float>(bits);
    return true;
  }

  /**
   * @brief Reads a unit-wrapped numeric value from the input.
   * @tparam UnitType The unit type.
   * @param value Reference to store the read value.
   * @return True if the read was successful, false otherwise.
   */
  template <typename UnitType>
  bool ReadUnit(UnitType& value) noexcept {
    float numeric_value = 0.0F;
    if (!ReadFloat(numeric_value)) {
      return false;
    }

    value = UnitType{numeric_value};
    return true;
  }

  /**
   * @brief Reads a sequence of bytes from the input.
   * @param bytes Pointer to the buffer to store the read bytes.
   * @param size Number of bytes to read.
   * @return True if the read was successful, false otherwise.
   */
  bool ReadBytes(char* const kBytes, const std::size_t kSize) noexcept {
    if (offset_ + kSize > input_.size()) {
      return false;
    }

    std::memcpy(kBytes, input_.data() + offset_, kSize);
    offset_ += kSize;
    return true;
  }

  [[nodiscard]] constexpr std::size_t Position() const noexcept { return offset_; }

 private:
  std::span<const std::byte> input_;
  std::size_t offset_{0U};
};

}  // namespace settings_codec_internal

/**
 * @brief Encodes and decodes the persistent settings image.
 */
class SettingsCodec {
 public:
  /// Number of bytes in one encoded settings image.
  static constexpr std::size_t kImageSize = kSettingsImageSize;

  /**
   * @brief Encodes settings into a fixed-size binary image.
   * @param settings Typed settings to encode.
   * @param image Destination buffer, exactly `kImageSize` bytes long.
   * @return True when the image was encoded completely.
   */
  [[nodiscard]] static bool Encode(const NvmSettings& settings, std::span<std::byte> image) noexcept {
    if (image.size() != kImageSize) {
      return false;
    }

    settings_codec_internal::Writer writer{image};
    return writer.WriteUint32(settings.magic) && writer.WriteUint16(settings.version) &&
           writer.WriteUint8(settings.node_id) &&
           writer.WriteBytes(std::span{settings.identity.name}.data(),
                             sizeof(settings.identity.name)) &&
           writer.WriteUint8(settings.identity.hw_version_major) &&
           writer.WriteUint8(settings.identity.hw_version_minor) &&
           writer.WriteUint8(settings.identity.sw_version_major) &&
           writer.WriteUint8(settings.identity.sw_version_minor) &&
           writer.WriteUint8(static_cast<uint8_t>(settings.motor_type)) &&
           writer.WriteUint8(settings.pole_pairs) &&
           writer.WriteUint8(static_cast<uint8_t>(settings.control_mode)) &&
           writer.WriteUnit(settings.stator_r) && writer.WriteUnit(settings.stator_l) &&
           writer.WriteUnit(settings.motor_i_max) && writer.WriteUnit(settings.motor_omega_max) &&
           writer.WriteUnit(settings.motor_omega_min) &&
           writer.WriteUnit(settings.battery_drive_current_max) &&
           writer.WriteUnit(settings.battery_charge_current_max) && writer.WriteUnit(settings.flux_pm) &&
           writer.WriteUnit(settings.l_d) && writer.WriteUnit(settings.l_q) &&
           writer.WriteUnit(settings.asm_r_r) && writer.WriteUnit(settings.asm_r_s) &&
           writer.WriteUnit(settings.asm_l_s) && writer.WriteUnit(settings.asm_l_r) &&
           writer.WriteUnit(settings.asm_l_m) && writer.WriteUnit(settings.motor_j) &&
           writer.WriteFloat(settings.mech_obs_q) && writer.WriteFloat(settings.mech_obs_r) &&
           writer.WriteUnit(settings.pmsm_flux_obs_c_d) && writer.WriteUnit(settings.pmsm_flux_obs_c_q) &&
           writer.WriteUnit(settings.asm_obs_g_i) && writer.WriteUnit(settings.asm_obs_g_flux) &&
           writer.WriteUnit(settings.asm_flux_kp) && writer.WriteUnit(settings.asm_flux_ki) &&
           writer.WriteUnit(settings.asm_flux_i_d_min) && writer.WriteUnit(settings.asm_flux_i_d_max) &&
           writer.WriteUnit(settings.fw_v_max) && writer.WriteUnit(settings.fw_ki) &&
           writer.WriteUnit(settings.fw_i_d_min) && writer.WriteUnit(settings.pwm_frequency) &&
           writer.WriteUnit(settings.current_kp_d) && writer.WriteUnit(settings.current_ki_d) &&
           writer.WriteUnit(settings.current_kp_q) && writer.WriteUnit(settings.current_ki_q) &&
           writer.WriteUnit(settings.current_v_max) && writer.WriteUnit(settings.svm_duty_min) &&
           writer.WriteUnit(settings.svm_duty_max) && writer.WriteUnit(settings.dtc_dead_time) &&
           writer.WriteUnit(settings.dtc_f_pwm) && writer.WriteUnit(settings.dtc_i_threshold) &&
           writer.WriteUnit(settings.hfi_v_inject) && writer.WriteUnit(settings.hfi_error_gain) &&
           writer.WriteUint8(settings.excitation_mode) && writer.WriteUnit(settings.excitation_l_m) &&
           writer.WriteUnit(settings.excitation_kp) && writer.WriteUnit(settings.excitation_ki) &&
           writer.WriteUnit(settings.excitation_i_f_min) && writer.WriteUnit(settings.excitation_i_f_max) &&
           writer.WriteUnit(settings.excitation_obs_tau) && writer.WriteUnit(settings.excitation_obs_l_m) &&
           writer.WriteUnit(settings.pos_kp_pos) && writer.WriteUnit(settings.pos_kp_speed) &&
           writer.WriteUnit(settings.pos_ki_speed) && writer.WriteUnit(settings.pos_speed_limit) &&
           writer.WriteUnit(settings.pos_accel_limit) && writer.WriteUnit(settings.pos_position_tolerance) &&
           writer.WriteUnit(settings.pos_speed_tolerance) && writer.WriteUnit(settings.pos_homing_speed) &&
           writer.WriteUnit(settings.adc_offset_a) && writer.WriteUnit(settings.adc_offset_b) &&
           writer.WriteUnit(settings.adc_gain_a) && writer.WriteUnit(settings.adc_gain_b) &&
           writer.WriteUnit(settings.adc_gain_vdc) && writer.WriteUnit(settings.phase_balance_a) &&
           writer.WriteUnit(settings.phase_balance_b) && writer.WriteUnit(settings.phase_balance_c) &&
           writer.Position() == kImageSize;
  }

  /**
   * @brief Decodes a fixed-size binary image into typed settings.
   * @param image Source image, exactly `kImageSize` bytes long.
   * @param settings Destination settings object, changed only on success.
   * @return True when the image has the expected complete format.
   */
  [[nodiscard]] static bool Decode(std::span<const std::byte> image, NvmSettings& settings) noexcept {
    if (image.size() != kImageSize) {
      return false;
    }

    NvmSettings decoded{};
    settings_codec_internal::Reader reader{image};
    uint8_t motor_type = 0U;
    uint8_t control_mode = 0U;

    if (!reader.ReadUint32(decoded.magic) || !reader.ReadUint16(decoded.version) ||
        !reader.ReadUint8(decoded.node_id) ||
        !reader.ReadBytes(std::span{decoded.identity.name}.data(), sizeof(decoded.identity.name)) ||
        !reader.ReadUint8(decoded.identity.hw_version_major) ||
        !reader.ReadUint8(decoded.identity.hw_version_minor) ||
        !reader.ReadUint8(decoded.identity.sw_version_major) ||
        !reader.ReadUint8(decoded.identity.sw_version_minor) || !reader.ReadUint8(motor_type) ||
        !reader.ReadUint8(decoded.pole_pairs) || !reader.ReadUint8(control_mode) ||
        !reader.ReadUnit(decoded.stator_r) || !reader.ReadUnit(decoded.stator_l) ||
        !reader.ReadUnit(decoded.motor_i_max) || !reader.ReadUnit(decoded.motor_omega_max) ||
        !reader.ReadUnit(decoded.motor_omega_min) || !reader.ReadUnit(decoded.battery_drive_current_max) ||
        !reader.ReadUnit(decoded.battery_charge_current_max) || !reader.ReadUnit(decoded.flux_pm) ||
        !reader.ReadUnit(decoded.l_d) || !reader.ReadUnit(decoded.l_q) || !reader.ReadUnit(decoded.asm_r_r) ||
        !reader.ReadUnit(decoded.asm_r_s) || !reader.ReadUnit(decoded.asm_l_s) ||
        !reader.ReadUnit(decoded.asm_l_r) || !reader.ReadUnit(decoded.asm_l_m) ||
        !reader.ReadUnit(decoded.motor_j) || !reader.ReadFloat(decoded.mech_obs_q) ||
        !reader.ReadFloat(decoded.mech_obs_r) || !reader.ReadUnit(decoded.pmsm_flux_obs_c_d) ||
        !reader.ReadUnit(decoded.pmsm_flux_obs_c_q) || !reader.ReadUnit(decoded.asm_obs_g_i) ||
        !reader.ReadUnit(decoded.asm_obs_g_flux) || !reader.ReadUnit(decoded.asm_flux_kp) ||
        !reader.ReadUnit(decoded.asm_flux_ki) || !reader.ReadUnit(decoded.asm_flux_i_d_min) ||
        !reader.ReadUnit(decoded.asm_flux_i_d_max) || !reader.ReadUnit(decoded.fw_v_max) ||
        !reader.ReadUnit(decoded.fw_ki) || !reader.ReadUnit(decoded.fw_i_d_min) ||
        !reader.ReadUnit(decoded.pwm_frequency) || !reader.ReadUnit(decoded.current_kp_d) ||
        !reader.ReadUnit(decoded.current_ki_d) || !reader.ReadUnit(decoded.current_kp_q) ||
        !reader.ReadUnit(decoded.current_ki_q) || !reader.ReadUnit(decoded.current_v_max) ||
        !reader.ReadUnit(decoded.svm_duty_min) || !reader.ReadUnit(decoded.svm_duty_max) ||
        !reader.ReadUnit(decoded.dtc_dead_time) || !reader.ReadUnit(decoded.dtc_f_pwm) ||
        !reader.ReadUnit(decoded.dtc_i_threshold) || !reader.ReadUnit(decoded.hfi_v_inject) ||
        !reader.ReadUnit(decoded.hfi_error_gain) || !reader.ReadUint8(decoded.excitation_mode) ||
        !reader.ReadUnit(decoded.excitation_l_m) || !reader.ReadUnit(decoded.excitation_kp) ||
        !reader.ReadUnit(decoded.excitation_ki) || !reader.ReadUnit(decoded.excitation_i_f_min) ||
        !reader.ReadUnit(decoded.excitation_i_f_max) || !reader.ReadUnit(decoded.excitation_obs_tau) ||
        !reader.ReadUnit(decoded.excitation_obs_l_m) || !reader.ReadUnit(decoded.pos_kp_pos) ||
        !reader.ReadUnit(decoded.pos_kp_speed) || !reader.ReadUnit(decoded.pos_ki_speed) ||
        !reader.ReadUnit(decoded.pos_speed_limit) || !reader.ReadUnit(decoded.pos_accel_limit) ||
        !reader.ReadUnit(decoded.pos_position_tolerance) || !reader.ReadUnit(decoded.pos_speed_tolerance) ||
        !reader.ReadUnit(decoded.pos_homing_speed) || !reader.ReadUnit(decoded.adc_offset_a) ||
        !reader.ReadUnit(decoded.adc_offset_b) || !reader.ReadUnit(decoded.adc_gain_a) ||
        !reader.ReadUnit(decoded.adc_gain_b) || !reader.ReadUnit(decoded.adc_gain_vdc) ||
        !reader.ReadUnit(decoded.phase_balance_a) || !reader.ReadUnit(decoded.phase_balance_b) ||
        !reader.ReadUnit(decoded.phase_balance_c) || reader.Position() != kImageSize) {
      return false;
    }

    decoded.motor_type = static_cast<system::MotorType>(motor_type);
    decoded.control_mode = static_cast<cyphal::ControlMode>(control_mode);
    settings = decoded;
    return true;
  }
};

}  // namespace unimoc::settings
