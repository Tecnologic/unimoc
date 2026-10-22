/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file trace_node.cpp
 *    @brief Cyphal services of the traces: the cymon protocol on the drive node.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "trace_node.hpp"
#include <cymon/ArmTrigger_0_1.h>
#include <cymon/GetBufferInfo_0_1.h>
#include <cymon/GetVariableList_0_1.h>
#include <cymon/ReadSamples_0_1.h>
#include <cymon/SetupCapture_0_1.h>
#include <cymon/TriggerEvent_0_1.h>
#include <algorithm>
#include <array>
#include <string_view>

namespace unimoc::trace {

namespace {

[[nodiscard]] const uint8_t* InputBytes(std::span<const std::byte> payload) noexcept {
  return reinterpret_cast<const uint8_t*>(payload.data());
}

[[nodiscard]] uint8_t* OutputBytes(std::span<std::byte> buffer) noexcept {
  return reinterpret_cast<uint8_t*>(buffer.data());
}

/// Copies @p text into a Nunavut uint8 array of @p capacity bytes.
template <typename Array>
void CopyText(const std::string_view text, const std::size_t capacity, Array& array) noexcept {
  const std::size_t kSize = std::min(text.size(), capacity);
  std::copy_n(text.begin(), kSize, std::begin(array.elements));
  array.count = kSize;
}

}  // namespace

bool TraceNode::Start(const settings::NvmSettings& settings, const float tick_period_us) noexcept {
  device_->SetTickPeriod(tick_period_us);
  variables_port_ = settings.srv_cymon_variables_id;
  buffer_info_port_ = settings.srv_cymon_buffer_info_id;
  setup_port_ = settings.srv_cymon_setup_id;
  arm_port_ = settings.srv_cymon_arm_id;
  read_port_ = settings.srv_cymon_read_id;
  trigger_subject_ = settings.sub_cymon_trigger_id;
  trigger_publisher_ = cyphal::Publisher{settings.pub_cymon_trigger_id, cyphal::Priority::kHigh, 0U};

  bool added = node_->AddServer(variables_port_, cymon_GetVariableList_Request_0_1_EXTENT_BYTES_, *this);
  added = node_->AddServer(buffer_info_port_, cymon_GetBufferInfo_Request_0_1_EXTENT_BYTES_, *this) && added;
  added = node_->AddServer(setup_port_, cymon_SetupCapture_Request_0_1_EXTENT_BYTES_, *this) && added;
  added = node_->AddServer(arm_port_, cymon_ArmTrigger_Request_0_1_EXTENT_BYTES_, *this) && added;
  added = node_->AddServer(read_port_, cymon_ReadSamples_Request_0_1_EXTENT_BYTES_, *this) && added;
  added = node_->Subscribe(trigger_subject_, cymon_TriggerEvent_0_1_EXTENT_BYTES_, *this) && added;
  added = node_->AddPublisher(trigger_publisher_) && added;
  return added;
}

void TraceNode::Spin() noexcept {
  cymon::Device::TriggerEvent event{};
  if (!device_->TakeTriggerEvent(&event)) {
    return;
  }
  cymon_TriggerEvent_0_1 message{};
  message.source_variable_id = event.source_variable_id;
  message.mode.value = static_cast<uint8_t>(event.mode);
  message.trigger_value = event.trigger_value;
  std::array<std::byte, cymon_TriggerEvent_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (cymon_TriggerEvent_0_1_serialize_(&message, OutputBytes(buffer), &size) >= 0) {
    (void)node_->Publish(trigger_publisher_, std::span<const std::byte>{buffer.data(), size});
  }
}

std::optional<std::size_t> TraceNode::OnRequest(const cyphal::Transfer& request,
                                                const std::span<std::byte> response) noexcept {
  // The port-IDs are unique: CyphalNode::AddServer() refuses a taken one.
  if (request.port_id == variables_port_) {
    return OnGetVariableList(request.payload, response);
  }
  if (request.port_id == buffer_info_port_) {
    return OnGetBufferInfo(response);
  }
  if (request.port_id == setup_port_) {
    return OnSetupCapture(request.payload, response);
  }
  if (request.port_id == arm_port_) {
    return OnArmTrigger(request.payload, response);
  }
  if (request.port_id == read_port_) {
    return OnReadSamples(request.payload, response);
  }
  return std::nullopt;
}

void TraceNode::OnMessage(const cyphal::Transfer& transfer, uint64_t /*now_us*/) noexcept {
  // Any trigger event of another node triggers the armed capture at the next sample.
  if (transfer.port_id == trigger_subject_) {
    device_->RequestTrigger();
  }
}

std::optional<std::size_t> TraceNode::OnGetVariableList(const std::span<const std::byte> request,
                                                        const std::span<std::byte> response) const noexcept {
  cymon_GetVariableList_Request_0_1 list_request{};
  std::size_t size = request.size();
  if (cymon_GetVariableList_Request_0_1_deserialize_(&list_request, InputBytes(request), &size) < 0) {
    return std::nullopt;
  }
  const cymon::Device::VariableInfo kVariable = device_->GetVariable(list_request.index);
  cymon_GetVariableList_Response_0_1 list_response{};
  CopyText(kVariable.name, cymon_GetVariableList_Response_0_1_name_ARRAY_CAPACITY_, list_response.name);
  CopyText(kVariable.unit, cymon_GetVariableList_Response_0_1_unit_ARRAY_CAPACITY_, list_response.unit);
  size = response.size();
  if (cymon_GetVariableList_Response_0_1_serialize_(&list_response, OutputBytes(response), &size) < 0) {
    return std::nullopt;
  }
  return size;
}

std::optional<std::size_t> TraceNode::OnGetBufferInfo(const std::span<std::byte> response) const noexcept {
  const cymon::Device::BufferInfo kInfo = device_->HandleGetBufferInfo();
  cymon_GetBufferInfo_Response_0_1 info{};
  info.buffer_bytes = kInfo.buffer_bytes;
  info.max_channels = kInfo.max_channels;
  info.max_frames = kInfo.max_frames;
  info.max_read_samples = kInfo.max_read_samples;
  info.tick_period_us = kInfo.tick_period_us;
  std::size_t size = response.size();
  if (cymon_GetBufferInfo_Response_0_1_serialize_(&info, OutputBytes(response), &size) < 0) {
    return std::nullopt;
  }
  return size;
}

std::optional<std::size_t> TraceNode::OnSetupCapture(const std::span<const std::byte> request,
                                                     const std::span<std::byte> response) noexcept {
  cymon_SetupCapture_Request_0_1 setup{};
  std::size_t size = request.size();
  if (cymon_SetupCapture_Request_0_1_deserialize_(&setup, InputBytes(request), &size) < 0) {
    return std::nullopt;
  }
  cymon::Device::CaptureConfig config{};
  // At most 16 IDs by the DSDL; more than kMaxChannels is answered with BAD_CONFIG.
  config.num_channels = static_cast<uint8_t>(setup.channel_ids.count);
  std::copy_n(std::begin(setup.channel_ids.elements),
              std::min(setup.channel_ids.count, cymon::kMaxChannels),
              config.channel_ids.begin());
  config.sample_period_us = setup.sample_period_us;
  config.num_samples = setup.num_samples;
  config.pretrigger_samples = setup.pretrigger_samples;
  const cymon::Device::SetupCaptureResult kResult = device_->HandleSetupCapture(config);

  cymon_SetupCapture_Response_0_1 setup_response{};
  setup_response.result.value = static_cast<uint8_t>(kResult.error);
  setup_response.actual_sample_period_us = kResult.actual_sample_period_us;
  size = response.size();
  if (cymon_SetupCapture_Response_0_1_serialize_(&setup_response, OutputBytes(response), &size) < 0) {
    return std::nullopt;
  }
  return size;
}

std::optional<std::size_t> TraceNode::OnArmTrigger(const std::span<const std::byte> request,
                                                   const std::span<std::byte> response) noexcept {
  cymon_ArmTrigger_Request_0_1 arm{};
  std::size_t size = request.size();
  if (cymon_ArmTrigger_Request_0_1_deserialize_(&arm, InputBytes(request), &size) < 0) {
    return std::nullopt;
  }
  cymon::Device::ArmTriggerConfig config{};
  config.arm = arm.arm;
  config.source_variable_id = arm.source_variable_id;
  // A raw value; the device rejects modes it does not know.
  config.mode = static_cast<cymon::TriggerMode>(arm.mode.value);
  config.level = arm.level;
  config.hysteresis_band = arm.hysteresis_band;
  const cymon::ErrorCode kResult = device_->HandleArmTrigger(config);

  cymon_ArmTrigger_Response_0_1 arm_response{};
  arm_response.result.value = static_cast<uint8_t>(kResult);
  size = response.size();
  if (cymon_ArmTrigger_Response_0_1_serialize_(&arm_response, OutputBytes(response), &size) < 0) {
    return std::nullopt;
  }
  return size;
}

std::optional<std::size_t> TraceNode::OnReadSamples(const std::span<const std::byte> request,
                                                    const std::span<std::byte> response) const noexcept {
  cymon_ReadSamples_Request_0_1 read{};
  std::size_t size = request.size();
  if (cymon_ReadSamples_Request_0_1_deserialize_(&read, InputBytes(request), &size) < 0) {
    return std::nullopt;
  }
  const cymon::Device::ReadSamplesResult kResult =
      device_->HandleReadSamples(read.frame_offset, read.max_frames);
  cymon_ReadSamples_Response_0_1 samples{};
  samples.state.value = static_cast<uint8_t>(kResult.state);
  samples.num_channels = kResult.num_channels;
  samples.num_samples = kResult.num_samples;
  samples.pretrigger_samples = kResult.pretrigger_samples;
  samples.sample_period_us = kResult.sample_period_us;
  samples.end_of_data = kResult.end_of_data;
  const std::span<const float> kSamples = kResult.Samples();
  std::copy(kSamples.begin(), kSamples.end(), std::begin(samples.samples.elements));
  samples.samples.count = kSamples.size();
  size = response.size();
  if (cymon_ReadSamples_Response_0_1_serialize_(&samples, OutputBytes(response), &size) < 0) {
    return std::nullopt;
  }
  return size;
}

}  // namespace unimoc::trace
