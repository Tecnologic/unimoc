/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file zephyr_can_media.cpp
 *    @brief Cyphal CAN interface on a Zephyr CAN controller.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "zephyr_can_media.hpp"
#include <algorithm>
#include <cstring>

namespace unimoc::hardware {

int ZephyrCanMedia::Init(const struct device* const device,
                         struct k_sem* const wake,
                         const bool use_fd) noexcept {
  if (device == nullptr || !device_is_ready(device)) {
    return -ENODEV;
  }
  device_ = device;
  wake_ = wake;
  k_msgq_init(&rx_queue_, rx_buffer_.data(), sizeof(struct can_frame), kRxQueueLength);

  can_mode_t capabilities = 0U;
  (void)can_get_capabilities(device, &capabilities);
  can_mode_t mode = CAN_MODE_NORMAL;
  if (use_fd && (capabilities & CAN_MODE_FD) != 0U) {
    mode = CAN_MODE_FD;
    mtu_ = cyphal::kCanFdMtu;
    brs_ = IS_ENABLED(CONFIG_UNIMOC_CYPHAL_CAN_BRS);
  }
  int result = can_set_mode(device, mode);
  if (result != 0) {
    return result;
  }

  // All extended data frames; Cyphal/CAN filters by the transfer metadata in software.
  const struct can_filter kFilter{.id = 0U, .mask = 0U, .flags = CAN_FILTER_IDE};
  result = can_add_rx_filter(device, OnFrame, this, &kFilter);
  if (result < 0) {
    return result;
  }
  state_callback_.self = this;
  can_init_state_change_callback(&state_callback_.callback, OnStateChange);
  result = can_add_state_change_callback(device, &state_callback_.callback);
  if (result != 0 && result != -EALREADY) {
    return result;
  }
  result = can_start(device);
  return (result == -EALREADY) ? 0 : result;
}

void ZephyrCanMedia::OnFrame(const struct device* /*device*/,
                             struct can_frame* const frame,
                             void* const user_data) {
  auto* self = static_cast<ZephyrCanMedia*>(user_data);
  if ((frame->flags & CAN_FRAME_RTR) != 0U) {
    return;
  }
  if (k_msgq_put(&self->rx_queue_, frame, K_NO_WAIT) != 0) {
    ++self->rx_overruns_;
  }
  if (self->wake_ != nullptr) {
    k_sem_give(self->wake_);
  }
}

void ZephyrCanMedia::OnStateChange(const struct device* /*device*/,
                                   struct can_state_change_callback* const callback,
                                   const enum can_state state,
                                   struct can_bus_err_cnt /*error_count*/) {
  ZephyrCanMedia* self = CONTAINER_OF(callback, StateCallback, callback)->self;
  cyphal::CanBusState bus_state = cyphal::CanBusState::kErrorActive;
  switch (state) {
    case CAN_STATE_ERROR_WARNING:
      bus_state = cyphal::CanBusState::kErrorWarning;
      break;
    case CAN_STATE_ERROR_PASSIVE:
      bus_state = cyphal::CanBusState::kErrorPassive;
      break;
    case CAN_STATE_BUS_OFF:
      bus_state = cyphal::CanBusState::kBusOff;
      self->bus_off_count_.fetch_add(1U, std::memory_order_relaxed);
      break;
    default:  // error active; stopped only before can_start()
      break;
  }
  self->bus_state_.store(static_cast<uint8_t>(bus_state), std::memory_order_release);
  if (self->wake_ != nullptr) {
    k_sem_give(self->wake_);
  }
}

cyphal::CanBusStatus ZephyrCanMedia::BusStatus() const noexcept {
  return cyphal::CanBusStatus{.state = static_cast<cyphal::CanBusState>(
                                  bus_state_.load(std::memory_order_acquire)),
                              .bus_off_count = bus_off_count_.load(std::memory_order_relaxed)};
}

bool ZephyrCanMedia::Push(const cyphal::CanFrame& frame) noexcept {
  struct can_frame out{};
  out.id = frame.extended_can_id & CAN_EXT_ID_MASK;
  out.flags = CAN_FRAME_IDE;
  if (mtu_ > cyphal::kCanClassicMtu) {
    out.flags |= CAN_FRAME_FDF | (brs_ ? CAN_FRAME_BRS : 0U);
  }
  out.dlc = can_bytes_to_dlc(frame.size);
  std::memcpy(std::begin(out.data), frame.data.data(), std::min<std::size_t>(frame.size, sizeof(out.data)));
  // With a callback, can_send() queues the frame and does not wait for the transmission.
  const int kResult = can_send(device_, &out, K_NO_WAIT, [](const struct device*, int, void*) {}, nullptr);
  return kResult == 0;
}

bool ZephyrCanMedia::Pop(cyphal::CanFrame& frame) noexcept {
  struct can_frame received{};
  if (device_ == nullptr || k_msgq_get(&rx_queue_, &received, K_NO_WAIT) != 0) {
    return false;
  }
  frame.extended_can_id = received.id;
  frame.size = can_dlc_to_bytes(received.dlc);
  std::memcpy(frame.data.data(),
              std::begin(received.data),
              std::min<std::size_t>(frame.size, frame.data.size()));
  return true;
}

}  // namespace unimoc::hardware
