/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file can_test_bus.hpp
 *    @brief In-memory CAN bus for Cyphal tests: every frame pushed by one
 *           medium arrives at all other media of the bus.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include "cyphal/can_media.hpp"

namespace unimoc::test {

class CanTestBus;

/// One CAN interface on a CanTestBus.
class CanTestMedia final : public cyphal::CanMedia {
 public:
  /// Frames buffered per medium.
  static constexpr std::size_t kQueueSize = 256U;

  /**
   * @brief Attaches the medium to a bus.
   * @param bus Bus that delivers the frames; must outlive the medium.
   * @param mtu kCanClassicMtu or kCanFdMtu.
   */
  CanTestMedia(CanTestBus& bus, std::size_t mtu) noexcept;

  [[nodiscard]] std::size_t Mtu() const noexcept override { return mtu_; }
  [[nodiscard]] bool Push(const cyphal::CanFrame& frame) noexcept override;
  [[nodiscard]] cyphal::CanBusStatus BusStatus() const noexcept override { return bus_status; }
  [[nodiscard]] bool Pop(cyphal::CanFrame& frame) noexcept override {
    if (count_ == 0U) {
      return false;
    }
    frame = queue_[head_];
    head_ = (head_ + 1U) % kQueueSize;
    --count_;
    return true;
  }

  /// Delivers a frame to this medium's receive queue; dropped while the medium is bus-off.
  void Deliver(const cyphal::CanFrame& frame) noexcept {
    if (count_ < kQueueSize && bus_status.state != cyphal::CanBusState::kBusOff) {
      queue_[(head_ + count_) % kQueueSize] = frame;
      ++count_;
    }
  }

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes): test probes.
  /// When false, Push() refuses every frame (bus off, mailboxes full).
  bool accept_tx{true};
  /// Frames sent through this medium.
  std::size_t sent{0U};
  /// Largest frame size sent through this medium.
  std::size_t largest_sent{0U};
  /// Reported by BusStatus(); kBusOff also stops sending and receiving.
  cyphal::CanBusStatus bus_status{};
  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  CanTestBus* bus_;
  std::size_t mtu_;
  std::array<cyphal::CanFrame, kQueueSize> queue_{};
  std::size_t head_{0U};
  std::size_t count_{0U};
};

/// Up to kMaxMedia media that see each other's frames.
class CanTestBus {
 public:
  static constexpr std::size_t kMaxMedia = 4U;

  void Attach(CanTestMedia& media) noexcept {
    if (count_ < kMaxMedia) {
      media_[count_++] = &media;
    }
  }

  void Broadcast(const CanTestMedia& sender, const cyphal::CanFrame& frame) noexcept {
    for (std::size_t i = 0U; i < count_; ++i) {
      if (media_[i] != &sender) {
        media_[i]->Deliver(frame);
      }
    }
  }

 private:
  std::array<CanTestMedia*, kMaxMedia> media_{};
  std::size_t count_{0U};
};

inline CanTestMedia::CanTestMedia(CanTestBus& bus, const std::size_t mtu) noexcept : bus_{&bus}, mtu_{mtu} {
  bus.Attach(*this);
}

inline bool CanTestMedia::Push(const cyphal::CanFrame& frame) noexcept {
  if (!accept_tx || bus_status.state == cyphal::CanBusState::kBusOff) {
    return false;
  }
  ++sent;
  largest_sent = (frame.size > largest_sent) ? frame.size : largest_sent;
  bus_->Broadcast(*this, frame);
  return true;
}

}  // namespace unimoc::test
