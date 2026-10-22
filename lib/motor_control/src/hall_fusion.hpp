/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hall_fusion.hpp
 *    @brief Hall sector and edge measurements for the mechanical observer.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <concepts>
#include <cstdint>
#include <optional>
#include <span>
#include "angle_wrap.hpp"
#include "hall_capture.hpp"
#include "hall_decoder.hpp"
#include "mechanical_observer.hpp"
#include "nvm_settings.hpp"
#include "units.hpp"

/**
 * @namespace unimoc::observer observer algorithms namespace
 */
namespace unimoc::observer {

/**
 * @brief Fuses the Hall sensors into the mechanical observer (docs/hotpath_plan.md, section 5).
 *
 * Two measurements come from the same Hall code, both as Kalman updates of
 * the shared `MechanicalObserver` (`UpdateAngle()`):
 *
 *  - **Edge:** a change of the code between adjacent sectors marks the
 *    boundary angle at the timestamp of the change. Projected to the
 *    reference time of the estimate with the speed estimate:
 *    ε = wrap(θ_edge + ω̂ · (t_ref − t_edge) − θ̂), noise `r_edge`.
 *  - **Sector:** the code at the reference time only says that the angle lies
 *    in its 60 degree sector; the innovation is zero inside and the distance
 *    to the nearest boundary outside (`HallDecoder::SectorInnovation()`),
 *    noise `r_sector`.
 *
 * Inside a sector the angle innovation is zero, so it neither slows a speed
 * estimate down nor stops the load-torque state from accelerating it. The
 * time τ since the last change of the code bounds the speed: on average
 * |ω| < 60° / τ. Two speed updates (`UpdateSpeed()`) use it:
 *
 *  - An estimate above twice the bound (the margin keeps the bound from
 *    clipping a correct estimate at constant speed, where it is tight just
 *    before each edge) is pulled to it, noise `r_sector` / τ². A stopping
 *    rotor so loses its speed estimate within a few sector times.
 *  - After kStandstillTime without an edge the rotor stands (it would be
 *    slower than 2 rad/s electrical otherwise): the estimate is held still
 *    (`HoldStill()`: no speed, torque balance) instead of drifting through
 *    the sector. A repeated zero-speed measurement would make the filter
 *    overconfident and slow to follow the next start.
 *
 * The first valid code after Reset() moves the estimate to the sector
 * center: the best start for an unknown angle (±30°), from where the sector
 * bound and the edges take over. An estimate at a sector boundary would
 * otherwise stay there until the first edge, up to 60° off. After a Hall
 * failure the estimate (now from the flux observer) is kept while it lies
 * in the sector of the returning code and moved to the center otherwise.
 *
 * Inside the sector the sector measurement is no measurement at all: an
 * update with zero innovation in every cycle would shrink the angle
 * covariance far below the real ±30° uncertainty, and the next edge would
 * then correct the speed and the load torque instead of the angle. The
 * sector update therefore only acts while the estimate lies outside.
 *
 * Invalid codes (0, 7) give no measurement and are counted; a change between
 * sectors that are not adjacent (a missed edge) gives no edge and is counted.
 * The fusion runs in the observer after the covariance prediction.
 *
 * @tparam T Floating-point type of the mechanical observer.
 */
template <std::floating_point T = float>
class HallFusion {
 public:
  /**
   * @brief Loads the Hall settings and resets the state.
   * @param settings Validated settings.
   */
  constexpr void Init(const settings::NvmSettings& settings) noexcept {
    enabled_ = settings.hall_enable != 0U;
    decoder_ = HallDecoder{settings.hall_offset};
    r_edge_ = settings.hall_r_edge;
    r_sector_ = settings.hall_r_sector;
    Reset();
  }

  /// Forgets the code and clears the counters.
  constexpr void Reset() noexcept {
    valid_ = false;
    started_ = false;
    code_ = 0U;
    last_change_ = 0U;
    edges_ = 0U;
    invalid_codes_ = 0U;
    invalid_transitions_ = 0U;
  }

  /// The Hall measurements are fused.
  [[nodiscard]] constexpr bool Enabled() const noexcept { return enabled_; }

  /**
   * @brief Fuses the events captured since the last call and the sector.
   *
   * @param events Hall events, oldest first.
   * @param reference Timestamp that the estimate of @p mech_obs refers to.
   * @param seconds_per_tick Period of the timestamp clock [s].
   * @param mech_obs Observer that receives the measurements; its covariance
   *                 must already be predicted for this cycle.
   */
  constexpr void Update(const std::span<const HallEvent> events,
                        const uint32_t reference,
                        const float seconds_per_tick,
                        MechanicalObserver<T>& mech_obs) noexcept {
    uint8_t code_at_reference = code_;
    for (const HallEvent& event : events) {
      if (event.code == code_) {
        continue;
      }
      const auto kTicksAfterEvent = static_cast<int32_t>(reference - event.timestamp);
      if (!HallDecoder::Sector(event.code).has_value()) {
        ++invalid_codes_;
      } else if (const std::optional<unit::Angle> kEdge = decoder_.EdgeAngle(code_, event.code)) {
        const unit::Time kAge{static_cast<float>(kTicksAfterEvent) * seconds_per_tick};
        const unit::Angle kMeasured = *kEdge + mech_obs.omega * kAge;
        mech_obs.UpdateAngle(WrapAngle(kMeasured - mech_obs.theta), r_edge_);
        ++edges_;
      } else if (HallDecoder::Sector(code_).has_value()) {
        ++invalid_transitions_;
      }
      code_ = event.code;
      last_change_ = event.timestamp;
      if (kTicksAfterEvent >= 0) {
        code_at_reference = code_;
      }
    }

    const std::optional<uint8_t> kSector = HallDecoder::Sector(code_at_reference);
    if (!kSector.has_value()) {
      valid_ = false;
      return;  // no valid code: no Hall information
    }
    const unit::Angle kSectorInnovation = decoder_.SectorInnovation(*kSector, mech_obs.theta);
    if (!started_ || (!valid_ && kSectorInnovation != unit::Angle{})) {
      const unit::Angle kCenter = decoder_.SectorStart(*kSector) + HallDecoder::kSectorWidth * 0.5F;
      mech_obs.Reset(WrapAngle(kCenter), mech_obs.omega);
    } else if (kSectorInnovation != unit::Angle{}) {
      mech_obs.UpdateAngle(kSectorInnovation, r_sector_);
    }
    valid_ = true;
    started_ = true;

    // Keep the age of the last change in range of the wrapping timestamps.
    if (static_cast<int32_t>(reference - last_change_) > kMaxAgeTicks) {
      last_change_ = reference - static_cast<uint32_t>(kMaxAgeTicks);
    }
    const auto kTicksSinceChange = static_cast<int32_t>(reference - last_change_);
    if (kTicksSinceChange <= 0) {
      return;
    }
    const float kSinceChange = static_cast<float>(kTicksSinceChange) * seconds_per_tick;
    const unit::AngularVelocity kBound{HallDecoder::kSectorWidth.Value() / kSinceChange};
    const unit::AngularVelocity kLimit = kBound * 2.0F;
    const unit::AngularVelocity kSpeed = mech_obs.omega;
    if (kSpeed > kLimit || kSpeed < -kLimit) {
      const unit::AngularVelocity kTarget = kSpeed > unit::AngularVelocity{} ? kLimit : -kLimit;
      mech_obs.UpdateSpeed(kTarget - kSpeed, r_sector_ / (kSinceChange * kSinceChange));
    }
    if (kSinceChange > kStandstillTime) {
      mech_obs.HoldStill();
    }
  }

  /// Latest Hall code seen (0 before the first event).
  [[nodiscard]] constexpr uint8_t Code() const noexcept { return code_; }

  /// Edges fused since Init().
  [[nodiscard]] constexpr uint32_t Edges() const noexcept { return edges_; }

  /// Events with an invalid code (0 or 7) since Init().
  [[nodiscard]] constexpr uint32_t InvalidCodes() const noexcept { return invalid_codes_; }

  /// Changes between sectors that are not adjacent since Init().
  [[nodiscard]] constexpr uint32_t InvalidTransitions() const noexcept { return invalid_transitions_; }

 private:
  /// Largest age of the last code change the speed bound uses [ticks].
  static constexpr int32_t kMaxAgeTicks = 0x4000'0000;

  /// Time without an edge after which the rotor counts as standing [s]: below
  /// 2 rad/s electrical a sector takes longer.
  static constexpr float kStandstillTime = 0.5F;

  HallDecoder decoder_;
  T r_edge_{static_cast<T>(3e-3)};
  T r_sector_{static_cast<T>(3e-2)};
  bool enabled_{false};
  bool valid_{false};    ///< The last cycle had a valid code.
  bool started_{false};  ///< A valid code was fused since Reset().
  uint8_t code_{0U};
  uint32_t last_change_{0U};
  uint32_t edges_{0U};
  uint32_t invalid_codes_{0U};
  uint32_t invalid_transitions_{0U};
};

}  // namespace unimoc::observer
