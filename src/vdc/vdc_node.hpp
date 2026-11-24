/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file vdc_node.hpp
 *    @brief Cyphal application of the vehicle dynamics controller (VDC): readiness of the
 *           drives, wheel command from a torque request, the drive status collected.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include "cyphal/node.hpp"
#include "cyphal/register_value.hpp"
#include "drive_monitor.hpp"
#include "pedal_map.hpp"
#include "units.hpp"
#include "wheel.hpp"

/**
 * @namespace unimoc::vdc the VDC application (apps/vdc)
 */
namespace unimoc::vdc {

using unit::operator""_Hz;
using unit::operator""_Nm;
using unit::operator""_km_per_h;
using unit::operator""_m_per_rad;
using unit::operator""_m_per_s2;
using unit::operator""_ratio;
using unit::operator""_s;

/// Value of a drive node-ID meaning "no drive on this wheel".
inline constexpr uint8_t kNoDrive = 0xFFU;

/**
 * @brief Factory parameters of the pedal map (docs/cart_mechanics_plan.md).
 *
 * The full drive torque in every gear; the gears differ in the speed limit
 * and the speed ramp: reverse 4 km/h, first gear (children, no release) 6
 * km/h, second 12 km/h, third 20 km/h without a ramp.
 */
[[nodiscard]] constexpr vehicle::PedalMapParameters FactoryPedalMap() noexcept {
  vehicle::PedalMapParameters parameters;
  parameters.gears[vehicle::GearIndex(vehicle::kReverse)] = {4.0_km_per_h, 0.5_m_per_s2};
  parameters.gears[vehicle::GearIndex(1)] = {6.0_km_per_h, 0.5_m_per_s2};
  parameters.gears[vehicle::GearIndex(2)] = {12.0_km_per_h, 1.0_m_per_s2};
  parameters.gears[vehicle::GearIndex(3)] = {20.0_km_per_h, 0.0_m_per_s2};
  parameters.drive_torque = 30.0_Nm;
  parameters.brake_torque = 30.0_Nm;
  return parameters;
}

/**
 * @brief Parameters of the VDC, the registers `unimoc.vdc.*`.
 *
 * The defaults match the simulated cluster (`scripts/sim_cluster.sh`: drive
 * k with node-ID 10 + k on wheel k). Not stored: every start begins with them.
 */
struct VdcParameters {
  /// Node-ID of the drive of each wheel, kNoDrive for none.
  vehicle::PerWheel<uint8_t> drive_node_ids{10U, 11U, 12U, 13U};
  /// The VDC arms its drives (readiness ENGAGED) instead of holding them in STANDBY.
  bool engage{false};
  /// Total wheel torque, positive forward.
  unit::Torque torque_request;
  /// Share of the front axle, 0 to 1.
  unit::Ratio front_share{0.5_ratio};
  /// Rate of the wheel command (system plan section 6.3: 250 Hz to start).
  unit::Frequency cycle_rate{250.0_Hz};
  /// Rate of the readiness of each drive; well inside the 0.5 s readiness timeout of the drives.
  unit::Frequency readiness_rate{10.0_Hz};
  /// Age after which the drive status of a wheel counts as lost (the drives publish at 50 Hz).
  unit::Time status_timeout{0.1_s};
  /// Rolling radius of the wheels: wheel speed times it is the speed of the cart (4.00-6 tyre).
  unit::LengthPerAngle rolling_radius{0.165_m_per_rad};
  /// The torque request comes from the pedal state instead of torque_request.
  bool pedal_control{false};
  /// Age after which the pedal state counts as lost.
  unit::Time pedal_timeout{0.1_s};
  /// Pedal map: gears, drive and brake torque, speed limits and ramps.
  vehicle::PedalMapParameters pedal_map{FactoryPedalMap()};
};

/**
 * @brief The VDC skeleton on a Cyphal node.
 *
 * - Publishes the readiness of each configured drive on 1024 + its node-ID:
 *   ENGAGED while `unimoc.vdc.engage` is set, STANDBY otherwise.
 * - Collects the unimoc.cart.DriveStatus of the four wheels (vehicle::DriveMonitor).
 * - Publishes the unimoc.cart.WheelCommand at the cycle rate: the torque
 *   request split onto the available wheels (vehicle::SplitTorque()), zero
 *   for every wheel while the VDC does not engage its drives. The request
 *   is `unimoc.vdc.torque_request`, or with `unimoc.vdc.pedal_control` the
 *   pedal map of the unimoc.cart.PedalState and the gear (vehicle::PedalMap:
 *   brake first, reverse, speed limit and speed ramp per gear, higher gears
 *   only with the volatile release; a lost pedal state brakes gently).
 * - Reports the collected state in read-only registers and the node health:
 *   ADVISORY while a configured drive is not available (not fresh, or not
 *   engaged while the VDC engages) or the pedal state is lost under pedal
 *   control.
 *
 * Runs in the Cyphal thread of the VDC; free of Zephyr headers.
 */
class VdcNode final : public cyphal::MessageListener, public cyphal::RegisterProvider {
 public:
  /// The register providers and subscriptions are added in Start().
  explicit VdcNode(cyphal::CyphalNode& node) noexcept : node_{&node} {}

  /**
   * @brief Adds the registers, ports and publishers and starts the node.
   * @param node_id Node-ID of the VDC, 0 to 127 (larger: plug-and-play).
   * @param now_us Monotonic time [us].
   * @return False when a port could not be added or the node did not start.
   */
  [[nodiscard]] bool Start(uint8_t node_id, uint64_t now_us) noexcept;

  /**
   * @brief Spins the node and publishes what is due.
   * @param now_us Monotonic time [us].
   */
  void Spin(uint64_t now_us) noexcept;

  /// The parameters in effect.
  [[nodiscard]] const VdcParameters& Parameters() const noexcept { return parameters_; }

  /// The collected drive status.
  [[nodiscard]] const vehicle::DriveMonitor& Drives() const noexcept { return drives_; }

  /// The pedal map (gear, release, last request).
  [[nodiscard]] const vehicle::PedalMap& PedalControl() const noexcept { return pedal_map_; }

  /// Wheel torques of the last wheel command.
  [[nodiscard]] const vehicle::PerWheel<unit::Torque>& WheelTorques() const noexcept {
    return wheel_torques_;
  }

  /// Readiness subject of the drive of @p wheel, kUnsetPortId without a drive.
  [[nodiscard]] uint16_t ReadinessPort(vehicle::Wheel wheel) const noexcept;

  // cyphal::MessageListener
  void OnMessage(const cyphal::Transfer& transfer, uint64_t now_us) noexcept override;

  // cyphal::RegisterProvider
  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;
  [[nodiscard]] cyphal::RegisterDescription Describe(std::size_t index) const noexcept override;

 private:
  void OnDriveStatus(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnPedalState(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  /// The total torque request of this cycle.
  [[nodiscard]] unit::Torque TorqueRequest(uint64_t now_us) noexcept;
  /// A pedal state arrived and is not older than the pedal timeout.
  [[nodiscard]] bool PedalsFresh(uint64_t now_us) const noexcept;
  /// Speed of the cart: the mean wheel speed of the fresh drives times the rolling radius.
  [[nodiscard]] unit::Velocity CartSpeed(uint64_t now_us) const noexcept;
  void PublishWheelCommand(uint64_t now_us) noexcept;
  void PublishReadiness() noexcept;
  void UpdateHealth(uint64_t now_us) noexcept;
  /// The gear limits a reverse.* or gear<n>.* register addresses.
  [[nodiscard]] const vehicle::GearLimits& LimitsOf(std::size_t index) const noexcept;
  /// Writes a pedal map register and hands the map to the pedal control.
  void WritePedalMap(std::size_t index, float number) noexcept;
  /// Writes the gear or the gear release.
  [[nodiscard]] cyphal::RegisterWriteResult WriteGear(std::size_t index,
                                                      std::optional<float> number) noexcept;
  /// Applies a changed rate or timeout to the schedule.
  void ApplyTiming() noexcept;

  cyphal::CyphalNode* node_;
  VdcParameters parameters_;
  vehicle::DriveMonitor drives_;
  vehicle::PerWheel<unit::Torque> wheel_torques_{};
  vehicle::PedalMap pedal_map_;
  vehicle::Pedals pedals_;
  bool has_pedals_{false};
  uint64_t pedals_us_{0U};
  bool has_request_cycle_{false};
  uint64_t request_cycle_us_{0U};
  cyphal::Publisher wheel_command_;
  vehicle::PerWheel<uint8_t> readiness_transfer_ids_{};
  uint64_t now_us_{0U};
  uint64_t cycle_period_us_{0U};
  uint64_t readiness_period_us_{0U};
  uint64_t next_cycle_us_{0U};
  uint64_t next_readiness_us_{0U};
};

}  // namespace unimoc::vdc
