/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file drive_node.hpp
 *    @brief Cyphal application of the drive: arming, setpoints, telemetry,
 *           settings registers and node commands.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include <span>
#include <string_view>
#include "cart_control.hpp"
#include "commissioning.hpp"
#include "cyphal/node.hpp"
#include "ebike_control.hpp"
#include "hardware_interface.hpp"
#include "hw_startup.hpp"
#include "outer_loop.hpp"
#include "settings_register_provider.hpp"
#include "settings_store.hpp"
#include "timing_registers.hpp"

namespace unimoc::drive {

/// Platform services the drive node needs from the Zephyr glue.
struct DriveHooks {
  /// Restarts the firmware (sys_reboot); called after the response went out.
  void (*restart)() noexcept {nullptr};
  /// Called after new settings were applied to the controller (native_sim: the plant follows); may be null.
  void (*settings_applied)(const settings::NvmSettings& settings) noexcept {nullptr};
};

/**
 * @brief Read-only registers of the drive: the data type names of its ports,
 *        the hardware unique-ID, the pending settings and the bus-off count.
 */
class DriveInfoRegisters final : public cyphal::RegisterProvider {
 public:
  /**
   * @param unique_id Unique-ID reported by `uavcan.node.unique_id`; must outlive the provider.
   * @param settings_pending Reported by `unimoc.settings.pending`; must outlive the provider.
   * @param bus_off_count Reported by `unimoc.can.bus_off_count`; must outlive the provider.
   */
  DriveInfoRegisters(std::span<const uint8_t> unique_id,
                     const bool& settings_pending,
                     const uint32_t& bus_off_count) noexcept
      : unique_id_{unique_id}, settings_pending_{&settings_pending}, bus_off_count_{&bus_off_count} {}

  [[nodiscard]] std::size_t Count() const noexcept override;
  [[nodiscard]] std::string_view Name(std::size_t index) const noexcept override;
  [[nodiscard]] cyphal::RegisterFlags Read(std::size_t index,
                                           cyphal::RegisterValue& value) const noexcept override;
  [[nodiscard]] cyphal::RegisterWriteResult Write(std::size_t index,
                                                  const cyphal::RegisterValue& value) noexcept override;
  [[nodiscard]] cyphal::RegisterDescription Describe(std::size_t index) const noexcept override;

 private:
  std::span<const uint8_t> unique_id_;
  const bool* settings_pending_;
  const uint32_t* bus_off_count_;
};

/**
 * @brief Cyphal application of one drive (motor 0).
 *
 * Ports (IDs from the settings, applied at start):
 *
 * | Register | Type | Effect |
 * |---|---|---|
 * | `uavcan.sub.readiness.id` | reg.udral.service.common.Readiness.0.1 | ENGAGED releases the power
 *   stage, SLEEP/STANDBY or a timeout disables it |
 * | `uavcan.sub.current_setpoint.id` | unimoc.drive.CurrentSetpoint.0.1 | d/q current (torque mode) |
 * | `uavcan.sub.torque_setpoint.id` | uavcan.si.unit.torque.Scalar.1.0 | torque, converted to i_q (PMSM) |
 * | `uavcan.sub.speed_setpoint.id` | uavcan.si.unit.angular_velocity.Scalar.1.0 | mechanical speed (speed
 *   mode) |
 * | `uavcan.sub.position_setpoint.id` | uavcan.si.unit.angle.Scalar.1.0 | mechanical position relative to
 *   home (position mode) |
 * | `uavcan.sub.servo_setpoint.id` | reg.udral.physics.dynamics.rotation.Planar.0.1 | UDRAL servo: the
 *   first finite of position, velocity, (acceleration), torque selects the mode |
 * | `uavcan.pub.telemetry.id` | unimoc.drive.Telemetry.0.1 | controller trace at `unimoc.telemetry.rate` |
 * | `uavcan.sub.wheel_command.id` | unimoc.cart.WheelCommand.0.1 | torque of this drive's wheel (cart) |
 * | `uavcan.sub.pedal_state.id` | unimoc.cart.PedalState.0.1 | pedals for limp-home mode (cart) |
 * | `uavcan.sub.battery_status.id` | unimoc.cart.BatteryStatus.0.1 | battery current limits of the BMS |
 * | `uavcan.pub.drive_status.id` | unimoc.cart.DriveStatus.0.1 | state for the VDC at
 * `unimoc.cart.status_rate` |
 *
 * The ports of the traces (`uavcan.srv.cymon_*`, `uavcan.*.cymon_trigger`)
 * are served by trace::TraceNode; this class only reports their `.type`
 * registers.
 *
 * The last setpoint selects the control mode. The drive hands it to the
 * outer loop (current_control::OuterLoop, control thread), which runs the
 * speed and position control and passes the current reference to the
 * control cycle. The motion-profile fields of a servo setpoint (the finite
 * fields after the commanded one) are ignored, as UDRAL allows.
 *
 * Setpoints apply only while engaged and fall back to zero current after
 * `unimoc.control.command_timeout`; the current is limited to
 * `unimoc.motor.limits.i_max`. A power stage fault or an overcurrent trip
 * while engaged disengages the drive; it engages again only after a
 * readiness other than ENGAGED (an explicit re-arm). STANDBY while
 * disengaged acknowledges a latched trip or fault without releasing the
 * outputs (a still missing external enable latches it again).
 *
 * Cart (CartControl, docs/system_plan.md, section 7): the first wheel
 * command after the engage makes the drive a cart wheel drive; its torque
 * then replaces the other setpoints until the disengage. Without the wheel
 * command the drive goes to limp-home on the pedals, and the readiness
 * timeout waits while it is in limp-home or still ramping its torque down.
 * The battery status of the BMS replaces the battery limits of the settings
 * while it is fresh. A bus-off counts as the loss of every input at once;
 * the controller rejoins the bus on its own and the drive logs it.
 *
 * The hardware startup aid (startup::HwStartup) runs in this thread through
 * the commissioning commands and status message (Commissioning). It owns the outputs
 * while it runs: the drive does not engage then, and a run is refused while
 * engaged. When it ends, the controller restarts with its calibration.
 *
 * Settings registers change the RAM settings. They are applied to the
 * controller (`current_control::StartRuntime()`) while the drive is
 * disengaged, at the latest when it engages; ExecuteCommand
 * STORE_PERSISTENT_STATES saves them to flash.
 *
 * Runs in the Cyphal thread; it reaches the control only through
 * `OuterLoop::SetSetpoint()` and reads the controller state for the trace.
 */
class DriveNode final : public cyphal::MessageListener,
                        public cyphal::CommandHandler,
                        public cyphal::NodeIdListener {
 public:
  /// Delay between the RESTART response and the restart [us].
  static constexpr uint64_t kRestartDelayUs = 200'000U;

  /**
   * @param node Cyphal node on the platform transport.
   * @param store Settings store of the firmware.
   * @param motor Hardware access of the motor the drive controls.
   * @param hooks Platform services.
   * @param unique_id Hardware unique-ID for `uavcan.node.unique_id`; must outlive the drive node.
   */
  DriveNode(cyphal::CyphalNode& node,
            settings::SettingsStore& store,
            const hardware::HardwareInterface& motor,
            const DriveHooks& hooks,
            std::span<const uint8_t> unique_id) noexcept;

  /**
   * @brief Adds the registers, subscribes the ports and starts the node.
   * @param now_us Monotonic time [us].
   * @return False when a port could not be subscribed.
   */
  [[nodiscard]] bool Start(uint64_t now_us) noexcept;

  /// Runs the node and the drive logic; call every millisecond or faster.
  void Spin(uint64_t now_us) noexcept;

  /// True while the power stage is released for the setpoint.
  [[nodiscard]] bool IsEngaged() const noexcept { return engaged_; }

  /**
   * @brief Makes the e-bike control the local source of the engagement and of the torque.
   *
   * With a local control the readiness ENGAGED of the bus no longer engages the drive (STANDBY and SLEEP
   * still disengage it and acknowledge a latched fault); the control asks for the drive while the rider needs
   * the assist, and its torque replaces the other setpoints while the drive is engaged. Call before Start().
   *
   * @param control E-bike control; must outlive the drive node. Null removes it.
   */
  void SetLocalControl(ebike::EbikeControl* control) noexcept { ebike_ = control; }

  /// The startup FSM of the drive (state and results).
  [[nodiscard]] const startup::HwStartup& Startup() const noexcept { return startup_; }

  void OnMessage(const cyphal::Transfer& transfer, uint64_t now_us) noexcept override;
  [[nodiscard]] cyphal::CommandResult OnCommand(uint16_t command,
                                                std::span<const uint8_t> parameter) noexcept override;
  void OnNodeIdAllocated(uint8_t node_id) noexcept override;

 private:
  void OnReadiness(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnCurrentSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnTorqueSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnSpeedSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnPositionSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnServoSetpoint(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnWheelCommand(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnPedalState(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  void OnBatteryStatus(const cyphal::Transfer& transfer, uint64_t now_us) noexcept;
  /// Follows the bus state; a bus-off drops the cart inputs.
  void WatchBus() noexcept;
  void SetCurrentSetpoint(unit::Current i_d, unit::Current i_q, uint64_t now_us) noexcept;
  void SetSpeedSetpoint(float speed, uint64_t now_us) noexcept;
  void SetPositionSetpoint(float position, uint64_t now_us) noexcept;
  /// Back to zero current in torque mode (no setpoint).
  void ClearSetpoint() noexcept;
  /// Runs the startup FSM and restarts the controller after it ends.
  void SpinStartup() noexcept;
  /// The e-bike control: engagement on demand and its torque (only with a local control).
  void SpinLocalControl(uint64_t now_us) noexcept;
  /// Cart torque, readiness and fault checks, setpoint timeout, battery limits.
  void SpinEngagement(uint64_t now_us) noexcept;
  void Engage() noexcept;
  void Disengage(std::string_view reason) noexcept;
  /// Takes the timing and limits of the drive from the live settings.
  void LoadParameters(const settings::NvmSettings& settings) noexcept;
  /// Restarts the controller with the live settings; refused while the outputs are released.
  [[nodiscard]] bool ApplySettings() noexcept;
  /// While engaged: hands changed gains to the running loops; other changes wait for the disengage.
  void TuneLive() noexcept;
  void PublishTelemetry() noexcept;
  void PublishDriveStatus() noexcept;
  void PublishCommissioning(uint64_t now_us) noexcept;
  void UpdateStatus() noexcept;

  cyphal::CyphalNode* node_;
  settings::SettingsStore* store_;
  const hardware::HardwareInterface* motor_;
  DriveHooks hooks_;
  SettingsRegisterProvider settings_registers_;
  DriveInfoRegisters info_registers_;
  startup::HwStartup startup_;
  Commissioning commissioning_;
  TimingRegisters timing_registers_{};
  bool startup_active_{false};

  // Settings captured at start or at the last apply.
  uint16_t readiness_port_{cyphal::kUnsetPortId};
  uint16_t current_setpoint_port_{cyphal::kUnsetPortId};
  uint16_t torque_setpoint_port_{cyphal::kUnsetPortId};
  uint16_t speed_setpoint_port_{cyphal::kUnsetPortId};
  uint16_t position_setpoint_port_{cyphal::kUnsetPortId};
  uint16_t servo_setpoint_port_{cyphal::kUnsetPortId};
  uint16_t wheel_command_port_{cyphal::kUnsetPortId};
  uint16_t pedal_state_port_{cyphal::kUnsetPortId};
  uint16_t battery_status_port_{cyphal::kUnsetPortId};
  uint64_t command_timeout_us_{500'000U};
  uint64_t telemetry_period_us_{0U};
  uint64_t drive_status_period_us_{0U};
  uint8_t cart_wheel_{0U};
  unit::Current current_limit_;
  unit::MagneticFlux torque_constant_;  ///< PMSM 3/2 p psi_pm [N m/A = Wb]
  uint32_t applied_writes_{0U};
  uint32_t checked_writes_{0U};             ///< Register writes looked at for live tuning.
  settings::NvmSettings applied_settings_;  ///< What the controller runs with.
  bool settings_pending_{false};
  bool settings_waiting_{false};   ///< Writes not applied yet (`unimoc.settings.pending`).
  bool waiting_announced_{false};  ///< A record said that settings wait for the disengage.

  cyphal::Publisher telemetry_{};
  uint64_t next_telemetry_us_{0U};
  cyphal::Publisher drive_status_{};
  cyphal::Publisher commissioning_publisher_{};
  uint64_t next_drive_status_us_{0U};

  CartControl cart_{};
  CartMode cart_mode_{CartMode::kOff};
  ebike::EbikeControl* ebike_{nullptr};  ///< Local source of engagement and torque; null: the bus decides.
  bool ebike_engage_{false};             ///< The e-bike control asks for the drive.
  unit::Torque ebike_torque_;            ///< Its torque, rotor frame.
  bool bus_lost_{false};                 ///< Bus-off seen and not yet back on the bus.
  uint32_t bus_off_count_{0U};           ///< `unimoc.can.bus_off_count`.

  bool engaged_{false};
  bool rearm_required_{false};
  bool emergency_stop_{false};
  bool engage_requested_{false};
  uint64_t last_readiness_us_{0U};
  uint64_t last_setpoint_us_{0U};
  bool setpoint_active_{false};
  /// Last setpoint; `active` is set from engaged_ when it is handed over.
  current_control::OuterSetpoint setpoint_{};
  bool restart_requested_{false};
  uint64_t restart_at_us_{0U};
  uint64_t now_us_{0U};
};

}  // namespace unimoc::drive
