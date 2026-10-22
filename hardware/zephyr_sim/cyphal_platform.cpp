/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_platform.cpp
 *    @brief Cyphal platform part of the native_sim drive.
 *
 *    - A serial-line CAN interface on a pseudo terminal (slcan_media.hpp)
 *      next to the SocketCAN interface, for hosts without vcan.
 *    - Registers of the simulated plant (`unimoc.sim.*`, volatile): shaft
 *      mode, imposed speed, load torque, DC-link voltage, external enable,
 *      and whether the plant follows the motor settings.
 *    - The plant state (unimoc.sim.Plant.0.1) on `uavcan.pub.sim_plant.id`
 *      at the telemetry rate, to compare with the controller's estimates.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal_platform.hpp"
#include <unimoc/sim/Plant_0_1.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <array>
#include <cmath>
#include <numbers>
#include <string_view>
#include <tuple>
#include "cyphal/register_value.hpp"
#include "sim_cmdline.h"
#include "sim_control.hpp"
#include "slcan_media.hpp"

LOG_MODULE_DECLARE(unimoc_cyphal, CONFIG_UNIMOC_LOG_LEVEL);

namespace unimoc::hardware {

namespace {

using cyphal::RegisterFlags;
using cyphal::RegisterValue;
using cyphal::RegisterWriteResult;

/// Volatile state of the simulated plant that the registers set.
struct PlantInputs {
  ::unimoc::sim::ShaftMode shaft_mode{::unimoc::sim::ShaftMode::kFree};
  float imposed_speed{0.0F};  ///< Mechanical speed in ShaftMode::kImposedSpeed [rad/s].
  float load_torque{0.0F};    ///< Load torque [N m]; positive opposes positive rotation.
  float dc_link_voltage{48.0F};
  bool external_enable{true};
  bool follow_settings{true};
};

PlantInputs g_inputs;
cyphal::Publisher g_plant_publisher{cyphal::kPubSimPlant, cyphal::Priority::kHigh, 0U};
uint64_t g_plant_period_us = 0U;
uint64_t g_next_plant_us = 0U;

/// Applies the cached inputs to the plant; the cycle timer is held off meanwhile.
void ApplyInputs() noexcept {
  const unsigned int kKey = irq_lock();
  auto& plant = sim::Plant();
  plant.SetShaftMode(g_inputs.shaft_mode, unit::AngularVelocity{g_inputs.imposed_speed});
  plant.SetInput(::unimoc::sim::SimulationInput{unit::Torque{g_inputs.load_torque}, unit::Voltage{}});
  plant.SetDcLinkVoltage(unit::Voltage{g_inputs.dc_link_voltage});
  irq_unlock(kKey);
  sim::SetExternalEnable(g_inputs.external_enable);
}

enum class SimRegister : uint8_t {
  kShaftMode,
  kImposedSpeed,
  kLoadTorque,
  kDcLinkVoltage,
  kExternalEnable,
  kFollowSettings,
  kPlantId,
  kPlantType,
};

constexpr std::array<std::string_view, 8> kNames{
    "unimoc.sim.shaft_mode",
    "unimoc.sim.imposed_speed",
    "unimoc.sim.load_torque",
    "unimoc.sim.dc_link_voltage",
    "unimoc.sim.external_enable",
    "unimoc.sim.plant_follows_settings",
    "uavcan.pub.sim_plant.id",
    "uavcan.pub.sim_plant.type",
};

/// Registers of the simulated plant; not persistent, the plant starts from the defaults.
class SimRegisters final : public cyphal::RegisterProvider {
 public:
  [[nodiscard]] std::size_t Count() const noexcept override { return kNames.size(); }

  [[nodiscard]] std::string_view Name(const std::size_t index) const noexcept override {
    return index < kNames.size() ? kNames[index] : std::string_view{};
  }

  [[nodiscard]] RegisterFlags Read(const std::size_t index, RegisterValue& value) const noexcept override {
    switch (static_cast<SimRegister>(index)) {
      case SimRegister::kShaftMode:
        cyphal::SetNatural8(value, static_cast<uint8_t>(g_inputs.shaft_mode));
        break;
      case SimRegister::kImposedSpeed:
        cyphal::SetReal32(value, g_inputs.imposed_speed);
        break;
      case SimRegister::kLoadTorque:
        cyphal::SetReal32(value, g_inputs.load_torque);
        break;
      case SimRegister::kDcLinkVoltage:
        cyphal::SetReal32(value, g_inputs.dc_link_voltage);
        break;
      case SimRegister::kExternalEnable:
        cyphal::SetBit(value, g_inputs.external_enable);
        break;
      case SimRegister::kFollowSettings:
        cyphal::SetBit(value, g_inputs.follow_settings);
        break;
      case SimRegister::kPlantId:
        cyphal::SetNatural16(value, g_plant_publisher.port_id);
        break;
      case SimRegister::kPlantType:
        cyphal::SetString(value, "unimoc.sim.Plant.0.1");
        return {false, false};
    }
    return {true, false};
  }

  [[nodiscard]] RegisterWriteResult Write(const std::size_t index,
                                          const RegisterValue& value) noexcept override {
    const auto kNumber = cyphal::GetNumber(value);
    if (!kNumber.has_value()) {
      return RegisterWriteResult::kTypeMismatch;
    }
    const float kValue = *kNumber;
    if (!std::isfinite(kValue)) {
      return RegisterWriteResult::kOutOfRange;
    }
    switch (static_cast<SimRegister>(index)) {
      case SimRegister::kShaftMode:
        if (kValue != 0.0F && kValue != 1.0F) {
          return RegisterWriteResult::kOutOfRange;
        }
        g_inputs.shaft_mode =
            (kValue == 0.0F) ? ::unimoc::sim::ShaftMode::kFree : ::unimoc::sim::ShaftMode::kImposedSpeed;
        break;
      case SimRegister::kImposedSpeed:
        g_inputs.imposed_speed = kValue;
        break;
      case SimRegister::kLoadTorque:
        g_inputs.load_torque = kValue;
        break;
      case SimRegister::kDcLinkVoltage:
        if (kValue <= 0.0F) {
          return RegisterWriteResult::kOutOfRange;
        }
        g_inputs.dc_link_voltage = kValue;
        break;
      case SimRegister::kExternalEnable:
        g_inputs.external_enable = kValue != 0.0F;
        break;
      case SimRegister::kFollowSettings:
        g_inputs.follow_settings = kValue != 0.0F;
        return RegisterWriteResult::kAccepted;
      case SimRegister::kPlantId: {
        const auto kId = cyphal::GetInteger(value);
        if (!kId.has_value() || (*kId > 8191 && *kId != cyphal::kUnsetPortId) || *kId < 0) {
          return RegisterWriteResult::kOutOfRange;
        }
        g_plant_publisher.port_id = static_cast<uint16_t>(*kId);
        return RegisterWriteResult::kAccepted;
      }
      case SimRegister::kPlantType:
        return RegisterWriteResult::kReadOnly;
    }
    ApplyInputs();
    return RegisterWriteResult::kAccepted;
  }
};

SimRegisters g_registers;

// The slcan pseudo terminals: the slcan-uarts phandles of zephyr,user (app overlay).
#define UNIMOC_SLCAN_NODE DT_PATH(zephyr_user)
#if defined(CONFIG_UNIMOC_CYPHAL_SLCAN) && DT_NODE_HAS_PROP(UNIMOC_SLCAN_NODE, slcan_uarts)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage): DT_FOREACH_PROP_ELEM takes a macro.
#define UNIMOC_SLCAN_UART(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),
const std::array kSlcanUarts{DT_FOREACH_PROP_ELEM(UNIMOC_SLCAN_NODE, slcan_uarts, UNIMOC_SLCAN_UART)};
#undef UNIMOC_SLCAN_UART
sim::SlcanMedia g_slcan;
std::array<cyphal::CanMedia*, 1> g_interfaces{};
#else
std::array<cyphal::CanMedia*, 0> g_interfaces{};
#endif

void SetPlantRate(const settings::NvmSettings& settings) noexcept {
  const float kRate = settings.telemetry_rate.Value();
  g_plant_period_us = (kRate > 0.0F) ? static_cast<uint64_t>(1.0e6F / kRate) : 0U;
}

}  // namespace

uint8_t CyphalPlatformNodeIdOverride() noexcept {
  const int32_t kNodeId = UnimocSimNodeIdOption();
  return (kNodeId >= 0 && kNodeId <= cyphal::kNodeIdMax) ? static_cast<uint8_t>(kNodeId)
                                                         : cyphal::kNodeIdUnset;
}

void CyphalPlatformInit() noexcept {
#if defined(CONFIG_UNIMOC_CYPHAL_SLCAN) && DT_NODE_HAS_PROP(UNIMOC_SLCAN_NODE, slcan_uarts)
  const std::size_t kPorts = g_slcan.Init(kSlcanUarts);
  if (kPorts > 0U) {
    g_interfaces[0] = &g_slcan;
    LOG_INF("slcan CAN interface on %u pseudo terminals (uart_1.. in the lines above)",
            static_cast<unsigned>(kPorts));
  }
#endif
}

std::span<cyphal::CanMedia* const> CyphalPlatformInterfaces() noexcept { return g_interfaces; }

void CyphalPlatformStart(cyphal::CyphalNode& node, const settings::NvmSettings& settings) noexcept {
  (void)node.AddRegisterProvider(g_registers);
  (void)node.AddPublisher(g_plant_publisher);
  SetPlantRate(settings);
  ApplyInputs();
}

void CyphalPlatformTraceSignals(cymon::Device& device) noexcept {
  const sim::PlantTrace& plant = sim::PlantTraceSignals();
  const std::array<std::tuple<const char*, const char*, const float*>, 9> kSignals{{
      {"sim.i_a", "A", &plant.i_a},
      {"sim.i_b", "A", &plant.i_b},
      {"sim.i_c", "A", &plant.i_c},
      {"sim.i_d", "A", &plant.i_d},
      {"sim.i_q", "A", &plant.i_q},
      {"sim.electrical_angle", "rad", &plant.electrical_angle},
      {"sim.electrical_speed", "rad/s", &plant.electrical_speed},
      {"sim.mechanical_speed", "rad/s", &plant.mechanical_speed},
      {"sim.torque", "N m", &plant.torque},
  }};
  for (const auto& [name, unit, source] : kSignals) {
    if (!device.RegisterVariable(name, unit, source)) {
      LOG_WRN("trace variable %s not registered", name);
    }
  }
}

void CyphalPlatformSpin(cyphal::CyphalNode& node, const uint64_t now_us) noexcept {
  if (g_plant_period_us == 0U || now_us < g_next_plant_us) {
    return;
  }
  g_next_plant_us = now_us + g_plant_period_us;

  const sim::Snapshot kSnapshot = sim::TakeSnapshot();
  const auto& output = kSnapshot.plant;
  constexpr float kTwoPi = 2.0F * std::numbers::pi_v<float>;
  float angle = std::fmod(output.electrical_angle.AbsoluteAngle().Value(), kTwoPi);
  angle = (angle < 0.0F) ? angle + kTwoPi : angle;

  unimoc_sim_Plant_0_1 plant{};
  plant.control_cycles = kSnapshot.cycle_count;
  plant.i_a = output.phase_currents.a.Value();
  plant.i_b = output.phase_currents.b.Value();
  plant.i_c = output.phase_currents.c.Value();
  plant.mechanical_angle = output.mechanical_angle.AbsoluteAngle().Value();
  plant.mechanical_speed = output.mechanical_speed.Value();
  plant.electrical_angle = angle;
  plant.electromagnetic_torque = output.electromagnetic_torque.Value();
  plant.dc_link_voltage = output.dc_link_voltage.Value();
  plant.hall_code = output.hall.code;

  std::array<std::byte, unimoc_sim_Plant_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_> buffer{};
  std::size_t size = buffer.size();
  if (unimoc_sim_Plant_0_1_serialize_(&plant, reinterpret_cast<uint8_t*>(buffer.data()), &size) >= 0) {
    (void)node.Publish(g_plant_publisher, std::span<const std::byte>{buffer.data(), size});
  }
}

void CyphalPlatformSettingsApplied(const settings::NvmSettings& settings) noexcept {
  SetPlantRate(settings);
  if (!g_inputs.follow_settings) {
    return;
  }
  // The simulated motor takes the motor parameters of the settings; the
  // plant restarts at standstill with the inputs of the registers.
  const unsigned int kKey = irq_lock();
  const auto kStatus = sim::Plant().Configure(settings);
  irq_unlock(kKey);
  if (kStatus != ::unimoc::sim::Status::kOk) {
    LOG_WRN("plant rejected the motor settings (%d)", static_cast<int>(kStatus));
  }
  ApplyInputs();
}

}  // namespace unimoc::hardware
