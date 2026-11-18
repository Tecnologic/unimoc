/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_excitation.cpp
 *    @brief The simulated excitation module of an EESM on native_sim.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "sim_excitation.hpp"
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include "sim_control.hpp"

namespace unimoc::hardware::sim {

namespace {

using unit::operator""_ms;
using unit::operator""_us;

/// Closed-loop time constant of the field current loop.
constexpr unit::Time kLoopTimeConstant = 5.0_ms;
/// A step longer than this (the first command, a pause) counts as one millisecond.
constexpr uint64_t kMaxStepUs = 20'000U;
constexpr uint64_t kDefaultStepUs = 1'000U;

struct Module {
  bool started{false};
  bool enabled{false};
  uint64_t last_us{0U};
  unit::Voltage integrator;
  unit::Voltage field_voltage;
};
Module g_module;

void ApplyFieldVoltage(const unit::Voltage voltage) noexcept {
  const unsigned int kKey = irq_lock();
  auto& plant = Plant();
  ::unimoc::sim::SimulationInput input = plant.Input();
  input.field_voltage = voltage;
  plant.SetInput(input);
  irq_unlock(kKey);
}

}  // namespace

void ExcitationCommand(const unit::Current field_current, const bool enable, const uint64_t now_us) noexcept {
  const uint64_t kStepUs = (g_module.started && now_us - g_module.last_us <= kMaxStepUs)
                               ? now_us - g_module.last_us
                               : kDefaultStepUs;
  g_module.started = true;
  g_module.last_us = now_us;
  g_module.enabled = enable;
  const auto& plant = Plant();
  const ::unimoc::sim::EesmParameters& winding = plant.Parameters().eesm;
  if (!enable || plant.Type() != system::MotorType::kEesm) {
    g_module.integrator = unit::Voltage{};
    g_module.field_voltage = unit::Voltage{};
    ApplyFieldVoltage(unit::Voltage{});
    return;
  }
  const unit::Time kStep = 1.0_us * static_cast<float>(kStepUs);
  const unit::Current kError = field_current - plant.Output().field_current;
  const unit::Voltage kRail = plant.Output().dc_link_voltage;
  // kp = L_f / tau, ki = R_f / tau: the loop follows the reference with tau.
  const unit::Resistance kProportional = winding.field_inductance / kLoopTimeConstant;
  const unit::InverseTime kIntegral = winding.field_resistance / winding.field_inductance;
  g_module.integrator =
      (g_module.integrator + ((kProportional * kError) * (kIntegral * kStep))).Clamp(unit::Voltage{}, kRail);
  g_module.field_voltage = ((kProportional * kError) + g_module.integrator).Clamp(unit::Voltage{}, kRail);
  ApplyFieldVoltage(g_module.field_voltage);
}

bool ExcitationStatus(drive::ExcitationState& state) noexcept {
  state = drive::ExcitationState{.field_current = Plant().Output().field_current,
                                 .field_voltage = g_module.field_voltage,
                                 .enabled = g_module.enabled,
                                 .fault = false};
  return true;
}

void ResetExcitation() noexcept {
  g_module = Module{};
  ApplyFieldVoltage(unit::Voltage{});
}

}  // namespace unimoc::hardware::sim
