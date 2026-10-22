/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file trace_signals.cpp
 *    @brief Float mirror of the control state and the trace device (cymon).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "trace_signals.hpp"
#include <array>
#include <atomic>
#include <optional>
#include "current_control_isr.hpp"

namespace unimoc::trace {

namespace {

/**
 * @brief Static storage of the traces.
 *
 * The device is constructed by the first TraceDevice() call (Cyphal thread):
 * its default state is not all zero, so a constant-initialized object would
 * be copied from flash (.data). In the optional it stays in .bss. The control
 * cycle samples it once it is published in `device`.
 */
struct TraceStorage {
  std::optional<cymon::Device> device_storage;
  std::atomic<cymon::Device*> device{nullptr};
  TraceSignals signals{};
};
static_assert(std::atomic<cymon::Device*>::is_always_lock_free, "read in the zero-latency context");

/// constinit: constant-initialized, so the control cycle reaches it without a guard variable.
TraceStorage& Storage() noexcept {
  constinit static TraceStorage storage{};
  return storage;
}

struct SignalInfo {
  const char* name;
  const char* unit;
  const float* source;
};

}  // namespace

cymon::Device& TraceDevice() noexcept {
  TraceStorage& storage = Storage();
  cymon::Device* device = storage.device.load(std::memory_order_acquire);
  if (device == nullptr) {
    device = &storage.device_storage.emplace();
    storage.device.store(device, std::memory_order_release);
  }
  return *device;
}

const TraceSignals& Signals() noexcept { return Storage().signals; }

bool RegisterControlSignals() noexcept {
  TraceSignals& signals = Storage().signals;
  const std::array<SignalInfo, 12> kSignals{{
      {"i_d", "A", &signals.i_d},
      {"i_q", "A", &signals.i_q},
      {"i_d_ref", "A", &signals.i_d_ref},
      {"i_q_ref", "A", &signals.i_q_ref},
      {"u_d", "V", &signals.u_d},
      {"u_q", "V", &signals.u_q},
      {"electrical_angle", "rad", &signals.electrical_angle},
      {"electrical_speed", "rad/s", &signals.electrical_speed},
      {"dc_link_voltage", "V", &signals.dc_link_voltage},
      {"i_a", "A", &signals.i_a},
      {"i_b", "A", &signals.i_b},
      {"i_c", "A", &signals.i_c},
  }};
  bool registered = true;
  for (const SignalInfo& signal : kSignals) {
    registered = TraceDevice().RegisterVariable(signal.name, signal.unit, signal.source) && registered;
  }
  return registered;
}

void OnControlCycle(const current_control::CurrentControlIsr& isr) noexcept {
  const current_control::CurrentControlState& state = isr.state;
  TraceSignals& signals = Storage().signals;
  signals.i_d = state.i_dq_last.d.Value();
  signals.i_q = state.i_dq_last.q.Value();
  signals.i_d_ref = state.i_ref.d.Value();
  signals.i_q_ref = state.i_ref.q.Value();
  signals.u_d = state.u_dq_last.d.Value();
  signals.u_q = state.u_dq_last.q.Value();
  signals.electrical_angle = isr.mech_obs.theta.Value();
  signals.electrical_speed = isr.mech_obs.omega.Value();
  signals.dc_link_voltage = state.raw_vdc.Value();
  signals.i_a = state.raw_phase_currents.a.Value();
  signals.i_b = state.raw_phase_currents.b.Value();
  signals.i_c = state.raw_phase_currents.c.Value();
  cymon::Device* const kDevice = Storage().device.load(std::memory_order_acquire);
  if (kDevice != nullptr) {
    (void)kDevice->Tick();
  }
}

}  // namespace unimoc::trace
