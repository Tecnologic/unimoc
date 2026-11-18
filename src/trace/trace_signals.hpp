/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file trace_signals.hpp
 *    @brief Float mirror of the control state and the trace device (cymon).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "cymon/device.hpp"

namespace unimoc::current_control {
class CurrentControlIsr;
struct OuterStatus;
}  // namespace unimoc::current_control

/**
 * @namespace unimoc::trace
 * @brief Traces of the drive: cymon captures of the control state over Cyphal.
 */
namespace unimoc::trace {

/**
 * @brief Plain float copy of the control state, sampled by the traces (SI units).
 *
 * The control state is held in unit types; cymon reads `const float*`. The
 * control cycle fills this mirror at its end (OnControlCycle()), then the
 * trace device samples it. Each field is registered once as a trace variable
 * (RegisterControlSignals()).
 */
struct TraceSignals {
  float i_d{0.0F};               ///< Measured d current of the last control step [A].
  float i_q{0.0F};               ///< Measured q current of the last control step [A].
  float i_d_ref{0.0F};           ///< d current reference of the cycle [A].
  float i_q_ref{0.0F};           ///< q current reference of the cycle [A].
  float u_d{0.0F};               ///< d voltage demand [V].
  float u_q{0.0F};               ///< q voltage demand [V].
  float electrical_angle{0.0F};  ///< Estimated electrical angle, [0, 2 pi) [rad]; one cycle late.
  float electrical_speed{0.0F};  ///< Estimated electrical speed [rad/s]; one cycle late.
  float dc_link_voltage{0.0F};   ///< DC-link voltage [V].
  float i_a{0.0F};               ///< Phase current a, newest sample of the cycle [A].
  float i_b{0.0F};               ///< Phase current b, newest sample of the cycle [A].
  float i_c{0.0F};               ///< Phase current c, newest sample of the cycle [A].
  // The thermal model (docs/motor_thermal_plan.md), written by the Cyphal
  // thread (SetThermalSignals()), single aligned words.
  float winding_temperature{0.0F};  ///< Estimated winding temperature [degC].
  float stator_temperature{0.0F};   ///< Estimated stator temperature [degC].
  float rs_estimate{0.0F};          ///< Last stator resistance estimate of the observer [Ohm].
  float rs_temperature{0.0F};       ///< Winding temperature of that estimate [degC].
};

/**
 * @brief The trace device of the drive, a static object (.bss) of about CONFIG_UNIMOC_TRACE_BUFFER_BYTES.
 *
 * Thread context only (the Cyphal thread). The first call constructs it;
 * the control cycle samples it from then on.
 */
[[nodiscard]] cymon::Device& TraceDevice() noexcept;

/// The mirror the control cycle fills.
[[nodiscard]] const TraceSignals& Signals() noexcept;

/**
 * @brief Copies the state of the thermal model into the mirror (Cyphal thread).
 *
 * The values change every 0.1 s; the thread writes them, the control cycle
 * samples them with the rest.
 *
 * @param status State of the outer loop.
 */
void SetThermalSignals(const current_control::OuterStatus& status) noexcept;

/**
 * @brief Registers the TraceSignals fields as the first trace variables (IDs 0..15).
 *
 * Thread context, once, before the trace services start. Platforms register
 * their own variables after these.
 *
 * @return False when the registry is full (CONFIG_UNIMOC_TRACE_VARS).
 */
[[nodiscard]] bool RegisterControlSignals() noexcept;

/**
 * @brief Fills the mirror from the control cycle that just ran and samples the traces.
 *
 * Zero-latency context, right after the CurrentControlIsr::OnSample() that
 * ended a control cycle (once per timing.steps_per_cycle control steps):
 * copies twelve floats (the thermal four come from the Cyphal thread) and runs cymon::Device::Tick(), which
 * takes no lock, calls no kernel or libm function and no function pointer.
 */
void OnControlCycle(const current_control::CurrentControlIsr& isr) noexcept;

}  // namespace unimoc::trace
