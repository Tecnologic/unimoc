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
  float i_d{0.0F};               ///< Measured d current, mean of the cycle [A].
  float i_q{0.0F};               ///< Measured q current, mean of the cycle [A].
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
 * @brief Registers the TraceSignals fields as the first trace variables (IDs 0..11).
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
 * Zero-latency context, right after CurrentControlIsr::OnControlCycle():
 * copies twelve floats and runs cymon::Device::Tick(), which takes no lock,
 * calls no kernel or libm function and no function pointer.
 */
void OnControlCycle(const current_control::CurrentControlIsr& isr) noexcept;

}  // namespace unimoc::trace
