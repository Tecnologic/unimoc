/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32h7_dt.hpp
 *    @brief Devicetree view of the foc0 power stage node on the STM32H7.
 *
 *    Pins, timer and ADC channel assignment come from the foc0 node
 *    (compatible unimoc,foc-stm32h7) of the board devicetree. Nothing in the
 *    driver names a pin, an ADC instance or a channel number.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <array>
#include <cstdint>

// NOLINTBEGIN(cppcoreguidelines-macro-usage): the devicetree API is macro-only;
// these helpers and the signal indices are pasted into DT_* macro names.
#define UNIMOC_FOC DT_NODELABEL(foc0)
#define UNIMOC_FOC_TIMER DT_PHANDLE(UNIMOC_FOC, timer)

/// ADC controller node of a named io-channel of foc0.
#define UNIMOC_FOC_ADC(name) DT_IO_CHANNELS_CTLR_BY_NAME(UNIMOC_FOC, name)
/// ADC input channel number of a named io-channel of foc0.
#define UNIMOC_FOC_ADC_CH(name) DT_IO_CHANNELS_INPUT_BY_NAME(UNIMOC_FOC, name)

/// Same as STM32_CLOCK_INFO, but with the designators in declaration order as C++ requires.
#define UNIMOC_CLOCK_INFO(clk_index, node_id)                                                        \
  {                                                                                                  \
      .bus = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bus) & GENMASK(STM32_CLOCK_DIV_SHIFT - 1, 0), \
      .div = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bus) >> STM32_CLOCK_DIV_SHIFT,                \
      .enr = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bits),                                        \
  }

// Position of a signal in io-channel-names / sample-times. Plain numbers:
// the devicetree macros paste them into identifiers.
#define UNIMOC_SIG_IA 0
#define UNIMOC_SIG_VA 1
#define UNIMOC_SIG_BRIDGE_TEMPERATURE 2
#define UNIMOC_SIG_IB 3
#define UNIMOC_SIG_VB 4
#define UNIMOC_SIG_MOTOR_TEMPERATURE 5
#define UNIMOC_SIG_IC 6
#define UNIMOC_SIG_VDC 7
#define UNIMOC_SIG_VC 8
#define UNIMOC_SIG_COUNT 9

#define UNIMOC_FOC_SAMPLE_CODE(index) DT_ENUM_IDX_BY_IDX(UNIMOC_FOC, sample_times, index)

#define UNIMOC_FOC_ADC_TRIPLE(first, second, third, first_index, second_index, third_index) \
  AdcDescription {                                                                          \
    DT_REG_ADDR(UNIMOC_FOC_ADC(first)),                                                     \
        {UNIMOC_FOC_ADC_CH(first), UNIMOC_FOC_ADC_CH(second), UNIMOC_FOC_ADC_CH(third)},    \
        {UNIMOC_FOC_SAMPLE_CODE(first_index),                                               \
         UNIMOC_FOC_SAMPLE_CODE(second_index),                                              \
         UNIMOC_FOC_SAMPLE_CODE(third_index)},                                              \
        UNIMOC_CLOCK_INFO(0, UNIMOC_FOC_ADC(first))                                         \
  }

#define UNIMOC_FOC_SAME_ADC(first, second) \
  (DT_REG_ADDR(UNIMOC_FOC_ADC(first)) == DT_REG_ADDR(UNIMOC_FOC_ADC(second)))
// NOLINTEND(cppcoreguidelines-macro-usage)

namespace unimoc::hardware::foc_stm32h7 {

/// Conversions in the injected sequence of every ADC.
inline constexpr uint32_t kRanks = 3U;

/// One ADC with an injected sequence of kRanks channels.
struct AdcDescription {
  uintptr_t base;
  std::array<uint8_t, kRanks> channels;
  std::array<uint8_t, kRanks> sample_times;  ///< SMP codes (0 = 1.5 ... 7 = 810.5 ADC clock cycles)
  struct stm32_pclken pclken;
};

// Each phase current is rank 0 of its own ADC; the triples fix the ranks.
inline constexpr AdcDescription kAdcPhaseA = UNIMOC_FOC_ADC_TRIPLE(ia,
                                                                   va,
                                                                   bridge_temperature,
                                                                   UNIMOC_SIG_IA,
                                                                   UNIMOC_SIG_VA,
                                                                   UNIMOC_SIG_BRIDGE_TEMPERATURE);
inline constexpr AdcDescription kAdcPhaseB = UNIMOC_FOC_ADC_TRIPLE(ib,
                                                                   vb,
                                                                   motor_temperature,
                                                                   UNIMOC_SIG_IB,
                                                                   UNIMOC_SIG_VB,
                                                                   UNIMOC_SIG_MOTOR_TEMPERATURE);
inline constexpr AdcDescription kAdcPhaseC =
    UNIMOC_FOC_ADC_TRIPLE(ic, vdc, vc, UNIMOC_SIG_IC, UNIMOC_SIG_VDC, UNIMOC_SIG_VC);

static_assert(UNIMOC_FOC_SAME_ADC(ia, va) && UNIMOC_FOC_SAME_ADC(ia, bridge_temperature),
              "ia, va and bridge-temperature share one ADC");
static_assert(UNIMOC_FOC_SAME_ADC(ib, vb) && UNIMOC_FOC_SAME_ADC(ib, motor_temperature),
              "ib, vb and motor-temperature share one ADC");
static_assert(UNIMOC_FOC_SAME_ADC(ic, vdc) && UNIMOC_FOC_SAME_ADC(ic, vc), "ic, vdc and vc share one ADC");
static_assert(!UNIMOC_FOC_SAME_ADC(ia, ib) && !UNIMOC_FOC_SAME_ADC(ia, ic) && !UNIMOC_FOC_SAME_ADC(ib, ic),
              "every phase current has its own ADC");

static_assert(DT_PROP_LEN(UNIMOC_FOC, sample_times) == UNIMOC_SIG_COUNT, "one sample time per io-channel");
static_assert(DT_PROP_LEN(UNIMOC_FOC, io_channel_names) == UNIMOC_SIG_COUNT, "signal order of the binding");

inline constexpr uint32_t kDeadTimeNs = DT_PROP(UNIMOC_FOC, dead_time_ns);
inline constexpr uintptr_t kTimerBase = DT_REG_ADDR(UNIMOC_FOC_TIMER);
static_assert(kTimerBase == TIM1_BASE || kTimerBase == TIM8_BASE, "the power stage timer is TIM1 or TIM8");

/// Timer update vector. The update interrupt itself stays disabled (UIE = 0, only the
/// DMA request UDE is used), so the vector is free for the software-pended observer.
inline constexpr unsigned kTimerUpdateIrq = DT_IRQ_BY_NAME(UNIMOC_FOC_TIMER, up, irq);
/// The sample interrupt is the end of the injected sequence of the phase A ADC.
inline constexpr unsigned kFastIrq = DT_IRQN(UNIMOC_FOC_ADC(ia));

// ----- Clock tree (board devicetree) ----------------------------------------

#define UNIMOC_RCC DT_NODELABEL(rcc)  // NOLINT(cppcoreguidelines-macro-usage): devicetree node

/// Core clock (SYSCLK / D1CPRE): the DWT cycle counter.
inline constexpr uint32_t kCoreClockHz = DT_PROP(UNIMOC_RCC, clock_frequency) / DT_PROP(UNIMOC_RCC, d1cpre);
/// AHB clock (HCLK).
inline constexpr uint32_t kHclkHz = kCoreClockHz / DT_PROP(UNIMOC_RCC, hpre);

// TIM1 and TIM8 are on APB2 (D2PPRE2). With TIMPRE = 0 the timer kernel clock is
// PCLK2 without an APB2 prescaler and 2 x PCLK2 with one (RM0433, timer clock).
static_assert(!DT_PROP(UNIMOC_RCC, timpre), "timer clock assumes TIMPRE = 0");
inline constexpr uint32_t kTimerClockHz =
    (DT_PROP(UNIMOC_RCC, d2ppre2) == 1) ? kHclkHz : 2U * (kHclkHz / DT_PROP(UNIMOC_RCC, d2ppre2));

// ADC kernel clock: pll2_p_ck, the reset value of ADCSEL (RCC_D3CCIPR); the
// Zephyr ADC driver that could change it stays off (ADC nodes disabled).
#define UNIMOC_PLL2 DT_NODELABEL(pll2)  // NOLINT(cppcoreguidelines-macro-usage): devicetree node
static_assert(DT_NODE_HAS_STATUS(UNIMOC_PLL2, okay) && DT_NODE_HAS_PROP(UNIMOC_PLL2, div_p),
              "the ADC kernel clock is PLL2 P");
inline constexpr uint32_t kAdcKernelClockHz = static_cast<uint32_t>(
    static_cast<uint64_t>(DT_PROP(DT_CLOCKS_CTLR(UNIMOC_PLL2), clock_frequency)) /
    DT_PROP(UNIMOC_PLL2, div_m) * DT_PROP(UNIMOC_PLL2, mul_n) / DT_PROP(UNIMOC_PLL2, div_p));

// The duty DMA uses DMA1 stream 0 directly; no Zephyr driver may own DMA1.
static_assert(!DT_NODE_HAS_STATUS(DT_NODELABEL(dma1), okay), "DMA1 belongs to the foc0 duty DMA");

}  // namespace unimoc::hardware::foc_stm32h7
