/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32g4_dt.hpp
 *    @brief Devicetree view of the foc0 power stage node.
 *
 *    Everything the driver needs to know about pins, timer and ADC channel
 *    assignment is read from the foc0 node in the board devicetree
 *    (boards/unimoc/bcc/bcc.dts). Nothing in the driver names a pin, an ADC
 *    instance or a channel number.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <cstdint>

#define UNIMOC_FOC DT_NODELABEL(foc0)
#define UNIMOC_FOC_TIMER DT_PHANDLE(UNIMOC_FOC, timer)

/// ADC controller node of a named io-channel of foc0.
#define UNIMOC_FOC_ADC(name) DT_IO_CHANNELS_CTLR_BY_NAME(UNIMOC_FOC, name)
/// ADC input channel number of a named io-channel of foc0.
#define UNIMOC_FOC_ADC_CH(name) DT_IO_CHANNELS_INPUT_BY_NAME(UNIMOC_FOC, name)

/// Same as STM32_CLOCK_INFO, but with the designators in declaration order as C++ requires.
#define UNIMOC_CLOCK_INFO(clk_index, node_id)                                                          \
  {                                                                                                    \
    .bus = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bus) & GENMASK(STM32_CLOCK_DIV_SHIFT - 1, 0),     \
    .div = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bus) >> STM32_CLOCK_DIV_SHIFT,                    \
    .enr = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bits),                                            \
  }

namespace unimoc::hardware::foc_stm32g4 {

/// One ADC with an injected sequence of two channels (rank 0 and rank 1).
struct AdcDescription {
  uintptr_t base;
  uint8_t channel_rank0;
  uint8_t channel_rank1;
  uint8_t sample_time_rank0;  ///< SMP register code (0 = 2.5 cycles ... 7 = 640.5 cycles)
  uint8_t sample_time_rank1;
  struct stm32_pclken pclken;
};

// Position of a signal in io-channel-names / sample-times. Plain numbers:
// the devicetree macros paste them into identifiers.
#define UNIMOC_SIG_IA 0
#define UNIMOC_SIG_VA 1
#define UNIMOC_SIG_IB 2
#define UNIMOC_SIG_VB 3
#define UNIMOC_SIG_IC 4
#define UNIMOC_SIG_VC 5
#define UNIMOC_SIG_VDC 6
#define UNIMOC_SIG_CRANK_TORQUE 7
#define UNIMOC_SIG_MOTOR_TEMPERATURE 8
#define UNIMOC_SIG_BRIDGE_TEMPERATURE 9
#define UNIMOC_SIG_COUNT 10

#define UNIMOC_FOC_SAMPLE_CODE(index) DT_ENUM_IDX_BY_IDX(UNIMOC_FOC, sample_times, index)

#define UNIMOC_FOC_ADC_PAIR(first, second, first_index, second_index)                        \
  AdcDescription {                                                                           \
    DT_REG_ADDR(UNIMOC_FOC_ADC(first)), UNIMOC_FOC_ADC_CH(first), UNIMOC_FOC_ADC_CH(second), \
        UNIMOC_FOC_SAMPLE_CODE(first_index), UNIMOC_FOC_SAMPLE_CODE(second_index),           \
        UNIMOC_CLOCK_INFO(0, UNIMOC_FOC_ADC(first))                                          \
  }

// Current / phase voltage of one phase share an ADC; the pairs below fix the
// injected ranks: rank 0 is the first name, rank 1 the second.
inline constexpr AdcDescription kAdcPhaseA = UNIMOC_FOC_ADC_PAIR(ia, va, UNIMOC_SIG_IA, UNIMOC_SIG_VA);
inline constexpr AdcDescription kAdcPhaseB = UNIMOC_FOC_ADC_PAIR(ib, vb, UNIMOC_SIG_IB, UNIMOC_SIG_VB);
inline constexpr AdcDescription kAdcPhaseC = UNIMOC_FOC_ADC_PAIR(ic, vc, UNIMOC_SIG_IC, UNIMOC_SIG_VC);
inline constexpr AdcDescription kAdcDcLink = UNIMOC_FOC_ADC_PAIR(vdc, crank_torque, UNIMOC_SIG_VDC, UNIMOC_SIG_CRANK_TORQUE);
inline constexpr AdcDescription kAdcTemperature = UNIMOC_FOC_ADC_PAIR(motor_temperature, bridge_temperature, UNIMOC_SIG_MOTOR_TEMPERATURE, UNIMOC_SIG_BRIDGE_TEMPERATURE);

static_assert(DT_REG_ADDR(UNIMOC_FOC_ADC(ia)) == DT_REG_ADDR(UNIMOC_FOC_ADC(va)), "ia and va share one ADC");
static_assert(DT_REG_ADDR(UNIMOC_FOC_ADC(ib)) == DT_REG_ADDR(UNIMOC_FOC_ADC(vb)), "ib and vb share one ADC");
static_assert(DT_REG_ADDR(UNIMOC_FOC_ADC(ic)) == DT_REG_ADDR(UNIMOC_FOC_ADC(vc)), "ic and vc share one ADC");
static_assert(DT_REG_ADDR(UNIMOC_FOC_ADC(vdc)) == DT_REG_ADDR(UNIMOC_FOC_ADC(crank_torque)),
              "vdc and crank-torque share one ADC");
static_assert(DT_REG_ADDR(UNIMOC_FOC_ADC(motor_temperature)) ==
                  DT_REG_ADDR(UNIMOC_FOC_ADC(bridge_temperature)),
              "motor and bridge temperature share one ADC");

static_assert(DT_PROP_LEN(UNIMOC_FOC, sample_times) == UNIMOC_SIG_COUNT, "one sample time per io-channel");
static_assert(DT_PROP_LEN(UNIMOC_FOC, io_channel_names) == UNIMOC_SIG_COUNT, "signal order of SignalIndex");

inline constexpr uint32_t kDeadTimeNs = DT_PROP(UNIMOC_FOC, dead_time_ns);
inline constexpr uintptr_t kTimerBase = DT_REG_ADDR(UNIMOC_FOC_TIMER);
/// TIM8 update vector. The update interrupt itself stays disabled (UIE = 0, only the
/// DMA request UDE is used), so the vector is free for the software-pended observer.
inline constexpr unsigned kTimerUpdateIrq = DT_IRQ_BY_NAME(UNIMOC_FOC_TIMER, up, irq);
/// The sample interrupt is the end of the injected sequence of the first ADC (phase A).
inline constexpr unsigned kFastIrq = DT_IRQN(UNIMOC_FOC_ADC(ia));

// The timer kernel clock equals the APB2 clock only without an APB2 prescaler.
static_assert(DT_PROP(DT_NODELABEL(rcc), apb2_prescaler) == 1, "timer clock assumes APB2 prescaler 1");

}  // namespace unimoc::hardware::foc_stm32g4
