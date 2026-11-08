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
#include <string_view>

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

#define UNIMOC_FOC_SAMPLE_CODE(index) DT_ENUM_IDX_BY_IDX(UNIMOC_FOC, sample_times, index)

/// One io-channel of foc0, in devicetree order.
#define UNIMOC_FOC_CHANNEL_ENTRY(node_id, prop, idx)                            \
  ChannelEntry{DT_PROP_BY_IDX(node_id, io_channel_names, idx),                  \
               DT_REG_ADDR(DT_IO_CHANNELS_CTLR_BY_IDX(node_id, idx)),           \
               static_cast<uint8_t>(DT_IO_CHANNELS_INPUT_BY_IDX(node_id, idx)), \
               static_cast<uint8_t>(DT_ENUM_IDX_BY_IDX(node_id, sample_times, idx))},
// NOLINTEND(cppcoreguidelines-macro-usage)

namespace unimoc::hardware::foc_stm32h7 {

/// Longest injected sequence of an STM32H7 ADC.
inline constexpr uint32_t kMaxRanks = 4U;

/// One entry of io-channels with its name, ADC, input channel and SMP code.
struct ChannelEntry {
  std::string_view name;
  uintptr_t adc;
  uint8_t channel;
  uint8_t sample_time;  ///< SMP code (0 = 1.5 ... 7 = 810.5 ADC clock cycles)
};

inline constexpr std::array kChannels{
    DT_FOREACH_PROP_ELEM(UNIMOC_FOC, io_channels, UNIMOC_FOC_CHANNEL_ENTRY)};

static_assert(DT_PROP_LEN(UNIMOC_FOC, sample_times) == kChannels.size(), "one sample time per io-channel");
static_assert(DT_PROP_LEN(UNIMOC_FOC, io_channel_names) == kChannels.size(), "one name per io-channel");

/// One ADC with its injected sequence: the entries of kChannels on it, in devicetree order.
struct AdcDescription {
  uintptr_t base;
  uint32_t ranks;
  std::array<uint8_t, kMaxRanks> channels;
  std::array<uint8_t, kMaxRanks> sample_times;
  struct stm32_pclken pclken;
};

[[nodiscard]] consteval AdcDescription MakeAdc(const uintptr_t base, const struct stm32_pclken pclken) {
  AdcDescription adc{base, 0U, {}, {}, pclken};
  for (const ChannelEntry& entry : kChannels) {
    if (entry.adc == base && adc.ranks < kMaxRanks) {
      adc.channels.at(adc.ranks) = entry.channel;
      adc.sample_times.at(adc.ranks) = entry.sample_time;
      ++adc.ranks;
    }
  }
  return adc;
}

[[nodiscard]] consteval uint32_t RanksOn(const uintptr_t base) {
  uint32_t ranks = 0U;
  for (const ChannelEntry& entry : kChannels) {
    ranks += (entry.adc == base) ? 1U : 0U;
  }
  return ranks;
}

// Each phase current has its own ADC; the other signals may sit on any of the three.
inline constexpr AdcDescription kAdcPhaseA =
    MakeAdc(DT_REG_ADDR(UNIMOC_FOC_ADC(ia)), UNIMOC_CLOCK_INFO(0, DT_PARENT(UNIMOC_FOC_ADC(ia))));
inline constexpr AdcDescription kAdcPhaseB =
    MakeAdc(DT_REG_ADDR(UNIMOC_FOC_ADC(ib)), UNIMOC_CLOCK_INFO(0, DT_PARENT(UNIMOC_FOC_ADC(ib))));
inline constexpr AdcDescription kAdcPhaseC =
    MakeAdc(DT_REG_ADDR(UNIMOC_FOC_ADC(ic)), UNIMOC_CLOCK_INFO(0, DT_PARENT(UNIMOC_FOC_ADC(ic))));
inline constexpr std::array<const AdcDescription*, 3> kAdcs{&kAdcPhaseA, &kAdcPhaseB, &kAdcPhaseC};

/// Where a signal is converted: the index into kAdcs and its rank (0 = JDR1).
struct SignalLocation {
  uint32_t adc;
  uint32_t rank;
};

[[nodiscard]] consteval bool HasSignal(const std::string_view name) {
  uint32_t found = 0U;
  for (const ChannelEntry& entry : kChannels) {
    found += (entry.name == name) ? 1U : 0U;
  }
  return found == 1U;
}

/// The signal appears at most once (the optional board inputs).
[[nodiscard]] consteval bool HasAtMostOnce(const std::string_view name) {
  uint32_t found = 0U;
  for (const ChannelEntry& entry : kChannels) {
    found += (entry.name == name) ? 1U : 0U;
  }
  return found <= 1U;
}

/// Every io-channel names a signal of the binding.
[[nodiscard]] consteval bool AllNamesKnown() {
  constexpr std::array<std::string_view, 12> kKnown{"ia",
                                                    "ib",
                                                    "ic",
                                                    "va",
                                                    "vb",
                                                    "vc",
                                                    "vdc",
                                                    "bridge-temperature",
                                                    "motor-temperature",
                                                    "gate-supply",
                                                    "accelerator",
                                                    "brake"};
  for (const ChannelEntry& entry : kChannels) {
    bool known = false;
    for (const std::string_view kName : kKnown) {
      known = known || entry.name == kName;
    }
    if (!known) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] consteval SignalLocation LocationOf(const std::string_view name) {
  for (uint32_t adc = 0U; adc < kAdcs.size(); ++adc) {
    uint32_t rank = 0U;
    for (const ChannelEntry& entry : kChannels) {
      if (entry.adc != kAdcs.at(adc)->base) {
        continue;
      }
      if (entry.name == name) {
        return SignalLocation{adc, rank};
      }
      ++rank;
    }
  }
  return SignalLocation{0U, kMaxRanks};
}

/// Length of an injected sequence in half ADC clock cycles (sampling plus the 6.5 cycles of a 12 bit
/// conversion).
[[nodiscard]] consteval uint32_t SequenceHalfCycles(const AdcDescription& adc) {
  constexpr std::array<uint32_t, 8> kSampleHalfCycles{3U, 5U, 17U, 33U, 65U, 129U, 775U, 1621U};
  constexpr uint32_t kConversionHalfCycles = 13U;
  uint32_t total = 0U;
  for (uint32_t rank = 0U; rank < adc.ranks; ++rank) {
    total += kSampleHalfCycles.at(adc.sample_times.at(rank)) + kConversionHalfCycles;
  }
  return total;
}

static_assert(HasSignal("ia") && HasSignal("va") && HasSignal("bridge-temperature") && HasSignal("ib") &&
                  HasSignal("vb") && HasSignal("motor-temperature") && HasSignal("ic") && HasSignal("vdc") &&
                  HasSignal("vc"),
              "foc0 io-channel-names: every signal of the binding exactly once");
static_assert(HasAtMostOnce("gate-supply") && HasAtMostOnce("accelerator") && HasAtMostOnce("brake"),
              "foc0 io-channel-names: the optional board inputs at most once");
static_assert(AllNamesKnown(), "foc0 io-channel-names: an unknown signal name");
static_assert(kAdcPhaseA.base != kAdcPhaseB.base && kAdcPhaseA.base != kAdcPhaseC.base &&
                  kAdcPhaseB.base != kAdcPhaseC.base,
              "every phase current has its own ADC");
static_assert(RanksOn(kAdcPhaseA.base) + RanksOn(kAdcPhaseB.base) + RanksOn(kAdcPhaseC.base) ==
                  kChannels.size(),
              "every io-channel is on the ADC of a phase current");
static_assert(kAdcPhaseA.ranks <= kMaxRanks && kAdcPhaseB.ranks <= kMaxRanks &&
                  kAdcPhaseC.ranks <= kMaxRanks && RanksOn(kAdcPhaseA.base) <= kMaxRanks &&
                  RanksOn(kAdcPhaseB.base) <= kMaxRanks && RanksOn(kAdcPhaseC.base) <= kMaxRanks,
              "at most four injected conversions per ADC");
static_assert(LocationOf("ia").rank == 0U && LocationOf("ib").rank == 0U && LocationOf("ic").rank == 0U,
              "each phase current is the first conversion of its ADC, so the three are sampled together");
// The sample interrupt is the end of the phase A sequence: the other sequences, with the DC link
// that the interrupt checks, must be complete by then.
static_assert(SequenceHalfCycles(kAdcPhaseA) >= SequenceHalfCycles(kAdcPhaseB) &&
                  SequenceHalfCycles(kAdcPhaseA) >= SequenceHalfCycles(kAdcPhaseC),
              "the injected sequence of the phase A ADC must be the longest");

inline constexpr SignalLocation kVdcLocation = LocationOf("vdc");

// Optional board inputs, sampled with the injected sequences and read by the drive thread.
inline constexpr bool kHasGateSupply = HasSignal("gate-supply");
inline constexpr bool kHasAccelerator = HasSignal("accelerator");
inline constexpr bool kHasBrake = HasSignal("brake");
static_assert(!kHasGateSupply || (DT_NODE_HAS_PROP(UNIMOC_FOC, gate_supply_divider_ohms) &&
                                  DT_NODE_HAS_PROP(UNIMOC_FOC, gate_supply_millivolts)),
              "gate-supply needs gate-supply-divider-ohms and gate-supply-millivolts");
#if DT_NODE_HAS_PROP(UNIMOC_FOC, gate_supply_divider_ohms)
inline constexpr uint32_t kGateSupplyDividerTopOhm = DT_PROP_BY_IDX(UNIMOC_FOC, gate_supply_divider_ohms, 0);
inline constexpr uint32_t kGateSupplyDividerBottomOhm =
    DT_PROP_BY_IDX(UNIMOC_FOC, gate_supply_divider_ohms, 1);
static_assert(DT_PROP_LEN(UNIMOC_FOC, gate_supply_divider_ohms) == 2,
              "gate-supply-divider-ohms: <top bottom>");
static_assert(kGateSupplyDividerBottomOhm > 0U, "gate-supply-divider-ohms: bottom resistor");
#else
inline constexpr uint32_t kGateSupplyDividerTopOhm = 0U;
inline constexpr uint32_t kGateSupplyDividerBottomOhm = 1U;
#endif
#if DT_NODE_HAS_PROP(UNIMOC_FOC, gate_supply_millivolts)
inline constexpr uint32_t kGateSupplyTripMillivolts = DT_PROP_BY_IDX(UNIMOC_FOC, gate_supply_millivolts, 0);
inline constexpr uint32_t kGateSupplyEngageMillivolts = DT_PROP_BY_IDX(UNIMOC_FOC, gate_supply_millivolts, 1);
static_assert(DT_PROP_LEN(UNIMOC_FOC, gate_supply_millivolts) == 2, "gate-supply-millivolts: <trip engage>");
static_assert(kGateSupplyTripMillivolts < kGateSupplyEngageMillivolts,
              "gate-supply-millivolts: trip below engage");
#else
inline constexpr uint32_t kGateSupplyTripMillivolts = 0U;
inline constexpr uint32_t kGateSupplyEngageMillivolts = 0U;
#endif

inline constexpr uint32_t kDeadTimeNs = DT_PROP(UNIMOC_FOC, dead_time_ns);

// Analog front end and limits of the board (foc0).
inline constexpr uint32_t kShuntNanoOhm = DT_PROP(UNIMOC_FOC, shunt_nano_ohm);
inline constexpr uint32_t kCurrentAmplifierGain = DT_PROP(UNIMOC_FOC, current_amplifier_gain);
inline constexpr uint32_t kDividerTopOhm = DT_PROP_BY_IDX(UNIMOC_FOC, voltage_divider_ohms, 0);
inline constexpr uint32_t kDividerBottomOhm = DT_PROP_BY_IDX(UNIMOC_FOC, voltage_divider_ohms, 1);
inline constexpr uint32_t kMaxPhaseCurrentAmps = DT_PROP(UNIMOC_FOC, max_phase_current_amps);
inline constexpr uint32_t kMaxMotorCurrentAmps = DT_PROP(UNIMOC_FOC, max_motor_current_amps);
/// Rated battery drive and charge current [A] (foc0 battery-current-amps).
inline constexpr uint32_t kBatteryDriveCurrentAmps = DT_PROP_BY_IDX(UNIMOC_FOC, battery_current_amps, 0);
inline constexpr uint32_t kBatteryChargeCurrentAmps = DT_PROP_BY_IDX(UNIMOC_FOC, battery_current_amps, 1);
inline constexpr uint32_t kDcLinkDerateMillivolts = DT_PROP_BY_IDX(UNIMOC_FOC, dc_link_millivolts, 0);
inline constexpr uint32_t kDcLinkTripMillivolts = DT_PROP_BY_IDX(UNIMOC_FOC, dc_link_millivolts, 1);
inline constexpr uint32_t kDcLinkMaxMillivolts = DT_PROP_BY_IDX(UNIMOC_FOC, dc_link_millivolts, 2);
static_assert(DT_PROP_LEN(UNIMOC_FOC, voltage_divider_ohms) == 2, "voltage-divider-ohms: <top bottom>");
static_assert(DT_PROP_LEN(UNIMOC_FOC, dc_link_millivolts) == 3, "dc-link-millivolts: <derate trip max>");
static_assert(kDcLinkDerateMillivolts < kDcLinkTripMillivolts &&
                  kDcLinkTripMillivolts <= kDcLinkMaxMillivolts,
              "dc-link-millivolts: derate below trip, trip at most max");
static_assert(kMaxMotorCurrentAmps <= kMaxPhaseCurrentAmps, "the motor current stays below the trip limit");
static_assert(DT_PROP_LEN(UNIMOC_FOC, battery_current_amps) == 2, "battery-current-amps is <drive charge>");
static_assert(kBatteryDriveCurrentAmps > 0U && kBatteryChargeCurrentAmps > 0U, "battery currents are set");
static_assert(kShuntNanoOhm > 0U && kCurrentAmplifierGain > 0U && kDividerBottomOhm > 0U, "front end values");
/// Phase currents from low-side shunts (current-sense = "low-side"), else inline.
inline constexpr bool kLowSideCurrentSense = DT_ENUM_IDX(UNIMOC_FOC, current_sense) == 1;
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
