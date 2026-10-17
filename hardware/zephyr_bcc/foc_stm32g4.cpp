/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32g4.cpp
 *    @brief TIM8 / ADC1..5 power stage driver of the battery case controller.
 *
 *    Direct register access (CMSIS). All pin, timer and channel assignments
 *    come from the foc0 devicetree node, see foc_stm32g4_dt.hpp.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "foc_stm32g4.hpp"

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <algorithm>

#include "foc_stm32g4_dt.hpp"
#include "hardware_interface_config.hpp"

PINCTRL_DT_DEFINE(UNIMOC_FOC);

namespace unimoc::hardware::foc_stm32g4 {

namespace {

// JEXTSEL 9 selects TIM8_TRGO on ADC1..5 (RM0440, injected trigger table).
constexpr uint32_t kInjectedTriggerTim8Trgo = 9U;
// Sample time register code 3: 24.5 ADC clock cycles.
constexpr uint32_t kSampleTime24Cycles5 = 3U;
constexpr uint32_t kCalibrationTimeoutLoops = 1'000'000U;

// OC4REF is TRGO (MMS = 0b0111); TRGO2 is the update event (MMS2 = 0b0010).
constexpr uint32_t kMasterModeCompareOc4 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1 | TIM_CR2_MMS_2;
constexpr uint32_t kMasterMode2Update = TIM_CR2_MMS2_1;

// Update every N periods in center aligned mode: RCR counts half periods.
constexpr uint32_t kRepetitionCounter = 2U * kSlowUpdateDivider - 1U;

TIM_TypeDef* const kTim = reinterpret_cast<TIM_TypeDef*>(kTimerBase);
ADC_TypeDef* const kAdcFast = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseA.base);

const struct device* const kRcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

HardwareInterface::CurrentControlCallback g_fast_callback = nullptr;
HardwareInterface::SlowUpdateCallback g_slow_callback = nullptr;
uint32_t g_timer_clock_hz = 0U;
uint32_t g_auto_reload = 0U;

constexpr ADC_TypeDef* Adc(const AdcDescription& d) noexcept { return reinterpret_cast<ADC_TypeDef*>(d.base); }

ADC_Common_TypeDef* CommonOf(const ADC_TypeDef* adc) noexcept {
  return (adc == ADC1 || adc == ADC2) ? ADC12_COMMON : ADC345_COMMON;
}

void SetSampleTime(ADC_TypeDef* adc, const uint32_t channel) noexcept {
  volatile uint32_t& reg = (channel < 10U) ? adc->SMPR1 : adc->SMPR2;
  const uint32_t shift = 3U * ((channel < 10U) ? channel : channel - 10U);
  reg = (reg & ~(7U << shift)) | (kSampleTime24Cycles5 << shift);
}

bool WaitWhile(const volatile uint32_t& reg, const uint32_t mask, const bool until_set) noexcept {
  for (uint32_t i = 0; i < kCalibrationTimeoutLoops; ++i) {
    if (((reg & mask) != 0U) == until_set) return true;
  }
  return false;
}

// Powers up, calibrates and enables one ADC and arms its injected sequence on
// the TIM8 trigger. The sequence starts with the first TRGO edge.
bool InitAdc(const AdcDescription& description, const bool fast_interrupt) noexcept {
  ADC_TypeDef* const adc = Adc(description);

  struct stm32_pclken pclken = description.pclken;
  if (clock_control_on(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken)) != 0) return false;

  // Synchronous clock, HCLK / 4 = 42.5 MHz (CKMODE = 0b11).
  ADC_Common_TypeDef* const common = CommonOf(adc);
  common->CCR = (common->CCR & ~ADC_CCR_CKMODE) | ADC_CCR_CKMODE_1 | ADC_CCR_CKMODE_0;

  adc->CR &= ~ADC_CR_DEEPPWD;
  adc->CR |= ADC_CR_ADVREGEN;
  k_busy_wait(20);  // tADCVREG_STUP

  adc->CFGR |= ADC_CFGR_JQDIS;  // plain JSQR, no context queue

  adc->CR &= ~ADC_CR_ADCALDIF;  // single ended
  adc->CR |= ADC_CR_ADCAL;
  if (!WaitWhile(adc->CR, ADC_CR_ADCAL, false)) return false;

  adc->ISR = ADC_ISR_ADRDY;
  adc->CR |= ADC_CR_ADEN;
  for (uint32_t i = 0; (adc->ISR & ADC_ISR_ADRDY) == 0U; ++i) {
    if (i > kCalibrationTimeoutLoops) return false;
    adc->CR |= ADC_CR_ADEN;  // ADEN is ignored for 4 ADC clocks after calibration
  }
  adc->ISR = ADC_ISR_ADRDY;

  SetSampleTime(adc, description.channel_rank0);
  SetSampleTime(adc, description.channel_rank1);

  adc->JSQR = (1U << ADC_JSQR_JL_Pos) |  // two conversions
              (kInjectedTriggerTim8Trgo << ADC_JSQR_JEXTSEL_Pos) | (1U << ADC_JSQR_JEXTEN_Pos) |  // rising
              (static_cast<uint32_t>(description.channel_rank0) << ADC_JSQR_JSQ1_Pos) |
              (static_cast<uint32_t>(description.channel_rank1) << ADC_JSQR_JSQ2_Pos);

  adc->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_JQOVF;
  adc->IER = fast_interrupt ? ADC_IER_JEOSIE : 0U;
  adc->CR |= ADC_CR_JADSTART;  // arm: waits for the hardware trigger
  return true;
}

// ----- Zero-latency interrupt service routines ------------------------------
// No kernel calls, no logging. Direct ISRs return 0: never a reschedule.

ISR_DIRECT_DECLARE(FastIsr) {
  kAdcFast->ISR = ADC_ISR_JEOS;  // the sequence is complete, JDR1/JDR2 are valid
  g_fast_callback();
  return 0;
}

ISR_DIRECT_DECLARE(SlowIsr) {
  kTim->SR = ~TIM_SR_UIF;  // rc_w0
  g_slow_callback();
  return 0;
}

bool InitTimer(const uint32_t pwm_hz, const uint32_t trigger_offset) noexcept {
  struct stm32_pclken pclken = UNIMOC_CLOCK_INFO(0, UNIMOC_FOC_TIMER);
  if (clock_control_on(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken)) != 0) return false;
  if (clock_control_get_rate(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken), &g_timer_clock_hz) != 0) {
    return false;
  }

  // Same relation the control core uses: ARR = f_timer / (2 f_pwm) - 1.
  const uint32_t half_periods = g_timer_clock_hz / pwm_hz;
  if (half_periods < 4U || half_periods / 2U - 1U > 0xFFFFU) return false;
  g_auto_reload = half_periods / 2U - 1U;

  kTim->CR1 = TIM_CR1_CMS;  // center aligned mode 3, counter disabled
  kTim->PSC = 0U;
  kTim->ARR = g_auto_reload;
  kTim->RCR = kRepetitionCounter;
  kTim->CR2 = kMasterModeCompareOc4 | kMasterMode2Update;

  // CH1..CH3: PWM mode 1, no CCR preload. The compare values are written
  // between update events (the update event only occurs every N periods).
  kTim->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | (6U << TIM_CCMR1_OC2M_Pos);
  kTim->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | (3U << TIM_CCMR2_OC4M_Pos);  // CH4: toggle, no pin
  kTim->CCR1 = 0U;
  kTim->CCR2 = 0U;
  kTim->CCR3 = 0U;
  kTim->CCR4 = std::min(trigger_offset, g_auto_reload);
  kTim->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  kTim->EGR = TIM_EGR_UG;  // load ARR, RCR; written before CEN: odd RCR updates at the peak
  kTim->SR = 0U;
  return true;
}

}  // namespace

bool Initialize(const unit::Frequency pwm_frequency,
                const uint32_t adc_trigger_offset,
                const HardwareInterface::CurrentControlCallback current_control,
                const HardwareInterface::SlowUpdateCallback slow_update) noexcept {
  const auto pwm_hz = static_cast<uint32_t>(pwm_frequency.Value());
  if (pwm_hz == 0U || current_control == nullptr || slow_update == nullptr) return false;
  g_fast_callback = current_control;
  g_slow_callback = slow_update;

  if (pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(UNIMOC_FOC), PINCTRL_STATE_DEFAULT) != 0) return false;
  if (!InitTimer(pwm_hz, adc_trigger_offset)) return false;

  if (!InitAdc(kAdcPhaseA, true)) return false;
  if (!InitAdc(kAdcPhaseB, false)) return false;
  if (!InitAdc(kAdcPhaseC, false)) return false;
  if (!InitAdc(kAdcDcLink, false)) return false;
  if (kBridgeTemperatureAvailable || kMotorTemperatureAvailable) {
    if (!InitAdc(kAdcTemperature, false)) return false;
  }

  // Zero latency: priority argument is ignored, the IRQ is never masked by irq_lock().
  IRQ_DIRECT_CONNECT(kFastIrq, 0, FastIsr, IRQ_ZERO_LATENCY);
  IRQ_DIRECT_CONNECT(kTimerUpdateIrq, 0, SlowIsr, IRQ_ZERO_LATENCY);
  kTim->DIER = TIM_DIER_UIE;
  irq_enable(kFastIrq);
  irq_enable(kTimerUpdateIrq);

  kTim->BDTR |= TIM_BDTR_MOE;  // outputs live; dead time and break are not configured yet
  kTim->CR1 |= TIM_CR1_CEN;
  return true;
}

uint32_t TimerClockFrequency() noexcept { return g_timer_clock_hz; }

uint32_t TimerAutoReload() noexcept { return g_auto_reload; }

void SetCompare(const uint32_t channel, const uint32_t ticks) noexcept {
  const uint32_t value = std::min(ticks, g_auto_reload);
  switch (channel) {
    case 1U: kTim->CCR1 = value; break;
    case 2U: kTim->CCR2 = value; break;
    case 3U: kTim->CCR3 = value; break;
    default: kTim->CCR4 = value; break;
  }
}

uint32_t GetCompare(const uint32_t channel) noexcept {
  switch (channel) {
    case 1U: return kTim->CCR1;
    case 2U: return kTim->CCR2;
    case 3U: return kTim->CCR3;
    default: return kTim->CCR4;
  }
}

RawSamples ReadRaw() noexcept {
  const ADC_TypeDef* const a = Adc(kAdcPhaseA);
  const ADC_TypeDef* const b = Adc(kAdcPhaseB);
  const ADC_TypeDef* const c = Adc(kAdcPhaseC);
  const ADC_TypeDef* const d = Adc(kAdcDcLink);
  const ADC_TypeDef* const t = Adc(kAdcTemperature);
  return RawSamples{a->JDR1, a->JDR2, b->JDR1, b->JDR2, c->JDR1, c->JDR2, d->JDR1, d->JDR2, t->JDR1, t->JDR2};
}

}  // namespace unimoc::hardware::foc_stm32g4
