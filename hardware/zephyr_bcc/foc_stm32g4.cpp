/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32g4.cpp
 *    @brief TIM8 / ADC1..5 / DMA power stage driver of the battery case controller.
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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <algorithm>
#include <cstddef>
#include "dead_time.hpp"
#include "foc_stm32g4_dt.hpp"
#include "hardware_interface_config.hpp"
#include "hot_path_interface.hpp"

PINCTRL_DT_DEFINE(UNIMOC_FOC);

namespace unimoc::hardware::foc_stm32g4 {

namespace {

// JEXTSEL 10 selects TIM8_TRGO2 on ADC1..5 (LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2).
constexpr uint32_t kInjectedTriggerTim8Trgo2 = 10U;
constexpr uint32_t kCalibrationTimeoutLoops = 1'000'000U;

// TRGO2 pulses on a rising OC4REF or a rising OC6REF (MMS2 = 0b1100,
// LL_TIM_TRGO2_OC4_RISING_OC6_RISING). OC4 is the trigger before the peak,
// OC6 the trigger before the valley, see InitTimer().
constexpr uint32_t kMasterMode2Oc4Oc6Rising = TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2;

// Center aligned with RCR = 0: an update event (and a duty DMA burst) at every
// peak and every valley.
constexpr uint32_t kRepetitionCounter = 0U;

TIM_TypeDef* const kTim = reinterpret_cast<TIM_TypeDef*>(kTimerBase);
ADC_TypeDef* const kAdcFast = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseA.base);
ADC_TypeDef* const kAdcB = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseB.base);
ADC_TypeDef* const kAdcC = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseC.base);
ADC_TypeDef* const kAdcVdc = reinterpret_cast<ADC_TypeDef*>(kAdcDcLink.base);

// Duty DMA: DMA1 channel 1 through DMAMUX1 channel 0 on the TIM8 update
// request. Each request is a burst of three word transfers into TIM8_DMAR,
// which TIM8_DCR maps onto CCR1..CCR3.
DMA_Channel_TypeDef* const kDma = DMA1_Channel1;
DMAMUX_Channel_TypeDef* const kDmaMux = DMAMUX1_Channel0;
constexpr uint32_t kDmaRequestTim8Up = 53U;  // LL_DMAMUX_REQ_TIM8_UP
constexpr uint32_t kBurstLength = 3U;        // CCR1, CCR2, CCR3
constexpr uint32_t kDmaTransfers = kBurstLength * kHalfPeriodsPerCycle;
constexpr uint32_t kCcr1WordOffset = offsetof(TIM_TypeDef, CCR1) / sizeof(uint32_t);

// Observer interrupt: the TIM8 update vector, pended by software after each
// control cycle. TIM8 never pends it itself: UIE stays 0 (only UDE is set).
constexpr unsigned kObserverIrq = kTimerUpdateIrq;
constexpr unsigned kObserverIrqPriority = 0U;  // highest normal (non zero-latency) priority

// Break input (net PWM_BREAK): the external enable. High = released, low or open = break.
const struct gpio_dt_spec kEnableInput = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), break_gpios, 0);

// BDTR: break input on, active low, 3 = 8 samples at f_DTS filter (about 50 ns);
// idle level (low) is driven while MOE = 0 (OSSI/OSSR), no automatic re-enable.
constexpr uint32_t kBreakConfig = TIM_BDTR_BKE | (3U << TIM_BDTR_BKF_Pos) | TIM_BDTR_OSSI | TIM_BDTR_OSSR;

const struct device* const kRcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

uint32_t g_timer_clock_hz = 0U;
uint32_t g_auto_reload = 0U;
uint32_t g_trigger_lead = 0U;

// Slot s holds CCR1..3 for the burst of the s-th update event of a cycle;
// slot 0 is the burst at the control boundary.
alignas(4) PhaseTicks g_dma_slots[kHalfPeriodsPerCycle]{};
static_assert(sizeof(g_dma_slots) == kDmaTransfers * sizeof(uint32_t), "slots are packed words");

// Sample interrupt state. Only the zero-latency context writes it.
CycleRaw g_cycle{};
uint8_t g_sample_index = 0U;
uint32_t g_overcurrent_low = 0U;
uint32_t g_overcurrent_high = 0xFFFU;

// What was active while the samples of the current cycle were taken.
TickPlan g_applied{};
PhaseTicks g_last_entry{};  // entry 3 of the last plan
bool g_control_at_peak = true;
uint32_t g_duty_overruns = 0U;

constexpr ADC_TypeDef* Adc(const AdcDescription& d) noexcept {
  return reinterpret_cast<ADC_TypeDef*>(d.base);
}

ADC_Common_TypeDef* CommonOf(const ADC_TypeDef* adc) noexcept {
  return (adc == ADC1 || adc == ADC2) ? ADC12_COMMON : ADC345_COMMON;
}

void SetSampleTime(ADC_TypeDef* adc, const uint32_t channel, const uint32_t code) noexcept {
  volatile uint32_t& reg = (channel < 10U) ? adc->SMPR1 : adc->SMPR2;
  const uint32_t shift = 3U * ((channel < 10U) ? channel : channel - 10U);
  reg = (reg & ~(7U << shift)) | (code << shift);
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

  SetSampleTime(adc, description.channel_rank0, description.sample_time_rank0);
  SetSampleTime(adc, description.channel_rank1, description.sample_time_rank1);

  adc->JSQR = (1U << ADC_JSQR_JL_Pos) |  // two conversions
              (kInjectedTriggerTim8Trgo2 << ADC_JSQR_JEXTSEL_Pos) | (1U << ADC_JSQR_JEXTEN_Pos) |  // rising
              (static_cast<uint32_t>(description.channel_rank0) << ADC_JSQR_JSQ1_Pos) |
              (static_cast<uint32_t>(description.channel_rank1) << ADC_JSQR_JSQ2_Pos);

  adc->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_JQOVF;
  adc->IER = fast_interrupt ? ADC_IER_JEOSIE : 0U;
  adc->CR |= ADC_CR_JADSTART;  // arm: waits for the hardware trigger
  return true;
}

// ----- Zero-latency interrupt service routine --------------------------------
// No kernel calls, no logging. Direct ISRs return 0: never a reschedule.

bool OutsideWindow(const uint32_t raw) noexcept {
  return raw < g_overcurrent_low || raw > g_overcurrent_high;
}

ISR_DIRECT_DECLARE(SampleIsr) {
  kAdcFast->ISR = ADC_ISR_JEOS;  // the sequence is complete, JDR1/JDR2 are valid
  const uint32_t ia = kAdcFast->JDR1;
  const uint32_t ib = kAdcB->JDR1;
  const uint32_t ic = kAdcC->JDR1;
  const uint8_t index = g_sample_index;
  g_cycle.currents[index] =
      RawCurrents{static_cast<uint16_t>(ia), static_cast<uint16_t>(ib), static_cast<uint16_t>(ic)};
  if (OutsideWindow(ia) || OutsideWindow(ib) || OutsideWindow(ic)) {
    kTim->EGR = TIM_EGR_BG;  // break: outputs off at once, BIF latched
    g_cycle.overcurrent = true;
  }

  if (index + 1U < kHalfPeriodsPerCycle) {
    g_sample_index = index + 1U;
    return 0;
  }
  g_sample_index = 0U;
  g_cycle.vdc = kAdcVdc->JDR1;
  ControlCycleHandler();
  g_cycle.overcurrent = false;
  NVIC_SetPendingIRQ(static_cast<IRQn_Type>(kObserverIrq));
  return 0;
}

// ----- Observer interrupt (normal priority) ---------------------------------

void ObserverIsr(const void*) { ObserverHandler(); }

void ApplyTriggerLead(const uint32_t ticks) noexcept {
  // At least one tick, at most a quarter period, so both triggers stay in
  // their own half period.
  const uint32_t lead = std::clamp(ticks, 1U, g_auto_reload / 2U);
  g_trigger_lead = lead;
  kTim->CCR4 = g_auto_reload - lead;
  kTim->CCR6 = lead;
}

void InitDma() noexcept {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
  (void)RCC->AHB1ENR;  // the clock is on before the first access

  const uint32_t neutral = g_auto_reload / 2U;
  for (auto& slot : g_dma_slots) slot = PhaseTicks{neutral, neutral, neutral};
  g_last_entry = PhaseTicks{neutral, neutral, neutral};
  for (auto& entry : g_applied) entry = g_last_entry;

  kDma->CCR = 0U;
  kDmaMux->CCR = kDmaRequestTim8Up;
  kDma->CPAR = reinterpret_cast<uintptr_t>(&kTim->DMAR);
  kDma->CMAR = reinterpret_cast<uintptr_t>(&g_dma_slots[0][0]);
  kDma->CNDTR = kDmaTransfers;
  // Memory to peripheral, words, memory increment, circular, very high priority.
  kDma->CCR = DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PL_1 |
              DMA_CCR_PL_0 | DMA_CCR_EN;

  kTim->DCR = ((kBurstLength - 1U) << TIM_DCR_DBL_Pos) | (kCcr1WordOffset << TIM_DCR_DBA_Pos);
}

bool ExternalEnablePresent() noexcept { return gpio_pin_get_raw(kEnableInput.port, kEnableInput.pin) > 0; }

bool InitTimer(const uint32_t pwm_hz, const uint32_t trigger_offset) noexcept {
  struct stm32_pclken pclken = UNIMOC_CLOCK_INFO(0, UNIMOC_FOC_TIMER);
  if (clock_control_on(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken)) != 0) return false;
  if (clock_control_get_rate(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken), &g_timer_clock_hz) !=
      0) {
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
  kTim->CR2 = kMasterMode2Oc4Oc6Rising;

  // CH1..CH3: PWM mode 1 with CCR preload: a compare value takes effect at
  // the next update event (peak or valley), never inside a half period.
  // CH4 (PWM mode 2) and CH6 (PWM mode 1) have no pin; they only generate the
  // ADC triggers (no preload, the lead changes rarely):
  //   OC4REF rises while counting up at CCR4 = ARR - lead  (lead before the peak)
  //   OC6REF rises while counting down at CCR6 = lead      (lead before the valley)
  kTim->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  kTim->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (7U << TIM_CCMR2_OC4M_Pos);
  kTim->CCMR3 = (6U << TIM_CCMR3_OC6M_Pos);
  // Neutral (50 %) start: enabling must not brake the phases.
  kTim->CCR1 = g_auto_reload / 2U;
  kTim->CCR2 = g_auto_reload / 2U;
  kTim->CCR3 = g_auto_reload / 2U;
  ApplyTriggerLead(trigger_offset);
  kTim->CCER =
      TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  // Dead time: t_DTS = t_CK_INT (CKD = 0), the timer clock.
  const auto dtg = EncodeDeadTime(DeadTimeTicks(kDeadTimeNs, g_timer_clock_hz));
  if (!dtg.has_value()) return false;

  // Break on, outputs held in the idle (off) state. MOE stays 0 until PowerStageEnable().
  kTim->BDTR = kBreakConfig | *dtg;

  kTim->EGR = TIM_EGR_UG;  // load ARR, RCR and the preloaded compare values
  kTim->SR = 0U;
  // Start between the trigger compares: neither OC4REF nor OC6REF is active
  // there, so starting the counter cannot produce a spurious ADC trigger. The
  // first boundary is then both the first sample and the first DMA burst.
  kTim->CNT = g_auto_reload / 2U;
  return true;
}

}  // namespace

bool Initialize(const unit::Frequency pwm_frequency, const uint32_t adc_trigger_offset) noexcept {
  const auto pwm_hz = static_cast<uint32_t>(pwm_frequency.Value());
  if (pwm_hz == 0U) return false;

  if (pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(UNIMOC_FOC), PINCTRL_STATE_DEFAULT) != 0) return false;
  if (!InitTimer(pwm_hz, adc_trigger_offset)) return false;

  if (!InitAdc(kAdcPhaseA, true)) return false;
  if (!InitAdc(kAdcPhaseB, false)) return false;
  if (!InitAdc(kAdcPhaseC, false)) return false;
  if (!InitAdc(kAdcDcLink, false)) return false;
  if (kBridgeTemperatureAvailable || kMotorTemperatureAvailable) {
    if (!InitAdc(kAdcTemperature, false)) return false;
  }

  InitDma();
  // The first sample belongs to the first boundary, which is also the first
  // DMA burst (slot 0): it is the last sample of a cycle.
  g_sample_index = kHalfPeriodsPerCycle - 1U;
  g_cycle = CycleRaw{};

  // Zero latency: priority argument is ignored, the IRQ is never masked by irq_lock().
  IRQ_DIRECT_CONNECT(kFastIrq, 0, SampleIsr, IRQ_ZERO_LATENCY);
  IRQ_CONNECT(kObserverIrq, kObserverIrqPriority, ObserverIsr, nullptr, 0);
  irq_enable(kObserverIrq);
  irq_enable(kFastIrq);

  kTim->DIER = TIM_DIER_UDE;  // after UG: the initial update must not start a burst
  kTim->CR1 |= TIM_CR1_CEN;
  return true;
}

bool PowerStageEnable() noexcept {
  if ((kTim->BDTR & TIM_BDTR_MOE) != 0U) return true;
  if (!ExternalEnablePresent()) return false;
  if ((kTim->SR & TIM_SR_BIF) != 0U) return false;  // fault latched
  kTim->BDTR |= TIM_BDTR_MOE;                       // ignored by the hardware while the break input is active
  return (kTim->BDTR & TIM_BDTR_MOE) != 0U;
}

void PowerStageDisable() noexcept { kTim->BDTR &= ~TIM_BDTR_MOE; }

void PowerStageTrip() noexcept { kTim->EGR = TIM_EGR_BG; }  // clears MOE, sets BIF

void PowerStageClearFault() noexcept { kTim->SR = ~TIM_SR_BIF; }  // rc_w0

PowerStageState GetPowerStageState() noexcept {
  if ((kTim->BDTR & TIM_BDTR_MOE) != 0U) return PowerStageState::kEnabled;
  if (!ExternalEnablePresent()) return PowerStageState::kBlocked;
  if ((kTim->SR & TIM_SR_BIF) != 0U) return PowerStageState::kFault;
  return PowerStageState::kDisabled;
}

uint32_t TimerClockFrequency() noexcept { return g_timer_clock_hz; }

uint32_t TimerAutoReload() noexcept { return g_auto_reload; }

void SetTriggerLead(const uint32_t ticks) noexcept { ApplyTriggerLead(ticks); }

uint32_t TriggerLead() noexcept { return g_trigger_lead; }

void SetDutyTicks(const TickPlan& ticks) noexcept {
  const uint32_t remaining = kDma->CNDTR;
  const bool counting_down = (kTim->CR1 & TIM_CR1_DIR) != 0U;

  for (uint8_t k = 1U; k < kHalfPeriodsPerCycle; ++k) g_dma_slots[k] = ticks[k];

  if (remaining == kDmaTransfers - kBurstLength) {
    // Normal case: the burst of the control boundary (slot 0) is done and the
    // next one has not started. Entry 0 goes into the preload now; slot 0 holds
    // the last entry as a fallback for a late next cycle.
    kTim->CCR1 = ticks[0][0];
    kTim->CCR2 = ticks[0][1];
    kTim->CCR3 = ticks[0][2];
    g_dma_slots[0] = ticks[kHalfPeriodsPerCycle - 1U];
    if (kDma->CNDTR != remaining) ++g_duty_overruns;  // the next update came during the write
    g_control_at_peak = counting_down;                // just after a peak the counter counts down
  } else if (remaining == kDmaTransfers) {
    // Early: the control boundary is still ahead (long trigger lead). Its
    // burst delivers entry 0.
    g_dma_slots[0] = ticks[0];
    g_control_at_peak = !counting_down;  // counting up towards a peak
  } else {
    ++g_duty_overruns;
  }

  // Sample k of the next cycle is taken in the half period ending at the
  // (k + 1)-th boundary after the control boundary.
  g_applied[0] = g_last_entry;
  for (uint8_t k = 1U; k < kHalfPeriodsPerCycle; ++k) g_applied[k] = ticks[k - 1U];
  g_last_entry = ticks[kHalfPeriodsPerCycle - 1U];
}

uint32_t DutyOverruns() noexcept { return g_duty_overruns; }

const CycleRaw& CycleSamples() noexcept { return g_cycle; }

const PhaseTicks& AppliedTicks(const uint8_t sample) noexcept { return g_applied[sample]; }

bool SampleAtPeak(const uint8_t sample) noexcept {
  // The last sample shares the boundary type with the control boundary; the
  // type alternates from boundary to boundary.
  const bool same_as_control = ((kHalfPeriodsPerCycle - 1U - sample) % 2U) == 0U;
  return same_as_control == g_control_at_peak;
}

void SetOvercurrentWindow(const uint32_t low, const uint32_t high) noexcept {
  g_overcurrent_low = low;
  g_overcurrent_high = high;
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
