/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32h7.cpp
 *    @brief TIM1/TIM8 / ADC1..3 power stage driver on the STM32H7.
 *
 *    Direct register access (CMSIS, RM0433). All pin, timer and channel
 *    assignments come from the foc0 devicetree node, see foc_stm32h7_dt.hpp.
 *    The timing is the one of the former STM32G4 driver of the BCC; the
 *    differences are the H7 ADC (clock, boost, linearity calibration,
 *    channel preselection) and the M7 cycle counter. The ADCs sample at every
 *    peak and valley (the overcurrent and overvoltage checks); the current
 *    control runs after the valley sample and writes the CCR preload, which
 *    the timer takes over at the peak only (RCR = 1, no duty DMA).
 *
 *    Not run on hardware yet.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "foc_stm32h7.hpp"
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <algorithm>
#include <array>
#include <atomic>
#include "foc_stm32h7_dt.hpp"
#include "hardware_interface_config.hpp"
#include "hot_path_interface.hpp"
#include "hot_path_timing.hpp"
#include "stm32_dead_time.hpp"

// Zephyr's pinctrl macro defines the pin table as a C array.
// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
PINCTRL_DT_DEFINE(UNIMOC_FOC);
// NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

namespace unimoc::hardware::foc_stm32h7 {

namespace {

constexpr bool kTimerIsTim1 = kTimerBase == TIM1_BASE;

// Injected trigger TRGO2 of the power stage timer (RM0433, ADC external
// triggers; LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2 / _TIM8_TRGO2).
constexpr uint32_t kInjectedTrigger = kTimerIsTim1 ? 8U : 10U;

constexpr uint32_t kCalibrationTimeoutLoops = 1'000'000U;

// TRGO2 pulses on a rising OC4REF or a rising OC6REF (MMS2 = 0b1100,
// LL_TIM_TRGO2_OC4_RISING_OC6_RISING). OC4 is the trigger before the peak,
// OC6 the trigger before the valley, see InitTimer().
constexpr uint32_t kMasterMode2Oc4Oc6Rising = TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2;

// Center aligned with RCR = 1: an update event (the preloaded duties take
// effect) once per PWM period, at the peak: the duties written after a valley
// sample change between two pulses, every pulse stays symmetric about its
// valley. The repetition counter counts every overflow and underflow and
// raises the update when it is 0; InitTimer() starts it at 0, so the first
// update is the first peak (measured on the NUCLEO-H743ZI, 2026-11-03: with
// the counter started at 1 every update fell on the valley).
constexpr uint32_t kRepetitionCounter = 1U;

// ADC clock: asynchronous from the kernel clock (pll2_p_ck) with PRESC = /4.
// Rev. Y allows 36 MHz; rev. V halves the clock internally once more.
constexpr uint32_t kAdcPrescalerCode = 2U;  // PRESC = 0b0010: divided by 4
constexpr uint32_t kAdcClockHz = kAdcKernelClockHz / 4U;
static_assert(kAdcClockHz <= 36'000'000U, "ADC clock above the rev. Y limit");

// Silicon revisions (DBGMCU_IDCODE REV_ID).
constexpr uint32_t kRevisionY = 0x1003U;

// 12 bit resolution, the same code on rev. Y and rev. V (LL_ADC_RESOLUTION_12B).
constexpr uint32_t kResolution12Bit = ADC_CFGR_RES_1;

TIM_TypeDef* const kTim = reinterpret_cast<TIM_TypeDef*>(kTimerBase);
ADC_TypeDef* const kAdcFast = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseA.base);
ADC_TypeDef* const kAdcB = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseB.base);
ADC_TypeDef* const kAdcC = reinterpret_cast<ADC_TypeDef*>(kAdcPhaseC.base);

// Observer interrupt: the timer update vector, pended by software after each
// control cycle. The timer never pends it itself: DIER stays 0.
constexpr unsigned kObserverIrq = kTimerUpdateIrq;
constexpr unsigned kObserverIrqPriority = 0U;  // highest normal (non zero-latency) priority

// M0 execution times of the interrupts (hot_path_timing.hpp).
constexpr bool kTiming = IS_ENABLED(CONFIG_UNIMOC_HOT_PATH_TIMING);

// Break input (net PWM_BREAK): the external enable. High = released, low or open = break.
const struct gpio_dt_spec kEnableInput = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), break_gpios, 0);

// BDTR: break input on, active low, 3 = 8 samples at f_DTS filter (about 40 ns
// at 200 MHz); idle level (low) is driven while MOE = 0 (OSSI/OSSR), no
// automatic re-enable.
constexpr uint32_t kBreakConfig = TIM_BDTR_BKE | (3U << TIM_BDTR_BKF_Pos) | TIM_BDTR_OSSI | TIM_BDTR_OSSR;

const struct device* const kRcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

uint32_t g_auto_reload = 0U;
uint32_t g_trigger_lead = 0U;

// Sample interrupt state. Only the zero-latency context writes it.
SampleRaw g_sample{};
bool g_sample_at_peak = true;  // the boundary of the last sample
// Trips of the samples since the control step last read one (peaks, and the
// valleys between two control steps): Sample() reports them with the next.
bool g_pending_overcurrent = false;
bool g_pending_overvoltage = false;

// Overcurrent window, low limit in bits 0..15 and high limit in bits 16..31.
// Written by the thread context (SetOvercurrentWindow()), read by the sample
// interrupt: one word, so it never sees the limits of two windows.
constexpr uint32_t PackWindow(const uint32_t low, const uint32_t high) noexcept {
  return (low & 0xFFFFU) | ((high & 0xFFFFU) << 16U);
}
std::atomic<uint32_t> g_overcurrent_window{PackWindow(0U, 0xFFFU)};
// Raw DC-link limit; written by the thread context, read by the sample interrupt.
std::atomic<uint32_t> g_overvoltage_counts{kOvervoltageCheckOff};
static_assert(std::atomic<uint32_t>::is_always_lock_free);

// Compare values of the last control step: active from the peak after their
// write, so also at the next valley sample.
PhaseTicks g_applied{};
bool g_started = false;  // Initialize() has programmed the hardware, Stop() has not run since
uint32_t g_duty_overruns = 0U;

inline ADC_TypeDef* Adc(const AdcDescription& d) noexcept { return reinterpret_cast<ADC_TypeDef*>(d.base); }

bool IsRevisionY() noexcept { return (DBGMCU->IDCODE >> DBGMCU_IDCODE_REV_ID_Pos) <= kRevisionY; }

// BOOST after the ADC clock (HAL ADC_ConfigureBoostMode()): rev. Y one bit
// above 20 MHz; rev. V two bits after its internal divider by 2.
uint32_t BoostBits() noexcept {
  if (IsRevisionY()) {
    return (kAdcClockHz > 20'000'000U) ? ADC_CR_BOOST_0 : 0U;
  }
  constexpr uint32_t kHalf = kAdcClockHz / 2U;
  if (kHalf <= 6'250'000U) {
    return 0U;
  }
  if (kHalf <= 12'500'000U) {
    return ADC_CR_BOOST_0;
  }
  if (kHalf <= 25'000'000U) {
    return ADC_CR_BOOST_1;
  }
  return ADC_CR_BOOST_1 | ADC_CR_BOOST_0;
}

void SetSampleTime(ADC_TypeDef* adc, const uint32_t channel, const uint32_t code) noexcept {
  volatile uint32_t& reg = (channel < 10U) ? adc->SMPR1 : adc->SMPR2;
  const uint32_t kShift = 3U * ((channel < 10U) ? channel : channel - 10U);
  reg = (reg & ~(7U << kShift)) | (code << kShift);
}

bool WaitWhile(const volatile uint32_t& reg, const uint32_t mask, const bool until_set) noexcept {
  for (uint32_t i = 0; i < kCalibrationTimeoutLoops; ++i) {
    if (((reg & mask) != 0U) == until_set) {
      return true;
    }
  }
  return false;
}

// The clock mode of a common block may only change while all its ADCs are
// disabled: set both before the first ADC is enabled.
bool InitAdcClocks() noexcept {
  for (const AdcDescription* description : {&kAdcPhaseA, &kAdcPhaseB, &kAdcPhaseC}) {
    struct stm32_pclken pclken = description->pclken;
    if (clock_control_on(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken)) != 0) {
      return false;
    }
  }
  for (ADC_Common_TypeDef* common : {ADC12_COMMON, ADC3_COMMON}) {
    common->CCR =
        (common->CCR & ~(ADC_CCR_CKMODE | ADC_CCR_PRESC)) | (kAdcPrescalerCode << ADC_CCR_PRESC_Pos);
  }
  return true;
}

// Powers up, calibrates and enables one ADC and arms its injected sequence on
// the timer trigger. The sequence starts with the first TRGO2 edge.
bool InitAdc(const AdcDescription& description, const bool fast_interrupt) noexcept {
  ADC_TypeDef* const kAdc = Adc(description);

  kAdc->CR &= ~ADC_CR_DEEPPWD;
  kAdc->CR |= ADC_CR_ADVREGEN;
  k_busy_wait(20);  // tADCVREG_STUP (rev. Y has no LDORDY flag)
  if (!IsRevisionY() && !WaitWhile(kAdc->ISR, ADC_ISR_LDORDY, true)) {
    return false;
  }
  kAdc->CR = (kAdc->CR & ~ADC_CR_BOOST) | BoostBits();

  kAdc->CFGR = kResolution12Bit | ADC_CFGR_JQDIS;  // plain JSQR, no context queue

  // Offset and linearity calibration, single ended.
  kAdc->CR = (kAdc->CR & ~ADC_CR_ADCALDIF) | ADC_CR_ADCALLIN;
  kAdc->CR |= ADC_CR_ADCAL;
  if (!WaitWhile(kAdc->CR, ADC_CR_ADCAL, false)) {
    return false;
  }

  kAdc->ISR = ADC_ISR_ADRDY;
  kAdc->CR |= ADC_CR_ADEN;
  if (!WaitWhile(kAdc->ISR, ADC_ISR_ADRDY, true)) {
    return false;
  }
  kAdc->ISR = ADC_ISR_ADRDY;

  uint32_t preselection = 0U;
  for (uint32_t rank = 0U; rank < kRanks; ++rank) {
    SetSampleTime(kAdc, description.channels[rank], description.sample_times[rank]);
    preselection |= 1U << description.channels[rank];
  }
  kAdc->PCSEL = preselection;  // the H7 ADC converts only preselected channels

  kAdc->JSQR = ((kRanks - 1U) << ADC_JSQR_JL_Pos) | (kInjectedTrigger << ADC_JSQR_JEXTSEL_Pos) |
               (1U << ADC_JSQR_JEXTEN_Pos) |  // rising edge
               (static_cast<uint32_t>(description.channels[0]) << ADC_JSQR_JSQ1_Pos) |
               (static_cast<uint32_t>(description.channels[1]) << ADC_JSQR_JSQ2_Pos) |
               (static_cast<uint32_t>(description.channels[2]) << ADC_JSQR_JSQ3_Pos);

  kAdc->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_JQOVF;
  kAdc->IER = fast_interrupt ? ADC_IER_JEOSIE : 0U;
  kAdc->CR |= ADC_CR_JADSTART;  // arm: waits for the hardware trigger
  return true;
}

// Stops the injected sequence and disables the ADC (RM0433: JADSTP only
// while JADSTART = 1, ADDIS only while ADEN = 1 and no conversion runs).
bool StopAdc(const AdcDescription& description) noexcept {
  ADC_TypeDef* const kAdc = Adc(description);
  if ((kAdc->CR & ADC_CR_JADSTART) != 0U) {
    kAdc->CR |= ADC_CR_JADSTP;
    if (!WaitWhile(kAdc->CR, ADC_CR_JADSTART, false)) {
      return false;
    }
  }
  kAdc->IER = 0U;
  if ((kAdc->CR & ADC_CR_ADEN) != 0U) {
    kAdc->CR |= ADC_CR_ADDIS;
    if (!WaitWhile(kAdc->CR, ADC_CR_ADEN, false)) {
      return false;
    }
  }
  return true;
}

// ----- Zero-latency interrupt service routine --------------------------------
// No kernel calls, no logging. Direct ISRs return 0: never a reschedule.

bool OutsideWindow(const uint32_t raw, const uint32_t window) noexcept {
  return raw < (window & 0xFFFFU) || raw > (window >> 16U);
}

ISR_DIRECT_DECLARE(SampleIsr) {
  // M0: from the first instruction, without the exception entry.
  const uint32_t kEntry = kTiming ? DWT->CYCCNT : 0U;
  // The sequence of a peak ends in the upper half of the counter, before or
  // after the peak, the one of a valley in the lower half (lead and sequence
  // shorter than a quarter period).
  g_sample_at_peak = kTim->CNT > g_auto_reload / 2U;
  kAdcFast->ISR = ADC_ISR_JEOS;  // the sequence is complete, JDR1..JDR3 are valid
  const uint32_t kIa = kAdcFast->JDR1;
  const uint32_t kIb = kAdcB->JDR1;
  const uint32_t kIc = kAdcC->JDR1;
  const uint32_t kWindow = g_overcurrent_window.load(std::memory_order_relaxed);
  const bool kOvercurrent =
      OutsideWindow(kIa, kWindow) || OutsideWindow(kIb, kWindow) || OutsideWindow(kIc, kWindow);
  // The DC link of every sample: rank 2 of the phase C ADC, converted with the same trigger and sample times.
  const uint32_t kVdc = kAdcC->JDR2;
  const bool kOvervoltage = kVdc >= g_overvoltage_counts.load(std::memory_order_relaxed);
  if (kOvercurrent || kOvervoltage) {
    kTim->EGR = TIM_EGR_BG;  // break: outputs off at once, BIF latched
  }
  g_pending_overcurrent = g_pending_overcurrent || kOvercurrent;
  g_pending_overvoltage = g_pending_overvoltage || kOvervoltage;
  if (g_sample_at_peak) {
    return 0;  // the control samples the valley, the middle of the high-side pulses
  }
  g_sample.ia = static_cast<uint16_t>(kIa);
  g_sample.ib = static_cast<uint16_t>(kIb);
  g_sample.ic = static_cast<uint16_t>(kIc);
  g_sample.vdc = kVdc;
  g_sample.overcurrent = g_pending_overcurrent;
  g_sample.overvoltage = g_pending_overvoltage;
  g_sample.timestamp = DWT->CYCCNT;  // in the clock of the Hall events

  bool observer = false;
  if constexpr (kTiming) {
    const uint32_t kStart = DWT->CYCCNT;
    observer = SampleHandler();
    timing::ProbeOf(timing::Section::kControlCycle).Record(DWT->CYCCNT - kStart);
  } else {
    observer = SampleHandler();
  }
  if (observer) {
    NVIC_SetPendingIRQ(static_cast<IRQn_Type>(kObserverIrq));
  }
  if constexpr (kTiming) {
    timing::ProbeOf(timing::Section::kSampleIsr).Record(DWT->CYCCNT - kEntry);
  }
  return 0;
}

// ----- Observer interrupt (normal priority) ---------------------------------

void ObserverIsr(const void* /*unused*/) {
  if constexpr (kTiming) {
    const uint32_t kStart = DWT->CYCCNT;
    ObserverHandler();
    timing::ProbeOf(timing::Section::kObserver).Record(DWT->CYCCNT - kStart);
  } else {
    ObserverHandler();
  }
}

void ApplyTriggerLead(const uint32_t ticks) noexcept {
  // At least one tick, at most a quarter period, so both triggers stay in
  // their own half period.
  const uint32_t kLead = std::clamp(ticks, 1U, g_auto_reload / 2U);
  g_trigger_lead = kLead;
  kTim->CCR4 = g_auto_reload - kLead;
  kTim->CCR6 = kLead;
}

void InitDutyState() noexcept {
  const uint32_t kNeutral = g_auto_reload / 2U;
  g_applied = PhaseTicks{kNeutral, kNeutral, kNeutral};
  g_pending_overcurrent = false;
  g_pending_overvoltage = false;
}

bool ExternalEnablePresent() noexcept { return gpio_pin_get_raw(kEnableInput.port, kEnableInput.pin) > 0; }

bool InitTimer(const uint32_t pwm_hz, const uint32_t trigger_offset) noexcept {
  struct stm32_pclken pclken = UNIMOC_CLOCK_INFO(0, UNIMOC_FOC_TIMER);
  if (clock_control_on(kRcc, reinterpret_cast<clock_control_subsys_t>(&pclken)) != 0) {
    return false;
  }

  // Same relation the control core uses: ARR = f_timer / (2 f_pwm) - 1.
  const uint32_t kHalfPeriods = kTimerClockHz / pwm_hz;
  if (kHalfPeriods < 4U || kHalfPeriods / 2U - 1U > 0xFFFFU) {
    return false;
  }
  g_auto_reload = kHalfPeriods / 2U - 1U;

  kTim->CR1 = TIM_CR1_CMS;  // center aligned mode 3, counter disabled
  kTim->PSC = 0U;
  kTim->ARR = g_auto_reload;
  kTim->RCR = 0U;  // loaded into the repetition counter by UG below
  kTim->CR2 = kMasterMode2Oc4Oc6Rising;

  // CH1..CH3: PWM mode 1 with CCR preload: a compare value takes effect at
  // the next update event (the peak), never inside a PWM period.
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
  const auto kDtg = stm32::EncodeDeadTime(stm32::DeadTimeTicks(kDeadTimeNs, kTimerClockHz));
  if (!kDtg.has_value()) {
    return false;
  }

  // Break on, outputs held in the idle (off) state. MOE stays 0 until PowerStageEnable().
  kTim->BDTR = kBreakConfig | *kDtg;

  // Load ARR, the preloaded compare values and a repetition counter of 0, then
  // RCR = 1 into its preload: the counter starts below the peak counting up,
  // updates at the first peak and from then on at every peak.
  kTim->EGR = TIM_EGR_UG;
  kTim->RCR = kRepetitionCounter;
  kTim->SR = 0U;
  // Start between the trigger compares: neither OC4REF nor OC6REF is active
  // there, so starting the counter cannot produce a spurious ADC trigger.
  kTim->CNT = g_auto_reload / 2U;
  return true;
}

// Cortex-M7: the DWT registers are write-locked until the lock access key is written.
void StartCycleCounter() noexcept {
  constexpr uint32_t kLockAccessKey = 0xC5ACCE55U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = kLockAccessKey;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

}  // namespace

bool Initialize(const unit::Frequency pwm_frequency, const uint32_t adc_trigger_offset) noexcept {
  const auto kPwmHz = static_cast<uint32_t>(pwm_frequency.Value());
  if (kPwmHz == 0U) {
    return false;
  }

  // A restart (new settings): stop what the last Initialize() started, also
  // when it failed half way.
  Stop();
  if (pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(UNIMOC_FOC), PINCTRL_STATE_DEFAULT) != 0) {
    return false;
  }
  g_started = true;
  if (!InitTimer(kPwmHz, adc_trigger_offset)) {
    return false;
  }

  if (!InitAdcClocks()) {
    return false;
  }
  if (!InitAdc(kAdcPhaseA, true)) {
    return false;
  }
  if (!InitAdc(kAdcPhaseB, false)) {
    return false;
  }
  if (!InitAdc(kAdcPhaseC, false)) {
    return false;
  }

  InitDutyState();
  // Timestamp clock (Hall edges, samples): the DWT cycle counter.
  StartCycleCounter();
  g_sample = SampleRaw{};

  // Zero latency: priority argument is ignored, the IRQ is never masked by irq_lock().
  IRQ_DIRECT_CONNECT(kFastIrq, 0, SampleIsr, IRQ_ZERO_LATENCY);
  IRQ_CONNECT(kObserverIrq, kObserverIrqPriority, ObserverIsr, nullptr, 0);
  irq_enable(kObserverIrq);
  irq_enable(kFastIrq);

  kTim->DIER = 0U;  // no timer interrupt or DMA request: the observer vector is pended by software
  kTim->CR1 |= TIM_CR1_CEN;
  return true;
}

void Stop() noexcept {
  if (!g_started) {
    return;
  }
  g_started = false;
  // First the interrupts: no control cycle or observer runs from here on.
  irq_disable(kFastIrq);
  irq_disable(kObserverIrq);

  kTim->BDTR &= ~TIM_BDTR_MOE;  // outputs off (the power stage is disabled already)
  kTim->CR1 &= ~TIM_CR1_CEN;    // no more ADC triggers

  // An ADC that does not stop is left as it is; the calibration in the
  // following Initialize() then fails and reports it.
  (void)StopAdc(kAdcPhaseA);
  (void)StopAdc(kAdcPhaseB);
  (void)StopAdc(kAdcPhaseC);

  NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(kFastIrq));
  NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(kObserverIrq));
}

bool PowerStageEnable() noexcept {
  if ((kTim->BDTR & TIM_BDTR_MOE) != 0U) {
    return true;
  }
  if (!ExternalEnablePresent()) {
    return false;
  }
  if ((kTim->SR & TIM_SR_BIF) != 0U) {
    return false;  // fault latched
  }
  kTim->BDTR |= TIM_BDTR_MOE;  // ignored by the hardware while the break input is active
  // The sample interrupt preempts this read-modify-write: a trip (BG) between
  // the BIF check and the write would be undone by setting MOE again.
  if ((kTim->SR & TIM_SR_BIF) != 0U) {
    kTim->BDTR &= ~TIM_BDTR_MOE;
    return false;
  }
  return (kTim->BDTR & TIM_BDTR_MOE) != 0U;
}

void PowerStageDisable() noexcept { kTim->BDTR &= ~TIM_BDTR_MOE; }

void PowerStageTrip() noexcept { kTim->EGR = TIM_EGR_BG; }  // clears MOE, sets BIF

void PowerStageClearFault() noexcept { kTim->SR = ~TIM_SR_BIF; }  // rc_w0

PowerStageState GetPowerStageState() noexcept {
  if ((kTim->BDTR & TIM_BDTR_MOE) != 0U) {
    return PowerStageState::kEnabled;
  }
  if (!ExternalEnablePresent()) {
    return PowerStageState::kBlocked;
  }
  if ((kTim->SR & TIM_SR_BIF) != 0U) {
    return PowerStageState::kFault;
  }
  return PowerStageState::kDisabled;
}

uint32_t TimerClockFrequency() noexcept { return kTimerClockHz; }

uint32_t TimerAutoReload() noexcept { return g_auto_reload; }

void SetTriggerLead(const uint32_t ticks) noexcept { ApplyTriggerLead(ticks); }

uint32_t TriggerLead() noexcept { return g_trigger_lead; }

void SetDutyTicks(const PhaseTicks& ticks) noexcept {
  kTim->CCR1 = ticks[0];
  kTim->CCR2 = ticks[1];
  kTim->CCR3 = ticks[2];
  // Late: the counter has passed the peak the preload was meant for and
  // counts down to the next valley; the duties take effect one period later.
  const bool kUpperHalf = kTim->CNT > g_auto_reload / 2U;
  const bool kCountingDown = (kTim->CR1 & TIM_CR1_DIR) != 0U;
  if (kUpperHalf && kCountingDown) {
    ++g_duty_overruns;
  }
  g_applied = ticks;
}

uint32_t DutyOverruns() noexcept { return g_duty_overruns; }

const SampleRaw& Sample() noexcept {
  // The trips are reported once: with this sample, not again with the next.
  g_pending_overcurrent = false;
  g_pending_overvoltage = false;
  return g_sample;
}

uint32_t Timestamp() noexcept { return DWT->CYCCNT; }

const PhaseTicks& AppliedTicks() noexcept { return g_applied; }

bool SampleAtPeak() noexcept { return g_sample_at_peak; }

void SetOvercurrentWindow(const uint32_t low, const uint32_t high) noexcept {
  g_overcurrent_window.store(PackWindow(low, high), std::memory_order_relaxed);
}

void SetOvervoltageCounts(const uint32_t counts) noexcept {
  g_overvoltage_counts.store(counts, std::memory_order_relaxed);
}

RawSamples ReadRaw() noexcept {
  const ADC_TypeDef* const kRegA = Adc(kAdcPhaseA);
  const ADC_TypeDef* const kRegB = Adc(kAdcPhaseB);
  const ADC_TypeDef* const kRegC = Adc(kAdcPhaseC);
  return RawSamples{kRegA->JDR1,
                    kRegA->JDR2,
                    kRegA->JDR3,
                    kRegB->JDR1,
                    kRegB->JDR2,
                    kRegB->JDR3,
                    kRegC->JDR1,
                    kRegC->JDR2,
                    kRegC->JDR3};
}

}  // namespace unimoc::hardware::foc_stm32h7
