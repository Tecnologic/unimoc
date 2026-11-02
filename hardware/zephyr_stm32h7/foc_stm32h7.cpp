/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file foc_stm32h7.cpp
 *    @brief TIM1/TIM8 / ADC1..3 / DMA power stage driver on the STM32H7.
 *
 *    Direct register access (CMSIS, RM0433). All pin, timer and channel
 *    assignments come from the foc0 devicetree node, see foc_stm32h7_dt.hpp.
 *    The timing is the one of the STM32G4 driver (hardware/zephyr_bcc); the
 *    differences are the H7 ADC (clock, boost, linearity calibration,
 *    channel preselection), the DMA stream and the M7 cycle counter.
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
#include <zephyr/linker/section_tags.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
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
// DMAMUX1 request of the timer update (LL_DMAMUX1_REQ_TIM1_UP / _TIM8_UP).
constexpr uint32_t kDmaRequestTimerUp = kTimerIsTim1 ? 15U : 51U;

constexpr uint32_t kCalibrationTimeoutLoops = 1'000'000U;

// TRGO2 pulses on a rising OC4REF or a rising OC6REF (MMS2 = 0b1100,
// LL_TIM_TRGO2_OC4_RISING_OC6_RISING). OC4 is the trigger before the peak,
// OC6 the trigger before the valley, see InitTimer().
constexpr uint32_t kMasterMode2Oc4Oc6Rising = TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2;

// Center aligned with RCR = 0: an update event (and a duty DMA burst) at every
// peak and every valley.
constexpr uint32_t kRepetitionCounter = 0U;

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

// Duty DMA: DMA1 stream 0, fed by DMAMUX1 channel 0 with the timer update
// request. Each request is a burst of three word transfers into TIMx_DMAR,
// which TIMx_DCR maps onto CCR1..CCR3.
DMA_Stream_TypeDef* const kDma = DMA1_Stream0;
DMAMUX_Channel_TypeDef* const kDmaMux = DMAMUX1_Channel0;
constexpr uint32_t kDmaStream0Flags =
    DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
constexpr uint32_t kBurstLength = 3U;  // CCR1, CCR2, CCR3
constexpr uint32_t kDmaTransfers = kBurstLength * kHalfPeriodsPerCycle;
constexpr uint32_t kCcr1WordOffset = offsetof(TIM_TypeDef, CCR1) / sizeof(uint32_t);

// Observer interrupt: the timer update vector, pended by software after each
// control cycle. The timer never pends it itself: UIE stays 0 (only UDE is set).
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

// Slot s holds CCR1..3 for the burst of the s-th update event of a cycle;
// slot 0 is the burst at the control boundary. DMA1 reads it from the AXI
// SRAM; __nocache keeps it out of the data cache (CONFIG_NOCACHE_MEMORY), so
// the DMA sees what the control cycle wrote.
__nocache std::array<PhaseTicks, kHalfPeriodsPerCycle> g_dma_slots{};
static_assert(sizeof(g_dma_slots) == kDmaTransfers * sizeof(uint32_t), "slots are packed words");

// Sample interrupt state. Only the zero-latency context writes it.
CycleRaw g_cycle{};
uint8_t g_sample_index = 0U;

// Overcurrent window, low limit in bits 0..15 and high limit in bits 16..31.
// Written by the thread context (SetOvercurrentWindow()), read by the sample
// interrupt: one word, so it never sees the limits of two windows.
constexpr uint32_t PackWindow(const uint32_t low, const uint32_t high) noexcept {
  return (low & 0xFFFFU) | ((high & 0xFFFFU) << 16U);
}
std::atomic<uint32_t> g_overcurrent_window{PackWindow(0U, 0xFFFU)};
static_assert(std::atomic<uint32_t>::is_always_lock_free);

// What was active while the samples of the current cycle were taken.
TickPlan g_applied{};
PhaseTicks g_last_entry{};  // entry 3 of the last plan
bool g_control_at_peak = true;
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
  kAdcFast->ISR = ADC_ISR_JEOS;  // the sequence is complete, JDR1..JDR3 are valid
  const uint32_t kIa = kAdcFast->JDR1;
  const uint32_t kIb = kAdcB->JDR1;
  const uint32_t kIc = kAdcC->JDR1;
  const uint8_t kIndex = g_sample_index;
  g_cycle.currents[kIndex] =
      RawCurrents{static_cast<uint16_t>(kIa), static_cast<uint16_t>(kIb), static_cast<uint16_t>(kIc)};
  const uint32_t kWindow = g_overcurrent_window.load(std::memory_order_relaxed);
  if (OutsideWindow(kIa, kWindow) || OutsideWindow(kIb, kWindow) || OutsideWindow(kIc, kWindow)) {
    kTim->EGR = TIM_EGR_BG;  // break: outputs off at once, BIF latched
    g_cycle.overcurrent = true;
  }

  if (kIndex + 1U < kHalfPeriodsPerCycle) {
    g_sample_index = kIndex + 1U;
    return 0;
  }
  g_sample_index = 0U;
  // Rank 1 of the phase C ADC: converted together with rank 1 of the phase A
  // ADC when both have the same sample times, else possibly of the last sample.
  g_cycle.vdc = kAdcC->JDR2;
  g_cycle.timestamp = DWT->CYCCNT;  // the control boundary, in the clock of the Hall events
  if constexpr (kTiming) {
    const uint32_t kStart = DWT->CYCCNT;
    ControlCycleHandler();
    timing::ProbeOf(timing::Section::kControlCycle).Record(DWT->CYCCNT - kStart);
  } else {
    ControlCycleHandler();
  }
  g_cycle.overcurrent = false;
  NVIC_SetPendingIRQ(static_cast<IRQn_Type>(kObserverIrq));
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

void InitDma() noexcept {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;  // DMAMUX1 has no own clock enable on the H7
  (void)RCC->AHB1ENR;                  // the clock is on before the first access

  const uint32_t kNeutral = g_auto_reload / 2U;
  for (auto& slot : g_dma_slots) {
    slot = PhaseTicks{kNeutral, kNeutral, kNeutral};
  }
  g_last_entry = PhaseTicks{kNeutral, kNeutral, kNeutral};
  for (auto& entry : g_applied) {
    entry = g_last_entry;
  }

  kDma->CR = 0U;
  (void)WaitWhile(kDma->CR, DMA_SxCR_EN, false);  // a stream stops at the end of its burst
  DMA1->LIFCR = kDmaStream0Flags;
  kDmaMux->CCR = kDmaRequestTimerUp;
  kDma->PAR = reinterpret_cast<uintptr_t>(&kTim->DMAR);
  kDma->M0AR = reinterpret_cast<uintptr_t>(g_dma_slots[0].data());
  kDma->NDTR = kDmaTransfers;
  kDma->FCR = 0U;  // direct mode (no FIFO): one word per request beat
  // Memory to peripheral, words, memory increment, circular, very high priority.
  kDma->CR = DMA_SxCR_DIR_0 | DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 |
             DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_EN;

  kTim->DCR = ((kBurstLength - 1U) << TIM_DCR_DBL_Pos) | (kCcr1WordOffset << TIM_DCR_DBA_Pos);
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
  const auto kDtg = stm32::EncodeDeadTime(stm32::DeadTimeTicks(kDeadTimeNs, kTimerClockHz));
  if (!kDtg.has_value()) {
    return false;
  }

  // Break on, outputs held in the idle (off) state. MOE stays 0 until PowerStageEnable().
  kTim->BDTR = kBreakConfig | *kDtg;

  kTim->EGR = TIM_EGR_UG;  // load ARR, RCR and the preloaded compare values
  kTim->SR = 0U;
  // Start between the trigger compares: neither OC4REF nor OC6REF is active
  // there, so starting the counter cannot produce a spurious ADC trigger. The
  // first boundary is then both the first sample and the first DMA burst.
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

  InitDma();
  // Timestamp clock (Hall edges, control boundary): the DWT cycle counter.
  StartCycleCounter();
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

void Stop() noexcept {
  if (!g_started) {
    return;
  }
  g_started = false;
  // First the interrupts: no control cycle or observer runs from here on.
  irq_disable(kFastIrq);
  irq_disable(kObserverIrq);

  kTim->BDTR &= ~TIM_BDTR_MOE;  // outputs off (the power stage is disabled already)
  kTim->DIER = 0U;              // no more DMA requests
  kTim->CR1 &= ~TIM_CR1_CEN;    // no more ADC triggers
  kDma->CR &= ~DMA_SxCR_EN;

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

void SetDutyTicks(const TickPlan& ticks) noexcept {
  const uint32_t kRemaining = kDma->NDTR;
  const bool kCountingDown = (kTim->CR1 & TIM_CR1_DIR) != 0U;

  for (uint8_t k = 1U; k < kHalfPeriodsPerCycle; ++k) {
    g_dma_slots[k] = ticks[k];
  }

  if (kRemaining == kDmaTransfers - kBurstLength) {
    // Normal case: the burst of the control boundary (slot 0) is done and the
    // next one has not started. Entry 0 goes into the preload now; slot 0 holds
    // the last entry as a fallback for a late next cycle.
    kTim->CCR1 = ticks[0][0];
    kTim->CCR2 = ticks[0][1];
    kTim->CCR3 = ticks[0][2];
    g_dma_slots[0] = ticks[kHalfPeriodsPerCycle - 1U];
    if (kDma->NDTR != kRemaining) {
      ++g_duty_overruns;  // the next update came during the write
    }
    g_control_at_peak = kCountingDown;  // just after a peak the counter counts down
  } else if (kRemaining == kDmaTransfers) {
    // Early: the control boundary is still ahead (long trigger lead). Its
    // burst delivers entry 0.
    g_dma_slots[0] = ticks[0];
    g_control_at_peak = !kCountingDown;  // counting up towards a peak
  } else {
    ++g_duty_overruns;
  }

  // Sample k of the next cycle is taken in the half period ending at the
  // (k + 1)-th boundary after the control boundary.
  g_applied[0] = g_last_entry;
  for (uint8_t k = 1U; k < kHalfPeriodsPerCycle; ++k) {
    g_applied[k] = ticks[k - 1U];
  }
  g_last_entry = ticks[kHalfPeriodsPerCycle - 1U];
}

uint32_t DutyOverruns() noexcept { return g_duty_overruns; }

const CycleRaw& CycleSamples() noexcept { return g_cycle; }

uint32_t Timestamp() noexcept { return DWT->CYCCNT; }

const PhaseTicks& AppliedTicks(const uint8_t sample) noexcept { return g_applied[sample]; }

bool SampleAtPeak(const uint8_t sample) noexcept {
  // The last sample shares the boundary type with the control boundary; the
  // type alternates from boundary to boundary.
  const bool kSameAsControl = ((kHalfPeriodsPerCycle - 1U - sample) % 2U) == 0U;
  return kSameAsControl == g_control_at_peak;
}

void SetOvercurrentWindow(const uint32_t low, const uint32_t high) noexcept {
  g_overcurrent_window.store(PackWindow(low, high), std::memory_order_relaxed);
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
