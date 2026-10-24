/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ebike_io.cpp
 *    @brief BCC e-bike hardware: crank cadence pulses, display line capture and transmit.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include "analog_scaling.hpp"
#include "ebike_interface.hpp"
#include "edge_uart_decoder.hpp"
#include "foc_stm32g4.hpp"
#include "hardware_interface_config.hpp"

LOG_MODULE_REGISTER(unimoc_ebike_io, LOG_LEVEL_INF);

// Zephyr's pinctrl, ring buffer and devicetree macros define C arrays and use C-style casts.
// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
#define UNIMOC_EBIKE DT_NODELABEL(ebike0)
PINCTRL_DT_DEFINE(UNIMOC_EBIKE);
RING_BUF_DECLARE(g_display_tx, 64);
// NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

namespace unimoc::hardware {

namespace {

constexpr uint32_t kDisplayBaud = 9600U;
// Normal (not zero-latency) priorities, below the observer interrupt (0).
constexpr unsigned kCaptureIrqPriority = 4U;
constexpr uint32_t kDisplayRxPin = 15U;  // PB15, GPIOB->IDR bit

/// Single producer (interrupt), single consumer (thread) queue of entries.
template <typename Entry, std::size_t kSize>
class EdgeQueue {
 public:
  void Push(const Entry& entry) noexcept {
    const uint32_t kHead = head_.load(std::memory_order_relaxed);
    const uint32_t kNext = (kHead + 1U) % kSize;
    if (kNext == tail_.load(std::memory_order_acquire)) {
      overruns_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    entries_[kHead] = entry;
    head_.store(kNext, std::memory_order_release);
  }

  bool Pop(Entry& entry) noexcept {
    const uint32_t kTail = tail_.load(std::memory_order_relaxed);
    if (kTail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    entry = entries_[kTail];
    tail_.store((kTail + 1U) % kSize, std::memory_order_release);
    return true;
  }

  [[nodiscard]] uint32_t Overruns() const noexcept { return overruns_.load(std::memory_order_relaxed); }

 private:
  std::array<Entry, kSize> entries_{};
  std::atomic<uint32_t> head_{0U};
  std::atomic<uint32_t> tail_{0U};
  std::atomic<uint32_t> overruns_{0U};
};

EdgeQueue<uint32_t, 32> g_crank_pulses;
EdgeQueue<ebike::UartEdge, 128> g_display_edges;
ebike::EdgeUartDecoder g_display_decoder{kTimestampFrequency / kDisplayBaud};

struct gpio_callback g_crank_callback;
const struct gpio_dt_spec kCrankPulse = GPIO_DT_SPEC_GET(UNIMOC_EBIKE, crank_pulse_gpios);
const struct device* const kDisplayUart = DEVICE_DT_GET(DT_PHANDLE(UNIMOC_EBIKE, display_uart));
TIM_TypeDef* const kCaptureTimer =
    reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_PHANDLE(UNIMOC_EBIKE, display_rx_timer)));
GPIO_TypeDef* const kDisplayRxPort = GPIOB;
static_assert(DT_REG_ADDR(DT_GPIO_CTLR(UNIMOC_EBIKE, display_rx_gpios)) == GPIOB_BASE &&
                  DT_GPIO_PIN(UNIMOC_EBIKE, display_rx_gpios) == kDisplayRxPin,
              "the display receive capture is written for PB15");

// Cadence pulse: timestamp only.
void CrankPulseIsr(const struct device* /*port*/,
                   struct gpio_callback* /*callback*/,
                   gpio_port_pins_t /*pins*/) {
  g_crank_pulses.Push(foc_stm32g4::Timestamp());
}

// Both edges of the display line: the timestamp and the level after the edge.
void DisplayEdgeIsr(const void* /*argument*/) {
  const uint32_t kTicks = foc_stm32g4::Timestamp();
  const bool kLevel = (kDisplayRxPort->IDR & (1U << kDisplayRxPin)) != 0U;
  (void)kCaptureTimer->CCR2;  // reading the capture register clears CC2IF
  kCaptureTimer->SR = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
  g_display_edges.Push(ebike::UartEdge{.ticks = kTicks, .level = kLevel});
}

void DisplayUartIsr(const struct device* device, void* /*user_data*/) {
  if (uart_irq_update(device) == 0 || uart_irq_tx_ready(device) == 0) {
    return;
  }
  std::array<uint8_t, 16> chunk{};
  const uint32_t kCount = ring_buf_get(&g_display_tx, chunk.data(), static_cast<uint32_t>(chunk.size()));
  if (kCount == 0U) {
    uart_irq_tx_disable(device);
    return;
  }
  (void)uart_fifo_fill(device, chunk.data(), static_cast<int>(kCount));
}

unit::Torque ReadCrankTorque() noexcept {
  return analog::CountsToTorque(foc_stm32g4::ReadRaw().crank_torque);
}

std::size_t ReadCadencePulses(const std::span<unit::Time> ages) noexcept {
  const uint32_t kNow = foc_stm32g4::Timestamp();
  std::size_t count = 0U;
  uint32_t ticks = 0U;
  while (g_crank_pulses.Pop(ticks)) {
    if (count < ages.size()) {
      ages[count++] = unit::Time{static_cast<float>(kNow - ticks) / static_cast<float>(kTimestampFrequency)};
    }
  }
  return count;
}

std::size_t ReadDisplay(const std::span<uint8_t> bytes) noexcept {
  ebike::UartEdge edge;
  while (g_display_edges.Pop(edge)) {
    g_display_decoder.Push(edge);
  }
  g_display_decoder.Poll(foc_stm32g4::Timestamp());
  std::size_t count = 0U;
  while (count < bytes.size() && g_display_decoder.Pop(bytes[count])) {
    ++count;
  }
  return count;
}

void WriteDisplay(const std::span<const uint8_t> bytes) noexcept {
  // A frame that does not fit is dropped as a whole: half a frame would only confuse the display.
  if (ring_buf_space_get(&g_display_tx) < bytes.size()) {
    return;
  }
  (void)ring_buf_put(&g_display_tx, bytes.data(), static_cast<uint32_t>(bytes.size()));
  uart_irq_tx_enable(kDisplayUart);
}

const EbikeIo kIo{.read_crank_torque = ReadCrankTorque,
                  .read_cadence_pulses = ReadCadencePulses,
                  .read_display = ReadDisplay,
                  .write_display = WriteDisplay};

}  // namespace

const EbikeIo& GetEbikeIo() noexcept { return kIo; }

namespace {

// PB15 as TIM15 channel 2 input, free-running timer, an interrupt on every edge of the line.
bool StartDisplayCapture() noexcept {
  if (pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(UNIMOC_EBIKE), PINCTRL_STATE_DEFAULT) != 0) {
    // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay):
    // the LOG macro declares a C array.
    LOG_ERR("display receive pin not configured");
    // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    return false;
  }
  // Channel 2 captures both edges of TI2 (CC2P and CC2NP together select both).
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
  kCaptureTimer->PSC = 0U;
  kCaptureTimer->ARR = 0xFFFFU;
  kCaptureTimer->CCMR1 = TIM_CCMR1_CC2S_0;  // CC2 is an input on TI2, no filter
  kCaptureTimer->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP;
  kCaptureTimer->SR = 0U;
  kCaptureTimer->DIER = TIM_DIER_CC2IE;
  kCaptureTimer->CR1 = TIM_CR1_CEN;
  IRQ_CONNECT(TIM1_BRK_TIM15_IRQn, kCaptureIrqPriority, DisplayEdgeIsr, nullptr, 0);
  irq_enable(TIM1_BRK_TIM15_IRQn);
  return true;
}

bool StartCrankPulse() noexcept {
  if (!gpio_is_ready_dt(&kCrankPulse) || gpio_pin_configure_dt(&kCrankPulse, GPIO_INPUT) != 0) {
    // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay):
    // the LOG macro declares a C array.
    LOG_ERR("crank pulse pin not ready");
    // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    return false;
  }
  gpio_init_callback(&g_crank_callback, CrankPulseIsr, BIT(kCrankPulse.pin));
  (void)gpio_add_callback(kCrankPulse.port, &g_crank_callback);
  (void)gpio_pin_interrupt_configure_dt(&kCrankPulse, GPIO_INT_EDGE_FALLING);
  return true;
}

bool StartDisplayUart() noexcept {
  if (!device_is_ready(kDisplayUart)) {
    // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay):
    // the LOG macro declares a C array.
    LOG_ERR("display UART not ready");
    // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    return false;
  }
  uart_irq_callback_user_data_set(kDisplayUart, DisplayUartIsr, nullptr);
  return true;
}

}  // namespace

void StartEbikeIo() noexcept {
  if (StartDisplayCapture() && StartCrankPulse() && StartDisplayUart()) {
    // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay):
    // the LOG macro declares a C array.
    LOG_INF("e-bike inputs started");
    // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  }
}

}  // namespace unimoc::hardware
