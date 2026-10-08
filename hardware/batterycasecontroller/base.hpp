/*
           __  ___   ________  _______  ______
          / / / / | / /  _/  |/  / __ \/ ____/
         / / / /  |/ // // /|_/ / / / / /
        / /_/ / /|  // // /  / / /_/ / /___
        \____/_/ |_/___/_/  /_/\____/\____/

        Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

        This file is part of UNIMOC.

        UNIMOC is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef UNIMOC_HARDWARE_BASE_H_
#define UNIMOC_HARDWARE_BASE_H_

#include <cstdint>
#include <modm/platform.hpp>

namespace unimoc::hardware {

using namespace modm::literals;
/**
 * @brief SystemClock class for the STM32G474RE microcontroller.
 *
 * This class provides the system clock frequencies and methods to enable the
 * system clock for the STM32G474RE microcontroller, which is running at 170MHz
 * generated from an external 8MHz oscillator.
 */
struct SystemClock {
  static constexpr uint32_t Frequency = 170_MHz;  // System clock frequency
  static constexpr uint32_t Ahb1 = Frequency;
  static constexpr uint32_t Ahb2 = Frequency;
  static constexpr uint32_t Apb1 = Frequency;
  static constexpr uint32_t Apb2 = Frequency;

  static constexpr uint32_t Cordic = Ahb1;
  static constexpr uint32_t Crc = Ahb1;
  static constexpr uint32_t Dma = Ahb1;
  static constexpr uint32_t Dma1 = Dma;
  static constexpr uint32_t Dma2 = Dma;
  static constexpr uint32_t DmaMux = Dma;
  static constexpr uint32_t Fmac = Ahb1;

  static constexpr uint32_t Adc = Ahb2;
  static constexpr uint32_t Adc1 = Adc;
  static constexpr uint32_t Adc2 = Adc;
  static constexpr uint32_t Adc3 = Adc;
  static constexpr uint32_t Adc4 = Adc;
  static constexpr uint32_t Adc5 = Adc;
  static constexpr uint32_t Dac = Ahb2;
  static constexpr uint32_t Dac1 = Dac;
  static constexpr uint32_t Dac2 = Dac;
  static constexpr uint32_t Dac3 = Dac;
  static constexpr uint32_t Dac4 = Dac;
  static constexpr uint32_t Rng = Ahb2;

  static constexpr uint32_t Can = Apb1;
  static constexpr uint32_t Fdcan1 = Can;
  static constexpr uint32_t Fdcan2 = Can;
  static constexpr uint32_t Fdcan3 = Can;
  static constexpr uint32_t I2c = Apb1;
  static constexpr uint32_t I2c1 = I2c;
  static constexpr uint32_t I2c2 = I2c;
  static constexpr uint32_t I2c3 = I2c;
  static constexpr uint32_t I2c4 = I2c;
  static constexpr uint32_t Lptim = Apb1;
  static constexpr uint32_t Lpuart = Apb1;
  static constexpr uint32_t Spi2 = Apb1;
  static constexpr uint32_t Spi3 = Apb1;
  static constexpr uint32_t Uart4 = Apb1;
  static constexpr uint32_t Uart5 = Apb1;
  static constexpr uint32_t Usart2 = Apb1;
  static constexpr uint32_t Usart3 = Apb1;
  static constexpr uint32_t Usb = Apb1;
  static constexpr uint32_t Apb1Timer = Apb1 * 1;
  static constexpr uint32_t Timer2 = Apb1Timer;
  static constexpr uint32_t Timer3 = Apb1Timer;
  static constexpr uint32_t Timer4 = Apb1Timer;
  static constexpr uint32_t Timer5 = Apb1Timer;
  static constexpr uint32_t Timer6 = Apb1Timer;
  static constexpr uint32_t Timer7 = Apb1Timer;

  static constexpr uint32_t Sai1 = Apb2;
  static constexpr uint32_t Spi1 = Apb2;
  static constexpr uint32_t Usart1 = Apb2;
  static constexpr uint32_t Apb2Timer = Apb2 * 1;
  static constexpr uint32_t Timer1 = Apb2Timer;
  static constexpr uint32_t Timer8 = Apb2Timer;
  static constexpr uint32_t Timer15 = Apb2Timer;
  static constexpr uint32_t Timer16 = Apb2Timer;
  static constexpr uint32_t Timer17 = Apb2Timer;
  static constexpr uint32_t Timer20 = Apb2Timer;
  static constexpr uint32_t Iwdg = modm::platform::Rcc::LsiFrequency;

  static constexpr uint32_t Rtc = 32.768_kHz;

  static bool inline enable() {
    using namespace modm::platform;

    Rcc::enableExternalClock();
    Rcc::PllFactors pllFactors{
        .pllM = 2,   //   8MHz / M= 2 ->   4MHz
        .pllN = 85,  //   4MHz * N=85 -> 340MHz
        .pllR = 2,   // 340MHz / R= 2 -> 170MHz = F_cpu
    };
    Rcc::enablePll(Rcc::PllSource::ExternalClock, pllFactors);
    Rcc::setFlashLatency<Frequency>();
    Rcc::setVoltageScaling(Rcc::VoltageScaling::Boost);  // recommended for >150 MHz
    // switch system clock to PLL output
    Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
    Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
    // APB1 has max. 170MHz
    Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div1);
    Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
    // update frequencies for busy-wait delay functions
    Rcc::updateCoreFrequency<Frequency>();

    Rcc::enableLowSpeedExternalCrystal();
    Rcc::enableRealTimeClock(Rcc::RealTimeClockSource::LowSpeedExternalCrystal);

    Rcc::setCanClockSource(Rcc::CanClockSource::Pclk);
    return true;
  }
};

}  // namespace unimoc::hardware

#endif  // UNIMOC_HARDWARE_BASE_H_