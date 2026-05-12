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

#include "Units.hpp"
#include "HardwareInterface.hpp"
#include <modm/platform.hpp>

using namespace modm::platform;
using namespace std::chrono_literals;


namespace unimoc::hardware::analog
{

//
// ADC Channel Mapping
// Pin		Channel			Signal
// ---------------------------------------------
// PA0		ADC1_IN1		Current IA
// PA2		ADC1_IN3		Voltage VA
//
using A1_IA = GpioA0::In1;
using A1_VA = GpioA2::In3;
// ---------------------------------------------
// PA1		ADC2_IN2		Current IB
// PA6		ADC2_IN3		Voltage VB
using A2_IB = GpioA1::In2;
using A2_VB = GpioA6::In3;
// ---------------------------------------------
// PB1		ADC3_IN1		Current IC
// PB13		ADC3_IN5		Voltage VC
using A3_IC = GpioB1::In1;
using A3_VC = GpioB13::In5;
// ---------------------------------------------
// PB12		ADC4_IN3		DC Link Voltage
// PB14		ADC4_IN4		Crank Torque
using A4_VDC = GpioB12::In3;
using A4_CRKT = GpioB14::In4;
// ---------------------------------------------
// PA8		ADC5_IN1		Motor Temperature
// PA9		ADC5_IN2		Bridge Temperature
using A5_MOTT = GpioA8::In1;
using A5_BRDGT = GpioA9::In2;

void initialize()
{
    // Initialize the GPIOs for ADC channels
    GpioA0::setAnalogInput();
    GpioA2::setAnalogInput();
    GpioA1::setAnalogInput();
    GpioA6::setAnalogInput();
    GpioB1::setAnalogInput();
    GpioB13::setAnalogInput();
    GpioB12::setAnalogInput();
    GpioB14::setAnalogInput();
    GpioA8::setAnalogInput();
    GpioA9::setAnalogInput();

    Adc1::initialize(Adc1::ClockMode::SynchronousPrescaler4,
                     Adc1::ClockSource::SystemClock,
                     Adc1::Prescaler::Disabled,
                     Adc1::CalibrationMode::SingleEndedInputsMode,
                     true);

    Adc1::connect<A1_IA, A1_VA>();
    Adc1::setInjectedConversionSequenceLength(4);
    Adc1::setInjectedConversionChannel<A1_IA>(0, Adc1::SampleTime::Cycles13);
    Adc1::setInjectedConversionChannel<A1_VA>(1, Adc1::SampleTime::Cycles13);
    Adc1::setInjectedConversionChannel<A1_IA>(2, Adc1::SampleTime::Cycles13);
    Adc1::setInjectedConversionChannel<A1_VA>(3, Adc1::SampleTime::Cycles13);

    Adc2::initialize(Adc2::ClockMode::SynchronousPrescaler4,
                     Adc2::ClockSource::SystemClock,
                     Adc2::Prescaler::Disabled,
                     Adc2::CalibrationMode::SingleEndedInputsMode,
                     true);

    Adc2::connect<A2_IB, A2_VB>();
    Adc2::setInjectedConversionSequenceLength(4);
    Adc2::setInjectedConversionChannel<A2_IB>(0, Adc2::SampleTime::Cycles13);
    Adc2::setInjectedConversionChannel<A2_VB>(1, Adc2::SampleTime::Cycles13);
    Adc2::setInjectedConversionChannel<A2_IB>(2, Adc2::SampleTime::Cycles13);
    Adc2::setInjectedConversionChannel<A2_VB>(3, Adc2::SampleTime::Cycles13);

    Adc3::initialize(Adc3::ClockMode::SynchronousPrescaler4,
                     Adc3::ClockSource::SystemClock,
                     Adc3::Prescaler::Disabled,
                     Adc3::CalibrationMode::SingleEndedInputsMode,
                     true);

    Adc3::connect<A3_IC, A3_VC>();
    Adc3::setInjectedConversionSequenceLength(4);
    Adc3::setInjectedConversionChannel<A3_IC>(0, Adc3::SampleTime::Cycles13);
    Adc3::setInjectedConversionChannel<A3_VC>(1, Adc3::SampleTime::Cycles13);
    Adc3::setInjectedConversionChannel<A3_IC>(2, Adc3::SampleTime::Cycles13);
    Adc3::setInjectedConversionChannel<A3_VC>(3, Adc3::SampleTime::Cycles13);

    Adc4::initialize(Adc4::ClockMode::SynchronousPrescaler4,
                     Adc4::ClockSource::SystemClock,
                     Adc4::Prescaler::Disabled,
                     Adc4::CalibrationMode::SingleEndedInputsMode,
                     true);
    Adc4::connect<A4_VDC, A4_CRKT>();
    Adc4::setInjectedConversionSequenceLength(4);
    Adc4::setInjectedConversionChannel<A4_VDC>(0, Adc4::SampleTime::Cycles13);
    Adc4::setInjectedConversionChannel<A4_CRKT>(1, Adc4::SampleTime::Cycles13);
    Adc4::setInjectedConversionChannel<A4_VDC>(2, Adc4::SampleTime::Cycles13);
    Adc4::setInjectedConversionChannel<A4_CRKT>(3, Adc4::SampleTime::Cycles13);

    Adc5::initialize(Adc5::ClockMode::SynchronousPrescaler4,
                     Adc5::ClockSource::SystemClock,
                     Adc5::Prescaler::Disabled,
                     Adc5::CalibrationMode::SingleEndedInputsMode,
                     true);
    Adc5::connect<A5_MOTT, A5_BRDGT>();
    Adc5::setInjectedConversionSequenceLength(4);
    Adc5::setInjectedConversionChannel<A5_MOTT>(0, Adc5::SampleTime::Cycles13);
    Adc5::setInjectedConversionChannel<A5_BRDGT>(1, Adc5::SampleTime::Cycles13);
    Adc5::setInjectedConversionChannel<A5_MOTT>(2, Adc5::SampleTime::Cycles13);
    Adc5::setInjectedConversionChannel<A5_BRDGT>(3, Adc5::SampleTime::Cycles13);
}

} // namespace unimoc::hardware::analog
