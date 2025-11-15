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

#include <modm/debug.hpp>
#include <modm/platform.hpp>

#include "hardware_interface.hpp"
#include "analog.hpp"
#include "pulse_width.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;
using namespace modm::literals;

namespace unimoc::hardware
{

bool initialize(void);

HardwareInterface motor[MOTORS] = {{// Initialize function
									initialize,

									// Get phase currents function
									analog::getPhaseCurrents,

									// Get phase voltages function
									analog::getPhaseVoltages,

									// Set phase duties function
									pulse_width::setPhaseDuties
									}};

/**
 * Initializes the hardware components of the battery case controller.
 *
 * This function sets up the system clock, initializes the ADCs, and configures
 * the pulse width modulation (PWM) for motor control.
 */
bool
initialize(void)
{
	if (!SystemClock::enable()) {
		MODM_LOG_ERROR << "Failed to enable system clock." << modm::endl;
		return false;  // Return false if system clock initialization fails
	}

	// Initialize the ADCs
	if (!analog::initialize()) {
		MODM_LOG_ERROR << "Failed to initialize ADCs." << modm::endl;
		return false;  // Return false if ADC initialization fails
	}

	// Initialize the pulse width modulation (PWM) for motor control
	if (!pulse_width::initialize()) {
		MODM_LOG_ERROR << "Failed to initialize PWM." << modm::endl;
		return false;  // Return false if PWM initialization fails
	}

	MODM_LOG_INFO << "Hardware initialization successful." << modm::endl;
	return true;  // Return true if all initializations are successful
}

}  // namespace unimoc::hardware


// Include the necessary headers for RTT logging
using LoggerDevice = modm::IODeviceWrapper<Rtt<0>, modm::IOBuffer::DiscardIfFull>;
static LoggerDevice rtt_device;
// Set all four logger streams to use RTT
modm::log::Logger modm::log::debug(rtt_device);
modm::log::Logger modm::log::info(rtt_device);
modm::log::Logger modm::log::warning(rtt_device);
modm::log::Logger modm::log::error(rtt_device);
