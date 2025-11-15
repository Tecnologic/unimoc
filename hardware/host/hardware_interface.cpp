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

#include "hardware_interface.hpp"

namespace unimoc
{
namespace hardware
{

// Stub function implementations
static bool initializeStub()
{
	// Stub: Always return successful initialization
	return true;
}

static system::ThreePhase getPhaseCurrentsStub()
{
	// Stub: Return zero currents
	return system::ThreePhase{0.0f, 0.0f, 0.0f};
}

static system::ThreePhase getPhaseVoltagesStub()
{
	// Stub: Return zero voltages
	return system::ThreePhase{0.0f, 0.0f, 0.0f};
}

static void setPhaseDutysStub(system::ThreePhase duties)
{
	// Stub: Accept duties but do nothing
	(void)duties;
}

// Initialize the motor array with stub implementations
HardwareInterface motor[MOTORS] = {
	HardwareInterface(
		initializeStub,
		getPhaseCurrentsStub,
		getPhaseVoltagesStub,
		setPhaseDutysStub
	)
};

}  // namespace hardware
}  // namespace unimoc