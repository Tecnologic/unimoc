/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

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
#include <modm/platform.hpp>
#include <modm/debug.hpp>
#include "HardwareInterface.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;



int main()
{
    GpioA3::setOutput();
    GpioA4::setOutput();

    unimoc::hardware::analog::initialize();

    while (true)
    {
        GpioA3::toggle();       

        GpioA4::toggle();


        modm::delay(0.5s);
    }
}