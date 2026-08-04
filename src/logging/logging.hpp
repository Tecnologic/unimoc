/*
       __  ___   ________  _______  ______
      / / / / | / /  _/  |/  / __ \/ ____/
     / / / /  |/ // // /|_/ / / / / /
    / /_/ / /|  // // /  / / /_/ / /___
    \____/_/ |_/___/_/  /_/\____/\____/

    Universal Motor Control 2025 Alexander <tecnologic86@gmail.com> Evers

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

modm::platform::Rtt rtt(0);
modm::IODeviceObjectWrapper<modm::platform::Rtt, modm::IOBuffer::DiscardIfFull> rtt_device(rtt);
// Set all four logger streams to use RTT
modm::log::Logger modm::log::debug(rtt_device);
modm::log::Logger modm::log::info(rtt_device);
modm::log::Logger modm::log::warning(rtt_device);
modm::log::Logger modm::log::error(rtt_device);