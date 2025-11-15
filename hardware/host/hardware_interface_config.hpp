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

#ifndef UNIMOC_HARDWARE_INTERFACE_CONFIG_H_
#define UNIMOC_HARDWARE_INTERFACE_CONFIG_H_


#include <cstdint>

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace hardware hardware namespace
 */
namespace hardware
{
constexpr uint_fast8_t VERSION_MAJOR = 1;  ///< Major version of the hardware interface
constexpr uint_fast8_t VERSION_MINOR = 0;  ///< Minor version of the hardware interface
constexpr uint_fast8_t VERSION_PATCH = 0;  ///< Patch version of the hardware interface

constexpr uint_fast32_t VERSION =
	(VERSION_MAJOR << 16) | (VERSION_MINOR << 8) | VERSION_PATCH;  ///< Combined hardware version

constexpr uint_fast8_t MOTORS = 1;  ///< Number of motors supported by the hardware interface


}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_INTERFACE_CONFIG_H_ */