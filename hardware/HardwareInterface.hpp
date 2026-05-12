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

#ifndef UNIMOC_HARDWARE_INTERFACE_H_
#define UNIMOC_HARDWARE_INTERFACE_H_

#include "Units.hpp"
#include <cmath>

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
/**
 * @namespace analog analog namespace
 */
namespace analog
{
void initialize();
}  // namespace analog

/**
 * @namespace angle angle namespace
 */
namespace angle
{

// sin(Angle) -> DimensionlessRatio
template<typename Rep, typename P>
unit::DimensionlessRatio
calculate_sine_cosine(const unit::Unit<Rep, P, unit::AngleTag>& angle)
{
	// Assumes angle.value() is in radians, which is true for the Angle alias (Period =
	// std::ratio<1>)
	return unit::DimensionlessRatio(std::sin(angle.value()));
}

}  // namespace angle


}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_INTERFACE_H_ */