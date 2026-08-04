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
#include <array>

#include "three_phase_system.hpp"
#include "stator_system.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace coordinate systems.
 */
namespace system
{
// clarke transformation
// transform abc 3 phase vector to alpha beta vector.
constexpr StatorReference
ThreePhase::clark() const noexcept
{
	constexpr float sqrt3by2 = std::sqrt(3.0f) / 2.0f;
	constexpr float _2by3 = 2.0f / 3.0f;

	return StatorReference{_2by3 * (a - (0.5f * b) - (0.5f * c)),
						   _2by3 * ((sqrt3by2 * b) - (sqrt3by2 * c))};
}

}  // namespace system
}  // namespace unimoc
