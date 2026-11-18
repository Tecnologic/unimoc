/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ebike_io.cpp
 *    @brief E-bike hardware of the STM32H7 platform: none.
 *
 *    The NUCLEO-H743ZI and the Jacobi drive have no crank sensor and no
 *    display. The e-bike control is off (CONFIG_UNIMOC_EBIKE depends on the
 *    simulation); these functions only satisfy the interface.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "ebike_interface.hpp"

namespace unimoc::hardware {

namespace {

unit::Torque ReadCrankTorque() noexcept { return unit::Torque{}; }

std::size_t ReadCadencePulses(std::span<unit::Time> /*ages*/) noexcept { return 0U; }

std::size_t ReadDisplay(std::span<uint8_t> /*bytes*/) noexcept { return 0U; }

void WriteDisplay(std::span<const uint8_t> /*bytes*/) noexcept {}

const EbikeIo kEbikeIo{ReadCrankTorque, ReadCadencePulses, ReadDisplay, WriteDisplay};

}  // namespace

const EbikeIo& GetEbikeIo() noexcept { return kEbikeIo; }

void StartEbikeIo() noexcept {}

}  // namespace unimoc::hardware
