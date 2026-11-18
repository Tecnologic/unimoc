/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file can_transceiver.hpp
 *    @brief Standby pin of the CAN transceiver of an STM32H7 board.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

namespace unimoc::hardware {

/**
 * @brief Wakes the CAN transceiver: drives `can-standby-gpios` of zephyr,user inactive.
 *
 * Shared by the drive and the VDC on the STM32H7 boards. A board without the
 * property has a transceiver without a standby pin; nothing happens then.
 * Call from the Cyphal thread before the CAN controller starts.
 */
void InitCanTransceiver() noexcept;

}  // namespace unimoc::hardware
