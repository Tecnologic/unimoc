/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_cmdline.c
 *    @brief Command-line options of the native_sim drive.
 *
 *    `--node-id=<n>` sets the Cyphal node-ID of this process in the RAM
 *    settings, so several simulated drives can share one bus without
 *    separate flash files. Combine it with `--device_id=<n>` (hwinfo) for
 *    distinct unique-IDs. `--cart-wheel=<n>` sets the cart wheel of the drive
 *    (`unimoc.cart.wheel`) the same way (scripts/sim_cluster.sh).
 *
 *    `--describe-registers=<file>` writes the register description file
 *    (registers.json) of the drive after its start and ends the process.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "sim_cmdline.h"
#include <cmdline.h>
#include <posix_native_task.h>
#include <stdbool.h>

static uint32_t node_id_option;
static bool node_id_given;
static char* describe_registers_option;
static uint32_t cart_wheel_option;
static bool cart_wheel_given;

// NOLINTNEXTLINE(readability-non-const-parameter): the signature of the native_sim option callback.
static void NodeIdFound(char* argv, int offset) {
  (void)argv;
  (void)offset;
  node_id_given = true;
}

// NOLINTNEXTLINE(readability-non-const-parameter): the signature of the native_sim option callback.
static void CartWheelFound(char* argv, int offset) {
  (void)argv;
  (void)offset;
  cart_wheel_given = true;
}

static void AddOptions(void) {
  static struct args_struct_t options[] = {
      {
          .option = "node-id",
          .name = "id",
          .type = 'u',
          .dest = (void*)&node_id_option,
          .call_when_found = NodeIdFound,
          .descript = "Cyphal node-ID of the simulated drive (0..127), "
                      "instead of uavcan.node.id from the settings.",
      },
      {
          .option = "cart-wheel",
          .name = "wheel",
          .type = 'u',
          .dest = (void*)&cart_wheel_option,
          .call_when_found = CartWheelFound,
          .descript = "Cart wheel of the simulated drive (0..3), "
                      "instead of unimoc.cart.wheel from the settings.",
      },
      {
          .option = "describe-registers",
          .name = "file",
          .type = 's',
          .dest = (void*)&describe_registers_option,
          .descript = "Write the register description file (JSON) after the start, then exit.",
      },
      ARG_TABLE_ENDMARKER,
  };

  native_add_command_line_opts(options);
}

NATIVE_TASK(AddOptions, PRE_BOOT_1, 20);

int32_t UnimocSimNodeIdOption(void) { return node_id_given ? (int32_t)node_id_option : -1; }

int32_t UnimocSimCartWheelOption(void) { return cart_wheel_given ? (int32_t)cart_wheel_option : -1; }

const char* UnimocSimDescribeRegistersOption(void) { return describe_registers_option; }
