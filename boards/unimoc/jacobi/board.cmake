# SPDX-License-Identifier: GPL-3.0-or-later

board_runner_args(jlink "--device=STM32H743VI" "--speed=4000")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
