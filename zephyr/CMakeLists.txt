# SPDX-License-Identifier: GPL-3.0-or-later
#
# UNIMOC as a Zephyr module: with CONFIG_UNIMOC=y the libraries of lib/ are
# declared as interface targets (unimoc_lib, unimoc_sim, ...), which the
# application and the tests link. Their sources use ZEPHYR_UNIMOC_MODULE_DIR.
if(CONFIG_UNIMOC)
  include(${CMAKE_CURRENT_LIST_DIR}/../cmake/UnimocZephyr.cmake)
  unimoc_zephyr_libraries()
endif()
