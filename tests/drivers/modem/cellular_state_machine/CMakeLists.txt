# Copyright (c) 2026 Embeint Inc
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
# The test driver binding is shared with the MODEM_CELLULAR_V2 tests
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../cellular_common)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(modem_cellular_state_machine_test)

set(common ${CMAKE_CURRENT_SOURCE_DIR}/../cellular_common)
target_sources(app PRIVATE
  ${common}/driver/modem_cellular_test.c
  ${common}/src/stubs.c
  ${common}/src/test_helpers.c
)
target_include_directories(app PRIVATE
  ${common}/driver
  ${common}/src
  ${ZEPHYR_BASE}/subsys/modem
)

# On-demand connect rejects modems with a network script, so it runs on its own devicetree
if(CONFIG_MODEM_CELLULAR_ON_DEMAND_CONNECT)
  target_sources(app PRIVATE
    src/on_demand.c
  )
else()
  target_sources(app PRIVATE
    src/api.c
    src/board_init.c
    src/gpio.c
    src/main.c
    src/modem_info.c
    src/network_status.c
    src/periodic.c
    src/ppp_dead.c
    src/ring.c
    src/script_errors.c
    src/signal.c
    src/suspend.c
    src/unsolicited.c
  )
endif()
