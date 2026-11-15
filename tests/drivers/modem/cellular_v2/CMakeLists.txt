# Copyright (c) 2026 Embeint Inc
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
# The test driver binding is shared with the original driver's tests
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../cellular_common)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(modem_cellular_v2_test)

set(common ${CMAKE_CURRENT_SOURCE_DIR}/../cellular_common)
target_sources(app PRIVATE
  ${common}/driver/modem_cellular_test.c
  ${common}/src/stubs.c
  ${common}/src/test_helpers.c
  src/apn.c
  src/board_init.c
  src/core.c
  src/gpio.c
  src/lte_rf.c
  src/modem_info.c
  src/periodic.c
  src/ppp.c
  src/registration.c
  src/script_errors.c
  src/sim.c
  src/suspend.c
)
target_include_directories(app PRIVATE
  ${common}/driver
  ${common}/src
  ${ZEPHYR_BASE}/subsys/modem
)
