# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(entropy_rp2040_rosc)
target_include_directories(app PRIVATE include)
target_sources(app PRIVATE src/main.c ${ZEPHYR_BASE}/drivers/entropy/entropy_rp2040_rosc.c)
