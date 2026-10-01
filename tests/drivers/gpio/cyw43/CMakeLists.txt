# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gpio_cyw43)
target_include_directories(app PRIVATE include)
target_compile_definitions(app PRIVATE CONFIG_GPIO_CYW43_INIT_PRIORITY=82)
target_sources(app PRIVATE src/main.c ${ZEPHYR_BASE}/drivers/gpio/gpio_cyw43.c)
