/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include_next <zephyr/sys/sys_io.h>
#include <zephyr/arch/common/sys_io.h>
#define sys_read32 rp2040_test_read32
uint32_t rp2040_test_read32(mem_addr_t address);
