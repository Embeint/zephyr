/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <zephyr/sys/byteorder.h>
#define htod32(value) sys_cpu_to_le32(value)
