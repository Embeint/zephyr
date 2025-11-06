/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/llext/symbol.h>

EXPORT_GROUP_SYMBOL(LIBC, strcpy);
EXPORT_GROUP_SYMBOL(LIBC, strncpy);
EXPORT_GROUP_SYMBOL(LIBC, strlen);
EXPORT_GROUP_SYMBOL(LIBC, strcmp);
EXPORT_GROUP_SYMBOL(LIBC, strncmp);
EXPORT_GROUP_SYMBOL(LIBC, memcmp);
EXPORT_GROUP_SYMBOL(LIBC, memcpy);
EXPORT_GROUP_SYMBOL(LIBC, memset);

#include <zephyr/syscall_exports_llext.c>
