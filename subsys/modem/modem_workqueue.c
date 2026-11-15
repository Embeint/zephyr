/*
 * Copyright (c) 2025 Embeint Pty Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel/dedicated_workq.h>

#include "modem_workqueue.h"

K_DEDICATED_WORK_QUEUE_DEFINE(modem, MODEM);

k_tid_t modem_work_queue_thread_get(void)
{
	return k_work_queue_thread_get(&modem_work_q);
}
