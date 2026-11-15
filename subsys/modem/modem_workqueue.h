/** @file
 * @brief Modem workqueue header file.
 */

/*
 * Copyright (c) 2025 Embeint Pty Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_MODEM_WORKQUEUE_H_
#define ZEPHYR_INCLUDE_MODEM_WORKQUEUE_H_

#include <zephyr/kernel.h>
#include <zephyr/kernel/dedicated_workq.h>

#ifdef __cplusplus
extern "C" {
#endif

K_DEDICATED_WORK_QUEUE_DECLARE(modem, MODEM);

#ifdef CONFIG_MODEM_DEDICATED_WORKQUEUE

/* Thread of the workqueue the modem modules submit their work to */
k_tid_t modem_work_queue_thread_get(void);

#else

static inline k_tid_t modem_work_queue_thread_get(void)
{
	return k_work_queue_thread_get(&k_sys_work_q);
}

#endif /* CONFIG_MODEM_DEDICATED_WORKQUEUE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MODEM_WORKQUEUE_H_ */
