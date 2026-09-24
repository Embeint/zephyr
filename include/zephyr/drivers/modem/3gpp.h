/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_3GPP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_3GPP_H_

#include <zephyr/drivers/cellular.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Access technology values used by the +CEDRXP command.
 *
 * See 3GPP TS 27.007, clause 7.41.
 */
enum modem_3gpp_edrx_act_type {
	MODEM_3GPP_EDRX_ACT_DISABLED = 0,
	MODEM_3GPP_EDRX_ACT_TYPE_WB_S1 = 4,
	MODEM_3GPP_EDRX_ACT_TYPE_NB_S1 = 5,
};

/**
 * @brief Decode the access technology reported by +CEDRXP
 *
 * @param act_type Access technology associated with the timer values.
 * @return int
 */
enum cellular_access_technology modem_3gpp_edrx_act_decode(enum modem_3gpp_edrx_act_type act_type);

/**
 * @brief Decode network-provided eDRX timers.
 *
 * Converts the four-bit eDRX cycle and paging time window strings specified by
 * 3GPP TS 24.008, clause 10.5.5.32, into seconds. Both quoted AT-command strings
 * and unquoted four-bit strings are accepted.
 *
 * @param act_type Access technology associated with the timer values.
 * @param edrx Network-provided eDRX cycle string.
 * @param ptw Network-provided paging time window string.
 * @param edrx_seconds Decoded eDRX cycle in seconds.
 * @param ptw_seconds Decoded paging time window in seconds.
 *
 * @retval 0 on success.
 * @retval -EINVAL if an argument or encoded timer is invalid.
 * @retval -ENOTSUP if @p act_type is not supported.
 */
int modem_3gpp_edrx_decode(enum modem_3gpp_edrx_act_type act_type, const char *edrx,
			   const char *ptw, float *edrx_seconds, float *ptw_seconds);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_3GPP_H_ */
