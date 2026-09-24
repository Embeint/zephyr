/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/modem/3gpp.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

enum cellular_access_technology modem_3gpp_edrx_act_decode(enum modem_3gpp_edrx_act_type act_type)
{
	switch (act_type) {
	case MODEM_3GPP_EDRX_ACT_TYPE_WB_S1:
		return CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN;
	case MODEM_3GPP_EDRX_ACT_TYPE_NB_S1:
		return CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN_NB_S1;
	default:
		return CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN;
	}
}

static int parse_timer_bits(const char *str, uint8_t *value)
{
	const char *bits;
	size_t len;

	if ((str == NULL) || (value == NULL)) {
		return -EINVAL;
	}

	bits = str;
	len = strlen(str);

	/* String-valued AT command parameters are normally quoted, but accept an
	 * unquoted value as well to accommodate modem-specific formatting.
	 */
	if ((len == 6) && (str[0] == '"') && (str[5] == '"')) {
		bits++;
	} else if (len != 4) {
		return -EINVAL;
	}

	*value = 0;
	for (uint8_t i = 0; i < 4; i++) {
		if ((bits[i] != '0') && (bits[i] != '1')) {
			return -EINVAL;
		}

		*value = (*value << 1) | (bits[i] - '0');
	}

	return 0;
}

int modem_3gpp_edrx_decode(enum modem_3gpp_edrx_act_type act_type, const char *edrx,
			   const char *ptw, float *edrx_seconds, float *ptw_seconds)
{
	/* Multipliers for the S1 mode T_eDRX values from 3GPP TS 24.008 10.5.5.32.
	 * The resulting interval is multiplier * 10.24 seconds, except for multiplier zero.
	 * Values that are not applicable to a mode are mapped as required by the specification.
	 */
	static const uint16_t edrx_multiplier_wb_s1[16] = {
		0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 32, 64, 128, 256, 256, 256,
	};
	static const uint16_t edrx_multiplier_nb_s1[16] = {
		2, 2, 2, 4, 2, 8, 2, 2, 2, 16, 32, 64, 128, 256, 512, 1024,
	};
	uint16_t edrx_multiplier;
	float ptw_multiplier;
	uint8_t edrx_index;
	uint8_t ptw_index;
	int ret;

	if ((edrx_seconds == NULL) || (ptw_seconds == NULL)) {
		return -EINVAL;
	}

	ret = parse_timer_bits(edrx, &edrx_index);
	if (ret < 0) {
		return ret;
	}

	ret = parse_timer_bits(ptw, &ptw_index);
	if (ret < 0) {
		return ret;
	}

	switch (act_type) {
	case MODEM_3GPP_EDRX_ACT_TYPE_WB_S1:
		edrx_multiplier = edrx_multiplier_wb_s1[edrx_index];
		ptw_multiplier = 1.28f;
		break;
	case MODEM_3GPP_EDRX_ACT_TYPE_NB_S1:
		edrx_multiplier = edrx_multiplier_nb_s1[edrx_index];
		ptw_multiplier = 2.56f;
		break;
	default:
		return -ENOTSUP;
	}

	*edrx_seconds = (edrx_multiplier == 0) ? 5.12f : edrx_multiplier * 10.24f;
	*ptw_seconds = (ptw_index + 1) * ptw_multiplier;

	return 0;
}
