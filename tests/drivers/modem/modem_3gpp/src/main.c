/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/modem/3gpp.h>
#include <zephyr/ztest.h>

#include <errno.h>
#include <stdint.h>

static uint32_t hundredths(float value)
{
	return (uint32_t)(value * 100.0f + 0.5f);
}

ZTEST(modem_3gpp, test_edrx_decode_act)
{
	zassert_equal(CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN,
		      modem_3gpp_edrx_act_decode(MODEM_3GPP_EDRX_ACT_DISABLED));
	zassert_equal(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN,
		      modem_3gpp_edrx_act_decode(MODEM_3GPP_EDRX_ACT_TYPE_WB_S1));
	zassert_equal(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN_NB_S1,
		      modem_3gpp_edrx_act_decode(MODEM_3GPP_EDRX_ACT_TYPE_NB_S1));
	zassert_equal(CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN, modem_3gpp_edrx_act_decode(255));
}

ZTEST(modem_3gpp, test_edrx_decode_wb_s1)
{
	float edrx;
	float ptw;

	zassert_ok(modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_WB_S1, "\"0000\"", "\"1111\"",
					  &edrx, &ptw));
	zassert_equal(hundredths(edrx), 512);
	zassert_equal(hundredths(ptw), 2048);

	/* WB-S1 interprets the NB-S1-only value 1111 as 1101. */
	zassert_ok(modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_WB_S1, "1111", "0000", &edrx,
					  &ptw));
	zassert_equal(hundredths(edrx), 262144);
	zassert_equal(hundredths(ptw), 128);
}

ZTEST(modem_3gpp, test_edrx_decode_nb_s1)
{
	float edrx;
	float ptw;

	/* NB-S1 interprets the WB-S1-only value 0000 as 0010. */
	zassert_ok(modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_NB_S1, "0000", "1111", &edrx,
					  &ptw));
	zassert_equal(hundredths(edrx), 2048);
	zassert_equal(hundredths(ptw), 4096);

	zassert_ok(modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_NB_S1, "1111", "0000", &edrx,
					  &ptw));
	zassert_equal(hundredths(edrx), 1048576);
	zassert_equal(hundredths(ptw), 256);
}

ZTEST(modem_3gpp, test_edrx_decode_invalid)
{
	float edrx;
	float ptw;

	zassert_equal(
		modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_WB_S1, "001", "0000", &edrx, &ptw),
		-EINVAL);
	zassert_equal(
		modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_WB_S1, "0020", "0000", &edrx, &ptw),
		-EINVAL);
	zassert_equal(modem_3gpp_edrx_decode(0, "0000", "0000", &edrx, &ptw), -ENOTSUP);
	zassert_equal(
		modem_3gpp_edrx_decode(MODEM_3GPP_EDRX_ACT_TYPE_WB_S1, "0000", "0000", NULL, &ptw),
		-EINVAL);
}

ZTEST_SUITE(modem_3gpp, NULL, NULL, NULL, NULL, NULL);
