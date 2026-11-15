/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MODEM_CELLULAR_V2 configures the data context with the APN before each LTE RF enable, and
 * rejects LTE RF requests while no APN is available.
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <modem_cellular_test.h>
#include <zephyr/ztest.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_MODEM_CELLULAR_V2));

#define OTHER_APN     "other"
#define OTHER_APN_CMD "AT+CGDCONT=1,\"IP\",\"" OTHER_APN "\""

/* Without a periodic script, so that only the commands under test are sent */
static const struct device *const modem = DEVICE_DT_GET(DT_NODELABEL(modem_no_periodic));
static const struct device *const modem_default_pdp =
	DEVICE_DT_GET(DT_NODELABEL(modem_default_pdp));

static void boot_to_ready(const struct device *dev)
{
	modem_restart(dev);
	advance_to(dev, BOOT_AT_READY);
}

/* Answer @p cmd with OK, or ERROR unless @p success, and check the driver read it */
static void answer(const char *cmd, bool success)
{
	expect_line(TEST_BACKEND_PIPE_DLCI1, cmd);
	if (success) {
		ok(TEST_BACKEND_PIPE_DLCI1);
	} else {
		respond_error(TEST_BACKEND_PIPE_DLCI1);
	}
	expect_read(TEST_BACKEND_PIPE_DLCI1);
}

ZTEST(modem_cellular_v2_apn, test_request_rejected_without_apn)
{
	boot_to_ready(modem);

	zassert_equal(-ENODATA, modem_cellular_lte_rf_request(modem));
	expect_no_line(K_MSEC(100));
	/* A rejected request is not held */
	zassert_equal(-EALREADY, modem_cellular_lte_rf_release(modem));
}

/* An APN set once a request was rejected lets the next one through */
ZTEST(modem_cellular_v2_apn, test_request_after_apn_set)
{
	boot_to_ready(modem);
	zassert_equal(-ENODATA, modem_cellular_lte_rf_request(modem));

	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(TEST_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

/* An empty APN counts as set, and is configured for the network to select the default APN */
ZTEST(modem_cellular_v2_apn, test_empty_apn)
{
	boot_to_ready(modem);
	zassert_equal(-ENODATA, modem_cellular_lte_rf_request(modem));

	zassert_ok(cellular_set_apn(modem, ""));
	zassert_equal(-EALREADY, cellular_set_apn(modem, ""));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer("AT+CGDCONT=1,\"IP\",\"\"", true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

ZTEST(modem_cellular_v2_apn, test_set_apn_validated)
{
	char too_long[MODEM_CELLULAR_DATA_APN_LEN + 1];

	memset(too_long, 'a', sizeof(too_long) - 1);
	too_long[sizeof(too_long) - 1] = '\0';

	zassert_equal(-EINVAL, cellular_set_apn(modem, NULL));
	zassert_equal(-EINVAL, cellular_set_apn(modem, too_long));
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_equal(-EALREADY, cellular_set_apn(modem, TEST_APN));
}

/* The APN cannot change under a held request, and the new one is used on the next enable */
ZTEST(modem_cellular_v2_apn, test_change_while_requested)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(TEST_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);

	zassert_equal(-EBUSY, cellular_set_apn(modem, OTHER_APN));

	zassert_ok(modem_cellular_lte_rf_release(modem));
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, true);
	zassert_ok(cellular_set_apn(modem, OTHER_APN));

	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(OTHER_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
}

/* Release, set the APN and request again before the driver gets to run */
static void change_apn_at_once(void)
{
	zassert_ok(modem_cellular_lte_rf_release(modem));
	zassert_ok(cellular_set_apn(modem, OTHER_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
}

/* An APN changed between a release and the next request is configured, however quickly the
 * request follows
 */
ZTEST(modem_cellular_v2_apn, test_change_between_release_and_request)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(TEST_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);

	change_apn_at_once();
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, true);
	answer(OTHER_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

/* As test_change_between_release_and_request, while the old APN is being configured */
ZTEST(modem_cellular_v2_apn, test_change_during_configuration)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_APN_CMD);

	change_apn_at_once();
	ok(TEST_BACKEND_PIPE_DLCI1);
	answer(OTHER_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

/* As test_change_between_release_and_request, while the LTE RF is being enabled */
ZTEST(modem_cellular_v2_apn, test_change_during_enable)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(TEST_APN_CMD, true);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);

	change_apn_at_once();
	ok(TEST_BACKEND_PIPE_DLCI1);
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, true);
	answer(OTHER_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

/* A failed configuration is retried after the LTE RF retry interval, the LTE RF staying off */
ZTEST(modem_cellular_v2_apn, test_failure_retried)
{
	int64_t start;

	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_APN_CMD);

	start = k_uptime_get();
	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS / 2));
	answer(TEST_APN_CMD, true);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS,
		     "retried before the LTE RF retry interval");
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
}

ZTEST(modem_cellular_v2_apn, test_failure_released)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(TEST_APN_CMD, false);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS * 2));
}

/* Releasing while the APN is configured leaves the LTE RF off */
ZTEST(modem_cellular_v2_apn, test_released_during_configuration)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_APN_CMD);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(100));
}

/* The APN is configured again on each enable, as the modem may have lost it */
ZTEST(modem_cellular_v2_apn, test_configured_on_each_enable)
{
	boot_to_ready(modem);
	zassert_ok(cellular_set_apn(modem, TEST_APN));
	zassert_ok(modem_cellular_lte_rf_request(modem));
	answer(TEST_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	suspend_with_lte_rf_and_shutdown(modem);

	zassert_ok(modem_cellular_pm_action(modem, PM_DEVICE_ACTION_RESUME), "resume failed");
	advance_to(modem, BOOT_AT_READY);
	answer(TEST_APN_CMD, true);
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
}

/* The default PDP context is left to the modem, so no APN is needed or configured */
ZTEST(modem_cellular_v2_apn, test_default_pdp_context)
{
	boot_to_ready(modem_default_pdp);

	zassert_ok(modem_cellular_lte_rf_request(modem_default_pdp));
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

ZTEST_SUITE(modem_cellular_v2_apn, NULL, modem_test_setup, modem_test_before, modem_test_after,
	    NULL);
