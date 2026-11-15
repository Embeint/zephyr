/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MODEM_CELLULAR_V2 leaves the LTE RF disabled once ready, as the init script does, and
 * enables it while at least one request for it is held.
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <modem_cellular_test.h>
#include <zephyr/ztest.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_MODEM_CELLULAR_V2));

/* Without a periodic script, so that only the commands under test are sent */
static const struct device *const modem = DEVICE_DT_GET(DT_NODELABEL(modem_no_periodic));

static void boot_to_ready(void)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
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

static void enable_rf(void)
{
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_apn_configured();
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
}

ZTEST(modem_cellular_v2_lte_rf, test_off_until_requested)
{
	boot_to_ready();
	expect_no_line(K_MSEC(100));

	enable_rf();
	expect_no_line(K_MSEC(100));
}

/* The LTE RF stays enabled until the last request is released */
ZTEST(modem_cellular_v2_lte_rf, test_requests_counted)
{
	boot_to_ready();
	enable_rf();
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_no_line(K_MSEC(100));

	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_no_line(K_MSEC(100));
	zassert_ok(modem_cellular_lte_rf_release(modem));
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, true);

	zassert_equal(-EALREADY, modem_cellular_lte_rf_release(modem));
	expect_no_line(K_MSEC(100));
}

/* A request held while the modem boots is acted on once it is ready */
ZTEST(modem_cellular_v2_lte_rf, test_requested_before_ready)
{
	modem_restart(modem);
	zassert_ok(modem_cellular_lte_rf_request(modem));

	advance_to(modem, BOOT_AT_READY);
	expect_apn_configured();
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
}

/* Requests outlive the modem being powered down, and are applied again once ready */
ZTEST(modem_cellular_v2_lte_rf, test_reapplied_after_power_cycle)
{
	boot_to_ready();
	enable_rf();
	suspend_with_lte_rf_and_shutdown(modem);
	expect_no_line(K_MSEC(100));

	zassert_ok(modem_cellular_pm_action(modem, PM_DEVICE_ACTION_RESUME), "resume failed");
	advance_to(modem, BOOT_AT_READY);
	expect_apn_configured();
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
}

ZTEST(modem_cellular_v2_lte_rf, test_enable_retried)
{
	int64_t start;

	boot_to_ready();
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);

	start = k_uptime_get();
	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS / 2));
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS,
		     "retried before the LTE RF retry interval");
}

/* A failed enable is not retried once the request is released, the LTE RF being off */
ZTEST(modem_cellular_v2_lte_rf, test_enable_failure_released)
{
	boot_to_ready();
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_apn_configured();
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, false);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS * 2));
}

ZTEST(modem_cellular_v2_lte_rf, test_disable_retried)
{
	int64_t start;

	boot_to_ready();
	enable_rf();
	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);

	start = k_uptime_get();
	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS / 2));
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, true);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS,
		     "retried before the LTE RF retry interval");
}

/* Releasing while the enable script runs disables the LTE RF again once it completes */
ZTEST(modem_cellular_v2_lte_rf, test_released_during_enable)
{
	boot_to_ready();
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	ok(TEST_BACKEND_PIPE_DLCI1);
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

/* Requesting while the disable script runs enables the LTE RF again once it completes */
ZTEST(modem_cellular_v2_lte_rf, test_requested_during_disable)
{
	boot_to_ready();
	enable_rf();
	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);

	zassert_ok(modem_cellular_lte_rf_request(modem));
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_apn_configured();
	answer(MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD, true);
	expect_no_line(K_MSEC(100));
}

/* A suspend waits for the LTE RF disable, so that the modem detaches before powering down */
ZTEST(modem_cellular_v2_lte_rf, test_suspend_waits_for_disable)
{
	boot_to_ready();
	enable_rf();
	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);

	start_suspend(modem);
	expect_no_line(K_MSEC(100));
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	finish_suspend(modem);
}

/* Losing the AT command channel while a suspend waits for the LTE RF disable powers it down */
ZTEST(modem_cellular_v2_lte_rf, test_suspend_during_disable_channel_lost)
{
	boot_to_ready();
	enable_rf();
	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);

	start_suspend(modem);
	expect_no_line(K_MSEC(100));
	test_backend_pipe_disconnect(TEST_BACKEND_PIPE_DLCI1);
	finish_suspend(modem);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MS * 2));
}

/* A suspend while waiting to retry a failed LTE RF disable tries it once more first */
ZTEST(modem_cellular_v2_lte_rf, test_suspend_during_disable_retry)
{
	boot_to_ready();
	enable_rf();
	zassert_ok(modem_cellular_lte_rf_release(modem));
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, false);
	flush_driver_events();

	/* The disable is tried once more, and the modem powered down whatever it answers */
	start_suspend(modem);
	answer(MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD, false);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	finish_suspend(modem);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));
}

/*
 * Requests and releases faster than the driver handles them are signalled rather than
 * queued, so a burst of them cannot crowd out a suspend that follows.
 */
ZTEST(modem_cellular_v2_lte_rf, test_request_burst_then_suspend)
{
	boot_to_ready();

	for (int i = 0; i < 32; i++) {
		zassert_ok(modem_cellular_lte_rf_request(modem));
		zassert_ok(modem_cellular_lte_rf_release(modem));
	}
	suspend_and_shutdown(modem);
}

ZTEST_SUITE(modem_cellular_v2_lte_rf, NULL, modem_test_setup, modem_test_before_with_apn,
	    modem_test_after, NULL);
