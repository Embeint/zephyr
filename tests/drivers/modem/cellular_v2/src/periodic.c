/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MODEM_CELLULAR_V2 runs the vendor periodic script every periodic script interval while it is
 * ready for commands, whether the LTE RF is enabled or not, reports each run as a
 * communications check, and recovers the modem once enough consecutive runs fail.
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <modem_cellular_test.h>
#include <zephyr/ztest.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_MODEM_CELLULAR_V2));

static const struct device *const modem = DEVICE_DT_GET(DT_ALIAS(modem));
static const struct device *const modem_no_periodic =
	DEVICE_DT_GET(DT_NODELABEL(modem_no_periodic));
static const struct device *const modem_conditional =
	DEVICE_DT_GET(DT_NODELABEL(modem_conditional));

K_MSGQ_DEFINE(comms_checks, sizeof(bool), 16, 4);

static void comms_check_handler(const struct device *dev, enum cellular_event event,
				const void *payload, void *user_data)
{
	const struct cellular_evt_modem_comms_check_result *result = payload;

	ARG_UNUSED(dev);
	ARG_UNUSED(event);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&comms_checks, &result->success, K_NO_WAIT);
}

static void expect_comms_check(bool success)
{
	bool reported;

	zassert_ok(k_msgq_get(&comms_checks, &reported, K_SECONDS(2)), "no comms check result");
	zassert_equal(success, reported, "comms check reported success=%d", reported);
}

static void boot_to_ready(const struct device *dev)
{
	modem_restart(dev);
	zassert_ok(cellular_set_callback(dev, CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT,
					 comms_check_handler, NULL));
	advance_to(dev, BOOT_AT_READY);
}

static void enable_rf(const struct device *dev)
{
	zassert_ok(modem_cellular_lte_rf_request(dev));
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
}

/* Answer the next periodic script run, and check it is reported */
static void answer_periodic(bool success)
{
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_PERIODIC_CMD);
	if (success) {
		ok(TEST_BACKEND_PIPE_DLCI1);
	} else {
		respond_error(TEST_BACKEND_PIPE_DLCI1);
	}
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_comms_check(success);
}

/* A run every periodic script interval once ready, with the LTE RF still disabled */
ZTEST(modem_cellular_v2_periodic, test_runs_when_ready)
{
	int64_t start;

	start = k_uptime_get();
	boot_to_ready(modem);
	answer_periodic(true);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS,
		     "ran before the periodic script interval");
	answer_periodic(true);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS,
		     "ran again before the periodic script interval");
}

/* Enabling and disabling the LTE RF leaves the script running */
ZTEST(modem_cellular_v2_periodic, test_runs_across_rf_changes)
{
	boot_to_ready(modem);
	enable_rf(modem);
	answer_periodic(true);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	answer_periodic(true);
}

/* A run due while an LTE RF script runs is made once it completes */
ZTEST(modem_cellular_v2_periodic, test_held_during_rf_script)
{
	boot_to_ready(modem);
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 3 / 2));

	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	answer_periodic(true);
}

/* Consecutive failures up to the limit recover the modem, which is likely still running */
ZTEST(modem_cellular_v2_periodic, test_failures_recover)
{
	boot_to_ready(modem);
	enable_rf(modem);

	for (int i = 0; i < CONFIG_MODEM_CELLULAR_MAX_SCRIPT_FAILURES; i++) {
		answer_periodic(false);
	}
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
}

/* A success between failures starts the count again */
ZTEST(modem_cellular_v2_periodic, test_success_resets_failures)
{
	boot_to_ready(modem);
	enable_rf(modem);

	for (int round = 0; round < 2; round++) {
		for (int i = 0; i < CONFIG_MODEM_CELLULAR_MAX_SCRIPT_FAILURES - 1; i++) {
			answer_periodic(false);
		}
		answer_periodic(true);
	}
	answer_periodic(true);
}

/* A run due while paused happens as soon as the script is resumed */
ZTEST(modem_cellular_v2_periodic, test_pause_resume)
{
	int64_t start;

	boot_to_ready(modem);
	enable_rf(modem);
	zassert_ok(cellular_modem_pause_periodic_script(modem));
	zassert_equal(-EINVAL, cellular_modem_pause_periodic_script(modem));
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));

	start = k_uptime_get();
	zassert_ok(cellular_modem_resume_periodic_script(modem));
	zassert_equal(-EINVAL, cellular_modem_resume_periodic_script(modem));
	answer_periodic(true);
	zassert_true(k_uptime_delta(&start) < CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS,
		     "skipped run not made on resume");
}

/* Resuming before a run was due restarts the interval instead */
ZTEST(modem_cellular_v2_periodic, test_resume_before_due)
{
	int64_t start;

	boot_to_ready(modem);
	enable_rf(modem);
	zassert_ok(cellular_modem_pause_periodic_script(modem));
	start = k_uptime_get();
	zassert_ok(cellular_modem_resume_periodic_script(modem));

	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS / 2));
	answer_periodic(true);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS,
		     "ran before the periodic script interval");
}

ZTEST(modem_cellular_v2_periodic, test_no_periodic_script)
{
	zassert_equal(-ENOTSUP, cellular_modem_pause_periodic_script(modem_no_periodic));
	zassert_equal(-ENOTSUP, cellular_modem_resume_periodic_script(modem_no_periodic));

	boot_to_ready(modem_no_periodic);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));
}

/* The serving cell the periodic script reports is the network status */
ZTEST(modem_cellular_v2_periodic, test_network_status)
{
	struct cellular_evt_network_status status;

	boot_to_ready(modem);
	enable_rf(modem);
	zassert_equal(-ENODATA, cellular_get_network_status(modem, &status));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_PERIODIC_CMD);
	test_backend_respond(TEST_BACKEND_PIPE_DLCI1,
			     MODEM_CELLULAR_TEST_SERVING_CELL "505,1,1234567,4660,3750,7,42,-95,-11\r");
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_comms_check(true);

	zassert_ok(cellular_get_network_status(modem, &status));
	zassert_equal(CELLULAR_REGISTRATION_REGISTERED_HOME, status.status);
	zassert_equal(505, status.cell.lte.mcc);
	zassert_equal(1234567, status.cell.lte.gci);
	zassert_equal(-95, status.cell.lte.rsrp);
}

/* Releasing the LTE RF while the periodic script runs waits for it */
ZTEST(modem_cellular_v2_periodic, test_rf_release_waits_for_script)
{
	boot_to_ready(modem);
	enable_rf(modem);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_PERIODIC_CMD);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_no_line(K_MSEC(100));
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_comms_check(true);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
}

/*
 * A release and the suspend that follows it while the periodic script runs wait for it, and
 * the LTE RF is disabled before the modem is powered down
 */
ZTEST(modem_cellular_v2_periodic, test_suspend_waits_for_script)
{
	boot_to_ready(modem);
	enable_rf(modem);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_PERIODIC_CMD);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	start_suspend(modem);
	expect_no_line(K_MSEC(100));
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_comms_check(true);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	finish_suspend(modem);
}

/* Answer a run of the conditional periodic script, expecting the commands @p cmds to be sent */
static void answer_conditional_periodic(const char *const *cmds, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		expect_line(TEST_BACKEND_PIPE_DLCI1, cmds[i]);
		ok(TEST_BACKEND_PIPE_DLCI1);
	}
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_comms_check(true);
}

/* Commands conditional on the LTE RF and on registration are sent only while each is true */
ZTEST(modem_cellular_v2_periodic, test_conditional_commands)
{
	static const char *const rf_off[] = {
		MODEM_CELLULAR_TEST_PERIODIC_CMD,
		MODEM_CELLULAR_TEST_PERIODIC_LTE_RF_OFF_CMD,
	};
	static const char *const rf_on[] = {
		MODEM_CELLULAR_TEST_PERIODIC_CMD,
		MODEM_CELLULAR_TEST_PERIODIC_LTE_RF_CMD,
	};
	static const char *const registered_cmds[] = {
		MODEM_CELLULAR_TEST_PERIODIC_CMD,
		MODEM_CELLULAR_TEST_PERIODIC_LTE_RF_CMD,
		MODEM_CELLULAR_TEST_PERIODIC_REGISTERED_CMD,
	};

	boot_to_ready(modem_conditional);
	answer_conditional_periodic(rf_off, ARRAY_SIZE(rf_off));

	enable_rf(modem_conditional);
	answer_conditional_periodic(rf_on, ARRAY_SIZE(rf_on));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	answer_conditional_periodic(registered_cmds, ARRAY_SIZE(registered_cmds));

	/* Both are lost with the LTE RF */
	zassert_ok(modem_cellular_lte_rf_release(modem_conditional));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	answer_conditional_periodic(rf_off, ARRAY_SIZE(rf_off));
}

static void periodic_before(void *fixture)
{
	modem_test_before_with_apn(fixture);
	k_msgq_purge(&comms_checks);
}

static void periodic_after(void *fixture)
{
	(void)cellular_set_callback(modem, 0, NULL, NULL);
	(void)cellular_set_callback(modem_no_periodic, 0, NULL, NULL);
	(void)cellular_set_callback(modem_conditional, 0, NULL, NULL);
	modem_test_after(fixture);
}

ZTEST_SUITE(modem_cellular_v2_periodic, NULL, modem_test_setup, periodic_before, periodic_after,
	    NULL);
