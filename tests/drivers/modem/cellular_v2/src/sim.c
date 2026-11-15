/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MODEM_CELLULAR_V2 queries the SIM status with AT+CPIN? once DLCI1 is set up, until the SIM
 * is ready, reports an error that will not clear, or the SIM timeout passes. The outcome is
 * reported with CELLULAR_EVENT_SIM_STATUS_CHANGED, after the vendor SIM ready script once the
 * SIM is ready, and the modem is brought up whatever it is.
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <modem_cellular_test.h>
#include <zephyr/ztest.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_MODEM_CELLULAR_V2));

/* Without a periodic script, so that only the commands under test are sent */
static const struct device *const modem = DEVICE_DT_GET(DT_NODELABEL(modem_no_periodic));
static const struct device *const modem_no_sim_ready =
	DEVICE_DT_GET(DT_NODELABEL(modem_no_sim_ready));

/* Time the driver waits between SIM status queries */
#define SIM_POLL_INTERVAL_MS 250

K_MSGQ_DEFINE(sim_events, sizeof(enum cellular_sim_status), 8, 4);
static atomic_t modem_failures;

static void event_handler(const struct device *dev, enum cellular_event event,
			  const void *payload, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (event == CELLULAR_EVENT_SIM_STATUS_CHANGED) {
		const struct cellular_evt_sim_status *evt = payload;

		(void)k_msgq_put(&sim_events, &evt->status, K_NO_WAIT);
	} else if (event == CELLULAR_EVENT_MODEM_FAILED) {
		atomic_inc(&modem_failures);
	}
}

static void expect_sim_event(enum cellular_sim_status expected)
{
	enum cellular_sim_status status;

	zassert_ok(k_msgq_get(&sim_events, &status, K_SECONDS(2)), "no event for status %d",
		   expected);
	zassert_equal(expected, status, "status %d reported, expected %d", status, expected);
}

static void expect_no_sim_event(void)
{
	enum cellular_sim_status status;

	flush_driver_events();
	zassert_not_equal(0, k_msgq_get(&sim_events, &status, K_NO_WAIT),
			  "unexpected event for status %d", status);
}

/* The modem is ready for commands, so an LTE RF request configures the APN */
static void expect_modem_ready(const struct device *dev)
{
	expect_no_line(K_MSEC(100));
	zassert_ok(modem_cellular_lte_rf_request(dev));
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
}

static void sim_before(void *fixture)
{
	modem_test_before_with_apn(fixture);
	k_msgq_purge(&sim_events);
	atomic_clear(&modem_failures);
	zassert_ok(cellular_set_callback(modem, CELLULAR_EVENT_SIM_STATUS_CHANGED |
							CELLULAR_EVENT_MODEM_FAILED,
					 event_handler, NULL));
	zassert_ok(cellular_set_callback(modem_no_sim_ready,
					 CELLULAR_EVENT_SIM_STATUS_CHANGED |
						 CELLULAR_EVENT_MODEM_FAILED,
					 event_handler, NULL));
}

static void sim_after(void *fixture)
{
	(void)cellular_set_callback(modem, 0, NULL, NULL);
	(void)cellular_set_callback(modem_no_sim_ready, 0, NULL, NULL);
	modem_test_after(fixture);
}

/* A ready SIM is reported once the vendor script has run, e.g. to read the IMSI */
ZTEST(modem_cellular_v2_sim, test_ready)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM);

	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	expect_no_sim_event();

	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
	expect_modem_ready(modem);
}

/* Some modems quote the status */
ZTEST(modem_cellular_v2_sim, test_ready_quoted)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM);

	respond_cpin("\"READY\"");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
}

/* A failed vendor script still leaves the SIM ready */
ZTEST(modem_cellular_v2_sim, test_sim_ready_script_failure)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM_READY);

	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
	expect_modem_ready(modem);
	zassert_equal(0, atomic_get(&modem_failures), "modem failure reported");
}

/* Without a vendor script a ready SIM is reported at once */
ZTEST(modem_cellular_v2_sim, test_no_sim_ready_script)
{
	modem_restart(modem_no_sim_ready);
	advance_to(modem_no_sim_ready, BOOT_SIM);

	respond_cpin("READY");
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
	expect_modem_ready(modem_no_sim_ready);
}

/*
 * Errors that will not clear by waiting are reported at once, without the vendor script, and
 * the modem is still brought up.
 */
ZTEST(modem_cellular_v2_sim, test_errors_reported)
{
	static const struct {
		const char *response;
		enum cellular_sim_status status;
	} cases[] = {
		{"+CPIN: SIM PIN\rOK\r", CELLULAR_SIM_STATUS_PIN_REQUIRED},
		{"+CPIN: \"SIM PIN\"\rOK\r", CELLULAR_SIM_STATUS_PIN_REQUIRED},
		{"+CPIN: SIM PUK\rOK\r", CELLULAR_SIM_STATUS_PUK_REQUIRED},
		{"+CPIN: PH-SIM PIN\rOK\r", CELLULAR_SIM_STATUS_LOCKED},
		{"+CPIN: PH-NET PIN\rOK\r", CELLULAR_SIM_STATUS_LOCKED},
		{"+CME ERROR: 10\r", CELLULAR_SIM_STATUS_NOT_INSERTED},
		{"+CME ERROR: 11\r", CELLULAR_SIM_STATUS_PIN_REQUIRED},
		{"+CME ERROR: 12\r", CELLULAR_SIM_STATUS_PUK_REQUIRED},
		{"+CME ERROR: 13\r", CELLULAR_SIM_STATUS_FAILURE},
		{"+CME ERROR: 15\r", CELLULAR_SIM_STATUS_FAILURE},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		TC_PRINT("%s\n", cases[i].response);
		modem_restart(modem);
		advance_to(modem, BOOT_SIM);

		test_backend_respond(TEST_BACKEND_PIPE_DLCI1, cases[i].response);
		expect_sim_event(cases[i].status);
		expect_no_line(K_MSEC(SIM_POLL_INTERVAL_MS * 2));
	}

	expect_modem_ready(modem);
	zassert_equal(0, atomic_get(&modem_failures), "modem failure reported");
}

/* Answers that may still change are retried at the poll interval until the SIM is ready */
ZTEST(modem_cellular_v2_sim, test_not_ready_retried)
{
	static const char *const responses[] = {
		"+CPIN: NOT READY\rOK\r",
		"ERROR\r",
		/* SIM busy */
		"+CME ERROR: 14\r",
		/* A status line the driver does not know, then OK */
		"OK\r",
	};
	int64_t start;

	modem_restart(modem);
	advance_to(modem, BOOT_SIM);

	for (size_t i = 0; i < ARRAY_SIZE(responses); i++) {
		start = k_uptime_get();
		test_backend_respond(TEST_BACKEND_PIPE_DLCI1, responses[i]);
		expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CPIN_CMD);
		zassert_true(k_uptime_delta(&start) >= SIM_POLL_INTERVAL_MS,
			     "queried again before the poll interval after %zu", i);
		expect_no_sim_event();
	}

	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
}

/* A SIM that never becomes ready is reported once the SIM timeout passes, without recovery */
ZTEST(modem_cellular_v2_sim, test_timeout)
{
	struct test_backend_line line;
	int64_t start = k_uptime_get();
	int queries = 1;

	modem_restart(modem);
	advance_to(modem, BOOT_SIM);

	while (true) {
		respond_error(TEST_BACKEND_PIPE_DLCI1);
		if (!test_backend_wait_for_line(&line, K_MSEC(SIM_POLL_INTERVAL_MS * 2))) {
			break;
		}
		zassert_equal(TEST_BACKEND_PIPE_DLCI1, line.pipe);
		zassert_str_equal(TEST_CPIN_CMD, line.data);
		queries++;
		zassert_true(queries <= (CONFIG_MODEM_CELLULAR_SIM_TIMEOUT_MS /
					 SIM_POLL_INTERVAL_MS) + 2,
			     "still querying after %d queries", queries);
	}

	expect_sim_event(CELLULAR_SIM_STATUS_TIMEOUT);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_SIM_TIMEOUT_MS,
		     "gave up before the SIM timeout");
	zassert_equal(TEST_BACKEND_STATE_CMUX, test_backend_state(modem), "modem not left up");
	expect_modem_ready(modem);
	zassert_equal(0, atomic_get(&modem_failures), "modem failure reported");
}

/* An LTE RF request made while the SIM is checked is acted on once the modem is ready */
ZTEST(modem_cellular_v2_sim, test_lte_rf_request_held)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM);

	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_no_line(K_MSEC(100));

	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	expect_no_line(K_MSEC(100));

	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
}

/* Suspending while the SIM status is awaited powers the modem down */
ZTEST(modem_cellular_v2_sim, test_suspend_during_query)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM);

	suspend_and_shutdown(modem);
	expect_no_sim_event();
}

/* Suspending between queries stops them */
ZTEST(modem_cellular_v2_sim, test_suspend_between_queries)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM);
	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	flush_driver_events();

	suspend_and_shutdown(modem);
	expect_no_line(K_MSEC(SIM_POLL_INTERVAL_MS * 2));
	expect_no_sim_event();
}

/* Suspending while the vendor script runs powers the modem down */
ZTEST(modem_cellular_v2_sim, test_suspend_during_sim_ready_script)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SIM_READY);

	suspend_and_shutdown(modem);
	expect_no_sim_event();
}

/* The SIM is checked again each time the modem is powered up */
ZTEST(modem_cellular_v2_sim, test_checked_on_each_boot)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	expect_sim_event(CELLULAR_SIM_STATUS_READY);
	suspend_and_shutdown(modem);

	zassert_ok(modem_cellular_pm_action(modem, PM_DEVICE_ACTION_RESUME));
	advance_to(modem, BOOT_SIM);
	respond_cpin("SIM PIN");
	expect_sim_event(CELLULAR_SIM_STATUS_PIN_REQUIRED);
}

ZTEST_SUITE(modem_cellular_v2_sim, NULL, modem_test_setup, sim_before, sim_after, NULL);
