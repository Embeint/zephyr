/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <modem_cellular_test.h>
#include <zephyr/ztest.h>

static const struct device *const modem = DEVICE_DT_GET(DT_ALIAS(modem));
static const struct device *const modem_no_power = DEVICE_DT_GET(DT_NODELABEL(modem_no_power));

struct test_event {
	enum cellular_event event;
};

K_MSGQ_DEFINE(test_events, sizeof(struct test_event), 16, 4);

static void event_handler(const struct device *dev, enum cellular_event event,
			  const void *payload, void *user_data)
{
	struct test_event evt = {.event = event};

	ARG_UNUSED(dev);
	ARG_UNUSED(payload);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&test_events, &evt, K_NO_WAIT);
}

static void listen_for_events(const struct device *dev, cellular_event_mask_t mask)
{
	k_msgq_purge(&test_events);
	zassert_ok(cellular_set_callback(dev, mask, event_handler, NULL));
}

static struct test_event expect_event(enum cellular_event event)
{
	struct test_event evt;

	zassert_ok(k_msgq_get(&test_events, &evt, K_SECONDS(2)), "timed out waiting for event 0x%x",
		   event);
	zassert_equal(event, evt.event, "got event 0x%x, expected 0x%x", evt.event, event);
	return evt;
}

/* A modem that saved a previous baudrate change will not answer at the old one */
ZTEST(modem_cellular_script_errors, test_set_baudrate)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SET_BAUDRATE);

	respond_error(TEST_BACKEND_PIPE_UART);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
}

/* Fail the init script @p count times, checking each failure restarts the modem */
static void fail_init_and_recover(int count)
{
	for (int i = 0; i < count; i++) {
		expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
		respond_error(TEST_BACKEND_PIPE_UART);
		expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
		ok(TEST_BACKEND_PIPE_UART);
	}
}

/* A boot that recovers within the budget carries on to CMUX */
ZTEST(modem_cellular_script_errors, test_init_recovers)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SET_BAUDRATE);
	ok(TEST_BACKEND_PIPE_UART);
	fail_init_and_recover(CONFIG_MODEM_CELLULAR_MAX_RECOVERIES);

	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	expect_state(modem, TEST_BACKEND_STATE_CMUX);
}

ZTEST(modem_cellular_script_errors, test_init_exhausts_recoveries)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SET_BAUDRATE);
	ok(TEST_BACKEND_PIPE_UART);
	listen_for_events(modem, CELLULAR_EVENT_MODEM_SUSPENDED);
	fail_init_and_recover(CONFIG_MODEM_CELLULAR_MAX_RECOVERIES);

	/* Parked in idle rather than resetting forever */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
	respond_error(TEST_BACKEND_PIPE_UART);
	expect_event(CELLULAR_EVENT_MODEM_SUSPENDED);
	expect_state(modem, TEST_BACKEND_STATE_OFF);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MS * 2));

	/* Parking restores the full recovery budget */
	zassert_ok(modem_cellular_pm_action(modem, PM_DEVICE_ACTION_RESUME), "resume failed");
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	fail_init_and_recover(CONFIG_MODEM_CELLULAR_MAX_RECOVERIES);
}

static void script_errors_after(void *fixture)
{
	(void)cellular_set_callback(modem, 0, NULL, NULL);
	(void)cellular_set_callback(modem_no_power, 0, NULL, NULL);
	modem_test_after(fixture);
}

ZTEST_SUITE(modem_cellular_script_errors, NULL, modem_test_setup, modem_test_before,
	    script_errors_after, NULL);
