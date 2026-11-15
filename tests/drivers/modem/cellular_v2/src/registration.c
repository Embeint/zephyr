/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MODEM_CELLULAR_V2 tracks the registration reported on its AT command channel, and forgets
 * it once the LTE RF is disabled or the modem powered down, which nothing reports.
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

K_MSGQ_DEFINE(reg_events, sizeof(enum cellular_registration_status), 16, 4);

static void reg_event_handler(const struct device *dev, enum cellular_event event,
			      const void *payload, void *user_data)
{
	const struct cellular_evt_registration_status *evt = payload;

	ARG_UNUSED(dev);
	ARG_UNUSED(event);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&reg_events, &evt->status, K_NO_WAIT);
}

static void expect_reg_event(enum cellular_registration_status expected)
{
	enum cellular_registration_status status;

	zassert_ok(k_msgq_get(&reg_events, &status, K_SECONDS(2)), "no event for status %d",
		   expected);
	zassert_equal(expected, status, "status %d reported, expected %d", status, expected);
}

static void expect_no_reg_event(void)
{
	enum cellular_registration_status status;

	flush_driver_events();
	zassert_not_equal(0, k_msgq_get(&reg_events, &status, K_NO_WAIT),
			  "unexpected event for status %d", status);
}

static void expect_status(enum cellular_access_technology tech,
			  enum cellular_registration_status expected)
{
	enum cellular_registration_status status;

	zassert_ok(cellular_get_registration_status(modem, tech, &status));
	zassert_equal(expected, status, "status %d, expected %d", status, expected);
}

/* Report @p report on the AT command channel, and wait for the driver to read it */
static void report(const char *report)
{
	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, report);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	flush_driver_events();
}

/* Boot until ready with the LTE RF enabled */
static void boot_with_rf(void)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	zassert_ok(modem_cellular_lte_rf_request(modem));
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_no_reg_event();
}

/* Static storage must not make an access technology look known before a long-form report. */
ZTEST(modem_cellular_v2_registration, test_access_technology_initially_unknown)
{
	const struct modem_cellular_data *data;

	modem_restart(modem);
	data = modem->data;
	zassert_equal(CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN, data->access_tech);
}

/* Both the read command answer and the notification are taken, reported on change only */
ZTEST(modem_cellular_v2_registration, test_reported_on_change)
{
	boot_with_rf();
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_NOT_REGISTERED);

	/* Read command answer, with <n> first */
	report("+CEREG: 1,2\r");
	expect_reg_event(CELLULAR_REGISTRATION_SEARCHING);

	/* Notification, with the area and access technology */
	report("+CEREG: 1,\"00C3\",\"0000ABCD\",7\r");
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_HOME);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_REGISTERED_HOME);

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_no_reg_event();

	report("+CEREG: 5\r");
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_ROAMING);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN,
		      CELLULAR_REGISTRATION_REGISTERED_ROAMING);
}

/* Each report is stored against its access technology */
ZTEST(modem_cellular_v2_registration, test_access_technologies)
{
	boot_with_rf();

	report("+CREG: 1\r");
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_HOME);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_GSM, CELLULAR_REGISTRATION_REGISTERED_HOME);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_NOT_REGISTERED);

	report("+CGREG: 5\r");
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_ROAMING);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_UTRAN, CELLULAR_REGISTRATION_REGISTERED_ROAMING);
}

/* Disabling the LTE RF loses the registration, without the modem reporting it */
ZTEST(modem_cellular_v2_registration, test_forgotten_on_rf_disable)
{
	boot_with_rf();
	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_HOME);

	zassert_ok(modem_cellular_lte_rf_release(modem));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_reg_event(CELLULAR_REGISTRATION_NOT_REGISTERED);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_NOT_REGISTERED);
}

/* Powering the modem down loses the registration */
ZTEST(modem_cellular_v2_registration, test_forgotten_on_power_down)
{
	boot_with_rf();
	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_HOME);

	suspend_with_lte_rf_and_shutdown(modem);
	expect_reg_event(CELLULAR_REGISTRATION_NOT_REGISTERED);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_NOT_REGISTERED);
}

/* A detach ends the packet domain registration, leaving the circuit-switched one */
ZTEST(modem_cellular_v2_registration, test_forgotten_on_detach)
{
	boot_with_rf();
	report("+CREG: 1\r");
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_HOME);
	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_reg_event(CELLULAR_REGISTRATION_REGISTERED_HOME);

	report("+CGEV: NW DETACH\r");
	expect_reg_event(CELLULAR_REGISTRATION_NOT_REGISTERED);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_NOT_REGISTERED);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_GSM, CELLULAR_REGISTRATION_REGISTERED_HOME);

	/* Nothing is reported when no packet domain registration was held */
	report("+CGEV: ME DETACH\r");
	expect_no_reg_event();
}

/* A detach leaves a status the modem reported as not registered, e.g. a denied attach */
ZTEST(modem_cellular_v2_registration, test_denied_kept_on_detach)
{
	boot_with_rf();
	report("+CEREG: 3,\"00C3\",\"0000ABCD\",7,0,19\r");
	expect_reg_event(CELLULAR_REGISTRATION_DENIED);

	report("+CGEV: NW DETACH\r");
	expect_no_reg_event();
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_DENIED);

	/* The next denied attempt is no change */
	report("+CEREG: 3,\"00C3\",\"0000ABCD\",7,0,19\r");
	expect_no_reg_event();
}

/* An EMM cause may follow <stat> without the location, which is then left empty */
ZTEST(modem_cellular_v2_registration, test_cause_without_location)
{
	const struct modem_cellular_data *data = modem->data;

	boot_with_rf();
	report("+CEREG: 3,,,,0,19\r");
	expect_reg_event(CELLULAR_REGISTRATION_DENIED);
	expect_status(CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN, CELLULAR_REGISTRATION_DENIED);
	zassert_equal(CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN, data->access_tech,
		      "empty access technology taken as %d", data->access_tech);
}

/* Nothing is reported when no registration was held */
ZTEST(modem_cellular_v2_registration, test_unregistered_power_down)
{
	boot_with_rf();
	suspend_with_lte_rf_and_shutdown(modem);
	expect_no_reg_event();
}

K_MSGQ_DEFINE(context_events, sizeof(bool), 8, 4);

static void context_event_handler(const struct device *dev, enum cellular_event event,
				  const void *payload, void *user_data)
{
	const struct cellular_evt_data_context *evt = payload;

	ARG_UNUSED(dev);
	ARG_UNUSED(event);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&context_events, &evt->active, K_NO_WAIT);
}

static void expect_context_event(bool active)
{
	bool reported;

	zassert_ok(k_msgq_get(&context_events, &reported, K_SECONDS(2)),
		   "no data context event");
	zassert_equal(active, reported, "data context reported %s",
		      reported ? "active" : "inactive");
}

/*
 * Reports for the data context of the data call are passed on, those for other contexts
 * dropped. Losing registration leaves the context to +CGEV, which reports a detach. A context
 * still active is forgotten when the modem powers down.
 */
ZTEST(modem_cellular_v2_registration, test_data_context_reported)
{
	bool reported;

	k_msgq_purge(&context_events);
	zassert_ok(cellular_set_callback(modem, CELLULAR_EVENT_DATA_CONTEXT_CHANGED,
					 context_event_handler, NULL));
	boot_with_rf();

	report("+CGEV: ME PDN ACT 1\r");
	expect_context_event(true);
	report("+CGEV: NW PDN DEACT 1\r");
	expect_context_event(false);
	report("+CGEV: NW PDN DEACT 3\r");
	zassert_not_equal(0, k_msgq_get(&context_events, &reported, K_NO_WAIT),
			  "other context reported");

	report("+CGEV: ME PDN ACT 1\r");
	expect_context_event(true);
	report("+CEREG: 1\r");
	report("+CEREG: 0\r");
	zassert_not_equal(0, k_msgq_get(&context_events, &reported, K_MSEC(100)),
			  "context lost with the registration");
	report("+CGEV: NW DETACH\r");
	expect_context_event(false);

	report("+CGEV: ME PDN ACT 1\r");
	expect_context_event(true);
	suspend_with_lte_rf_and_shutdown(modem);
	expect_context_event(false);
}

static void registration_before(void *fixture)
{
	modem_test_before_with_apn(fixture);
	k_msgq_purge(&reg_events);
	zassert_ok(cellular_set_callback(modem, CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED,
					 reg_event_handler, NULL));
}

static void registration_after(void *fixture)
{
	(void)cellular_set_callback(modem, 0, NULL, NULL);
	modem_test_after(fixture);
}

ZTEST_SUITE(modem_cellular_v2_registration, NULL, modem_test_setup, registration_before,
	    registration_after, NULL);
