/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The MODEM_CELLULAR_V2 PPP device dials the data call once the modem is attached while the
 * PPP network interface is up, holding a runtime PM reference and an LTE RF request on the
 * core for as long, and dials it again whenever the call ends. The core under test is only
 * powered while something holds such a reference.
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <modem_cellular_test.h>
#include <zephyr/modem/ppp.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ppp.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/ztest.h>

#include <string.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_MODEM_CELLULAR_V2));
BUILD_ASSERT(IS_ENABLED(CONFIG_NET_MGMT_EVENT_INFO));

static const struct device *const core = DEVICE_DT_GET(DT_NODELABEL(modem_runtime));

static const struct device *ppp_device(void)
{
	const struct device *dev =
		device_get_binding(MODEM_CELLULAR_PPP_DEVICE_NAME(DT_NODELABEL(modem_runtime)));

	zassert_not_null(dev, "no PPP device");
	return dev;
}

static struct net_if *ppp_iface(void)
{
	const struct modem_cellular_ppp_config *config = ppp_device()->config;

	return modem_ppp_get_iface(config->ppp);
}

static bool wait_for_pm_state(const struct device *dev, enum pm_device_state state,
			      k_timeout_t timeout)
{
	k_timepoint_t end = sys_timepoint_calc(timeout);
	enum pm_device_state current;

	do {
		if ((pm_device_state_get(dev, &current) == 0) && (current == state)) {
			return true;
		}
		k_msleep(1);
	} while (!sys_timepoint_expired(end));

	return false;
}

static void expect_pm_state(const struct device *dev, enum pm_device_state state)
{
	enum pm_device_state current = PM_DEVICE_STATE_OFF;

	if (!wait_for_pm_state(dev, state, K_SECONDS(2))) {
		(void)pm_device_state_get(dev, &current);
		zassert_unreachable("%s is %s, expected %s", dev->name,
				    pm_device_state_str(current), pm_device_state_str(state));
	}
}

/* Bring the interface up until dialling, which waits for the modem to register */
static void boot_and_dial(void)
{
	zassert_ok(net_if_up(ppp_iface()));
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_ACTIVE);
	expect_pm_state(core, PM_DEVICE_STATE_ACTIVE);

	advance_to(core, BOOT_AT_READY);
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(100));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_DLCI2, MODEM_CELLULAR_TEST_DIAL_CMD);
}

/* Bring the interface up, answering the boot, the LTE RF enable and the dial */
static void link_up(void)
{
	boot_and_dial();
	ok(TEST_BACKEND_PIPE_DLCI2);
	expect_state(core, TEST_BACKEND_STATE_LINK_UP);
}

/* Answer the LTE RF disable and the core's shutdown once the interface is taken down */
static void expect_powered_down(void)
{
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	expect_state(core, TEST_BACKEND_STATE_OFF);
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
}

/* Take the interface down, answering the LTE RF disable and the core's shutdown */
static void link_down(void)
{
	zassert_ok(net_if_down(ppp_iface()));
	expect_powered_down();
}

/* The vendor dial script runs with the core's data, as the scripts the core runs itself do */
ZTEST(modem_cellular_v2_ppp, test_dial_script_user_data)
{
	const struct modem_cellular_ppp_data *data = ppp_device()->data;

	zassert_equal_ptr(core->data, data->chat.user_data,
			  "dial script run with other data than the core's");
}

/* Nothing holds the core until the interface comes up */
ZTEST(modem_cellular_v2_ppp, test_core_waits_for_interface)
{
	expect_no_line(K_MSEC(200));
	zassert_equal(TEST_BACKEND_STATE_OFF, test_backend_state(core));
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
}

/* The call is dialled on DLCI2 and PPP attached to it, every time the interface comes up */
ZTEST(modem_cellular_v2_ppp, test_link_up_and_down)
{
	link_up();
	link_down();

	link_up();
	link_down();
}

/*
 * Closing LCP from net_if_down() reports the link dead before the peer acknowledges it. The call
 * must stay attached until then, or the acknowledgement is lost and net_if_down() times out.
 */
ZTEST(modem_cellular_v2_ppp, test_down_terminates_lcp)
{
	const struct ppp_context *ctx = net_if_l2_data(ppp_iface());
	k_timepoint_t end = sys_timepoint_calc(K_SECONDS(1));

	link_up();

	/* As the real module does on attaching, which starts LCP */
	net_if_carrier_on(ppp_iface());
	while ((ctx->phase == PPP_DEAD) && !sys_timepoint_expired(end)) {
		k_msleep(1);
	}
	zassert_not_equal(PPP_DEAD, ctx->phase, "LCP not started");

	link_down();
}

/* The LTE RF request is only released once the PPP channel has closed */
ZTEST(modem_cellular_v2_ppp, test_lte_rf_disabled_after_channel_closed)
{
	link_up();

	test_backend_pipe_close_hold(TEST_BACKEND_PIPE_DLCI2);
	zassert_ok(net_if_down(ppp_iface()));
	zassert_true(test_backend_pipe_wait_for_close(TEST_BACKEND_PIPE_DLCI2, K_SECONDS(1)),
		     "PPP channel not closed");
	expect_no_line(K_MSEC(200));

	test_backend_pipe_close_finish(TEST_BACKEND_PIPE_DLCI2);
	expect_powered_down();
}

/* A failed dial is retried after the periodic script interval */
ZTEST(modem_cellular_v2_ppp, test_dial_retried)
{
	int64_t start;

	boot_and_dial();

	start = k_uptime_get();
	respond_error(TEST_BACKEND_PIPE_DLCI2);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS / 2));
	expect_line(TEST_BACKEND_PIPE_DLCI2, MODEM_CELLULAR_TEST_DIAL_CMD);
	zassert_true(k_uptime_delta(&start) >= CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS,
		     "dial retried before the periodic interval");
	ok(TEST_BACKEND_PIPE_DLCI2);
	expect_state(core, TEST_BACKEND_STATE_LINK_UP);

	link_down();
}

/* Taking the interface down mid boot powers the modem down without dialling */
ZTEST(modem_cellular_v2_ppp, test_down_while_booting)
{
	zassert_ok(net_if_up(ppp_iface()));
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);

	zassert_ok(net_if_down(ppp_iface()));
	expect_state(core, TEST_BACKEND_STATE_OFF);
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
	expect_no_line(K_MSEC(100));
}

/* Taking the interface down mid dial closes the PPP channel before the core shuts down */
ZTEST(modem_cellular_v2_ppp, test_down_while_dialling)
{
	boot_and_dial();
	zassert_true(test_backend_pipe_is_open(TEST_BACKEND_PIPE_DLCI2));

	link_down();
	zassert_false(test_backend_pipe_is_open(TEST_BACKEND_PIPE_DLCI2), "DLCI2 left open");
}

/*
 * The PPP device only drops its own reference and LTE RF request, so another user keeps the
 * modem powered with the LTE RF disabled.
 */
ZTEST(modem_cellular_v2_ppp, test_core_shared)
{
	zassert_ok(pm_device_runtime_get(core));
	link_up();

	zassert_ok(net_if_down(ppp_iface()));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED);
	expect_state(core, TEST_BACKEND_STATE_CMUX);
	expect_no_line(K_MSEC(200));
	expect_pm_state(core, PM_DEVICE_STATE_ACTIVE);

	/* The core suspend waits for the shutdown script, which this thread answers */
	zassert_ok(pm_device_runtime_put_async(core, K_NO_WAIT));
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	expect_state(core, TEST_BACKEND_STATE_OFF);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
}

/* The call has ended, so expect it dialled again once the hangup settles, and answer it */
static void expect_redial(void)
{
	expect_state(core, TEST_BACKEND_STATE_CMUX);
	expect_line(TEST_BACKEND_PIPE_DLCI2, MODEM_CELLULAR_TEST_DIAL_CMD);
	ok(TEST_BACKEND_PIPE_DLCI2);
	expect_state(core, TEST_BACKEND_STATE_LINK_UP);
}

/* The modem ending the call, e.g. with NO CARRIER, leaves PPP to report the link dead */
ZTEST(modem_cellular_v2_ppp, test_redialled_after_link_dead)
{
	link_up();

	net_mgmt_event_notify(NET_EVENT_PPP_PHASE_DEAD, ppp_iface());
	expect_redial();
	link_down();
}

/* Losing the data context ends the call, although the modem is still registered */
ZTEST(modem_cellular_v2_ppp, test_redialled_after_context_lost)
{
	link_up();

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CGEV: NW PDN DEACT 1\r");
	expect_redial();
	link_down();
}

/* Events for other contexts leave the call up */
ZTEST(modem_cellular_v2_ppp, test_other_context_ignored)
{
	link_up();

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CGEV: NW PDN DEACT 3\r");
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));
	zassert_equal(TEST_BACKEND_STATE_LINK_UP, test_backend_state(core));
	link_down();
}

/*
 * Losing the registration makes the interface dormant, and ends the call once PPP has not
 * reported the link dead within the periodic script interval. The call is dialled again once
 * registered.
 */
ZTEST(modem_cellular_v2_ppp, test_redialled_after_deregistration)
{
	link_up();

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CEREG: 1,2\r");
	expect_state(core, TEST_BACKEND_STATE_DIALED);
	expect_state(core, TEST_BACKEND_STATE_CMUX);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_redial();
	link_down();
}

/* Registering again before the call ends keeps it, with the interface no longer dormant */
ZTEST(modem_cellular_v2_ppp, test_reregistration_keeps_call)
{
	link_up();

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CEREG: 1,2\r");
	expect_state(core, TEST_BACKEND_STATE_DIALED);
	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_state(core, TEST_BACKEND_STATE_LINK_UP);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));
	zassert_equal(TEST_BACKEND_STATE_LINK_UP, test_backend_state(core));
	link_down();
}

/*
 * Losing registration while the data context is active, e.g. while searching during a cell
 * reselection, leaves the interface up, as only +CGEV reports the context lost
 */
ZTEST(modem_cellular_v2_ppp, test_active_context_keeps_call)
{
	link_up();

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CGEV: ME PDN ACT 1\r");
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CEREG: 1,2\r");
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));
	zassert_equal(TEST_BACKEND_STATE_LINK_UP, test_backend_state(core));

	/* The detach ends the call, which is dialled again once registered */
	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CGEV: NW DETACH\r");
	expect_state(core, TEST_BACKEND_STATE_CMUX);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_redial();
	link_down();
}

/*
 * A circuit-switched +CREG registration does not attach the modem, so it neither dials the
 * call, nor dials it again after a detach from the packet domain
 */
ZTEST(modem_cellular_v2_ppp, test_cs_registration_does_not_attach)
{
	zassert_ok(net_if_up(ppp_iface()));
	advance_to(core, BOOT_AT_READY);
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(100));

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CREG: 1\r");
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(200));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_DLCI2, MODEM_CELLULAR_TEST_DIAL_CMD);
	ok(TEST_BACKEND_PIPE_DLCI2);
	expect_state(core, TEST_BACKEND_STATE_LINK_UP);

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CGEV: NW DETACH\r");
	expect_state(core, TEST_BACKEND_STATE_CMUX);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS * 2));

	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_redial();
	link_down();
}

/* An active data context stands in for the registration of modems that report it late */
ZTEST(modem_cellular_v2_ppp, test_context_stands_in_for_registration)
{
	zassert_ok(net_if_up(ppp_iface()));
	advance_to(core, BOOT_AT_READY);
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(100));

	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CGEV: ME PDN ACT 1\r");
	expect_line(TEST_BACKEND_PIPE_DLCI2, MODEM_CELLULAR_TEST_DIAL_CMD);
	ok(TEST_BACKEND_PIPE_DLCI2);
	expect_state(core, TEST_BACKEND_STATE_LINK_UP);
	link_down();
}

K_MSGQ_DEFINE(app_events, sizeof(enum cellular_registration_status), 8, 4);

static void app_event_handler(const struct device *dev, enum cellular_event event,
			      const void *payload, void *user_data)
{
	const struct cellular_evt_registration_status *evt = payload;

	ARG_UNUSED(dev);
	ARG_UNUSED(event);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&app_events, &evt->status, K_NO_WAIT);
}

static void expect_app_event(enum cellular_registration_status expected)
{
	enum cellular_registration_status status;

	zassert_ok(k_msgq_get(&app_events, &status, K_SECONDS(2)), "no event for status %d",
		   expected);
	zassert_equal(expected, status, "status %d reported, expected %d", status, expected);
}

/* The PPP device has a callback of its own, leaving the application's in place */
ZTEST(modem_cellular_v2_ppp, test_application_callback)
{
	k_msgq_purge(&app_events);
	zassert_ok(cellular_set_callback(core, CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED,
					 app_event_handler, NULL));

	link_up();
	expect_app_event(CELLULAR_REGISTRATION_REGISTERED_HOME);
	link_down();
	expect_app_event(CELLULAR_REGISTRATION_NOT_REGISTERED);

	zassert_ok(cellular_set_callback(core, 0, NULL, NULL));
}

K_MSGQ_DEFINE(fatal_errors, sizeof(int), 4, 4);
static struct net_mgmt_event_callback fatal_error_cb;
static struct net_if *fatal_error_iface;

static void fatal_error_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				struct net_if *iface)
{
	int reason = 0;

	if ((mgmt_event != NET_EVENT_CONN_IF_FATAL_ERROR) || (iface != fatal_error_iface)) {
		return;
	}
	if ((cb->info != NULL) && (cb->info_length == sizeof(reason))) {
		memcpy(&reason, cb->info, sizeof(reason));
	}
	(void)k_msgq_put(&fatal_errors, &reason, K_NO_WAIT);
}

K_MSGQ_DEFINE(app_failures, sizeof(int), 4, 4);

/* The application receives the failure of the modem as the PPP device does */
static void app_failure_handler(const struct device *dev, enum cellular_event event,
				const void *payload, void *user_data)
{
	const struct cellular_evt_modem_failed *evt = payload;

	ARG_UNUSED(dev);
	ARG_UNUSED(event);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&app_failures, &evt->reason, K_NO_WAIT);
}

/* Fail the boot of the core, and every retry of it */
static void fail_boot(void)
{
	for (int i = 0; i <= CONFIG_MODEM_CELLULAR_MAX_RECOVERIES; i++) {
		expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
		ok(TEST_BACKEND_PIPE_UART);
		expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
		respond_error(TEST_BACKEND_PIPE_UART);
	}
}

/*
 * A modem that cannot be brought up within the recovery attempts is reported as a fatal
 * connectivity error. Taking the interface down and up again starts over.
 */
ZTEST(modem_cellular_v2_ppp, test_fatal_error_on_failed_boot)
{
	int reason;

	k_msgq_purge(&app_failures);
	zassert_ok(cellular_set_callback(core, CELLULAR_EVENT_MODEM_FAILED, app_failure_handler,
					 NULL));
	fatal_error_iface = ppp_iface();
	k_msgq_purge(&fatal_errors);
	net_mgmt_init_event_callback(&fatal_error_cb, fatal_error_handler,
				     NET_EVENT_CONN_IF_FATAL_ERROR);
	net_mgmt_add_event_callback(&fatal_error_cb);

	zassert_ok(net_if_up(ppp_iface()));
	fail_boot();
	zassert_ok(k_msgq_get(&fatal_errors, &reason, K_SECONDS(2)), "no fatal error raised");
	zassert_equal(-EIO, reason, "reason %d", reason);
	zassert_ok(k_msgq_get(&app_failures, &reason, K_NO_WAIT), "application not told");
	zassert_equal(-EIO, reason, "reason %d", reason);
	expect_state(core, TEST_BACKEND_STATE_OFF);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MS *
			      (CONFIG_MODEM_CELLULAR_MAX_RECOVERIES + 1)));

	zassert_ok(net_if_down(ppp_iface()));
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);

	zassert_ok(net_if_up(ppp_iface()));
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	zassert_equal(0, k_msgq_num_used_get(&fatal_errors), "fatal error raised again");
}

/* Take the interface down and wait for both devices to suspend, answering nothing */
K_SEM_DEFINE(no_configuration, 0, 1);
static struct net_mgmt_event_callback no_configuration_cb;

static void no_configuration_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				     struct net_if *iface)
{
	ARG_UNUSED(cb);

	if ((mgmt_event == NET_EVENT_CONN_IF_NO_CONFIGURATION) && (iface == ppp_iface())) {
		k_sem_give(&no_configuration);
	}
}

/*
 * Without an APN the data call cannot be made, which is reported as missing configuration
 * without powering the modem up. Taking the interface down and up again once it is set
 * starts over.
 */
/*
 * An LTE RF request holds the modem resumed, and its release disables the LTE RF before the
 * modem is suspended
 */
ZTEST(modem_cellular_v2_ppp, test_lte_rf_request_holds_modem)
{
	zassert_ok(modem_cellular_lte_rf_request(core));
	expect_pm_state(core, PM_DEVICE_STATE_ACTIVE);
	advance_to(core, BOOT_AT_READY);
	expect_apn_configured();
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_ENABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_no_line(K_MSEC(100));

	zassert_ok(modem_cellular_lte_rf_release(core));
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_LTE_RF_DISABLE_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	ok(TEST_BACKEND_PIPE_UART);
	expect_state(core, TEST_BACKEND_STATE_OFF);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
}

/* A request that fails for a missing APN leaves the modem suspended */
ZTEST(modem_cellular_v2_ppp, test_lte_rf_request_without_apn)
{
	/* Clears the APN set for the suite */
	modem_cellular_ztest_halt(core);
	zassert_equal(-ENODATA, modem_cellular_lte_rf_request(core));
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
	expect_no_line(K_MSEC(100));
}

ZTEST(modem_cellular_v2_ppp, test_no_configuration_without_apn)
{
	/* Clears the APN set for the suite */
	modem_cellular_ztest_halt(core);
	k_sem_reset(&no_configuration);
	net_mgmt_init_event_callback(&no_configuration_cb, no_configuration_handler,
				     NET_EVENT_CONN_IF_NO_CONFIGURATION);
	net_mgmt_add_event_callback(&no_configuration_cb);

	zassert_ok(net_if_up(ppp_iface()));
	zassert_ok(k_sem_take(&no_configuration, K_SECONDS(2)), "no configuration not raised");
	expect_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED);
	expect_pm_state(core, PM_DEVICE_STATE_SUSPENDED);
	expect_no_line(K_MSEC(100));

	zassert_ok(cellular_set_apn(core, TEST_APN));
	zassert_ok(net_if_down(ppp_iface()));
	link_up();
	zassert_equal(0, k_sem_count_get(&no_configuration), "no configuration raised again");
	link_down();
}

static bool settle(void)
{
	/* Fails when the interface is already down */
	(void)net_if_down(ppp_iface());

	/* An unanswered shutdown script times out, then the modem is left as it is */
	return wait_for_pm_state(ppp_device(), PM_DEVICE_STATE_SUSPENDED, K_SECONDS(5)) &&
	       wait_for_pm_state(core, PM_DEVICE_STATE_SUSPENDED, K_SECONDS(5));
}

static void v2_ppp_before(void *fixture)
{
	zassert_true(settle(), "PPP device or core left active");
	modem_test_before_with_apn(fixture);
	modem_reset_state(core);
}

static void v2_ppp_after(void *fixture)
{
	/* A failed test can leave the interface up, or a callback set */
	(void)cellular_set_callback(core, 0, NULL, NULL);
	net_mgmt_del_event_callback(&fatal_error_cb);
	net_mgmt_del_event_callback(&no_configuration_cb);
	(void)settle();
	modem_test_after(fixture);
}

ZTEST_SUITE(modem_cellular_v2_ppp, NULL, modem_test_setup, v2_ppp_before, v2_ppp_after, NULL);
