/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MODEM_CELLULAR_V2 brings the modem up until the CMUX connects and it has set up the AT
 * command channel on DLCI1, and no further. The PPP channel on DLCI2 is then handed out
 * through a pipelink, as are the user pipes.
 */

#include "test_helpers.h"

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <modem_cellular_test.h>
#include <modem_workqueue.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/ztest.h>

#include <string.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_MODEM_CELLULAR_V2));

#define TARGET_BAUDRATE          DT_PROP(DT_ALIAS(modem), target_baudrate)
#define BAUDRATE_UPDATE_DELAY_MS DT_PROP(DT_ALIAS(modem), baudrate_update_delay_ms)

static const struct device *const modem = DEVICE_DT_GET(DT_ALIAS(modem));
static const struct device *const modem_slow = DEVICE_DT_GET(DT_NODELABEL(modem_slow));
static const struct device *const modem_no_power = DEVICE_DT_GET(DT_NODELABEL(modem_no_power));
static const struct device *const modem_no_shutdown =
	DEVICE_DT_GET(DT_NODELABEL(modem_no_shutdown));
static const struct device *const modem_no_dlci_setup =
	DEVICE_DT_GET(DT_NODELABEL(modem_no_dlci_setup));

MODEM_PIPELINK_DT_DECLARE(DT_ALIAS(modem), MODEM_CELLULAR_PIPELINK_PPP);
MODEM_PIPELINK_DT_DECLARE(DT_ALIAS(modem), user_pipe_0);
MODEM_PIPELINK_DT_DECLARE(DT_ALIAS(modem), user_pipe_1);
MODEM_PIPELINK_DT_DECLARE(DT_NODELABEL(modem_no_dlci_setup), MODEM_CELLULAR_PIPELINK_PPP);

/* Every pipelink of the modem, and the DLCI it carries */
static const struct {
	struct modem_pipelink *link;
	enum test_backend_pipe_id pipe;
} pipelinks[] = {
	{MODEM_PIPELINK_DT_GET(DT_ALIAS(modem), MODEM_CELLULAR_PIPELINK_PPP),
	 TEST_BACKEND_PIPE_DLCI2},
	{MODEM_PIPELINK_DT_GET(DT_ALIAS(modem), user_pipe_0), TEST_BACKEND_PIPE_USER3},
	{MODEM_PIPELINK_DT_GET(DT_ALIAS(modem), user_pipe_1), TEST_BACKEND_PIPE_USER4},
};

static void expect_pipelinks_connected(bool connected)
{
	for (size_t i = 0; i < ARRAY_SIZE(pipelinks); i++) {
		zassert_equal(connected, modem_pipelink_is_connected(pipelinks[i].link),
			      "pipelink %zu connected is %d", i, !connected);
	}
}

/* The driver keeps the AT command channel, and brings nothing else up */
ZTEST(modem_cellular_v2_core, test_stops_when_at_ready)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	expect_pipelinks_connected(true);
	zassert_true(test_backend_pipe_is_open(TEST_BACKEND_PIPE_DLCI1), "DLCI1 not opened");
	zassert_false(test_backend_pipe_is_open(TEST_BACKEND_PIPE_DLCI2), "DLCI2 opened");

	/* The driver chat reads the AT command channel */
	registered(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);

	/* Nothing until the periodic script is due */
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS / 2));
	zassert_equal(TEST_BACKEND_STATE_CMUX, test_backend_state(modem));
}

/* The other channels are only handed out once DLCI1 is set up and the SIM checked */
ZTEST(modem_cellular_v2_core, test_pipelinks_wait_for_dlci_setup)
{
	modem_restart(modem);
	advance_to(modem, BOOT_CMUX_PENDING);
	expect_pipelinks_connected(false);

	test_backend_cmux_connect_finish(true);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_DLCI_SETUP_CMD);
	expect_pipelinks_connected(false);

	ok(TEST_BACKEND_PIPE_DLCI1);
	answer_reports();
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CPIN_CMD);
	expect_pipelinks_connected(false);

	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	expect_pipelinks_connected(false);

	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	flush_driver_events();
	expect_pipelinks_connected(true);
}

/* A failed DLCI setup is not critical, the channel still takes commands */
ZTEST(modem_cellular_v2_core, test_dlci_setup_failure)
{
	modem_restart(modem);
	advance_to(modem, BOOT_DLCI1_SETUP);

	respond_error(TEST_BACKEND_PIPE_DLCI1);
	answer_reports();
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CPIN_CMD);
	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	flush_driver_events();
	expect_pipelinks_connected(true);
	expect_no_line(K_MSEC(100));
}

/* Reports the modem cannot enable are logged, and the modem brought up regardless */
ZTEST(modem_cellular_v2_core, test_reports_failure)
{
	modem_restart(modem);
	advance_to(modem, BOOT_REPORTS);

	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CEREG_CMD);
	respond_error(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CGEREP_CMD);
	test_backend_respond(TEST_BACKEND_PIPE_DLCI1, "+CME ERROR: 4\r");
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CPIN_CMD);
	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	flush_driver_events();
	expect_pipelinks_connected(true);
}

/* Without a DLCI setup script the reports are enabled as soon as DLCI1 opens */
ZTEST(modem_cellular_v2_core, test_no_dlci_setup_script)
{
	struct modem_pipelink *ppp = MODEM_PIPELINK_DT_GET(DT_NODELABEL(modem_no_dlci_setup),
							   MODEM_CELLULAR_PIPELINK_PPP);

	modem_restart(modem_no_dlci_setup);
	advance_to(modem_no_dlci_setup, BOOT_CMUX_CONNECTED);
	answer_reports();
	expect_line(TEST_BACKEND_PIPE_DLCI1, TEST_CPIN_CMD);
	respond_cpin("READY");
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_SIM_READY_CMD);
	ok(TEST_BACKEND_PIPE_DLCI1);
	expect_read(TEST_BACKEND_PIPE_DLCI1);
	flush_driver_events();
	zassert_true(modem_pipelink_is_connected(ppp), "PPP pipelink not connected");
	zassert_true(test_backend_pipe_is_open(TEST_BACKEND_PIPE_DLCI1), "DLCI1 not opened");
	expect_no_line(K_MSEC(100));
}

/* Suspending before DLCI1 is set up never hands out the other channels */
ZTEST(modem_cellular_v2_core, test_suspend_during_dlci_setup)
{
	modem_restart(modem);
	advance_to(modem, BOOT_DLCI1_SETUP);

	suspend_and_shutdown(modem);
	expect_pipelinks_connected(false);
}

/* Pipelinks each carry their own DLCI */
ZTEST(modem_cellular_v2_core, test_pipelinks)
{
	static const char cmd[] = "AT+USER\r";

	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	for (size_t i = 0; i < ARRAY_SIZE(pipelinks); i++) {
		struct modem_pipe *pipe = modem_pipelink_get_pipe(pipelinks[i].link);

		zassert_ok(modem_pipe_open(pipe, K_SECONDS(2)));
		zassert_true(test_backend_pipe_is_open(pipelinks[i].pipe));
		zassert_equal(strlen(cmd),
			      modem_pipe_transmit(pipe, (const uint8_t *)cmd, strlen(cmd)));
		expect_line(pipelinks[i].pipe, "AT+USER");
	}

	suspend_and_shutdown(modem);
	expect_pipelinks_connected(false);
}

/* Halting the driver disconnects the pipelinks, as powering down does */
ZTEST(modem_cellular_v2_core, test_pipelinks_halt)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	expect_pipelinks_connected(true);

	modem_cellular_ztest_halt(modem);
	expect_pipelinks_connected(false);
}

/* The modem powers up at its default baudrate, so the UART is restored first */
ZTEST(modem_cellular_v2_core, test_suspend_and_resume)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	zassert_equal(TARGET_BAUDRATE, uart_baudrate(modem),
		      "UART baudrate was not switched");

	suspend_and_shutdown(modem);

	zassert_ok(modem_cellular_pm_action(modem, PM_DEVICE_ACTION_RESUME), "resume failed");
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	zassert_equal(modem_power_up_baudrate(modem), uart_baudrate(modem),
		      "UART baudrate was not restored on power up");
	ok(TEST_BACKEND_PIPE_UART);

	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
	zassert_equal(TARGET_BAUDRATE, uart_baudrate(modem),
		      "UART baudrate was not switched");
	ok(TEST_BACKEND_PIPE_UART);
	expect_line(TEST_BACKEND_PIPE_DLCI1, MODEM_CELLULAR_TEST_DLCI_SETUP_CMD);
}

static K_SEM_DEFINE(modem_ready, 0, K_SEM_MAX_LIMIT);

static void modem_ready_handler(const struct device *dev, enum cellular_event event,
				const void *payload, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(event);
	ARG_UNUSED(payload);
	ARG_UNUSED(user_data);

	k_sem_give(&modem_ready);
}

/* Readiness is reported once per boot, and not on suspend */
ZTEST(modem_cellular_v2_core, test_ready_reported_each_boot)
{
	k_sem_reset(&modem_ready);
	zassert_ok(cellular_set_callback(modem, CELLULAR_EVENT_MODEM_READY, modem_ready_handler,
					 NULL));

	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	zassert_ok(k_sem_take(&modem_ready, K_SECONDS(1)), "ready not reported");

	suspend_and_shutdown(modem);
	zassert_equal(0, k_sem_count_get(&modem_ready), "ready reported on suspend");

	zassert_ok(modem_cellular_pm_action(modem, PM_DEVICE_ACTION_RESUME), "resume failed");
	advance_to(modem, BOOT_AT_READY);
	zassert_ok(k_sem_take(&modem_ready, K_SECONDS(1)), "ready not reported on resume");
	zassert_equal(0, k_sem_count_get(&modem_ready), "ready reported more than once");

	zassert_ok(cellular_set_callback(modem, 0, NULL, NULL));
}

/*
 * Without a reset or power key, recovery boots the modem again as it is, at its power up
 * baudrate, until the recovery attempts are spent.
 */
ZTEST(modem_cellular_v2_core, test_recovery_without_gpios)
{
	modem_restart(modem_no_power);
	advance_to(modem_no_power, BOOT_SET_BAUDRATE);
	ok(TEST_BACKEND_PIPE_UART);

	for (int i = 0; i < CONFIG_MODEM_CELLULAR_MAX_RECOVERIES; i++) {
		expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
		respond_error(TEST_BACKEND_PIPE_UART);
		expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
		zassert_equal(modem_power_up_baudrate(modem_no_power),
			      uart_baudrate(modem_no_power),
			      "UART baudrate was not restored for the retry");
		ok(TEST_BACKEND_PIPE_UART);
	}

	/* Spent, so powered down rather than retried */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
	respond_error(TEST_BACKEND_PIPE_UART);
	expect_state(modem_no_power, TEST_BACKEND_STATE_OFF);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MS *
			      (CONFIG_MODEM_CELLULAR_MAX_RECOVERIES + 1)));
}

ZTEST(modem_cellular_v2_core, test_uart_backend_init_failure)
{
	modem_reset_state(modem);
	test_backend_uart_init_fail_once();
	zassert_equal(-EIO, modem_cellular_init(modem));

	/* A later initialization can still succeed. */
	modem_restart(modem);
	advance_to(modem, BOOT_SET_BAUDRATE);
}

ZTEST(modem_cellular_v2_core, test_uart_open_failure_recovers)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SET_BAUDRATE);
	test_backend_pipe_fail_next_open(TEST_BACKEND_PIPE_UART);
	ok(TEST_BACKEND_PIPE_UART);

	/* The failed init-script open enters recovery and starts boot again. */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
}

ZTEST(modem_cellular_v2_core, test_uart_close_failure_recovers)
{
	modem_restart(modem);
	advance_to(modem, BOOT_INIT);
	test_backend_pipe_fail_next_close(TEST_BACKEND_PIPE_UART);
	ok(TEST_BACKEND_PIPE_UART);

	/* The failed close after init enters recovery and starts boot again. */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
}

ZTEST(modem_cellular_v2_core, test_dlci1_open_failure_recovers)
{
	modem_restart(modem);
	advance_to(modem, BOOT_CMUX_PENDING);
	test_backend_pipe_fail_next_open(TEST_BACKEND_PIPE_DLCI1);
	test_backend_cmux_connect_finish(true);

	/* The failed command-channel open enters recovery and starts boot again. */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
}

ZTEST(modem_cellular_v2_core, test_dlci1_open_timeout_recovers)
{
	modem_restart(modem);
	advance_to(modem, BOOT_CMUX_PENDING);
	test_backend_pipe_close_next_open(TEST_BACKEND_PIPE_DLCI1);
	test_backend_cmux_connect_finish(true);

	/* The CMUX reports a closed pipe after its open retries are exhausted. */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
}

/* A command-channel failure remains visible after the chat has attached to DLCI1. */
ZTEST(modem_cellular_v2_core, test_dlci1_close_after_attach_recovers)
{
	modem_restart(modem_no_dlci_setup);
	advance_to(modem_no_dlci_setup, BOOT_SIM);

	test_backend_pipe_disconnect(TEST_BACKEND_PIPE_DLCI1);

	/* Recovery restarts the modem instead of waiting for a future script failure. */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
}

/* The init script waits for the UART to close and reopen at the new baudrate, however long the
 * close takes
 */
ZTEST(modem_cellular_v2_core, test_baudrate_waits_for_close)
{
	modem_restart(modem);
	advance_to(modem, BOOT_SET_BAUDRATE);
	test_backend_pipe_close_hold(TEST_BACKEND_PIPE_UART);
	ok(TEST_BACKEND_PIPE_UART);
	zassert_true(test_backend_pipe_wait_for_close(TEST_BACKEND_PIPE_UART, K_SECONDS(1)),
		     "UART not closed after the set baudrate script");

	expect_no_line(K_MSEC(BAUDRATE_UPDATE_DELAY_MS + 100));
	zassert_equal(modem_power_up_baudrate(modem), uart_baudrate(modem),
		      "baudrate changed before the UART closed");

	test_backend_pipe_close_finish(TEST_BACKEND_PIPE_UART);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
	zassert_equal(TARGET_BAUDRATE, uart_baudrate(modem),
		      "init script not run at the new baudrate");
}

static K_SEM_DEFINE(workqueue_release, 0, 1);

static void workqueue_block_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)k_sem_take(&workqueue_release, K_FOREVER);
}

/* An open that completes after the startup time ran out, with its event queued behind the
 * timeout, does not restart the script started once the UART is reopened
 */
ZTEST(modem_cellular_v2_core, test_late_uart_open_ignored)
{
	static struct k_work block;

	modem_reset_state(modem_slow);
	test_backend_pipe_open_hold(TEST_BACKEND_PIPE_UART);
	zassert_ok(modem_cellular_init(modem_slow));
	zassert_true(test_backend_pipe_wait_for_open(TEST_BACKEND_PIPE_UART, K_SECONDS(2)),
		     "UART never opened");

	/* Hold the driver until the startup time is out, then complete the open */
	k_sem_reset(&workqueue_release);
	k_work_init(&block, workqueue_block_handler);
	zassert_equal(1, modem_work_submit(&block));
	k_msleep(DT_PROP(DT_NODELABEL(modem_slow), startup_time_ms) * 2);
	test_backend_pipe_open_finish(TEST_BACKEND_PIPE_UART);
	k_sem_give(&workqueue_release);

	/* The reopened UART reports open at once, and the second report changes nothing */
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	flush_driver_events();
	zassert_true(test_backend_pipe_is_open(TEST_BACKEND_PIPE_UART),
		     "set baudrate script given up on");

	ok(TEST_BACKEND_PIPE_UART);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
}

/* Without a reset or power key, recovery takes a modem left in CMUX mode back to AT mode
 * before booting it again
 */
ZTEST(modem_cellular_v2_core, test_recovery_without_gpios_disconnects_cmux)
{
	modem_restart(modem_no_power);
	advance_to(modem_no_power, BOOT_AT_READY);

	test_backend_pipe_disconnect(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	zassert_equal(1, test_backend_cmux_disconnects(), "CMUX not disconnected");
	ok(TEST_BACKEND_PIPE_UART);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_INIT_CMD);
}

/* A fault after the CMUX connects does not reset the recovery attempts */
ZTEST(modem_cellular_v2_core, test_recovery_spent_after_cmux)
{
	modem_restart(modem_no_dlci_setup);

	for (int i = 0; i <= CONFIG_MODEM_CELLULAR_MAX_RECOVERIES; i++) {
		advance_to(modem_no_dlci_setup, BOOT_SIM);
		test_backend_pipe_disconnect(TEST_BACKEND_PIPE_DLCI1);
	}

	/* Spent, so powered down rather than retried */
	expect_state(modem_no_dlci_setup, TEST_BACKEND_STATE_OFF);
	expect_no_line(K_MSEC(CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MS *
			      (CONFIG_MODEM_CELLULAR_MAX_RECOVERIES + 1)));
}

static struct {
	const struct gpio_dt_spec *spec;
	struct gpio_callback cb;
	int last;
	atomic_t presses;
} power_key;

static void power_key_handler(const struct device *port, struct gpio_callback *cb,
			      gpio_port_pins_t pins)
{
	int value = gpio_emul_output_get_dt(power_key.spec);

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if ((value == 1) && (power_key.last == 0)) {
		atomic_inc(&power_key.presses);
	}
	power_key.last = value;
}

/* Count the presses of the power key of @p dev from now on */
static void power_key_monitor_start(const struct device *dev)
{
	power_key.spec = test_backend_power_gpio(dev);
	power_key.last = gpio_emul_output_get_dt(power_key.spec);
	atomic_clear(&power_key.presses);
	gpio_init_callback(&power_key.cb, power_key_handler, BIT(power_key.spec->pin));
	zassert_ok(gpio_add_callback_dt(power_key.spec, &power_key.cb));
}

static atomic_val_t power_key_monitor_stop(void)
{
	(void)gpio_remove_callback_dt(power_key.spec, &power_key.cb);
	return atomic_get(&power_key.presses);
}

/* A modem that never reached CMUX is off, so recovery presses the power key once */
ZTEST(modem_cellular_v2_core, test_recovery_powers_on_stopped_modem)
{
	modem_restart(modem);
	advance_to(modem, BOOT_RECOVERY);
	power_key_monitor_start(modem);

	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	zassert_equal(1, power_key_monitor_stop(), "power key not pressed once");
}

/* A fault after the CMUX connects leaves the modem running, so recovery turns it off first */
ZTEST(modem_cellular_v2_core, test_recovery_power_cycles_running_modem)
{
	modem_restart(modem_no_dlci_setup);
	advance_to(modem_no_dlci_setup, BOOT_SIM);
	power_key_monitor_start(modem_no_dlci_setup);

	test_backend_pipe_disconnect(TEST_BACKEND_PIPE_DLCI1);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SET_BAUDRATE_CMD);
	zassert_equal(2, power_key_monitor_stop(), "power key not pressed twice");
}

/* Already suspended, so a second suspend returns without touching the modem */
ZTEST(modem_cellular_v2_core, test_suspend_when_idle)
{
	modem_restart(modem);
	advance_to(modem, BOOT_AT_READY);
	suspend_and_shutdown(modem);

	start_suspend(modem);
	finish_suspend(modem);
	expect_no_line(K_MSEC(100));
}

/* Software shutdown failed, fall back to the power key */
ZTEST(modem_cellular_v2_core, test_shutdown_script_failure)
{
	modem_restart(modem_slow);
	advance_to(modem_slow, BOOT_AT_READY);

	start_suspend(modem_slow);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	respond_error(TEST_BACKEND_PIPE_UART);
	expect_gpio(test_backend_power_gpio(modem_slow), 1);
	expect_gpio(test_backend_power_gpio(modem_slow), 0);
	finish_suspend(modem_slow);
	expect_no_line(K_MSEC(100));
}

/* Without a power key a failed shutdown script leaves the modem as it is */
ZTEST(modem_cellular_v2_core, test_shutdown_without_power_key)
{
	modem_restart(modem_no_power);
	advance_to(modem_no_power, BOOT_AT_READY);

	start_suspend(modem_no_power);
	expect_line(TEST_BACKEND_PIPE_UART, MODEM_CELLULAR_TEST_SHUTDOWN_CMD);
	respond_error(TEST_BACKEND_PIPE_UART);
	finish_suspend(modem_no_power);
	expect_no_line(K_MSEC(100));
}

/* Without a shutdown script the modem is powered down with the power key alone */
ZTEST(modem_cellular_v2_core, test_no_shutdown_script)
{
	modem_restart(modem_no_shutdown);
	advance_to(modem_no_shutdown, BOOT_AT_READY);

	start_suspend(modem_no_shutdown);
	finish_suspend(modem_no_shutdown);
	expect_no_line(K_MSEC(100));
}

ZTEST_SUITE(modem_cellular_v2_core, NULL, modem_test_setup, modem_test_before, modem_test_after,
	    NULL);
