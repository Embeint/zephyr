/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Cellular modem driver built on the State Machine Framework.
 *
 * Boots the modem until CMUX is connected and the AT command channel is open, and powers
 * it down again on suspend. The driver keeps the AT command channel on DLCI 1 for its
 * chat, while the PPP channel on DLCI 2 is handed out through a pipelink.
 *
 *   IDLE                    Modem powered down
 *   ACTIVE                  Modem powered, or being powered up or down
 *     BOOT                  Bringing the modem up, a suspend returns to IDLE
 *       RESET_PULSE
 *       AWAIT_RESET
 *       POWER_ON_PULSE
 *       AWAIT_POWER_ON
 *       SET_BAUDRATE
 *       RUN_INIT_SCRIPT
 *       RECOVERY
 *       RECOVERY_CMUX_DISCONNECT  Returning a modem without power or reset control to AT mode
 *       CONNECT_CMUX
 *     CMUX_CONNECTED        CMUX up, chat on the AT command channel once open, tracking
 *                           the registration and data context it reports
 *       OPEN_DLCI1
 *       RUN_DLCI_SETUP
 *       ENABLE_REPORTS      Enabling the error codes, registration and data context reports
 *                           the driver relies on, on the AT command channel
 *       AWAIT_SIM           Querying the SIM status until it is known or the SIM timeout
 *                           passes, reported with CELLULAR_EVENT_SIM_STATUS_CHANGED
 *       RUN_SIM_READY       Vendor script run once the SIM is ready
 *       RUN_BOARD_INIT      Board-specific script, if any
 *       READY               AT command channel set up, PPP and user pipelinks connected,
 *                           reported with CELLULAR_EVENT_MODEM_READY, the periodic script
 *                           due every periodic script interval
 *         LTE_RF_OFF        As the init script leaves it, until the LTE RF is requested
 *         CONFIGURE_APN     Configuring the data context with the APN while the LTE RF is
 *                           off, retried every LTE RF retry interval while it fails,
 *                           skipped with the default PDP context
 *         LTE_RF_ENABLE
 *         LTE_RF_ON         Until the last LTE RF request is released
 *         LTE_RF_DISABLE
 *         RUN_PERIODIC      Periodic script, run from LTE_RF_OFF or LTE_RF_ON and returning
 *                           to it, enough consecutive failures of which recover the modem
 *     POWER_DOWN           Taking the modem down, a suspend completes in IDLE
 *       CMUX_DISCONNECT
 *       RUN_SHUTDOWN_SCRIPT
 *       POWER_OFF_PULSE
 *       AWAIT_POWER_OFF
 *
 * Events run through the state machine on the modem workqueue, which the chat, CMUX and PPP
 * modules also run on, so that the reports they parse are serialised with it. Events that
 * report a level, the requested power state and the LTE RF requests, are signalled through
 * atomic flags, which coalesce rather than being lost when they arrive faster than they are
 * handled. Other events, which carry data or report edges, are queued.
 *
 * The state timer expires on the same workqueue. Its timeout is run once the events queued
 * before it are handled, and only if neither a transition nor a restart of the timer has
 * superseded it, so a timeout is only ever seen by the state that started the timer.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/cmux.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/smf.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_cellular, CONFIG_MODEM_LOG_LEVEL);

#include <stdlib.h>
#include <string.h>

#include "modem_cellular_common.h"
#include "modem_workqueue.h"

/* Time for the modem to switch its UART into CMUX mode after the init script */
#define MODEM_CELLULAR_CMUX_SWITCH_DELAY       K_MSEC(100)
/* Time for the modem to acknowledge a CMUX disconnect */
#define MODEM_CELLULAR_CMUX_DISCONNECT_TIMEOUT K_SECONDS(2)
/* Time between SIM status queries while the SIM is not ready */
#define MODEM_CELLULAR_SIM_POLL_INTERVAL       K_MSEC(250)
/* Time for the AT channel to settle before a board-specific script */
#define MODEM_CELLULAR_BOARD_INIT_DELAY        K_MSEC(200)

enum modem_cellular_event {
	MODEM_CELLULAR_EVENT_RESUME,
	MODEM_CELLULAR_EVENT_SUSPEND,
	MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS,
	MODEM_CELLULAR_EVENT_SCRIPT_FAILED,
	MODEM_CELLULAR_EVENT_CMUX_CONNECTED,
	MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED,
	MODEM_CELLULAR_EVENT_DLCI1_OPENED,
	MODEM_CELLULAR_EVENT_DLCI1_CLOSED,
	MODEM_CELLULAR_EVENT_TIMEOUT,
	MODEM_CELLULAR_EVENT_BUS_OPENED,
	MODEM_CELLULAR_EVENT_BUS_CLOSED,
	MODEM_CELLULAR_EVENT_BUS_ERROR,
	MODEM_CELLULAR_EVENT_MODEM_READY,
	MODEM_CELLULAR_EVENT_LTE_RF_CHANGED,
	MODEM_CELLULAR_EVENT_PERIODIC,
	MODEM_CELLULAR_EVENT_PERIODIC_RESUMED,
};

enum modem_cellular_v2_core_state {
	STATE_IDLE,
	STATE_ACTIVE,
	STATE_BOOT,
	STATE_RESET_PULSE,
	STATE_AWAIT_RESET,
	STATE_POWER_ON_PULSE,
	STATE_AWAIT_POWER_ON,
	STATE_SET_BAUDRATE,
	STATE_RUN_INIT_SCRIPT,
	STATE_RECOVERY,
	STATE_RECOVERY_CMUX_DISCONNECT,
	STATE_CONNECT_CMUX,
	STATE_CMUX_CONNECTED,
	STATE_OPEN_DLCI1,
	STATE_RUN_DLCI_SETUP,
	STATE_ENABLE_REPORTS,
	STATE_AWAIT_SIM,
	STATE_RUN_SIM_READY,
	STATE_RUN_BOARD_INIT,
	STATE_READY,
	STATE_LTE_RF_OFF,
	STATE_CONFIGURE_APN,
	STATE_LTE_RF_ENABLE,
	STATE_LTE_RF_ON,
	STATE_LTE_RF_DISABLE,
	STATE_RUN_PERIODIC,
	STATE_POWER_DOWN,
	STATE_CMUX_DISCONNECT,
	STATE_RUN_SHUTDOWN_SCRIPT,
	STATE_POWER_OFF_PULSE,
	STATE_AWAIT_POWER_OFF,
	STATE_COUNT,
};

static void modem_cellular_set_state(struct modem_cellular_data *data,
				     enum modem_cellular_v2_core_state state);

static void modem_cellular_delegate_event_ptr(struct modem_cellular_data *data,
					      enum modem_cellular_event evt, const void *ptr);

static void modem_cellular_delegate_event(struct modem_cellular_data *data,
					  enum modem_cellular_event evt);

static const struct modem_chat_script *
modem_cellular_board_init_script(const struct modem_cellular_data *data);

/* Level signals, run in this order once the queued events are handled */
enum modem_cellular_signal {
	/* The LTE RF requests changed, handled first so that a release is acted on before a
	 * suspend that follows it
	 */
	MODEM_CELLULAR_SIGNAL_LTE_RF,
	/* The periodic script is due */
	MODEM_CELLULAR_SIGNAL_PERIODIC,
	/* The periodic script was resumed */
	MODEM_CELLULAR_SIGNAL_PERIODIC_RESUMED,
	/* The requested power state changed */
	MODEM_CELLULAR_SIGNAL_PM,
	MODEM_CELLULAR_SIGNAL_COUNT,
};

static void modem_cellular_signal(struct modem_cellular_data *data,
				  enum modem_cellular_signal signal);

static const char *const modem_cellular_state_names[STATE_COUNT] = {
	[STATE_IDLE] = "idle",
	[STATE_ACTIVE] = "active",
	[STATE_BOOT] = "boot",
	[STATE_RESET_PULSE] = "reset pulse",
	[STATE_AWAIT_RESET] = "await reset",
	[STATE_POWER_ON_PULSE] = "power on pulse",
	[STATE_AWAIT_POWER_ON] = "await power on",
	[STATE_SET_BAUDRATE] = "set baudrate",
	[STATE_RUN_INIT_SCRIPT] = "run init script",
	[STATE_RECOVERY] = "recovery",
	[STATE_RECOVERY_CMUX_DISCONNECT] = "recovery cmux disconnect",
	[STATE_CONNECT_CMUX] = "connect cmux",
	[STATE_CMUX_CONNECTED] = "cmux connected",
	[STATE_OPEN_DLCI1] = "open dlci1",
	[STATE_RUN_DLCI_SETUP] = "run dlci setup",
	[STATE_ENABLE_REPORTS] = "enable reports",
	[STATE_AWAIT_SIM] = "await sim",
	[STATE_RUN_SIM_READY] = "run sim ready",
	[STATE_RUN_BOARD_INIT] = "run board init",
	[STATE_READY] = "ready",
	[STATE_LTE_RF_OFF] = "lte rf off",
	[STATE_CONFIGURE_APN] = "configure apn",
	[STATE_LTE_RF_ENABLE] = "lte rf enable",
	[STATE_LTE_RF_ON] = "lte rf on",
	[STATE_LTE_RF_DISABLE] = "lte rf disable",
	[STATE_RUN_PERIODIC] = "run periodic",
	[STATE_POWER_DOWN] = "power down",
	[STATE_CMUX_DISCONNECT] = "cmux disconnect",
	[STATE_RUN_SHUTDOWN_SCRIPT] = "run shutdown script",
	[STATE_POWER_OFF_PULSE] = "power off pulse",
	[STATE_AWAIT_POWER_OFF] = "await power off",
};

static const char *const modem_cellular_event_names[] = {
	[MODEM_CELLULAR_EVENT_RESUME] = "resume",
	[MODEM_CELLULAR_EVENT_SUSPEND] = "suspend",
	[MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS] = "script success",
	[MODEM_CELLULAR_EVENT_SCRIPT_FAILED] = "script failed",
	[MODEM_CELLULAR_EVENT_CMUX_CONNECTED] = "cmux connected",
	[MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED] = "cmux disconnected",
	[MODEM_CELLULAR_EVENT_DLCI1_OPENED] = "dlci1 opened",
	[MODEM_CELLULAR_EVENT_DLCI1_CLOSED] = "dlci1 closed",
	[MODEM_CELLULAR_EVENT_TIMEOUT] = "timeout",
	[MODEM_CELLULAR_EVENT_BUS_OPENED] = "bus opened",
	[MODEM_CELLULAR_EVENT_BUS_CLOSED] = "bus closed",
	[MODEM_CELLULAR_EVENT_BUS_ERROR] = "bus error",
	[MODEM_CELLULAR_EVENT_MODEM_READY] = "modem ready",
	[MODEM_CELLULAR_EVENT_LTE_RF_CHANGED] = "lte rf changed",
	[MODEM_CELLULAR_EVENT_PERIODIC] = "periodic",
	[MODEM_CELLULAR_EVENT_PERIODIC_RESUMED] = "periodic resumed",
};

static struct modem_cellular_data *modem_cellular_data_from_obj(void *obj)
{
	return CONTAINER_OF(obj, struct modem_cellular_data, smf);
}

static const struct modem_cellular_config *modem_cellular_config(struct modem_cellular_data *data)
{
	return data->dev->config;
}

static const struct modem_cellular_config_scripts *
modem_cellular_scripts(struct modem_cellular_data *data)
{
	return &modem_cellular_config(data)->vendor->common->scripts;
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static void modem_cellular_start_timer(struct modem_cellular_data *data, k_timeout_t timeout)
{
	data->timer_gen++;
	modem_work_reschedule(&data->timeout_work, timeout);
}

/* Return the UART to the baudrate the modem powers up at */
static void modem_cellular_restore_baudrate(struct modem_cellular_data *data)
{
	if (data->original_baudrate != 0U) {
		uint32_t previous = modem_cellular_baudrate_update(data, data->original_baudrate);

		/* A zero result means the UART was not changed; retain the target for recovery. */
		if (previous != 0U) {
			data->original_baudrate = 0U;
		}
	}
}

static void modem_cellular_bus_pipe_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
					    void *user_data)
{
	struct modem_cellular_data *data = user_data;

	ARG_UNUSED(pipe);

	switch (event) {
	case MODEM_PIPE_EVENT_OPENED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_OPENED);
		break;
	case MODEM_PIPE_EVENT_CLOSED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_CLOSED);
		break;
	default:
		break;
	}
}

static void modem_cellular_dlci1_pipe_handler(struct modem_pipe *pipe, enum modem_pipe_event event,
					      void *user_data)
{
	struct modem_cellular_data *data = user_data;

	/* The chat also uses the UART during boot and shutdown. Only DLCI1 lifetime
	 * events describe the command channel of the connected CMUX session.
	 */
	if (pipe != data->dlci1_pipe) {
		return;
	}

	if (event == MODEM_PIPE_EVENT_OPENED) {
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_DLCI1_OPENED);
	} else if (event == MODEM_PIPE_EVENT_CLOSED) {
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_DLCI1_CLOSED);
	}
}

/* Hand out the channels the driver does not use itself */
static void modem_cellular_notify_pipelinks_connected(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_pipelink_notify_connected(config->ppp_pipelink);
	modem_cellular_notify_user_pipes_connected(data);
}

static void modem_cellular_notify_pipelinks_disconnected(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_pipelink_notify_disconnected(config->ppp_pipelink);
	modem_cellular_notify_user_pipes_disconnected(data);
}

static int modem_cellular_open_bus(struct modem_cellular_data *data)
{
	modem_pipe_attach(data->uart_pipe, modem_cellular_bus_pipe_handler, data);
	return modem_pipe_open_async(data->uart_pipe);
}

static int modem_cellular_close_bus(struct modem_cellular_data *data)
{
	modem_pipe_attach(data->uart_pipe, modem_cellular_bus_pipe_handler, data);
	return modem_pipe_close_async(data->uart_pipe);
}

/*
 * Start @p script, recording it as the one whose result the current state waits for.
 * A script that cannot be started is reported as failed.
 */
static void modem_cellular_run_script(struct modem_cellular_data *data,
				      const struct modem_chat_script *script)
{
	int ret;

	data->script_running = script;
	ret = modem_chat_run_script_async(&data->chat, script);
	if (ret < 0) {
		LOG_ERR("Failed to start script %s (%d)", script->name, ret);
		modem_cellular_delegate_event_ptr(data, MODEM_CELLULAR_EVENT_SCRIPT_FAILED, script);
	}
}

/* First state that talks to the modem over the UART */
static void modem_cellular_enter_at_state(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);

	if ((config->vendor->instance != NULL) &&
	    (config->vendor->instance->set_baudrate != NULL)) {
		modem_cellular_set_state(data, STATE_SET_BAUDRATE);
	} else {
		modem_cellular_set_state(data, STATE_RUN_INIT_SCRIPT);
	}
}

static void modem_cellular_begin_power_off_pulse(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);

	(void)modem_pipe_close_async(data->uart_pipe);

	/* The power key would turn on a modem the status GPIO shows is already off */
	if (modem_cellular_gpio_is_enabled(&config->power_gpio) &&
	    (!modem_cellular_gpio_is_enabled(&config->status_gpio) ||
	     modem_cellular_modem_is_powered(config))) {
		modem_cellular_set_state(data, STATE_POWER_OFF_PULSE);
	} else {
		modem_cellular_set_state(data, STATE_IDLE);
	}
}

/* Run the shutdown script if there is a channel to run it on, else use the power key */
static void modem_cellular_begin_shutdown(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);

	if ((config->vendor->common->scripts.shutdown != NULL) && (data->cmd_pipe != NULL)) {
		modem_cellular_set_state(data, STATE_RUN_SHUTDOWN_SCRIPT);
		return;
	}

	modem_cellular_begin_power_off_pulse(data);
}

/* Power off a modem that may be left in CMUX mode by a failed or lost CMUX connection */
static void modem_cellular_power_off_after_cmux(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_cmux_release(&data->cmux);

	if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
		/* The shutdown script would only time out in CMUX mode */
		modem_cellular_begin_power_off_pulse(data);
	} else if (data->cmux_bus_opened) {
		/* Without a power key the shutdown script is the only way left */
		data->cmd_pipe = data->uart_pipe;
		modem_cellular_begin_shutdown(data);
	} else {
		/* The UART never opened, so there is no way to reach the modem */
		modem_cellular_begin_power_off_pulse(data);
	}
}

static void modem_cellular_emit_failure(struct modem_cellular_data *data, int reason)
{
	struct cellular_evt_modem_failed evt = {
		.reason = reason,
	};

	/* Kept for the PPP device, which may resume after the event */
	data->failure_reason = reason;
	modem_cellular_emit_event(data, CELLULAR_EVENT_MODEM_FAILED, &evt);
}

/* Power off a modem left running by a fault, or leave one that cannot be reached as it is */
static void modem_cellular_stop_after_fault(struct modem_cellular_data *data)
{
	if (data->cmux_connect_failed) {
		modem_cellular_power_off_after_cmux(data);
	} else {
		modem_cellular_set_state(data, STATE_IDLE);
	}
}

/*
 * Restart the modem after a fault, backing off for longer with each attempt. Once the
 * attempt budget is spent the modem is left powered down, and the failure reported. A
 * suspend that was waiting for the failed step completes instead.
 */
static void modem_cellular_enter_recovery(struct modem_cellular_data *data)
{
	if (data->suspend_pending) {
		modem_cellular_stop_after_fault(data);
		return;
	}

	if ((CONFIG_MODEM_CELLULAR_MAX_RECOVERIES != 0) &&
	    (data->recovery_count >= CONFIG_MODEM_CELLULAR_MAX_RECOVERIES)) {
		LOG_ERR("Recovery attempts exhausted (%u), suspending", data->recovery_count);
		modem_cellular_stop_after_fault(data);
		modem_cellular_emit_failure(data, -EIO);
		return;
	}

	/* Saturate rather than wrap so the backoff stays monotonic in unlimited mode */
	if (data->recovery_count < UINT8_MAX) {
		data->recovery_count++;
	}

	modem_cellular_set_state(data, STATE_RECOVERY);
}

static void modem_cellular_connect_cmux_failed(struct modem_cellular_data *data)
{
	/* The modem may still be running in CMUX mode, which a suspend has to power off */
	data->cmux_connect_failed = true;
	modem_cellular_enter_recovery(data);
}

/* ------------------------------------------------------------------------- */
/* IDLE                                                                      */
/* ------------------------------------------------------------------------- */

static void modem_cellular_idle_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	data->recovery_count = 0;
	data->power_on_skipped = false;
	data->suspend_pending = false;
	data->cmux_connect_failed = false;
	data->cmux_bus_opened = false;
	data->power_cycle = false;
}

static enum smf_state_result modem_cellular_idle_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_RESUME:
		/* Starting over clears any failure */
		data->failure_reason = 0;
		if (config->autostarts || config->vendor->common->force_autostart) {
			modem_cellular_set_state(data, STATE_AWAIT_POWER_ON);
		} else if (modem_cellular_gpio_is_enabled(&config->reset_gpio) &&
			   config->reset_on_resume) {
			modem_cellular_set_state(data, STATE_RESET_PULSE);
		} else if (modem_cellular_gpio_is_enabled(&config->power_gpio) &&
			   !modem_cellular_modem_is_powered(config)) {
			modem_cellular_set_state(data, STATE_POWER_ON_PULSE);
		} else {
			/* Pulsing the power key of a running modem would turn it off. The
			 * init script confirms the modem responds.
			 */
			data->power_on_skipped =
				modem_cellular_gpio_is_enabled(&config->power_gpio);
			modem_cellular_enter_at_state(data);
		}
		break;
	case MODEM_CELLULAR_EVENT_SUSPEND:
		k_sem_give(&data->suspended_sem);
		break;
	default:
		break;
	}

	return SMF_EVENT_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* ACTIVE                                                                    */
/* ------------------------------------------------------------------------- */

static void modem_cellular_active_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);
	/* Without a status GPIO the modem stays powered unless a power GPIO can cycle it */
	const bool powered = modem_cellular_gpio_is_enabled(&config->status_gpio)
				     ? modem_cellular_modem_is_powered(config)
				     : !modem_cellular_gpio_is_enabled(&config->power_gpio);
	const bool baudrate_kept = data->baudrate_kept && powered &&
				   !(modem_cellular_gpio_is_enabled(&config->reset_gpio) &&
				     config->hold_reset_on_suspend);

	k_sem_take(&data->suspended_sem, K_NO_WAIT);
	data->baudrate_kept = false;

	/* A boot that skips the power-on and reset pulses would otherwise reach the
	 * baudrate script at the new speed, unless the modem was left powered at it.
	 */
	if (!baudrate_kept) {
		modem_cellular_restore_baudrate(data);
	}

	if (modem_cellular_gpio_is_enabled(&config->reset_gpio) && config->hold_reset_on_suspend) {
		gpio_pin_set_dt(&config->reset_gpio, 0);
	}

	if (modem_cellular_gpio_is_enabled(&config->wake_gpio)) {
		gpio_pin_set_dt(&config->wake_gpio, 1);
	}
}

/* The modem is powered down, or left as it is when it cannot be reached */
static void modem_cellular_active_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	if (modem_cellular_gpio_is_enabled(&config->wake_gpio)) {
		gpio_pin_set_dt(&config->wake_gpio, 0);
	}

	if (modem_cellular_gpio_is_enabled(&config->reset_gpio) && config->hold_reset_on_suspend) {
		gpio_pin_set_dt(&config->reset_gpio, 1);
	}

	modem_chat_release(&data->chat);
	modem_cmux_release(&data->cmux);
	(void)modem_pipe_close_async(data->uart_pipe);
	data->cmd_pipe = NULL;

	k_sem_give(&data->suspended_sem);
	modem_cellular_emit_event(data, CELLULAR_EVENT_MODEM_SUSPENDED, NULL);
}

/* ------------------------------------------------------------------------- */
/* BOOT                                                                      */
/* ------------------------------------------------------------------------- */

static enum smf_state_result modem_cellular_boot_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event == MODEM_CELLULAR_EVENT_SUSPEND) {
		/* The modem is not running anything worth shutting down */
		modem_cellular_set_state(data, STATE_IDLE);
		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

/* Suspend once the boot has turned the modem on, powering it off with the power key unless
 * the reset held while suspended keeps it down
 */
static void modem_cellular_suspend_booted(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = modem_cellular_config(data);
	const bool held_in_reset = modem_cellular_gpio_is_enabled(&config->reset_gpio) &&
				   config->hold_reset_on_suspend;

	if (modem_cellular_gpio_is_enabled(&config->power_gpio) && !held_in_reset) {
		modem_chat_release(&data->chat);
		modem_cellular_begin_power_off_pulse(data);
	} else {
		modem_cellular_set_state(data, STATE_IDLE);
	}
}

static void modem_cellular_reset_pulse_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	if (modem_cellular_gpio_is_enabled(&config->wake_gpio)) {
		gpio_pin_set_dt(&config->wake_gpio, 0);
	}

	modem_cellular_restore_baudrate(data);
	gpio_pin_set_dt(&config->reset_gpio, 1);
	modem_cellular_start_timer(data, K_MSEC(config->vendor->common->reset_pulse_duration_ms));
}

static enum smf_state_result modem_cellular_reset_pulse_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	if (data->smf_event != MODEM_CELLULAR_EVENT_TIMEOUT) {
		return SMF_EVENT_PROPAGATE;
	}

	if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
		modem_cellular_set_state(data, STATE_AWAIT_RESET);
	} else {
		modem_cellular_set_state(data, STATE_AWAIT_POWER_ON);
	}

	return SMF_EVENT_HANDLED;
}

static void modem_cellular_reset_pulse_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	gpio_pin_set_dt(&config->reset_gpio, 0);

	if (modem_cellular_gpio_is_enabled(&config->wake_gpio)) {
		gpio_pin_set_dt(&config->wake_gpio, 1);
	}
}

/* Wait for the modem to come out of reset, or to power off in a power cycle */
static void modem_cellular_await_reset_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_cellular_start_timer(
		data, data->power_cycle ? K_MSEC(config->vendor->common->shutdown_time_ms)
					: K_MSEC(config->vendor->common->reset_power_on_delay_ms));
}

static enum smf_state_result modem_cellular_await_reset_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event != MODEM_CELLULAR_EVENT_TIMEOUT) {
		return SMF_EVENT_PROPAGATE;
	}

	/* Only entered by modems with a power key */
	data->power_cycle = false;
	modem_cellular_set_state(data, STATE_POWER_ON_PULSE);
	return SMF_EVENT_HANDLED;
}

static void modem_cellular_power_on_pulse_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_cellular_restore_baudrate(data);
	gpio_pin_set_dt(&config->power_gpio, 1);
	modem_cellular_start_timer(data, K_MSEC(config->vendor->common->power_pulse_duration_ms));
}

static enum smf_state_result modem_cellular_power_on_pulse_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event != MODEM_CELLULAR_EVENT_TIMEOUT) {
		return SMF_EVENT_PROPAGATE;
	}

	if (data->power_cycle) {
		/* The pulse turned the modem off, wait for it before turning it back on */
		modem_cellular_set_state(data, STATE_AWAIT_RESET);
	} else {
		modem_cellular_set_state(data, STATE_AWAIT_POWER_ON);
	}
	return SMF_EVENT_HANDLED;
}

static void modem_cellular_power_on_pulse_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	gpio_pin_set_dt(&config->power_gpio, 0);
}

static void modem_cellular_await_power_on_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);
	int ret;

	modem_cellular_start_timer(data, K_MSEC(config->vendor->common->startup_time_ms));

	/* Listen for the modem announcing it is ready */
	ret = modem_cellular_open_bus(data);
	if (ret < 0) {
		LOG_ERR("Failed to open bus (%d)", ret);
	}
}

static enum smf_state_result modem_cellular_await_power_on_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		modem_chat_attach(&data->chat, data->uart_pipe);
		return SMF_EVENT_HANDLED;
	case MODEM_CELLULAR_EVENT_MODEM_READY:
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_enter_at_state(data);
		return SMF_EVENT_HANDLED;
	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_suspend_booted(data);
		return SMF_EVENT_HANDLED;
	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_set_baudrate_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	int ret;

	ret = modem_cellular_open_bus(data);
	if (ret < 0) {
		LOG_ERR("Failed to open bus (%d)", ret);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_ERROR);
	}
}

static enum smf_state_result modem_cellular_set_baudrate_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);
	uint32_t previous;

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		/* A late open from the previous state must not restart the script */
		if (data->script_running == NULL) {
			modem_chat_attach(&data->chat, data->uart_pipe);
			modem_cellular_run_script(data, config->vendor->instance->set_baudrate);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		/* Modems that save the new baudrate on the first change no longer
		 * answer at the old one, so carry on as if it is already set.
		 */
		LOG_DBG("no reply from modem, assuming baudrate is already set");
		__fallthrough;
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		modem_chat_release(&data->chat);
		if (modem_cellular_close_bus(data) < 0) {
			modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_ERROR);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_BUS_CLOSED:
		previous = modem_cellular_baudrate_update(
			data, config->vendor->instance->target_baudrate);
		if (data->original_baudrate == 0U) {
			data->original_baudrate = previous;
		}
		/* Let the modem reconfigure, timed from the close so that the init script
		 * never opens the UART before it has closed.
		 */
		modem_cellular_start_timer(
			data, K_MSEC(config->vendor->common->baudrate_update_delay_ms));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_BUS_ERROR:
		modem_cellular_enter_recovery(data);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_set_state(data, STATE_RUN_INIT_SCRIPT);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_suspend_booted(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_run_init_script_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	int ret;

	ret = modem_cellular_open_bus(data);
	if (ret < 0) {
		LOG_ERR("Failed to open bus (%d)", ret);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_ERROR);
	}
}

static enum smf_state_result modem_cellular_run_init_script_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		/* A late open from the previous state must not restart the script */
		if (data->script_running == NULL) {
			modem_chat_attach(&data->chat, data->uart_pipe);
			modem_cellular_run_script(data, config->vendor->common->scripts.init);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		/* The modem responded, so skipping the power-on pulse was right */
		data->power_on_skipped = false;
		modem_chat_release(&data->chat);
		if (modem_cellular_close_bus(data) < 0) {
			modem_cellular_enter_recovery(data);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_BUS_CLOSED:
		modem_cellular_set_state(data, STATE_CONNECT_CMUX);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
	case MODEM_CELLULAR_EVENT_BUS_ERROR:
		if (data->power_on_skipped) {
			/* The modem was assumed to be running, but it did not respond */
			data->power_on_skipped = false;
			modem_cellular_set_state(data, STATE_POWER_ON_PULSE);
		} else {
			modem_cellular_enter_recovery(data);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		/* Without a power key the modem is left powered at the init script baudrate */
		data->baudrate_kept = true;
		modem_cellular_suspend_booted(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_recovery_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	/* Growing the delay with the attempt count keeps a persistent fault from
	 * resetting the modem back-to-back, and gives a slow modem time to settle.
	 */
	uint32_t backoff_ms = MIN(CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MS * data->recovery_count,
				  CONFIG_MODEM_CELLULAR_RECOVERY_BACKOFF_MAX_MS);

	LOG_DBG("recovery attempt %u, backoff %u ms", data->recovery_count, backoff_ms);
	modem_cellular_start_timer(data, K_MSEC(backoff_ms));
}

/* Detach the CMUX so the UART pipe reattaches cleanly on the next attempt */
static void modem_cellular_detach_cmux(struct modem_cellular_data *data)
{
	modem_cmux_release(&data->cmux);
	data->cmux_connect_failed = false;
}

/*
 * Without anything to restart the modem with, boot it again as it is. A modem that kept a
 * baudrate it was switched to is found at it again by the baudrate script failing.
 */
static void modem_cellular_reboot_as_is(struct modem_cellular_data *data)
{
	modem_cellular_detach_cmux(data);
	modem_cellular_restore_baudrate(data);
	modem_cellular_enter_at_state(data);
}

static enum smf_state_result modem_cellular_recovery_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	bool running;

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		/* A fault after the CMUX connect attempt leaves the modem running */
		running = modem_cellular_gpio_is_enabled(&config->status_gpio)
				  ? modem_cellular_modem_is_powered(config)
				  : data->cmux_connect_failed;

		if (modem_cellular_gpio_is_enabled(&config->reset_gpio) &&
		    config->reset_on_recovery) {
			modem_cellular_detach_cmux(data);
			modem_cellular_set_state(data, STATE_RESET_PULSE);
		} else if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
			modem_cellular_detach_cmux(data);
			/* The power key turns a running modem off, so pulse it twice */
			data->power_cycle = running;
			modem_cellular_set_state(data, STATE_POWER_ON_PULSE);
		} else if (data->cmux_connect_failed) {
			/* A modem left in CMUX mode would not answer the AT commands of the boot */
			modem_cellular_set_state(data, STATE_RECOVERY_CMUX_DISCONNECT);
		} else {
			modem_cellular_reboot_as_is(data);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		if (data->cmux_connect_failed) {
			modem_cellular_power_off_after_cmux(data);
			return SMF_EVENT_HANDLED;
		}
		/* An unresponsive modem is left as it is */
		return SMF_EVENT_PROPAGATE;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_recovery_cmux_disconnect_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (modem_cmux_disconnect_async(&data->cmux) == 0) {
		modem_cellular_start_timer(data, MODEM_CELLULAR_CMUX_DISCONNECT_TIMEOUT);
	} else {
		/* Not connected, so nothing to disconnect */
		modem_cellular_start_timer(data, K_NO_WAIT);
	}
}

static enum smf_state_result modem_cellular_recovery_cmux_disconnect_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED:
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_reboot_as_is(data);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		/* The modem may still be in CMUX mode, as in RECOVERY */
		modem_cellular_power_off_after_cmux(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_connect_cmux_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	data->suspend_pending = false;
	data->cmux_connect_failed = false;
	data->cmux_bus_opened = false;
	data->cmd_pipe = NULL;

	/* Some modems disable UART RX while switching to CMUX, which would show up as
	 * receive errors on the bus.
	 */
	modem_cellular_start_timer(data, MODEM_CELLULAR_CMUX_SWITCH_DELAY);
}

static enum smf_state_result modem_cellular_connect_cmux_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	int ret;

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		ret = modem_cellular_open_bus(data);
		if (ret < 0) {
			LOG_ERR("Failed to open bus (%d)", ret);
			modem_cellular_connect_cmux_failed(data);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		data->cmux_bus_opened = true;
		ret = modem_cmux_attach(&data->cmux, data->uart_pipe);
		if (ret == 0) {
			ret = modem_cmux_connect_async(&data->cmux);
		}
		if (ret < 0) {
			LOG_ERR("Failed to start CMUX connect (%d)", ret);
			modem_cellular_connect_cmux_failed(data);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_CMUX_CONNECTED:
		if (data->suspend_pending) {
			modem_cellular_set_state(data, STATE_CMUX_DISCONNECT);
		} else {
			modem_cellular_set_state(data, STATE_OPEN_DLCI1);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED:
		modem_cellular_connect_cmux_failed(data);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		/* No channel for the shutdown script until the connect attempt settles */
		data->suspend_pending = true;
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_emit_data_context(struct modem_cellular_data *data, bool active)
{
	struct cellular_evt_data_context evt = {
		.active = active,
	};

	modem_cellular_emit_event(data, CELLULAR_EVENT_DATA_CONTEXT_CHANGED, &evt);
}

/*
 * Without the LTE RF or the AT command channel any registration is lost, and no report
 * will arrive to say so.
 */
static void modem_cellular_forget_registration(struct modem_cellular_data *data)
{
	bool reported = (data->registration_status_gsm != CELLULAR_REGISTRATION_NOT_REGISTERED) ||
			(data->registration_status_gprs != CELLULAR_REGISTRATION_NOT_REGISTERED) ||
			(data->registration_status_lte != CELLULAR_REGISTRATION_NOT_REGISTERED) ||
			(data->registration_status_5g != CELLULAR_REGISTRATION_NOT_REGISTERED);
	bool pdn_active = data->pdn_active;

	/* Also forgets the data context */
	modem_cellular_clear_registration_status(data);
	if (reported) {
		modem_cellular_invalidate_network_status(data);
		modem_cellular_emit_reg_state(data, CELLULAR_REGISTRATION_NOT_REGISTERED);
	}
	if (pdn_active) {
		modem_cellular_emit_data_context(data, false);
	}
}

/* ------------------------------------------------------------------------- */
/* CMUX_CONNECTED                                                            */
/* ------------------------------------------------------------------------- */

static enum smf_state_result modem_cellular_cmux_connected_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_set_state(data, STATE_CMUX_DISCONNECT);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED:
		LOG_WRN("CMUX disconnected");
		data->cmd_pipe = NULL;
		/* The modem is still running and has to be powered off on suspend */
		data->cmux_connect_failed = true;
		modem_cellular_enter_recovery(data);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_DLCI1_CLOSED:
		LOG_WRN("DLCI1 closed");
		data->cmd_pipe = NULL;
		data->cmux_connect_failed = true;
		modem_cellular_enter_recovery(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_cmux_connected_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	/* The command pipe is kept, as the shutdown script falls back to it when the
	 * CMUX does not disconnect.
	 */
	modem_chat_release(&data->chat);
	modem_cellular_forget_registration(data);
}

static void modem_cellular_open_dlci1_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	int ret;

	modem_pipe_attach(data->dlci1_pipe, modem_cellular_dlci1_pipe_handler, data);
	ret = modem_pipe_open_async(data->dlci1_pipe);
	if (ret < 0) {
		LOG_ERR("Failed to open DLCI1 (%d)", ret);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_DLCI1_CLOSED);
	}
}

static void modem_cellular_dlci_setup_complete(struct modem_cellular_data *data)
{
	modem_cellular_set_state(data, STATE_ENABLE_REPORTS);
}

static enum smf_state_result modem_cellular_open_dlci1_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event == MODEM_CELLULAR_EVENT_DLCI1_CLOSED) {
		LOG_ERR("Failed to open DLCI1");
		data->cmux_connect_failed = true;
		modem_cellular_enter_recovery(data);
		return SMF_EVENT_HANDLED;
	}

	if (data->smf_event != MODEM_CELLULAR_EVENT_DLCI1_OPENED) {
		return SMF_EVENT_PROPAGATE;
	}

	data->cmd_pipe = data->dlci1_pipe;
	modem_chat_attach(&data->chat, data->cmd_pipe);

	if (modem_cellular_scripts(data)->dlci_setup != NULL) {
		modem_cellular_set_state(data, STATE_RUN_DLCI_SETUP);
	} else {
		modem_cellular_dlci_setup_complete(data);
	}
	return SMF_EVENT_HANDLED;
}

static void modem_cellular_run_dlci_setup_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_cellular_run_script(data, modem_cellular_scripts(data)->dlci_setup);
}

static enum smf_state_result modem_cellular_run_dlci_setup_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		/* Unlikely to be critical, the channel still takes commands */
		LOG_WRN("DLCI1 setup failed");
		__fallthrough;
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		modem_cellular_dlci_setup_complete(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_chat_on_cmee_error(struct modem_chat *chat, char **argv,
					      uint16_t argc, void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(argv);
	ARG_UNUSED(argc);
	ARG_UNUSED(user_data);

	LOG_WRN("Failed to enable +CME ERROR codes, SIM errors cannot be told apart");
}

static void modem_cellular_chat_on_cereg_error(struct modem_chat *chat, char **argv,
					       uint16_t argc, void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(argv);
	ARG_UNUSED(argc);
	ARG_UNUSED(user_data);

	LOG_WRN("Failed to enable +CEREG reports, LTE registration changes go unnoticed");
}

static void modem_cellular_chat_on_cgerep_error(struct modem_chat *chat, char **argv,
						uint16_t argc, void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(argv);
	ARG_UNUSED(argc);
	ARG_UNUSED(user_data);

	LOG_WRN("Failed to enable +CGEV reports, a lost data context only ends a call once PPP "
		"notices");
}

/* Each command completes on an error, so that a modem lacking one still gets the others */
MODEM_CHAT_MATCHES_DEFINE(modem_cellular_cmee_matches, MODEM_CHAT_MATCH("OK", "", NULL),
			  MODEM_CHAT_MATCH("ERROR", "", modem_cellular_chat_on_cmee_error),
			  MODEM_CHAT_MATCH("+CME ERROR", "", modem_cellular_chat_on_cmee_error));
MODEM_CHAT_MATCHES_DEFINE(modem_cellular_cereg_matches, MODEM_CHAT_MATCH("OK", "", NULL),
			  MODEM_CHAT_MATCH("ERROR", "", modem_cellular_chat_on_cereg_error),
			  MODEM_CHAT_MATCH("+CME ERROR", "", modem_cellular_chat_on_cereg_error));
MODEM_CHAT_MATCHES_DEFINE(modem_cellular_cgerep_matches, MODEM_CHAT_MATCH("OK", "", NULL),
			  MODEM_CHAT_MATCH("ERROR", "", modem_cellular_chat_on_cgerep_error),
			  MODEM_CHAT_MATCH("+CME ERROR", "", modem_cellular_chat_on_cgerep_error));

/*
 * Reports are enabled per channel on some modems, so they are enabled on the AT command
 * channel once it is set up. Numeric +CME ERROR codes come first, as the SIM status is read
 * from them, and so that the commands after report why they fail. <n> 3 reports the access
 * technology along with the registration status, and the EMM cause when registration fails.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(
	modem_cellular_enable_reports_script_cmds,
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CMEE=1", modem_cellular_cmee_matches),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CEREG=3", modem_cellular_cereg_matches),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGEREP=1", modem_cellular_cgerep_matches));

MODEM_CHAT_SCRIPT_NO_ABORT_DEFINE(modem_cellular_enable_reports_script,
				  modem_cellular_enable_reports_script_cmds,
				  modem_cellular_chat_callback_handler, 5);

static void modem_cellular_enable_reports_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_cellular_run_script(data, &modem_cellular_enable_reports_script);
}

static enum smf_state_result modem_cellular_enable_reports_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		/* The modem did not answer, which the SIM query that follows also finds out */
		LOG_WRN("Failed to enable reports");
		__fallthrough;
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		modem_cellular_set_state(data, STATE_AWAIT_SIM);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

/*
 * The SIM status is known from a +CPIN report, or from a +CME ERROR code that will not clear
 * by waiting. Anything else, such as a bare ERROR or "SIM busy", leaves it unknown, and the
 * SIM is queried again until the SIM timeout passes.
 */
static void modem_cellular_chat_on_cpin(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = user_data;
	const char *code;
	size_t len;

	ARG_UNUSED(chat);

	if (argc != 2) {
		return;
	}

	/* Some modems quote the code */
	code = argv[1];
	len = strlen(code);
	if ((len >= 2U) && (code[0] == '"') && (code[len - 1U] == '"')) {
		code++;
		len -= 2U;
	}

	if ((len == strlen("NOT READY")) && (strncmp(code, "NOT READY", len) == 0)) {
		return;
	}

	if ((len == strlen("READY")) && (strncmp(code, "READY", len) == 0)) {
		data->sim_status = CELLULAR_SIM_STATUS_READY;
	} else if ((len == strlen("SIM PIN")) && (strncmp(code, "SIM PIN", len) == 0)) {
		data->sim_status = CELLULAR_SIM_STATUS_PIN_REQUIRED;
	} else if ((len == strlen("SIM PUK")) && (strncmp(code, "SIM PUK", len) == 0)) {
		data->sim_status = CELLULAR_SIM_STATUS_PUK_REQUIRED;
	} else {
		data->sim_status = CELLULAR_SIM_STATUS_LOCKED;
	}
	data->sim_status_known = true;
}

/* 3GPP TS 27.007 numeric +CME ERROR codes for the SIM */
#define MODEM_CELLULAR_CME_SIM_NOT_INSERTED 10
#define MODEM_CELLULAR_CME_SIM_PIN_REQUIRED 11
#define MODEM_CELLULAR_CME_SIM_PUK_REQUIRED 12
#define MODEM_CELLULAR_CME_SIM_FAILURE      13
#define MODEM_CELLULAR_CME_SIM_WRONG        15

static void modem_cellular_chat_on_cme_error(struct modem_chat *chat, char **argv,
					     uint16_t argc, void *user_data)
{
	struct modem_cellular_data *data = user_data;

	ARG_UNUSED(chat);

	if (argc != 2) {
		return;
	}

	switch (strtol(argv[1], NULL, 10)) {
	case MODEM_CELLULAR_CME_SIM_NOT_INSERTED:
		data->sim_status = CELLULAR_SIM_STATUS_NOT_INSERTED;
		break;
	case MODEM_CELLULAR_CME_SIM_PIN_REQUIRED:
		data->sim_status = CELLULAR_SIM_STATUS_PIN_REQUIRED;
		break;
	case MODEM_CELLULAR_CME_SIM_PUK_REQUIRED:
		data->sim_status = CELLULAR_SIM_STATUS_PUK_REQUIRED;
		break;
	case MODEM_CELLULAR_CME_SIM_FAILURE:
	case MODEM_CELLULAR_CME_SIM_WRONG:
		data->sim_status = CELLULAR_SIM_STATUS_FAILURE;
		break;
	default:
		return;
	}
	data->sim_status_known = true;
}

/*
 * Errors are expected while the SIM starts up, e.g. +CME ERROR: 14 for SIM busy, so they
 * complete the query rather than abort it, which the chat would log as a warning. The SIM
 * status is known from the answer, or polled for again.
 */
MODEM_CHAT_MATCHES_DEFINE(modem_cellular_cpin_matches,
			  MODEM_CHAT_MATCH_INITIALIZER("+CPIN: ", "", modem_cellular_chat_on_cpin,
						       false, true),
			  MODEM_CHAT_MATCH("OK", "", NULL), MODEM_CHAT_MATCH("ERROR", "", NULL),
			  MODEM_CHAT_MATCH("+CME ERROR: ", "", modem_cellular_chat_on_cme_error));
MODEM_CHAT_SCRIPT_CMDS_DEFINE(modem_cellular_cpin_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CPIN?",
							      modem_cellular_cpin_matches));
MODEM_CHAT_SCRIPT_NO_ABORT_DEFINE(modem_cellular_cpin_script, modem_cellular_cpin_script_cmds,
				  modem_cellular_chat_callback_handler, 2);

/* Report the SIM status, and carry on bringing the modem up whatever it is */
static void modem_cellular_sim_complete(struct modem_cellular_data *data,
					enum cellular_sim_status status)
{
	struct cellular_evt_sim_status evt = {
		.status = status,
	};

	if (status != CELLULAR_SIM_STATUS_READY) {
		LOG_WRN("SIM status %d", status);
	}
	modem_cellular_emit_event(data, CELLULAR_EVENT_SIM_STATUS_CHANGED, &evt);
	modem_cellular_set_state(data, modem_cellular_board_init_script(data) != NULL
					       ? STATE_RUN_BOARD_INIT
					       : STATE_LTE_RF_OFF);
}

static void modem_cellular_query_sim(struct modem_cellular_data *data)
{
	data->sim_status_known = false;
	modem_cellular_run_script(data, &modem_cellular_cpin_script);
}

static void modem_cellular_await_sim_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	data->sim_deadline = sys_timepoint_calc(K_MSEC(CONFIG_MODEM_CELLULAR_SIM_TIMEOUT_MS));
	modem_cellular_query_sim(data);
}

static enum smf_state_result modem_cellular_await_sim_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		if (!data->sim_status_known) {
			if (sys_timepoint_expired(data->sim_deadline)) {
				modem_cellular_sim_complete(data, CELLULAR_SIM_STATUS_TIMEOUT);
			} else {
				/* No result is awaited until the next query */
				data->script_running = NULL;
				modem_cellular_start_timer(data, MODEM_CELLULAR_SIM_POLL_INTERVAL);
			}
		} else if ((data->sim_status == CELLULAR_SIM_STATUS_READY) &&
			   (modem_cellular_scripts(data)->sim_ready != NULL)) {
			modem_cellular_set_state(data, STATE_RUN_SIM_READY);
		} else {
			modem_cellular_sim_complete(data, data->sim_status);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_query_sim(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_run_sim_ready_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_cellular_run_script(data, modem_cellular_scripts(data)->sim_ready);
}

static enum smf_state_result modem_cellular_run_sim_ready_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		/* The SIM is still ready, only what the script reads is missing */
		LOG_WRN("SIM ready script failed");
		__fallthrough;
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		modem_cellular_sim_complete(data, CELLULAR_SIM_STATUS_READY);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static const struct modem_chat_script *
modem_cellular_board_init_script(const struct modem_cellular_data *data)
{
	STRUCT_SECTION_FOREACH(modem_cellular_board_init, board_init) {
		if (board_init->dev == data->dev) {
			return board_init->script;
		}
	}

	return NULL;
}

static void modem_cellular_run_board_init_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_chat_script *script = modem_cellular_board_init_script(data);

	__ASSERT_NO_MSG(script != NULL);
	data->board_init_script = *script;
	modem_chat_script_set_callback(&data->board_init_script,
				       modem_cellular_chat_callback_handler);
	/* Failures of the periodic script before a recovery or suspend do not count */
	data->script_failure_counter = 0;
	modem_cellular_start_timer(data, MODEM_CELLULAR_BOARD_INIT_DELAY);
}

static enum smf_state_result modem_cellular_run_board_init_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_run_script(data, &data->board_init_script);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		data->script_failure_counter = 0;
		modem_cellular_set_state(data, STATE_LTE_RF_OFF);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		if (data->script_failure_counter < UINT8_MAX) {
			data->script_failure_counter++;
		}
		if (data->script_failure_counter >= CONFIG_MODEM_CELLULAR_MAX_SCRIPT_FAILURES) {
			LOG_ERR("Board init script failed %u times, suspending",
				data->script_failure_counter);
			data->script_failure_counter = 0;
			modem_cellular_set_state(data, STATE_CMUX_DISCONNECT);
			modem_cellular_emit_failure(data, -EIO);
		} else {
			modem_cellular_start_timer(
				data, K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS));
		}
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static bool modem_cellular_has_periodic_script(const struct modem_cellular_config *config)
{
	return config->vendor->common->scripts.periodic != NULL;
}

static void modem_cellular_periodic_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_cellular_data *data =
		CONTAINER_OF(dwork, struct modem_cellular_data, periodic_work);

	modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PERIODIC);
}

static void modem_cellular_schedule_periodic(struct modem_cellular_data *data)
{
	modem_work_reschedule(&data->periodic_work,
			      K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS));
}

static void modem_cellular_ready_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	data->script_failure_counter = 0;
	data->periodic_pending = false;
	if (modem_cellular_has_periodic_script(modem_cellular_config(data))) {
		/* The fault driving recovery has cleared once the periodic script succeeds */
		modem_cellular_schedule_periodic(data);
	} else {
		/* Nothing left to check the modem with */
		data->recovery_count = 0;
	}
	/* Only hand out the other channels once the AT command channel is ready for commands */
	modem_cellular_notify_pipelinks_connected(data);
	modem_cellular_emit_event(data, CELLULAR_EVENT_MODEM_READY, NULL);
}

/*
 * Power down once ready, disabling the LTE RF first if it may be enabled, which detaches the
 * modem from the network rather than leaving it to the shutdown script or the power key
 */
static void modem_cellular_ready_suspend(struct modem_cellular_data *data)
{
	modem_cellular_set_state(data, data->lte_rf_enabled ? STATE_LTE_RF_DISABLE
							  : STATE_CMUX_DISCONNECT);
}

static enum smf_state_result modem_cellular_ready_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SUSPEND:
		/* A running script is let complete, the LTE RF settling, which acts on it */
		data->suspend_pending = true;
		if (data->script_running == NULL) {
			modem_cellular_ready_suspend(data);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_PERIODIC:
		/* Due while an LTE RF script runs, so held until it completes */
		data->periodic_pending = true;
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_PERIODIC_RESUMED:
		/* Acted on once the LTE RF settles */
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_ready_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	(void)k_work_cancel_delayable(&data->periodic_work);
	data->periodic_skipped = false;
	data->periodic_pending = false;
	modem_cellular_notify_pipelinks_disconnected(data);
}

/*
 * Run what became due while the LTE RF was changing, now that it has settled, or the suspend
 * that was held back for it
 */
static void modem_cellular_periodic_settled(struct modem_cellular_data *data)
{
	if (data->suspend_pending) {
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PM);
		return;
	}

	if (data->periodic_pending) {
		data->periodic_pending = false;
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PERIODIC);
	} else if (data->periodic_skipped && !atomic_get(&data->periodic_paused)) {
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PERIODIC_RESUMED);
	}
}

/* Periodic script events of the states the LTE RF is settled in */
static enum smf_state_result modem_cellular_periodic_run(struct modem_cellular_data *data)
{
	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_PERIODIC:
		if (atomic_get(&data->periodic_paused)) {
			/* Run once resumed, the timer restarting after it */
			data->periodic_skipped = true;
		} else {
			modem_cellular_set_state(data, STATE_RUN_PERIODIC);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_PERIODIC_RESUMED:
		if (data->periodic_skipped) {
			data->periodic_skipped = false;
			modem_cellular_set_state(data, STATE_RUN_PERIODIC);
		} else {
			modem_cellular_schedule_periodic(data);
		}
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static bool modem_cellular_lte_rf_requested(struct modem_cellular_data *data)
{
	return atomic_get(&data->lte_rf_refs) > 0;
}

/* Requests are only reported when they change, so act on those held on entry as if changed */
static void modem_cellular_lte_rf_check_requests(struct modem_cellular_data *data,
						 bool act_when_requested)
{
	if (modem_cellular_lte_rf_requested(data) == act_when_requested) {
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_LTE_RF);
	}
}

static void modem_cellular_lte_rf_off_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	data->lte_rf_enabled = false;
	modem_cellular_forget_registration(data);
	modem_cellular_lte_rf_check_requests(data, true);
	modem_cellular_periodic_settled(data);
}

static enum smf_state_result modem_cellular_lte_rf_off_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event != MODEM_CELLULAR_EVENT_LTE_RF_CHANGED) {
		return modem_cellular_periodic_run(data);
	}

	if (modem_cellular_lte_rf_requested(data)) {
		modem_cellular_set_state(data, modem_cellular_config(data)->use_default_pdp_context
						       ? STATE_LTE_RF_ENABLE
						       : STATE_CONFIGURE_APN);
	}
	return SMF_EVENT_HANDLED;
}

MODEM_CHAT_MATCHES_DEFINE(apn_abort_matches, MODEM_CHAT_MATCH("ERROR", "", NULL),
			  MODEM_CHAT_MATCH("+CME ERROR", "", NULL));

/* Whether the data context was configured with an APN that has since been replaced */
static bool modem_cellular_apn_changed(struct modem_cellular_data *data)
{
	return !modem_cellular_config(data)->use_default_pdp_context &&
	       atomic_get(&data->apn_changed);
}

/* The data context is configured while the LTE RF is disabled, as some modems require */
static void modem_cellular_configure_apn_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	/* The APN cannot change while the LTE RF is requested, the lock orders the read */
	k_mutex_lock(&data->api_lock, K_FOREVER);
	modem_cellular_build_apn_script(data);
	atomic_clear(&data->apn_changed);
	k_mutex_unlock(&data->api_lock);

	modem_cellular_run_script(data, &data->apn_script);
}

static enum smf_state_result modem_cellular_configure_apn_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		if (data->suspend_pending) {
			/* The LTE RF is still disabled, so there is no point enabling it */
			modem_cellular_ready_suspend(data);
		} else if (!modem_cellular_lte_rf_requested(data)) {
			modem_cellular_set_state(data, STATE_LTE_RF_OFF);
		} else if (modem_cellular_apn_changed(data)) {
			/* Released and requested again with another APN while configuring */
			modem_cellular_set_state(data, STATE_CONFIGURE_APN);
		} else {
			modem_cellular_set_state(data, STATE_LTE_RF_ENABLE);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		LOG_WRN("APN configuration failed");
		if (data->suspend_pending) {
			modem_cellular_ready_suspend(data);
			return SMF_EVENT_HANDLED;
		}
		/* No result is awaited until the retry, so a suspend is acted on at once */
		data->script_running = NULL;
		modem_cellular_start_timer(data, K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		if (modem_cellular_lte_rf_requested(data)) {
			modem_cellular_set_state(data, STATE_CONFIGURE_APN);
		} else {
			modem_cellular_set_state(data, STATE_LTE_RF_OFF);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_LTE_RF_CHANGED:
		/* Acted on once the script completes */
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_lte_rf_enable_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_cellular_run_script(data, modem_cellular_scripts(data)->lte_rf_enable);
}

static enum smf_state_result modem_cellular_lte_rf_enable_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		modem_cellular_set_state(data, STATE_LTE_RF_ON);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		LOG_WRN("LTE RF enable failed");
		if (data->suspend_pending) {
			/* Taken as still disabled, as in the retry below */
			modem_cellular_ready_suspend(data);
			return SMF_EVENT_HANDLED;
		}
		/* No result is awaited until the retry, so a suspend is acted on at once */
		data->script_running = NULL;
		modem_cellular_start_timer(data, K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		/* The LTE RF is still disabled, so only retry while it is requested */
		if (modem_cellular_lte_rf_requested(data)) {
			modem_cellular_set_state(data, STATE_LTE_RF_ENABLE);
		} else {
			modem_cellular_set_state(data, STATE_LTE_RF_OFF);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_LTE_RF_CHANGED:
		/* Acted on once the script completes */
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_lte_rf_on_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	data->lte_rf_enabled = true;
	if (modem_cellular_apn_changed(data)) {
		/* Released and requested again with another APN while enabling */
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_LTE_RF);
	} else {
		modem_cellular_lte_rf_check_requests(data, false);
	}
	modem_cellular_periodic_settled(data);
}

static enum smf_state_result modem_cellular_lte_rf_on_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event != MODEM_CELLULAR_EVENT_LTE_RF_CHANGED) {
		return modem_cellular_periodic_run(data);
	}

	/* A request released and made again with another APN, before the release was seen,
	 * also takes the LTE RF down so that the data context can be configured with it
	 */
	if (!modem_cellular_lte_rf_requested(data) || modem_cellular_apn_changed(data)) {
		modem_cellular_set_state(data, STATE_LTE_RF_DISABLE);
	}
	return SMF_EVENT_HANDLED;
}

/* The state the LTE RF is settled in, to return to after a periodic script run */
static enum modem_cellular_v2_core_state
modem_cellular_lte_rf_settled_state(const struct modem_cellular_data *data)
{
	return data->lte_rf_enabled ? STATE_LTE_RF_ON : STATE_LTE_RF_OFF;
}

static void modem_cellular_run_periodic_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_cellular_run_script(data, modem_cellular_scripts(data)->periodic);
}

static void modem_cellular_emit_comms_check(struct modem_cellular_data *data, bool success)
{
	struct cellular_evt_modem_comms_check_result evt = {
		.success = success,
	};

	modem_cellular_emit_event(data, CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT, &evt);
}

static enum smf_state_result modem_cellular_run_periodic_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		data->script_failure_counter = 0;
		data->recovery_count = 0;
		modem_cellular_emit_comms_check(data, true);
		modem_cellular_schedule_periodic(data);
		modem_cellular_set_state(data, modem_cellular_lte_rf_settled_state(data));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		modem_cellular_emit_comms_check(data, false);
		if (data->script_failure_counter < UINT8_MAX) {
			data->script_failure_counter++;
		}
		if (data->script_failure_counter >= CONFIG_MODEM_CELLULAR_MAX_SCRIPT_FAILURES) {
			LOG_WRN("Periodic script failed %u times, recovering",
				data->script_failure_counter);
			/* The modem is still running, likely in CMUX mode */
			data->cmux_connect_failed = true;
			modem_cellular_enter_recovery(data);
			return SMF_EVENT_HANDLED;
		}
		modem_cellular_schedule_periodic(data);
		modem_cellular_set_state(data, modem_cellular_lte_rf_settled_state(data));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_LTE_RF_CHANGED:
	case MODEM_CELLULAR_EVENT_PERIODIC_RESUMED:
		/* Acted on once the script completes */
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_lte_rf_disable_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_cellular_run_script(data, modem_cellular_scripts(data)->lte_rf_disable);
}

static enum smf_state_result modem_cellular_lte_rf_disable_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		if (data->suspend_pending) {
			modem_cellular_set_state(data, STATE_CMUX_DISCONNECT);
		} else {
			modem_cellular_set_state(data, STATE_LTE_RF_OFF);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		LOG_WRN("LTE RF disable failed");
		if (data->suspend_pending) {
			modem_cellular_set_state(data, STATE_CMUX_DISCONNECT);
			return SMF_EVENT_HANDLED;
		}
		/* No result is awaited until the retry */
		data->script_running = NULL;
		modem_cellular_start_timer(data, K_MSEC(CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		if (data->script_running == NULL) {
			/* Waiting to retry, so there is nothing to finish first */
			return SMF_EVENT_PROPAGATE;
		}
		/* Let the modem detach from the network before it is powered down */
		data->suspend_pending = true;
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		/* The LTE RF is still enabled, so only retry while it is not requested */
		if (modem_cellular_lte_rf_requested(data)) {
			modem_cellular_set_state(data, STATE_LTE_RF_ON);
		} else {
			modem_cellular_set_state(data, STATE_LTE_RF_DISABLE);
		}
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_LTE_RF_CHANGED:
		/* Acted on once the script completes */
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

/* ------------------------------------------------------------------------- */
/* POWER_DOWN                                                                */
/* ------------------------------------------------------------------------- */

static enum smf_state_result modem_cellular_power_down_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	/* Already powering down, the suspend completes once idle is reached */
	if (data->smf_event == MODEM_CELLULAR_EVENT_SUSPEND) {
		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

static void modem_cellular_cmux_disconnect_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	(void)modem_cmux_disconnect_async(&data->cmux);
	modem_cellular_start_timer(data, MODEM_CELLULAR_CMUX_DISCONNECT_TIMEOUT);
}

static enum smf_state_result modem_cellular_cmux_disconnect_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);
	uint16_t timeout_ms;

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED:
		data->cmd_pipe = data->uart_pipe;
		/* Listen on the UART for the modem returning to AT mode */
		modem_chat_attach(&data->chat, data->cmd_pipe);
		timeout_ms = (config->vendor->common->cmux_disconnect_timeout_ms > 0U)
				     ? config->vendor->common->cmux_disconnect_timeout_ms
				     : config->vendor->common->reset_pulse_duration_ms;
		modem_cellular_start_timer(data, K_MSEC(timeout_ms));
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_MODEM_READY:
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		if (data->cmd_pipe == data->uart_pipe) {
			modem_chat_release(&data->chat);
		}
		modem_cellular_begin_shutdown(data);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_run_shutdown_script_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_chat_attach(&data->chat, data->cmd_pipe);
	modem_cellular_run_script(data, config->vendor->common->scripts.shutdown);
}

static enum smf_state_result modem_cellular_run_shutdown_script_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	switch (data->smf_event) {
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		/* Fall back to the power key, if there is one */
		data->cmd_pipe = NULL;
		modem_cellular_begin_power_off_pulse(data);
		return SMF_EVENT_HANDLED;

	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		(void)modem_pipe_close_async(data->uart_pipe);
		data->cmd_pipe = NULL;
		modem_cellular_set_state(data, STATE_IDLE);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_run_shutdown_script_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	modem_chat_release(&data->chat);
}

static void modem_cellular_power_off_pulse_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	data->cmd_pipe = NULL;
	gpio_pin_set_dt(&config->power_gpio, 1);
	modem_cellular_start_timer(data, K_MSEC(config->vendor->common->power_pulse_duration_ms));
}

static enum smf_state_result modem_cellular_power_off_pulse_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event != MODEM_CELLULAR_EVENT_TIMEOUT) {
		return SMF_EVENT_PROPAGATE;
	}

	modem_cellular_set_state(data, STATE_AWAIT_POWER_OFF);
	return SMF_EVENT_HANDLED;
}

static void modem_cellular_power_off_pulse_exit(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	gpio_pin_set_dt(&config->power_gpio, 0);
}

static void modem_cellular_await_power_off_entry(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);
	const struct modem_cellular_config *config = modem_cellular_config(data);

	modem_cellular_start_timer(data, K_MSEC(config->vendor->common->shutdown_time_ms));
}

static enum smf_state_result modem_cellular_await_power_off_run(void *obj)
{
	struct modem_cellular_data *data = modem_cellular_data_from_obj(obj);

	if (data->smf_event != MODEM_CELLULAR_EVENT_TIMEOUT) {
		return SMF_EVENT_PROPAGATE;
	}

	modem_cellular_set_state(data, STATE_IDLE);
	return SMF_EVENT_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* State machine                                                             */
/* ------------------------------------------------------------------------- */

#define MODEM_CELLULAR_STATE(_entry, _run, _exit, _parent)                                         \
	SMF_CREATE_STATE(_entry, _run, _exit, _parent, NULL)

#define STATE_PARENT(_state) (&modem_cellular_states[_state])

static const struct smf_state modem_cellular_states[STATE_COUNT] = {
	[STATE_IDLE] = MODEM_CELLULAR_STATE(modem_cellular_idle_entry, modem_cellular_idle_run,
					    NULL, NULL),
	[STATE_ACTIVE] = MODEM_CELLULAR_STATE(modem_cellular_active_entry, NULL,
					      modem_cellular_active_exit, NULL),
	[STATE_BOOT] = MODEM_CELLULAR_STATE(NULL, modem_cellular_boot_run, NULL,
					    STATE_PARENT(STATE_ACTIVE)),
	[STATE_RESET_PULSE] = MODEM_CELLULAR_STATE(
		modem_cellular_reset_pulse_entry, modem_cellular_reset_pulse_run,
		modem_cellular_reset_pulse_exit, STATE_PARENT(STATE_BOOT)),
	[STATE_AWAIT_RESET] = MODEM_CELLULAR_STATE(modem_cellular_await_reset_entry,
						   modem_cellular_await_reset_run, NULL,
						   STATE_PARENT(STATE_BOOT)),
	[STATE_POWER_ON_PULSE] = MODEM_CELLULAR_STATE(
		modem_cellular_power_on_pulse_entry, modem_cellular_power_on_pulse_run,
		modem_cellular_power_on_pulse_exit, STATE_PARENT(STATE_BOOT)),
	[STATE_AWAIT_POWER_ON] = MODEM_CELLULAR_STATE(modem_cellular_await_power_on_entry,
						      modem_cellular_await_power_on_run, NULL,
						      STATE_PARENT(STATE_BOOT)),
	[STATE_SET_BAUDRATE] = MODEM_CELLULAR_STATE(modem_cellular_set_baudrate_entry,
						    modem_cellular_set_baudrate_run, NULL,
						    STATE_PARENT(STATE_BOOT)),
	[STATE_RUN_INIT_SCRIPT] = MODEM_CELLULAR_STATE(modem_cellular_run_init_script_entry,
						       modem_cellular_run_init_script_run, NULL,
						       STATE_PARENT(STATE_BOOT)),
	[STATE_RECOVERY] =
		MODEM_CELLULAR_STATE(modem_cellular_recovery_entry, modem_cellular_recovery_run,
				     NULL, STATE_PARENT(STATE_BOOT)),
	[STATE_RECOVERY_CMUX_DISCONNECT] = MODEM_CELLULAR_STATE(
		modem_cellular_recovery_cmux_disconnect_entry,
		modem_cellular_recovery_cmux_disconnect_run, NULL, STATE_PARENT(STATE_BOOT)),
	[STATE_CONNECT_CMUX] = MODEM_CELLULAR_STATE(modem_cellular_connect_cmux_entry,
						    modem_cellular_connect_cmux_run, NULL,
						    STATE_PARENT(STATE_BOOT)),
	[STATE_CMUX_CONNECTED] = MODEM_CELLULAR_STATE(NULL, modem_cellular_cmux_connected_run,
						      modem_cellular_cmux_connected_exit,
						      STATE_PARENT(STATE_ACTIVE)),
	[STATE_OPEN_DLCI1] =
		MODEM_CELLULAR_STATE(modem_cellular_open_dlci1_entry, modem_cellular_open_dlci1_run,
				     NULL, STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_RUN_DLCI_SETUP] = MODEM_CELLULAR_STATE(modem_cellular_run_dlci_setup_entry,
						      modem_cellular_run_dlci_setup_run, NULL,
						      STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_ENABLE_REPORTS] = MODEM_CELLULAR_STATE(modem_cellular_enable_reports_entry,
						      modem_cellular_enable_reports_run, NULL,
						      STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_AWAIT_SIM] = MODEM_CELLULAR_STATE(modem_cellular_await_sim_entry,
						 modem_cellular_await_sim_run, NULL,
						 STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_RUN_SIM_READY] = MODEM_CELLULAR_STATE(modem_cellular_run_sim_ready_entry,
						     modem_cellular_run_sim_ready_run, NULL,
						     STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_RUN_BOARD_INIT] = MODEM_CELLULAR_STATE(modem_cellular_run_board_init_entry,
						      modem_cellular_run_board_init_run, NULL,
						      STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_READY] =
		MODEM_CELLULAR_STATE(modem_cellular_ready_entry, modem_cellular_ready_run,
				     modem_cellular_ready_exit, STATE_PARENT(STATE_CMUX_CONNECTED)),
	[STATE_LTE_RF_OFF] =
		MODEM_CELLULAR_STATE(modem_cellular_lte_rf_off_entry, modem_cellular_lte_rf_off_run,
				     NULL, STATE_PARENT(STATE_READY)),
	[STATE_CONFIGURE_APN] = MODEM_CELLULAR_STATE(modem_cellular_configure_apn_entry,
						     modem_cellular_configure_apn_run, NULL,
						     STATE_PARENT(STATE_READY)),
	[STATE_LTE_RF_ENABLE] = MODEM_CELLULAR_STATE(modem_cellular_lte_rf_enable_entry,
						     modem_cellular_lte_rf_enable_run, NULL,
						     STATE_PARENT(STATE_READY)),
	[STATE_LTE_RF_ON] =
		MODEM_CELLULAR_STATE(modem_cellular_lte_rf_on_entry, modem_cellular_lte_rf_on_run,
				     NULL, STATE_PARENT(STATE_READY)),
	[STATE_LTE_RF_DISABLE] = MODEM_CELLULAR_STATE(modem_cellular_lte_rf_disable_entry,
						      modem_cellular_lte_rf_disable_run, NULL,
						      STATE_PARENT(STATE_READY)),
	[STATE_RUN_PERIODIC] = MODEM_CELLULAR_STATE(modem_cellular_run_periodic_entry,
						    modem_cellular_run_periodic_run, NULL,
						    STATE_PARENT(STATE_READY)),
	[STATE_POWER_DOWN] = MODEM_CELLULAR_STATE(NULL, modem_cellular_power_down_run, NULL,
						  STATE_PARENT(STATE_ACTIVE)),
	[STATE_CMUX_DISCONNECT] = MODEM_CELLULAR_STATE(modem_cellular_cmux_disconnect_entry,
						       modem_cellular_cmux_disconnect_run, NULL,
						       STATE_PARENT(STATE_POWER_DOWN)),
	[STATE_RUN_SHUTDOWN_SCRIPT] = MODEM_CELLULAR_STATE(
		modem_cellular_run_shutdown_script_entry, modem_cellular_run_shutdown_script_run,
		modem_cellular_run_shutdown_script_exit, STATE_PARENT(STATE_POWER_DOWN)),
	[STATE_POWER_OFF_PULSE] = MODEM_CELLULAR_STATE(
		modem_cellular_power_off_pulse_entry, modem_cellular_power_off_pulse_run,
		modem_cellular_power_off_pulse_exit, STATE_PARENT(STATE_POWER_DOWN)),
	[STATE_AWAIT_POWER_OFF] = MODEM_CELLULAR_STATE(modem_cellular_await_power_off_entry,
						       modem_cellular_await_power_off_run, NULL,
						       STATE_PARENT(STATE_POWER_DOWN)),
};

static const char *modem_cellular_current_state_name(const struct modem_cellular_data *data)
{
	const struct smf_state *state = smf_get_current_leaf_state(&data->smf);

	return modem_cellular_state_names[state - modem_cellular_states];
}

static void modem_cellular_set_state(struct modem_cellular_data *data,
				     enum modem_cellular_v2_core_state state)
{
	/* Each timer belongs to the state that started it */
	data->timer_gen++;
	(void)k_work_cancel_delayable(&data->timeout_work);
	data->script_running = NULL;
	smf_set_state(&data->smf, &modem_cellular_states[state]);
}

static void modem_cellular_run_event(struct modem_cellular_data *data,
				     enum modem_cellular_event evt, const void *ptr)
{
	const struct smf_state *previous = smf_get_current_leaf_state(&data->smf);

	/* Drop results of scripts the current state did not start */
	if (((evt == MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS) ||
	     (evt == MODEM_CELLULAR_EVENT_SCRIPT_FAILED)) &&
	    (ptr != data->script_running)) {
		LOG_DBG("script result dropped in %s", modem_cellular_current_state_name(data));
		return;
	}

	LOG_DBG("event %s", (evt < ARRAY_SIZE(modem_cellular_event_names)) &&
					    (modem_cellular_event_names[evt] != NULL)
				    ? modem_cellular_event_names[evt]
				    : "unknown");

	data->smf_event = evt;
	(void)smf_run_state(&data->smf);

	if (smf_get_current_leaf_state(&data->smf) != previous) {
		LOG_DBG("switch from %s to %s",
			modem_cellular_state_names[previous - modem_cellular_states],
			modem_cellular_current_state_name(data));
	}
}

static void modem_cellular_event_dispatch_handler(struct k_work *item);

static void modem_cellular_timeout_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_cellular_data *data =
		CONTAINER_OF(dwork, struct modem_cellular_data, timeout_work);
	uint32_t timer_gen;

	/* Restarted after it expired, but before this ran, so meant for later */
	if (k_work_delayable_remaining_get(dwork) != 0) {
		LOG_DBG("timeout dropped");
		return;
	}

	timer_gen = data->timer_gen;

	/* Events that arrived before the timer expired are handled first, a timer they stop
	 * or restart having been meant for the state before them
	 */
	modem_cellular_event_dispatch_handler(&data->event_dispatch_work);
	if (data->timer_gen != timer_gen) {
		LOG_DBG("timeout dropped");
		return;
	}

	modem_cellular_run_event(data, MODEM_CELLULAR_EVENT_TIMEOUT, NULL);
}

static enum modem_cellular_event modem_cellular_signal_event(struct modem_cellular_data *data,
							     enum modem_cellular_signal signal)
{
	switch (signal) {
	case MODEM_CELLULAR_SIGNAL_LTE_RF:
		return MODEM_CELLULAR_EVENT_LTE_RF_CHANGED;
	case MODEM_CELLULAR_SIGNAL_PERIODIC:
		return MODEM_CELLULAR_EVENT_PERIODIC;
	case MODEM_CELLULAR_SIGNAL_PERIODIC_RESUMED:
		return MODEM_CELLULAR_EVENT_PERIODIC_RESUMED;
	default:
		break;
	}

	return atomic_get(&data->pm_resume) ? MODEM_CELLULAR_EVENT_RESUME
					    : MODEM_CELLULAR_EVENT_SUSPEND;
}

static void modem_cellular_event_dispatch_handler(struct k_work *item)
{
	struct modem_cellular_data *data =
		CONTAINER_OF(item, struct modem_cellular_data, event_dispatch_work);
	struct modem_cellular_event_pkg pkg;
	atomic_val_t signals;

	do {
		while (k_msgq_get(&data->event_queue, &pkg, K_NO_WAIT) == 0) {
			modem_cellular_run_event(data, pkg.event, pkg.ptr);
		}

		signals = atomic_clear(&data->signals);
		for (int i = 0; i < MODEM_CELLULAR_SIGNAL_COUNT; i++) {
			if ((signals & BIT(i)) != 0) {
				modem_cellular_run_event(data, modem_cellular_signal_event(data, i),
							 NULL);
			}
		}
	} while (signals != 0);
}

static void modem_cellular_delegate_event_ptr(struct modem_cellular_data *data,
					      enum modem_cellular_event evt, const void *ptr)
{
	struct modem_cellular_event_pkg pkg = {.event = evt, .ptr = ptr};

	if (k_msgq_put(&data->event_queue, &pkg, K_NO_WAIT) < 0) {
		LOG_WRN("Event %d dropped", evt);
		return;
	}

	modem_work_submit(&data->event_dispatch_work);
}

static void modem_cellular_signal(struct modem_cellular_data *data,
				  enum modem_cellular_signal signal)
{
	atomic_set_bit(&data->signals, signal);
	modem_work_submit(&data->event_dispatch_work);
}

static void modem_cellular_delegate_event(struct modem_cellular_data *data,
					  enum modem_cellular_event evt)
{
	modem_cellular_delegate_event_ptr(data, evt, NULL);
}

static void modem_cellular_cmux_handler(struct modem_cmux *cmux, enum modem_cmux_event event,
					void *user_data)
{
	struct modem_cellular_data *data = user_data;

	ARG_UNUSED(cmux);

	switch (event) {
	case MODEM_CMUX_EVENT_CONNECTED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_CMUX_CONNECTED);
		break;
	case MODEM_CMUX_EVENT_DISCONNECTED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED);
		break;
	default:
		break;
	}
}

/* ------------------------------------------------------------------------- */
/* Chat callbacks                                                            */
/* ------------------------------------------------------------------------- */

void modem_cellular_chat_callback_handler(struct modem_chat *chat,
					  enum modem_chat_script_result result,
					  const struct modem_chat_script_completion_info *info,
					  void *user_data)
{
	ARG_UNUSED(chat);

	modem_cellular_delegate_event_ptr(user_data,
					  (result == MODEM_CHAT_SCRIPT_RESULT_SUCCESS)
						  ? MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS
						  : MODEM_CELLULAR_EVENT_SCRIPT_FAILED,
					  info->script);
}

void modem_cellular_chat_on_modem_ready(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(argv);
	ARG_UNUSED(argc);

	modem_cellular_delegate_event(user_data, MODEM_CELLULAR_EVENT_MODEM_READY);
}

/*
 * Registration reports arrive on the AT command channel, and are reported on change. The data
 * context is left to +CGEV, which reports a detach: losing registration alone, e.g. while
 * searching during a cell reselection, keeps the bearers.
 */
void modem_cellular_chat_on_cxreg(struct modem_chat *chat, char **argv, uint16_t argc,
				  void *user_data)
{
	struct modem_cellular_data *data = user_data;
	enum cellular_registration_status status;
	enum cellular_registration_status previous;

	ARG_UNUSED(chat);

	if (modem_cellular_store_registration(data, argv, argc, &status, &previous) < 0) {
		return;
	}

	if (status != previous) {
		/* Any cached serving cell belongs to the previous registration */
		modem_cellular_invalidate_network_status(data);
		modem_cellular_emit_reg_state(data, status);
	}
}

/* Forget @p status if registered, returning whether it was */
static bool modem_cellular_detach_status(enum cellular_registration_status *status)
{
	if (!modem_cellular_status_is_registered(*status)) {
		return false;
	}
	*status = CELLULAR_REGISTRATION_NOT_REGISTERED;
	return true;
}

/*
 * A detach from the packet domain ends its registration, leaving circuit-switched +CREG. A
 * status that was not registered, e.g. denied or searching, is left as the modem reported it.
 */
static void modem_cellular_on_detach(struct modem_cellular_data *data)
{
	bool gprs = modem_cellular_detach_status(&data->registration_status_gprs);
	bool lte = modem_cellular_detach_status(&data->registration_status_lte);
	bool nr = modem_cellular_detach_status(&data->registration_status_5g);

	if (gprs || lte || nr) {
		/* Any cached serving cell belongs to the previous registration */
		modem_cellular_invalidate_network_status(data);
		modem_cellular_emit_reg_state(data, CELLULAR_REGISTRATION_NOT_REGISTERED);
	}
}

/*
 * Data context reports arrive on the AT command channel. An active context stands in for the
 * registration of modems that report it late, and losing it ends the call.
 */
void modem_cellular_chat_on_cgev(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = user_data;
	bool connected;
	int ret;

	ARG_UNUSED(chat);

	ret = modem_cellular_parse_cgev(data, argv, argc, &connected);
	if (ret < 0) {
		return;
	}

	if (ret == 1) {
		modem_cellular_on_detach(data);
	}

	/* Every report is passed on, as the context may be lost without it having been reported
	 * active, and registration continue without it.
	 */
	data->pdn_active = connected;
	modem_cellular_emit_data_context(data, connected);
}

/* ------------------------------------------------------------------------- */
/* Device API                                                                */
/* ------------------------------------------------------------------------- */

void modem_cellular_set_ppp_callback(const struct device *dev, cellular_event_mask_t mask,
				     cellular_event_cb_t cb, void *user_data)
{
	struct modem_cellular_data *data = dev->data;

	k_mutex_lock(&data->api_lock, K_FOREVER);
	data->ppp_cb.fn = cb;
	data->ppp_cb.mask = (cb != NULL) ? mask : 0;
	data->ppp_cb.user_data = (cb != NULL) ? user_data : NULL;
	k_mutex_unlock(&data->api_lock);
}

/* Set and cleared on the workqueue the state machine runs on, which reads it from there */
int modem_cellular_failure_reason(const struct modem_cellular_data *data)
{
	return data->failure_reason;
}

/*
 * The LTE RF state as last settled, which cannot change while the periodic script runs. In the
 * LTE RF scripts it is the state being changed from.
 */
bool modem_cellular_run_check_lte_rf_enabled(void *user_data)
{
	const struct modem_cellular_data *data = user_data;

	return data->lte_rf_enabled;
}

bool modem_cellular_run_check_lte_rf_disabled(void *user_data)
{
	return !modem_cellular_run_check_lte_rf_enabled(user_data);
}

/* Whether the data context can be configured, which an LTE RF request depends on */
static bool modem_cellular_has_apn(const struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = data->dev->config;

	return config->use_default_pdp_context || !modem_cellular_needs_apn(data);
}

/*
 * Each request holds a runtime PM reference on the modem, so that the modem is only suspended
 * once the LTE RF is no longer requested, and its release is always acted on, disabling the
 * LTE RF, before the suspend that follows it.
 */
int modem_cellular_lte_rf_request(const struct device *dev)
{
	struct modem_cellular_data *data = dev->data;
	int ret;

	/* Checked again under the lock, but first so that the modem is not powered up for a
	 * request that fails
	 */
	if (!modem_cellular_has_apn(data)) {
		return -ENODATA;
	}

	/* Taken outside the lock, as it waits for a suspend in progress, which emits events */
	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	/* Ordered against the APN being set */
	k_mutex_lock(&data->api_lock, K_FOREVER);
	if (!modem_cellular_has_apn(data)) {
		ret = -ENODATA;
	} else if (atomic_inc(&data->lte_rf_refs) == 0) {
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_LTE_RF);
	}
	k_mutex_unlock(&data->api_lock);

	if (ret < 0) {
		(void)pm_device_runtime_put_async(dev, K_NO_WAIT);
	}
	return ret;
}

int modem_cellular_lte_rf_release(const struct device *dev)
{
	struct modem_cellular_data *data = dev->data;
	atomic_val_t refs;

	do {
		refs = atomic_get(&data->lte_rf_refs);
		if (refs == 0) {
			return -EALREADY;
		}
	} while (!atomic_cas(&data->lte_rf_refs, refs, refs - 1));

	/* Signalled before the reference is released, and handled before the suspend */
	if (refs == 1) {
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_LTE_RF);
	}
	/* Asynchronous, as a suspend waits for the modem to power down */
	return pm_device_runtime_put_async(dev, K_NO_WAIT);
}

static int modem_cellular_set_apn(const struct device *dev, const char *apn)
{
	struct modem_cellular_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->api_lock, K_FOREVER);

	ret = modem_cellular_check_apn(data, apn);
	if (ret < 0) {
		goto out;
	}

	/* The APN in use is only replaced once the LTE RF is no longer requested */
	if (atomic_get(&data->lte_rf_refs) > 0) {
		ret = -EBUSY;
		goto out;
	}

	atomic_set(&data->apn_changed, 1);
	modem_cellular_store_apn(data, apn);

out:
	k_mutex_unlock(&data->api_lock);
	return ret;
}

int cellular_modem_pause_periodic_script(const struct device *dev)
{
	struct modem_cellular_data *data = dev->data;

	if (!modem_cellular_has_periodic_script(dev->config)) {
		return -ENOTSUP;
	}

	if (!atomic_cas(&data->periodic_paused, 0, 1)) {
		return -EINVAL;
	}

	return 0;
}

int cellular_modem_resume_periodic_script(const struct device *dev)
{
	struct modem_cellular_data *data = dev->data;

	if (!modem_cellular_has_periodic_script(dev->config)) {
		return -ENOTSUP;
	}

	if (!atomic_cas(&data->periodic_paused, 1, 0)) {
		return -EINVAL;
	}

	modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PERIODIC_RESUMED);
	return 0;
}

DEVICE_API(cellular, modem_cellular_api) = {
	.get_modem_info = modem_cellular_get_modem_info,
	.get_registration_status = modem_cellular_get_registration_status,
	.get_network_status = modem_cellular_get_network_status,
	.set_callback = modem_cellular_set_callback,
	.set_apn = modem_cellular_set_apn,
};

int modem_cellular_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct modem_cellular_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		atomic_set(&data->pm_resume, 1);
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PM);
		return 0;

	case PM_DEVICE_ACTION_SUSPEND:
		/* The suspend completes on the workqueue the state machine runs on */
		if (k_current_get() == modem_work_queue_thread_get()) {
			LOG_ERR("Cannot suspend from the modem workqueue");
			return -EDEADLK;
		}
		atomic_set(&data->pm_resume, 0);
		modem_cellular_signal(data, MODEM_CELLULAR_SIGNAL_PM);
		/* Every shutdown step is bounded by a script or state timer. Waiting for the
		 * transition to finish keeps the PM state consistent with the hardware even
		 * when a vendor's shutdown script legitimately takes more than 30 seconds.
		 */
		return k_sem_take(&data->suspended_sem, K_FOREVER);

	default:
		return -ENOTSUP;
	}
}

/* Releasing a pipe leaves a real UART backend receiving into the instance */
#if defined(CONFIG_ZTEST) && !defined(CONFIG_MODEM_BACKEND_UART)
void modem_cellular_ztest_halt(const struct device *dev)
{
	struct modem_cellular_data *data = dev->data;
	const struct modem_cellular_config *config = dev->config;
	struct modem_cellular_event_pkg event;
	struct k_work_sync sync;

	k_work_cancel_delayable_sync(&data->timeout_work, &sync);
	k_work_cancel_delayable_sync(&data->periodic_work, &sync);
	k_work_cancel_sync(&data->event_dispatch_work, &sync);

	modem_cellular_notify_pipelinks_disconnected(data);
	modem_chat_release(&data->chat);
	modem_cmux_release(&data->cmux);

	if (data->uart_pipe != NULL) {
		modem_pipe_release(data->uart_pipe);
	}

	if (data->dlci1_pipe != NULL) {
		modem_pipe_release(data->dlci1_pipe);
	}

	if (data->dlci2_pipe != NULL) {
		modem_pipe_release(data->dlci2_pipe);
	}

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		if (config->user_pipes[i].pipe != NULL) {
			modem_pipe_release(config->user_pipes[i].pipe);
		}
	}

	/* The timer runs the state machine directly, and may have queued events */
	while (k_msgq_get(&data->event_queue, &event, K_NO_WAIT) == 0) {
	}
	atomic_clear(&data->signals);
	k_work_cancel_sync(&data->event_dispatch_work, &sync);
	k_work_cancel_delayable_sync(&data->timeout_work, &sync);
	k_work_cancel_delayable_sync(&data->periodic_work, &sync);

	/* Return to the state modem_cellular_init() expects on a fresh boot */
	data->original_baudrate = 0U;
	data->recovery_count = 0;
	data->power_on_skipped = false;
	data->baudrate_kept = false;
	data->suspend_pending = false;
	data->cmux_connect_failed = false;
	data->cmux_bus_opened = false;
	data->power_cycle = false;
	data->failure_reason = 0;
	data->cmd_pipe = NULL;
	data->script_running = NULL;
	/* Requests normally outlive power cycles, but not tests */
	atomic_clear(&data->lte_rf_refs);
	atomic_clear(&data->pm_resume);
	atomic_clear(&data->periodic_paused);
	data->periodic_skipped = false;
	data->periodic_pending = false;
	data->access_tech = CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN;
	data->script_failure_counter = 0;
	modem_cellular_clear_registration_status(data);
	data->apn[0] = '\0';
	data->apn_set = false;
	atomic_clear(&data->apn_changed);
	modem_cellular_init_apn(data, apn_abort_matches, ARRAY_SIZE(apn_abort_matches));
	smf_set_initial(&data->smf, &modem_cellular_states[STATE_IDLE]);
}
#endif /* CONFIG_ZTEST && !CONFIG_MODEM_BACKEND_UART */

static int modem_cellular_configure_gpio(const struct gpio_dt_spec *gpio, gpio_flags_t flags,
					 const char *name)
{
	int ret;

	if (!modem_cellular_gpio_is_enabled(gpio)) {
		return 0;
	}
	if (!gpio_is_ready_dt(gpio)) {
		LOG_ERR("Modem %s GPIO is not ready", name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(gpio, flags);
	if (ret < 0) {
		LOG_ERR("Failed to configure modem %s GPIO (%d)", name, ret);
	}

	return ret;
}

/* Scripts the core runs report their results through the callback they are defined with */
#define MODEM_CELLULAR_ASSERT_SCRIPT(_dev, _scripts, _name)                                        \
	__ASSERT(((_scripts)->_name == NULL) ||                                                    \
			 ((_scripts)->_name->callback == modem_cellular_chat_callback_handler),    \
		 "%s: " #_name " script must report to modem_cellular_chat_callback_handler",     \
		 (_dev)->name)

/* Validate the vendor scripts against what the driver expects of them */
static void modem_cellular_assert_scripts(const struct device *dev)
{
	const struct modem_cellular_config *config = dev->config;
	const struct modem_cellular_config_scripts *scripts = &config->vendor->common->scripts;
	const struct modem_cellular_vendor_config_inst *instance = config->vendor->instance;

	ARG_UNUSED(scripts);
	ARG_UNUSED(instance);

	__ASSERT(scripts->init != NULL, "%s: init script is required", dev->name);
	/* Run by the PPP device, which replaces the callback of its copy */
	__ASSERT(scripts->dial != NULL, "%s: dial script is required", dev->name);
	__ASSERT(scripts->lte_rf_enable != NULL, "%s: LTE RF enable script is required",
		 dev->name);
	__ASSERT(scripts->lte_rf_disable != NULL, "%s: LTE RF disable script is required",
		 dev->name);

	if (instance != NULL) {
		MODEM_CELLULAR_ASSERT_SCRIPT(dev, instance, set_baudrate);
	}
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, init);
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, dlci_setup);
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, sim_ready);
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, lte_rf_enable);
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, lte_rf_disable);
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, periodic);
	MODEM_CELLULAR_ASSERT_SCRIPT(dev, scripts, shutdown);
}

int modem_cellular_init(const struct device *dev)
{
	struct modem_cellular_data *data = dev->data;
	const struct modem_cellular_config *config = dev->config;
	const struct gpio_dt_spec *dtr_gpio = NULL;
	int ret;

	modem_cellular_assert_scripts(dev);

	data->dev = dev;

	k_mutex_init(&data->api_lock);
	k_work_init_delayable(&data->timeout_work, modem_cellular_timeout_handler);
	k_work_init_delayable(&data->periodic_work, modem_cellular_periodic_handler);
	k_work_init(&data->event_dispatch_work, modem_cellular_event_dispatch_handler);
	k_msgq_init(&data->event_queue, (char *)data->event_buf, sizeof(data->event_buf[0]),
		    ARRAY_SIZE(data->event_buf));
	k_sem_init(&data->suspended_sem, 0, 1);
	data->access_tech = CELLULAR_ACCESS_TECHNOLOGY_UNKNOWN;

	ret = modem_cellular_configure_gpio(&config->wake_gpio, GPIO_OUTPUT_INACTIVE, "wake");
	if (ret < 0) {
		return ret;
	}

	ret = modem_cellular_configure_gpio(&config->power_gpio, GPIO_OUTPUT_INACTIVE, "power");
	if (ret < 0) {
		return ret;
	}

	ret = modem_cellular_configure_gpio(
		&config->reset_gpio,
		config->hold_reset_on_suspend ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE, "reset");
	if (ret < 0) {
		return ret;
	}

	ret = modem_cellular_configure_gpio(&config->dtr_gpio, GPIO_OUTPUT_INACTIVE, "DTR");
	if (ret < 0) {
		return ret;
	}
	if (modem_cellular_gpio_is_enabled(&config->dtr_gpio)) {
		dtr_gpio = &config->dtr_gpio;
	}

	ret = modem_cellular_configure_gpio(&config->status_gpio, GPIO_INPUT, "status");
	if (ret < 0) {
		return ret;
	}

	{
		const struct modem_backend_uart_config uart_backend_config = {
			.uart = config->uart,
			.dtr_gpio = dtr_gpio,
			.receive_buf = data->uart_backend_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->uart_backend_receive_buf),
			.transmit_buf = data->uart_backend_transmit_buf,
			.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf),
		};

		data->uart_pipe =
			modem_backend_uart_init(&data->uart_backend, &uart_backend_config);
		if (data->uart_pipe == NULL) {
			LOG_ERR("Failed to initialize UART backend");
			return -EIO;
		}
		data->cmd_pipe = NULL;
	}

	{
		const struct modem_cmux_config cmux_config = {
			.callback = modem_cellular_cmux_handler,
			.user_data = data,
			.receive_buf = data->cmux_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->cmux_receive_buf),
			.transmit_buf = data->cmux_transmit_buf,
			.transmit_buf_size = ARRAY_SIZE(data->cmux_transmit_buf),
			.enable_runtime_power_management = config->cmux_enable_runtime_power_save,
			.close_pipe_on_power_save = config->cmux_close_pipe_on_power_save,
			.no_powersave_handshake = config->cmux_no_powersave_handshake,
			.idle_timeout = config->cmux_idle_timeout,
		};

		modem_cmux_init(&data->cmux, &cmux_config);
	}

	{
		const struct modem_cmux_dlci_config dlci1_config = {
			.dlci_address = 1,
			.receive_buf = data->dlci1_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->dlci1_receive_buf),
		};
		const struct modem_cmux_dlci_config dlci2_config = {
			.dlci_address = 2,
			.receive_buf = data->dlci2_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->dlci2_receive_buf),
		};

		data->dlci1_pipe = modem_cmux_dlci_init(&data->cmux, &data->dlci1, &dlci1_config);
		data->dlci2_pipe = modem_cmux_dlci_init(&data->cmux, &data->dlci2, &dlci2_config);
		modem_pipelink_init(config->ppp_pipelink, data->dlci2_pipe);
	}

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		struct modem_cellular_user_pipe *user_pipe = &config->user_pipes[i];
		const struct modem_cmux_dlci_config user_dlci_config = {
			.dlci_address = user_pipe->dlci_address,
			.receive_buf = user_pipe->dlci_receive_buf,
			.receive_buf_size = user_pipe->dlci_receive_buf_size,
		};

		user_pipe->pipe =
			modem_cmux_dlci_init(&data->cmux, &user_pipe->dlci, &user_dlci_config);
		modem_pipelink_init(user_pipe->pipelink, user_pipe->pipe);
	}

	{
		const struct modem_chat_config chat_config = {
			.user_data = data,
			.pipe_callback = modem_cellular_dlci1_pipe_handler,
			.receive_buf = data->chat_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->chat_receive_buf),
			.delimiter = (const uint8_t *)config->vendor->common->chat_delimiter,
			.delimiter_size = strlen(config->vendor->common->chat_delimiter),
			.filter = (const uint8_t *)config->vendor->common->chat_filter,
			.filter_size = (config->vendor->common->chat_filter != NULL)
					       ? strlen(config->vendor->common->chat_filter)
					       : 0,
			.argv = data->chat_argv,
			.argv_size = ARRAY_SIZE(data->chat_argv),
			.unsol_matches = config->vendor->common->unsol_matches.matches,
			.unsol_matches_size = config->vendor->common->unsol_matches.size,
		};

		modem_chat_init(&data->chat, &chat_config);
	}

	modem_cellular_init_apn(data, apn_abort_matches, ARRAY_SIZE(apn_abort_matches));
	smf_set_initial(&data->smf, &modem_cellular_states[STATE_IDLE]);

	return pm_device_driver_init(dev, modem_cellular_pm_action);
}
