/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PPP link of the State Machine Framework based cellular modem driver.
 *
 * Each modem has a PPP device, which its PPP network interface holds a runtime PM reference
 * on while the interface is up. The PPP device in turn holds a runtime PM reference on the
 * modem itself, the core, which powers the modem up, and a request for its LTE RF. Once the
 * core has set up its AT command channel it connects the PPP pipelink, and the data call is
 * dialled and PPP attached.
 *
 *   SUSPENDED               Interface down, no reference held on the core
 *   ACTIVE                  Interface up
 *     AWAIT_LINK            Waiting for the core to connect the PPP pipelink
 *     LINKED                PPP channel in use, closed again on leaving, which hangs up
 *       OPEN_PIPE           Opening the PPP channel, once any earlier close has completed
 *       AWAIT_ATTACH        Waiting for the core to report the modem attached
 *       DIAL
 *       DIAL_BACKOFF        Dial failed, retried after the periodic script interval
 *       CONNECTED           Call up and PPP attached, the interface dormant while the modem
 *                           is not attached, until the call ends
 *     HANGUP                Call ended, dialled again after the periodic script interval
 *                           once PPP is dead
 *
 * The modem is attached while registered in the packet domain, by +CGREG, +CEREG or +C5GREG, or
 * while its data context is active, which stands in for the registration of modems that report it
 * late. A circuit-switched +CREG registration alone does not attach it. Only +CGEV reports the
 * context lost, so on modems that report +CGEV a call survives losing registration while the
 * context stays active, e.g. while searching during a cell reselection. The call ends when PPP
 * reports the link dead, e.g. on NO CARRIER, when the data context is lost, including while
 * dialling, or when the modem stays detached for the periodic script interval. Making the interface
 * dormant closes PPP, so in practice a detach ends the call as soon as LCP terminates.
 *
 * PPP reports the link dead asynchronously, so a call is not dialled again until the link of
 * the previous one is dead, which keeps its report from ending the next call.
 *
 * When the core cannot bring the modem up within its recovery attempts, it powers the modem
 * down and reports CELLULAR_EVENT_MODEM_FAILED, which the PPP device raises as
 * NET_EVENT_CONN_IF_FATAL_ERROR on its interface. Taking the interface down and up again
 * starts over.
 *
 * Resuming the PPP device fails with -ENODATA when no APN is available for the data context,
 * which is raised as NET_EVENT_CONN_IF_NO_CONFIGURATION on its interface. Taking the
 * interface down and up again once cellular_set_apn() has set one starts over.
 *
 * As in the core, events are run through the state machine on the modem workqueue, with the
 * events that report a level signalled through atomic flags, and the runtime PM references
 * on the core are taken and released from the PM actions. The core reports changes to the
 * link with the cellular events an application receives, through a callback of its own for
 * the PPP device, leaving the one set with cellular_set_callback() to the application.
 *
 * A timeout runs after the events pending when it expired, and is dropped if they stopped or
 * restarted the timer.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/modem/ppp.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ppp.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/smf.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(modem_cellular, CONFIG_MODEM_LOG_LEVEL);

#include <string.h>

#include "modem_cellular_common.h"
#include "modem_workqueue.h"

enum modem_cellular_ppp_event {
	PPP_EVENT_RESUME,
	PPP_EVENT_SUSPEND,
	PPP_EVENT_LINK_CHANGED,
	PPP_EVENT_LINK_LOST,
	PPP_EVENT_PIPE_OPENED,
	/* Only for a close requested on leaving LINKED */
	PPP_EVENT_PIPE_CLOSED,
	PPP_EVENT_SCRIPT_SUCCESS,
	PPP_EVENT_SCRIPT_FAILED,
	PPP_EVENT_TIMEOUT,
	PPP_EVENT_ATTACH_CHANGED,
	/* The data context was lost */
	PPP_EVENT_CALL_ENDED,
	/* PPP reported the link dead */
	PPP_EVENT_PPP_DEAD,
	PPP_EVENT_LINK_FAILED,
	PPP_EVENT_COUNT,
};

static const char *const modem_cellular_ppp_event_names[PPP_EVENT_COUNT] = {
	[PPP_EVENT_RESUME] = "resume",
	[PPP_EVENT_SUSPEND] = "suspend",
	[PPP_EVENT_LINK_CHANGED] = "link changed",
	[PPP_EVENT_LINK_LOST] = "link lost",
	[PPP_EVENT_PIPE_OPENED] = "pipe opened",
	[PPP_EVENT_PIPE_CLOSED] = "pipe closed",
	[PPP_EVENT_SCRIPT_SUCCESS] = "script success",
	[PPP_EVENT_SCRIPT_FAILED] = "script failed",
	[PPP_EVENT_TIMEOUT] = "timeout",
	[PPP_EVENT_ATTACH_CHANGED] = "attach changed",
	[PPP_EVENT_CALL_ENDED] = "call ended",
	[PPP_EVENT_PPP_DEAD] = "ppp dead",
	[PPP_EVENT_LINK_FAILED] = "link failed",
};

enum modem_cellular_ppp_state {
	PPP_STATE_SUSPENDED,
	PPP_STATE_ACTIVE,
	PPP_STATE_AWAIT_LINK,
	PPP_STATE_LINKED,
	PPP_STATE_OPEN_PIPE,
	PPP_STATE_AWAIT_ATTACH,
	PPP_STATE_DIAL,
	PPP_STATE_DIAL_BACKOFF,
	PPP_STATE_CONNECTED,
	PPP_STATE_HANGUP,
	PPP_STATE_COUNT,
};

static const char *const modem_cellular_ppp_state_names[PPP_STATE_COUNT] = {
	[PPP_STATE_SUSPENDED] = "suspended",
	[PPP_STATE_ACTIVE] = "active",
	[PPP_STATE_AWAIT_LINK] = "await link",
	[PPP_STATE_LINKED] = "linked",
	[PPP_STATE_OPEN_PIPE] = "open pipe",
	[PPP_STATE_AWAIT_ATTACH] = "await attach",
	[PPP_STATE_DIAL] = "dial",
	[PPP_STATE_DIAL_BACKOFF] = "dial backoff",
	[PPP_STATE_CONNECTED] = "connected",
	[PPP_STATE_HANGUP] = "hangup",
};

static void modem_cellular_ppp_set_state(struct modem_cellular_ppp_data *data,
					 enum modem_cellular_ppp_state state);

static void modem_cellular_ppp_delegate_event_ptr(struct modem_cellular_ppp_data *data,
						  enum modem_cellular_ppp_event evt,
						  const void *ptr);

static void modem_cellular_ppp_delegate_event(struct modem_cellular_ppp_data *data,
					      enum modem_cellular_ppp_event evt)
{
	modem_cellular_ppp_delegate_event_ptr(data, evt, NULL);
}

/*
 * Level signals, run in this order once the queued events are handled. The order matters: the
 * core disconnects the pipelink before it reports the modem detached, the data context lost
 * or the modem failed, so those are handled once the channel is let go of.
 */
enum modem_cellular_ppp_signal {
	/* The pipelink disconnected since last handled, even if connected again since */
	PPP_SIGNAL_LINK_LOST,
	/* The pipelink connection changed */
	PPP_SIGNAL_LINK,
	/* The modem attachment changed */
	PPP_SIGNAL_ATTACH,
	/* The data context was lost */
	PPP_SIGNAL_CALL_ENDED,
	/* PPP reported the link dead */
	PPP_SIGNAL_PPP_DEAD,
	/* The core gave up on the modem */
	PPP_SIGNAL_LINK_FAILED,
	/* The requested power state changed */
	PPP_SIGNAL_PM,
	PPP_SIGNAL_COUNT,
};

static void modem_cellular_ppp_signal(struct modem_cellular_ppp_data *data,
				      enum modem_cellular_ppp_signal signal)
{
	atomic_set_bit(&data->signals, signal);
	modem_work_submit(&data->event_dispatch_work);
}

static struct modem_cellular_ppp_data *modem_cellular_ppp_data_from_obj(void *obj)
{
	return CONTAINER_OF(obj, struct modem_cellular_ppp_data, smf);
}

static const struct modem_cellular_ppp_config *
modem_cellular_ppp_config(struct modem_cellular_ppp_data *data)
{
	return data->dev->config;
}

static const struct modem_cellular_config *
modem_cellular_ppp_core_config(struct modem_cellular_ppp_data *data)
{
	return modem_cellular_ppp_config(data)->core->config;
}

static struct modem_pipelink *modem_cellular_ppp_pipelink(struct modem_cellular_ppp_data *data)
{
	return modem_cellular_ppp_core_config(data)->ppp_pipelink;
}

static struct modem_pipe *modem_cellular_ppp_pipe(struct modem_cellular_ppp_data *data)
{
	return modem_pipelink_get_pipe(modem_cellular_ppp_pipelink(data));
}

static struct net_if *modem_cellular_ppp_iface(struct modem_cellular_ppp_data *data)
{
	return modem_ppp_get_iface(modem_cellular_ppp_config(data)->ppp);
}

static bool modem_cellular_ppp_attached(struct modem_cellular_ppp_data *data)
{
	return modem_cellular_is_attached(modem_cellular_ppp_config(data)->core->data);
}

/* Whether PPP has finished with the link, which it reports asynchronously */
static bool modem_cellular_ppp_link_dead(struct modem_cellular_ppp_data *data)
{
	const struct ppp_context *ctx = net_if_l2_data(modem_cellular_ppp_iface(data));

	return ctx->phase == PPP_DEAD;
}

/*
 * Whether net_if_down() is taking the link down. It disables PPP before closing LCP, which
 * reports the link dead before the peer has acknowledged the termination.
 */
static bool modem_cellular_ppp_closing(struct modem_cellular_ppp_data *data)
{
	const struct ppp_context *ctx = net_if_l2_data(modem_cellular_ppp_iface(data));

	return !ctx->is_enabled;
}

static void modem_cellular_ppp_start_timer(struct modem_cellular_ppp_data *data,
					   k_timeout_t timeout)
{
	data->timer_gen++;
	modem_work_reschedule(&data->timeout_work, timeout);
}

static void modem_cellular_ppp_stop_timer(struct modem_cellular_ppp_data *data)
{
	data->timer_gen++;
	(void)k_work_cancel_delayable(&data->timeout_work);
}

/* Events of the core about the link, alongside those of the application's callback */
#define MODEM_CELLULAR_PPP_CORE_EVENTS                                                             \
	(CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED | CELLULAR_EVENT_DATA_CONTEXT_CHANGED |        \
	 CELLULAR_EVENT_MODEM_FAILED)

static void modem_cellular_ppp_core_handler(const struct device *dev, enum cellular_event event,
					    const void *payload, void *user_data)
{
	struct modem_cellular_ppp_data *data = user_data;

	ARG_UNUSED(dev);

	switch (event) {
	case CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED:
		modem_cellular_ppp_signal(data, PPP_SIGNAL_ATTACH);
		break;

	case CELLULAR_EVENT_DATA_CONTEXT_CHANGED:
		/* Losing the context ends any call on it, however the modem is registered */
		if (!((const struct cellular_evt_data_context *)payload)->active) {
			modem_cellular_ppp_signal(data, PPP_SIGNAL_CALL_ENDED);
		}
		modem_cellular_ppp_signal(data, PPP_SIGNAL_ATTACH);
		break;

	case CELLULAR_EVENT_MODEM_FAILED:
		/* Reported from the workqueue the state machine runs on */
		data->failure_reason = ((const struct cellular_evt_modem_failed *)payload)->reason;
		modem_cellular_ppp_signal(data, PPP_SIGNAL_LINK_FAILED);
		break;

	default:
		break;
	}
}

/* PPP reports the link dead when the modem ends the call, e.g. with NO CARRIER */
static void modem_cellular_ppp_dead_handler(struct net_mgmt_event_callback *cb,
					    uint64_t mgmt_event, struct net_if *iface)
{
	struct modem_cellular_ppp_data *data =
		CONTAINER_OF(cb, struct modem_cellular_ppp_data, ppp_dead_cb);

	if ((mgmt_event == NET_EVENT_PPP_PHASE_DEAD) && (iface == modem_cellular_ppp_iface(data))) {
		modem_cellular_ppp_signal(data, PPP_SIGNAL_PPP_DEAD);
	}
}

static void modem_cellular_ppp_pipelink_callback(struct modem_pipelink *link,
						 enum modem_pipelink_event event, void *user_data)
{
	struct modem_cellular_ppp_data *data = user_data;

	ARG_UNUSED(link);

	if (event == MODEM_PIPELINK_EVENT_DISCONNECTED) {
		/* Kept until handled, so that the channel is let go of even if connected again */
		atomic_set_bit(&data->signals, PPP_SIGNAL_LINK_LOST);
	}
	modem_cellular_ppp_signal(data, PPP_SIGNAL_LINK);
}

static void modem_cellular_ppp_pipe_callback(struct modem_pipe *pipe,
					     enum modem_pipe_event event, void *user_data)
{
	ARG_UNUSED(pipe);

	if (event == MODEM_PIPE_EVENT_OPENED) {
		modem_cellular_ppp_delegate_event(user_data, PPP_EVENT_PIPE_OPENED);
	} else if (event == MODEM_PIPE_EVENT_CLOSED) {
		modem_cellular_ppp_delegate_event(user_data, PPP_EVENT_PIPE_CLOSED);
	}
}

static void modem_cellular_ppp_chat_callback(struct modem_chat *chat,
					     enum modem_chat_script_result result,
					     const struct modem_chat_script_completion_info *info,
					     void *user_data)
{
	/* The user data is the core's, as for the scripts it runs itself */
	struct modem_cellular_ppp_data *data = CONTAINER_OF(chat, struct modem_cellular_ppp_data,
							    chat);

	ARG_UNUSED(user_data);

	modem_cellular_ppp_delegate_event_ptr(data,
					      (result == MODEM_CHAT_SCRIPT_RESULT_SUCCESS)
						      ? PPP_EVENT_SCRIPT_SUCCESS
						      : PPP_EVENT_SCRIPT_FAILED,
					      info->script);
}

/* ------------------------------------------------------------------------- */
/* SUSPENDED                                                                 */
/* ------------------------------------------------------------------------- */

static enum smf_state_result modem_cellular_ppp_suspended_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	switch (data->smf_event) {
	case PPP_EVENT_RESUME:
		modem_cellular_ppp_set_state(data, PPP_STATE_AWAIT_LINK);
		break;
	case PPP_EVENT_SUSPEND:
		if (!data->pipe_closing) {
			k_sem_give(&data->suspended_sem);
		}
		break;
	case PPP_EVENT_PIPE_CLOSED:
		/* The close started on leaving LINKED, which the suspend was held for */
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

static void modem_cellular_ppp_active_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	k_sem_take(&data->suspended_sem, K_NO_WAIT);

	/* Attached here rather than at init, which the core's init would undo */
	modem_pipelink_attach(modem_cellular_ppp_pipelink(data),
			      modem_cellular_ppp_pipelink_callback, data);
	modem_cellular_set_ppp_callback(modem_cellular_ppp_config(data)->core,
					MODEM_CELLULAR_PPP_CORE_EVENTS,
					modem_cellular_ppp_core_handler, data);
	net_mgmt_add_event_callback(&data->ppp_dead_cb);

	/* The core may have failed before, if held resumed by another user, and stays so */
	data->failure_reason =
		modem_cellular_failure_reason(modem_cellular_ppp_config(data)->core->data);
	if (data->failure_reason != 0) {
		modem_cellular_ppp_signal(data, PPP_SIGNAL_LINK_FAILED);
	}
}

static enum smf_state_result modem_cellular_ppp_active_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	switch (data->smf_event) {
	case PPP_EVENT_SUSPEND:
		modem_cellular_ppp_set_state(data, PPP_STATE_SUSPENDED);
		return SMF_EVENT_HANDLED;

	case PPP_EVENT_LINK_FAILED:
		/* The core stays powered down until the interface is taken down and up again */
		LOG_ERR("Modem failed, no further connection attempts");
		net_mgmt_event_notify_with_info(NET_EVENT_CONN_IF_FATAL_ERROR,
						modem_cellular_ppp_iface(data),
						&data->failure_reason,
						sizeof(data->failure_reason));
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_ppp_active_exit(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	net_mgmt_del_event_callback(&data->ppp_dead_cb);
	modem_cellular_set_ppp_callback(modem_cellular_ppp_config(data)->core, 0, NULL, NULL);
	modem_pipelink_release(modem_cellular_ppp_pipelink(data));
	/* A suspend completes once the channel has closed, so that the LTE RF request is only
	 * released after it, rather than the LTE RF being disabled while the channel closes
	 */
	if (!data->pipe_closing) {
		k_sem_give(&data->suspended_sem);
	}
}

static void modem_cellular_ppp_await_link_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	/* The pipelink only reports changes, so check whether it is already connected */
	if (modem_pipelink_is_connected(modem_cellular_ppp_pipelink(data))) {
		modem_cellular_ppp_signal(data, PPP_SIGNAL_LINK);
	}
}

static enum smf_state_result modem_cellular_ppp_await_link_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	if (data->smf_event != PPP_EVENT_LINK_CHANGED) {
		return SMF_EVENT_PROPAGATE;
	}

	if (modem_pipelink_is_connected(modem_cellular_ppp_pipelink(data))) {
		modem_cellular_ppp_set_state(data, PPP_STATE_OPEN_PIPE);
	}
	return SMF_EVENT_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* LINKED                                                                    */
/* ------------------------------------------------------------------------- */

static enum smf_state_result modem_cellular_ppp_linked_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	if (data->smf_event == PPP_EVENT_LINK_LOST) {
		LOG_DBG("PPP channel lost");
		modem_cellular_ppp_set_state(data, PPP_STATE_AWAIT_LINK);
		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

/*
 * Closing the channel hangs up any call on it. The pipe callback is kept attached for the
 * close to be reported, which the channel is not opened again before, as the pipe reports
 * itself open until then.
 */
static void modem_cellular_ppp_linked_exit(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);
	struct modem_pipe *pipe = modem_cellular_ppp_pipe(data);
	int ret;

	modem_chat_release(&data->chat);
	modem_pipe_attach(pipe, modem_cellular_ppp_pipe_callback, data);
	data->pipe_closing = true;
	ret = modem_pipe_close_async(pipe);
	if (ret < 0) {
		LOG_WRN("Failed to close PPP channel (%d)", ret);
		data->pipe_closing = false;
	}
}

static void modem_cellular_ppp_open_pipe(struct modem_cellular_ppp_data *data)
{
	struct modem_pipe *pipe = modem_cellular_ppp_pipe(data);
	int ret;

	modem_pipe_attach(pipe, modem_cellular_ppp_pipe_callback, data);
	ret = modem_pipe_open_async(pipe);
	if (ret < 0) {
		LOG_ERR("Failed to open PPP channel (%d)", ret);
	}
}

static void modem_cellular_ppp_open_pipe_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	if (data->pipe_closing) {
		LOG_DBG("Waiting for PPP channel to close");
		return;
	}

	modem_cellular_ppp_open_pipe(data);
}

static enum smf_state_result modem_cellular_ppp_open_pipe_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	switch (data->smf_event) {
	case PPP_EVENT_PIPE_CLOSED:
		/* The close the open was held back for has completed */
		modem_cellular_ppp_open_pipe(data);
		return SMF_EVENT_HANDLED;

	case PPP_EVENT_PIPE_OPENED:
		modem_cellular_ppp_set_state(data, PPP_STATE_AWAIT_ATTACH);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_ppp_await_attach_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	/* The core only reports changes, so check whether the modem is already attached */
	if (modem_cellular_ppp_attached(data)) {
		modem_cellular_ppp_signal(data, PPP_SIGNAL_ATTACH);
	}
}

static enum smf_state_result modem_cellular_ppp_await_attach_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	if (data->smf_event != PPP_EVENT_ATTACH_CHANGED) {
		return SMF_EVENT_PROPAGATE;
	}

	if (modem_cellular_ppp_attached(data)) {
		modem_cellular_ppp_set_state(data, PPP_STATE_DIAL);
	}
	return SMF_EVENT_HANDLED;
}

static void modem_cellular_ppp_dial_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);
	int ret;

	data->dial_call_ended = false;
	modem_chat_attach(&data->chat, modem_cellular_ppp_pipe(data));
	data->script_running = &data->dial_script;
	ret = modem_chat_run_script_async(&data->chat, &data->dial_script);
	if (ret < 0) {
		LOG_ERR("Failed to start dial script (%d)", ret);
		modem_cellular_ppp_delegate_event_ptr(data, PPP_EVENT_SCRIPT_FAILED,
						      &data->dial_script);
	}
}

static enum smf_state_result modem_cellular_ppp_dial_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	switch (data->smf_event) {
	case PPP_EVENT_CALL_ENDED:
		/* Acted on once the dial completes. A detach is not, CONNECTED checks for it */
		data->dial_call_ended = true;
		return SMF_EVENT_HANDLED;

	case PPP_EVENT_SCRIPT_SUCCESS:
		if (data->dial_call_ended) {
			LOG_INF("Data context lost while dialling");
			modem_cellular_ppp_set_state(data, PPP_STATE_HANGUP);
		} else {
			modem_cellular_ppp_set_state(data, PPP_STATE_CONNECTED);
		}
		return SMF_EVENT_HANDLED;

	case PPP_EVENT_SCRIPT_FAILED:
		LOG_WRN("Dial failed");
		modem_cellular_ppp_set_state(data, PPP_STATE_DIAL_BACKOFF);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_ppp_dial_backoff_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	modem_chat_release(&data->chat);
	modem_cellular_ppp_start_timer(data, K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS));
}

static enum smf_state_result modem_cellular_ppp_dial_backoff_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	if (data->smf_event != PPP_EVENT_TIMEOUT) {
		return SMF_EVENT_PROPAGATE;
	}

	/* The modem may have been detached since the failed dial */
	modem_cellular_ppp_set_state(data, PPP_STATE_AWAIT_ATTACH);
	return SMF_EVENT_HANDLED;
}

static void modem_cellular_ppp_connected_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	modem_chat_release(&data->chat);
	(void)modem_ppp_attach(modem_cellular_ppp_config(data)->ppp, modem_cellular_ppp_pipe(data));

	/* Detaches while dialling are not acted on, so the attachment is checked here */
	if (modem_cellular_ppp_attached(data)) {
		net_if_dormant_off(modem_cellular_ppp_iface(data));
	} else {
		modem_cellular_ppp_start_timer(data,
					       K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS));
	}
}

/*
 * The interface is dormant while the modem is not attached, which closes PPP. The call ends
 * once PPP reports the link dead, or after the periodic script interval as PPP may never do
 * so, unless the modem is attached again first.
 */
static enum smf_state_result modem_cellular_ppp_connected_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	switch (data->smf_event) {
	case PPP_EVENT_ATTACH_CHANGED:
		if (modem_cellular_ppp_attached(data)) {
			modem_cellular_ppp_stop_timer(data);
			net_if_dormant_off(modem_cellular_ppp_iface(data));
		} else {
			net_if_dormant_on(modem_cellular_ppp_iface(data));
			modem_cellular_ppp_start_timer(
				data, K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS));
		}
		return SMF_EVENT_HANDLED;

	case PPP_EVENT_PPP_DEAD:
		if (modem_cellular_ppp_closing(data)) {
			/* The call ends with the suspend that follows, once LCP has terminated
			 * over the channel, which releasing PPP from it now would prevent
			 */
			return SMF_EVENT_HANDLED;
		}
		__fallthrough;
	case PPP_EVENT_CALL_ENDED:
	case PPP_EVENT_TIMEOUT:
		LOG_INF("Call ended");
		modem_cellular_ppp_set_state(data, PPP_STATE_HANGUP);
		return SMF_EVENT_HANDLED;

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void modem_cellular_ppp_connected_exit(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	/* The carrier is turned off first, so that PPP takes the link down at once rather
	 * than terminating LCP over a channel that is no longer attached to it
	 */
	modem_ppp_release(modem_cellular_ppp_config(data)->ppp);
	net_if_dormant_on(modem_cellular_ppp_iface(data));
}

/*
 * Leaving LINKED closed the channel, give the modem time to settle before dialling again,
 * and PPP time to take the link down, so that its report does not end the next call
 */
static void modem_cellular_ppp_hangup_entry(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	data->hangup_elapsed = false;
	modem_cellular_ppp_start_timer(data, K_MSEC(CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS));
}

static enum smf_state_result modem_cellular_ppp_hangup_run(void *obj)
{
	struct modem_cellular_ppp_data *data = modem_cellular_ppp_data_from_obj(obj);

	switch (data->smf_event) {
	case PPP_EVENT_TIMEOUT:
		data->hangup_elapsed = true;
		break;
	case PPP_EVENT_PPP_DEAD:
		break;
	default:
		return SMF_EVENT_PROPAGATE;
	}

	if (!data->hangup_elapsed) {
		return SMF_EVENT_HANDLED;
	}

	/* PPP reports the link dead after it sets the phase, so the report follows if not */
	if (!modem_cellular_ppp_link_dead(data)) {
		LOG_DBG("Waiting for PPP link to go down");
		return SMF_EVENT_HANDLED;
	}

	modem_cellular_ppp_set_state(data, PPP_STATE_AWAIT_LINK);
	return SMF_EVENT_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* State machine                                                             */
/* ------------------------------------------------------------------------- */

#define PPP_STATE(_entry, _run, _exit, _parent) SMF_CREATE_STATE(_entry, _run, _exit, _parent, NULL)

#define PPP_STATE_PARENT(_state) (&modem_cellular_ppp_states[_state])

static const struct smf_state modem_cellular_ppp_states[PPP_STATE_COUNT] = {
	[PPP_STATE_SUSPENDED] = PPP_STATE(NULL, modem_cellular_ppp_suspended_run, NULL, NULL),
	[PPP_STATE_ACTIVE] = PPP_STATE(modem_cellular_ppp_active_entry,
				       modem_cellular_ppp_active_run,
				       modem_cellular_ppp_active_exit, NULL),
	[PPP_STATE_AWAIT_LINK] = PPP_STATE(modem_cellular_ppp_await_link_entry,
					   modem_cellular_ppp_await_link_run, NULL,
					   PPP_STATE_PARENT(PPP_STATE_ACTIVE)),
	[PPP_STATE_LINKED] = PPP_STATE(NULL, modem_cellular_ppp_linked_run,
				       modem_cellular_ppp_linked_exit,
				       PPP_STATE_PARENT(PPP_STATE_ACTIVE)),
	[PPP_STATE_OPEN_PIPE] = PPP_STATE(modem_cellular_ppp_open_pipe_entry,
					  modem_cellular_ppp_open_pipe_run, NULL,
					  PPP_STATE_PARENT(PPP_STATE_LINKED)),
	[PPP_STATE_AWAIT_ATTACH] = PPP_STATE(modem_cellular_ppp_await_attach_entry,
					     modem_cellular_ppp_await_attach_run, NULL,
					     PPP_STATE_PARENT(PPP_STATE_LINKED)),
	[PPP_STATE_DIAL] = PPP_STATE(modem_cellular_ppp_dial_entry, modem_cellular_ppp_dial_run,
				     NULL, PPP_STATE_PARENT(PPP_STATE_LINKED)),
	[PPP_STATE_DIAL_BACKOFF] = PPP_STATE(modem_cellular_ppp_dial_backoff_entry,
					     modem_cellular_ppp_dial_backoff_run, NULL,
					     PPP_STATE_PARENT(PPP_STATE_LINKED)),
	[PPP_STATE_CONNECTED] = PPP_STATE(modem_cellular_ppp_connected_entry,
					  modem_cellular_ppp_connected_run,
					  modem_cellular_ppp_connected_exit,
					  PPP_STATE_PARENT(PPP_STATE_LINKED)),
	[PPP_STATE_HANGUP] = PPP_STATE(modem_cellular_ppp_hangup_entry,
				       modem_cellular_ppp_hangup_run, NULL,
				       PPP_STATE_PARENT(PPP_STATE_ACTIVE)),
};

static const char *modem_cellular_ppp_state_name(const struct smf_state *state)
{
	return modem_cellular_ppp_state_names[state - modem_cellular_ppp_states];
}

static void modem_cellular_ppp_set_state(struct modem_cellular_ppp_data *data,
					 enum modem_cellular_ppp_state state)
{
	/* Each timer belongs to the state that started it */
	modem_cellular_ppp_stop_timer(data);
	data->script_running = NULL;
	smf_set_state(&data->smf, &modem_cellular_ppp_states[state]);
}

static void modem_cellular_ppp_run_event(struct modem_cellular_ppp_data *data,
					 enum modem_cellular_ppp_event evt, const void *ptr)
{
	const struct smf_state *previous = smf_get_current_leaf_state(&data->smf);

	/* Drop results of scripts the current state did not start */
	if (((evt == PPP_EVENT_SCRIPT_SUCCESS) || (evt == PPP_EVENT_SCRIPT_FAILED)) &&
	    (ptr != data->script_running)) {
		LOG_DBG("PPP script result dropped in %s", modem_cellular_ppp_state_name(previous));
		return;
	}

	/* Only a close requested on leaving LINKED is of interest, whatever state it completes
	 * in, the CMUX reporting a lost channel closed as well
	 */
	if (evt == PPP_EVENT_PIPE_CLOSED) {
		if (!data->pipe_closing) {
			return;
		}
		data->pipe_closing = false;
	}

	LOG_DBG("PPP event %s", modem_cellular_ppp_event_names[evt]);

	data->smf_event = evt;
	(void)smf_run_state(&data->smf);

	if (smf_get_current_leaf_state(&data->smf) != previous) {
		LOG_DBG("PPP switch from %s to %s", modem_cellular_ppp_state_name(previous),
			modem_cellular_ppp_state_name(smf_get_current_leaf_state(&data->smf)));
	}
}

static void modem_cellular_ppp_event_dispatch_handler(struct k_work *item);

static void modem_cellular_ppp_timeout_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_cellular_ppp_data *data =
		CONTAINER_OF(dwork, struct modem_cellular_ppp_data, timeout_work);
	uint32_t timer_gen;

	/* Restarted after it expired, but before this ran, so meant for later */
	if (k_work_delayable_remaining_get(dwork) != 0) {
		LOG_DBG("PPP timeout dropped");
		return;
	}

	timer_gen = data->timer_gen;

	/* Events that arrived before the timer expired are handled first, a timer they stop
	 * or restart having been meant for the state before them
	 */
	modem_cellular_ppp_event_dispatch_handler(&data->event_dispatch_work);
	if (data->timer_gen != timer_gen) {
		LOG_DBG("PPP timeout dropped");
		return;
	}

	modem_cellular_ppp_run_event(data, PPP_EVENT_TIMEOUT, NULL);
}

static enum modem_cellular_ppp_event
modem_cellular_ppp_signal_event(struct modem_cellular_ppp_data *data,
				enum modem_cellular_ppp_signal signal)
{
	switch (signal) {
	case PPP_SIGNAL_LINK_LOST:
		return PPP_EVENT_LINK_LOST;
	case PPP_SIGNAL_LINK:
		return PPP_EVENT_LINK_CHANGED;
	case PPP_SIGNAL_ATTACH:
		return PPP_EVENT_ATTACH_CHANGED;
	case PPP_SIGNAL_CALL_ENDED:
		return PPP_EVENT_CALL_ENDED;
	case PPP_SIGNAL_PPP_DEAD:
		return PPP_EVENT_PPP_DEAD;
	case PPP_SIGNAL_LINK_FAILED:
		return PPP_EVENT_LINK_FAILED;
	default:
		return atomic_get(&data->pm_resume) ? PPP_EVENT_RESUME : PPP_EVENT_SUSPEND;
	}
}

static void modem_cellular_ppp_event_dispatch_handler(struct k_work *item)
{
	struct modem_cellular_ppp_data *data =
		CONTAINER_OF(item, struct modem_cellular_ppp_data, event_dispatch_work);
	struct modem_cellular_event_pkg pkg;
	atomic_val_t signals;

	do {
		while (k_msgq_get(&data->event_queue, &pkg, K_NO_WAIT) == 0) {
			modem_cellular_ppp_run_event(data, pkg.event, pkg.ptr);
		}

		signals = atomic_clear(&data->signals);
		for (int i = 0; i < PPP_SIGNAL_COUNT; i++) {
			if ((signals & BIT(i)) != 0) {
				modem_cellular_ppp_run_event(
					data, modem_cellular_ppp_signal_event(data, i), NULL);
			}
		}
	} while (signals != 0);
}

static void modem_cellular_ppp_delegate_event_ptr(struct modem_cellular_ppp_data *data,
						  enum modem_cellular_ppp_event evt,
						  const void *ptr)
{
	struct modem_cellular_event_pkg pkg = {.event = evt, .ptr = ptr};

	if (k_msgq_put(&data->event_queue, &pkg, K_NO_WAIT) < 0) {
		LOG_WRN("PPP event %d dropped", evt);
		return;
	}

	modem_work_submit(&data->event_dispatch_work);
}

/* ------------------------------------------------------------------------- */
/* Device                                                                    */
/* ------------------------------------------------------------------------- */

int modem_cellular_ppp_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct modem_cellular_ppp_config *config = dev->config;
	struct modem_cellular_ppp_data *data = dev->data;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* The data call needs the LTE RF. The request also holds the runtime PM reference
		 * on the core, and fails on a missing APN before the modem is powered up.
		 */
		ret = modem_cellular_lte_rf_request(config->core);
		if (ret == -ENODATA) {
			/* No call can be made until an APN is set with cellular_set_apn() */
			LOG_WRN("No APN configured");
			net_mgmt_event_notify(NET_EVENT_CONN_IF_NO_CONFIGURATION,
					      modem_cellular_ppp_iface(data));
			return ret;
		}
		if (ret < 0) {
			LOG_ERR("Failed to resume %s (%d)", config->core->name, ret);
			return ret;
		}
		atomic_set(&data->pm_resume, 1);
		modem_cellular_ppp_signal(data, PPP_SIGNAL_PM);
		return 0;

	case PM_DEVICE_ACTION_SUSPEND:
		/* The suspend completes on the workqueue the state machine runs on */
		if (k_current_get() == modem_work_queue_thread_get()) {
			LOG_ERR("Cannot suspend from the modem workqueue");
			return -EDEADLK;
		}
		atomic_set(&data->pm_resume, 0);
		modem_cellular_ppp_signal(data, PPP_SIGNAL_PM);
		/* Leaving ACTIVE does not block, so this only waits for the workqueue and for the
		 * PPP channel to close, which CMUX completes within its retries whether the modem
		 * answers or not. Giving up would leave the state machine suspended while the
		 * device is still resumed.
		 */
		(void)k_sem_take(&data->suspended_sem, K_FOREVER);
		/* The core disables the LTE RF on the release, before the reference on it is
		 * released and it is suspended. The release is asynchronous, as the core suspend
		 * blocks until the modem is powered down.
		 */
		return modem_cellular_lte_rf_release(config->core);

	default:
		return -ENOTSUP;
	}
}

int modem_cellular_ppp_init(const struct device *dev)
{
	const struct modem_cellular_ppp_config *config = dev->config;
	const struct modem_cellular_config *core_config = config->core->config;
	const struct modem_cellular_vendor_config_common *vendor = core_config->vendor->common;
	struct modem_cellular_ppp_data *data = dev->data;

	__ASSERT_NO_MSG(vendor->scripts.dial != NULL);

	data->dev = dev;

	k_work_init(&data->event_dispatch_work, modem_cellular_ppp_event_dispatch_handler);
	k_msgq_init(&data->event_queue, (char *)data->event_buf, sizeof(data->event_buf[0]),
		    ARRAY_SIZE(data->event_buf));
	k_work_init_delayable(&data->timeout_work, modem_cellular_ppp_timeout_handler);
	k_sem_init(&data->suspended_sem, 0, 1);

	{
		/* The vendor dial script, its matches and run checks expect the core's data */
		const struct modem_chat_config chat_config = {
			.user_data = config->core->data,
			.receive_buf = data->chat_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->chat_receive_buf),
			.delimiter = (const uint8_t *)vendor->chat_delimiter,
			.delimiter_size = strlen(vendor->chat_delimiter),
			.filter = (const uint8_t *)vendor->chat_filter,
			.filter_size = (vendor->chat_filter != NULL) ? strlen(vendor->chat_filter)
								     : 0,
			.argv = data->chat_argv,
			.argv_size = ARRAY_SIZE(data->chat_argv),
		};

		modem_chat_init(&data->chat, &chat_config);
	}

	/* The vendor script reports to the core, so report to this device instead */
	data->dial_script = *vendor->scripts.dial;
	modem_chat_script_set_callback(&data->dial_script, modem_cellular_ppp_chat_callback);

	net_mgmt_init_event_callback(&data->ppp_dead_cb, modem_cellular_ppp_dead_handler,
				     NET_EVENT_PPP_PHASE_DEAD);

	smf_set_initial(&data->smf, &modem_cellular_ppp_states[PPP_STATE_SUSPENDED]);

	/* Suspended until the network interface takes a reference */
	pm_device_init_suspended(dev);
	return pm_device_runtime_enable(dev);
}
