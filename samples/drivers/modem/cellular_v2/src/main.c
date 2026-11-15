/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ppp.h>
#include <zephyr/pm/device_runtime.h>

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cellular_v2_sample, LOG_LEVEL_INF);

#define EVENT_REGISTERED BIT(0)
#define EVENT_SUSPENDED  BIT(1)
#define EVENT_FAILED     BIT(2)
#define EVENT_READY      BIT(3)
#define EVENT_CONNECTED  BIT(4)
#define EVENT_DNS_SERVER BIT(5)
#define EVENT_DNS_DONE   BIT(6)

#define READY_TIMEOUT_S      60
#define SUSPEND_TIMEOUT_S    30
/* PPP configures its DNS servers along with its address */
#define DNS_SERVER_TIMEOUT_S 5
#define DNS_QUERY_TIMEOUT_MS 10000

static K_EVENT_DEFINE(modem_events);

MODEM_PIPELINK_DT_DECLARE(DT_ALIAS(modem), MODEM_CELLULAR_PIPELINK_PPP);

static struct net_mgmt_event_callback ipv4_cb;
static struct net_mgmt_event_callback conn_cb;
static struct net_mgmt_event_callback dns_cb;

static bool is_registered(enum cellular_registration_status status)
{
	return (status == CELLULAR_REGISTRATION_REGISTERED_HOME) ||
	       (status == CELLULAR_REGISTRATION_REGISTERED_ROAMING);
}

/* Runs on the modem workqueue, so only logs and wakes the main thread */
static void modem_event_handler(const struct device *dev, enum cellular_event event,
				const void *payload, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (event) {
	case CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED: {
		const struct cellular_evt_registration_status *evt = payload;

		LOG_INF("Registration status %d", evt->status);
		if (is_registered(evt->status)) {
			k_event_post(&modem_events, EVENT_REGISTERED);
		}
		break;
	}
	case CELLULAR_EVENT_SIM_STATUS_CHANGED: {
		const struct cellular_evt_sim_status *evt = payload;

		LOG_INF("SIM status %d", evt->status);
		break;
	}
	case CELLULAR_EVENT_MODEM_READY:
		k_event_post(&modem_events, EVENT_READY);
		break;
	case CELLULAR_EVENT_MODEM_SUSPENDED:
		LOG_INF("Modem powered down");
		k_event_post(&modem_events, EVENT_SUSPENDED);
		break;
	case CELLULAR_EVENT_MODEM_FAILED: {
		const struct cellular_evt_modem_failed *evt = payload;

		LOG_ERR("Modem failed (%d) and was powered down", evt->reason);
		k_event_post(&modem_events, EVENT_FAILED);
		break;
	}
	default:
		break;
	}
}

/* The data call is up once PPP has negotiated an IPv4 address */
static void ipv4_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			       struct net_if *iface)
{
	char addr[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(iface);

	if ((mgmt_event != NET_EVENT_IPV4_ADDR_ADD) || (cb->info == NULL)) {
		return;
	}

	LOG_INF("IPv4 address %s",
		net_addr_ntop(NET_AF_INET, cb->info, addr, sizeof(addr)) != NULL ? addr : "?");
	k_event_post(&modem_events, EVENT_CONNECTED);
}

/* The PPP device gives up once the modem could not be brought up */
static void conn_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			       struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event == NET_EVENT_CONN_IF_FATAL_ERROR) {
		LOG_ERR("Interface failed, take it down and up again to retry");
		k_event_post(&modem_events, EVENT_FAILED);
	}
}

static void dns_server_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				     struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	/* An unchanged set of servers is only reported as reconfigured */
	if ((mgmt_event == NET_EVENT_DNS_SERVER_ADD) ||
	    (mgmt_event == NET_EVENT_DNS_SERVERS_RECONFIGURED)) {
		k_event_post(&modem_events, EVENT_DNS_SERVER);
	}
}

static void dns_result_handler(enum dns_resolve_status status, struct dns_addrinfo *info,
			       void *user_data)
{
	char addr[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(user_data);

	switch (status) {
	case DNS_EAI_INPROGRESS:
		/* One call per address */
		if ((info != NULL) && (info->ai_family == NET_AF_INET)) {
			LOG_INF("%s resolved to %s", CONFIG_SAMPLE_DNS_HOSTNAME,
				net_addr_ntop(
					NET_AF_INET,
					&net_sin((struct net_sockaddr *)&info->ai_addr_storage)
						 ->sin_addr,
					addr, sizeof(addr)) != NULL
					? addr
					: "?");
		}
		break;
	case DNS_EAI_ALLDONE:
		k_event_post(&modem_events, EVENT_DNS_DONE);
		break;
	default:
		LOG_ERR("DNS query failed (%d)", status);
		k_event_post(&modem_events, EVENT_DNS_DONE);
		break;
	}
}

static void print_modem_info(const struct device *modem)
{
	static const struct {
		enum cellular_modem_info_type type;
		const char *name;
	} fields[] = {
		{CELLULAR_MODEM_INFO_MANUFACTURER, "Manufacturer"},
		{CELLULAR_MODEM_INFO_MODEL_ID, "Model"},
		{CELLULAR_MODEM_INFO_FW_VERSION, "Firmware"},
		{CELLULAR_MODEM_INFO_IMEI, "IMEI"},
		{CELLULAR_MODEM_INFO_SIM_IMSI, "IMSI"},
		{CELLULAR_MODEM_INFO_SIM_ICCID, "ICCID"},
	};
	char value[66];

	for (size_t i = 0; i < ARRAY_SIZE(fields); i++) {
		if (cellular_get_modem_info(modem, fields[i].type, value, sizeof(value)) == 0) {
			LOG_INF("%-12s %s", fields[i].name, value);
		}
	}
}

/*
 * Select the APN from the home network in the SIM's IMSI, its MCC followed by its 2 or 3 digit
 * MNC. Only a few networks are listed, as an example: MVNOs and IoT SIMs can share the range of
 * their host network but need an APN of their own.
 */
static void select_apn(const struct device *modem)
{
	static const struct {
		const char *imsi_prefix;
		const char *apn;
	} apns[] = {
		{"50501", "telstra.internet"},   /* Telstra, Australia */
		{"310260", "fast.t-mobile.com"}, /* T-Mobile, United States */
		{"26201", "internet.telekom"},   /* Telekom, Germany */
	};
	const char *apn = CONFIG_MODEM_CELLULAR_APN;
	char imsi[16];
	int ret;

	if (cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_SIM_IMSI, imsi, sizeof(imsi)) == 0) {
		for (size_t i = 0; i < ARRAY_SIZE(apns); i++) {
			if (strncmp(imsi, apns[i].imsi_prefix, strlen(apns[i].imsi_prefix)) == 0) {
				apn = apns[i].apn;
				break;
			}
		}
	}

	/* An empty APN has the network select the default APN of the subscription */
	LOG_INF("Using APN \"%s\"", apn);
	ret = cellular_set_apn(modem, apn);
	if ((ret != 0) && (ret != -EALREADY)) {
		LOG_ERR("Failed to set the APN (%d)", ret);
	}
}

/*
 * Open and close the PPP channel, CMUX DLCI 2, without a call on it. Closing it completes either
 * way, as CMUX closes it locally once its retries are spent, so the time taken shows whether the
 * modem answered.
 */
static void test_ppp_channel_close(void)
{
	struct modem_pipelink *link =
		MODEM_PIPELINK_DT_GET(DT_ALIAS(modem), MODEM_CELLULAR_PIPELINK_PPP);
	struct modem_pipe *pipe;
	int64_t start;
	int ret;

	if (!modem_pipelink_is_connected(link)) {
		LOG_WRN("PPP channel not available");
		return;
	}
	pipe = modem_pipelink_get_pipe(link);

	start = k_uptime_get();
	ret = modem_pipe_open(pipe, K_SECONDS(2));
	LOG_INF("PPP channel opened (%d) in %lld ms", ret, k_uptime_delta(&start));
	if (ret != 0) {
		return;
	}

	start = k_uptime_get();
	ret = modem_pipe_close(pipe, K_SECONDS(2));
	LOG_INF("PPP channel closed (%d) in %lld ms, without an answer it takes %d ms", ret,
		k_uptime_delta(&start), 4 * CONFIG_MODEM_CMUX_T1_TIMEOUT);
}

/* Wait for the modem to power down, which a failed modem already has */
static void wait_for_power_down(uint32_t events)
{
	if (((events & EVENT_FAILED) == 0U) && (k_event_wait(&modem_events, EVENT_SUSPENDED, false,
							     K_SECONDS(SUSPEND_TIMEOUT_S)) == 0U)) {
		LOG_WRN("Modem not powered down within %d s", SUSPEND_TIMEOUT_S);
	}
}

/* Resolve CONFIG_SAMPLE_DNS_HOSTNAME over the data call, once PPP has configured DNS servers */
static void resolve_hostname(void)
{
	uint16_t dns_id;
	int ret;

	if (k_event_wait(&modem_events, EVENT_DNS_SERVER, false, K_SECONDS(DNS_SERVER_TIMEOUT_S)) ==
	    0U) {
		LOG_WRN("No DNS server configured");
		return;
	}

	ret = dns_get_addr_info(CONFIG_SAMPLE_DNS_HOSTNAME, DNS_QUERY_TYPE_A, &dns_id,
				dns_result_handler, NULL, DNS_QUERY_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("Failed to start the DNS query (%d)", ret);
		return;
	}

	/* The resolver reports a timeout itself */
	(void)k_event_wait(&modem_events, EVENT_DNS_DONE, false,
			   K_MSEC(DNS_QUERY_TIMEOUT_MS + 1000));
}

/*
 * Bring the PPP interface up on the registered modem, resolve a hostname over the data call,
 * then take the interface down again. The interface holds its own LTE RF request, so the
 * modem stays registered once it is down.
 */
static uint32_t resolve_over_ppp(struct net_if *iface)
{
	uint32_t events;
	int ret;

	k_event_clear(&modem_events, EVENT_CONNECTED | EVENT_DNS_SERVER | EVENT_DNS_DONE);

	LOG_INF("Bringing the PPP interface up");
	ret = net_if_up(iface);
	if (ret != 0) {
		LOG_ERR("Failed to bring the interface up (%d)", ret);
		return 0U;
	}

	events = k_event_wait(&modem_events, EVENT_CONNECTED | EVENT_FAILED, false,
			      K_SECONDS(CONFIG_SAMPLE_CONNECT_TIMEOUT_S));
	if ((events & EVENT_CONNECTED) == 0U) {
		if (events == 0U) {
			LOG_WRN("Not connected within %d s", CONFIG_SAMPLE_CONNECT_TIMEOUT_S);
		}
	} else {
		resolve_hostname();
	}

	/* Hangs up, while the LTE RF request below keeps the modem registered */
	LOG_INF("Taking the PPP interface down");
	ret = net_if_down(iface);
	if (ret != 0) {
		LOG_ERR("Failed to take the interface down (%d)", ret);
	}

	return events & EVENT_FAILED;
}

/*
 * Phase 1: control the modem directly, powering it up with a runtime PM reference and enabling
 * the LTE RF with a request, only bringing the PPP interface up once registered.
 */
static void run_controlled_phase(const struct device *modem, struct net_if *iface)
{
	uint32_t events = 0U;
	int ret;

	k_event_clear(&modem_events, UINT32_MAX);

	/* The modem is powered up while a runtime PM reference is held on it */
	ret = pm_device_runtime_get(modem);
	if (ret != 0) {
		LOG_ERR("Failed to resume the modem (%d)", ret);
		return;
	}

	/* The modem info has been read once the modem is ready for commands */
	events = k_event_wait(&modem_events, EVENT_READY | EVENT_FAILED, false,
			      K_SECONDS(READY_TIMEOUT_S));
	if ((events & EVENT_READY) == 0U) {
		if (events == 0U) {
			LOG_WRN("Modem not ready within %d s", READY_TIMEOUT_S);
		}
		goto power_down;
	}

	LOG_INF("Modem has booted, RF not yet enabled");
	print_modem_info(modem);

	if (IS_ENABLED(CONFIG_SAMPLE_PPP_CHANNEL_CLOSE_TEST)) {
		test_ppp_channel_close();
	}

	/* If the modem has GNSS support, it could be used here without activating LTE */

	/* The APN can only be changed while the LTE RF is not requested */
	select_apn(modem);

	/* The LTE RF is enabled once the modem is ready for commands.
	 * We could also request the PPP link here instead.
	 */
	ret = modem_cellular_lte_rf_request(modem);
	if (ret != 0) {
		LOG_ERR("Failed to request the LTE RF (%d)", ret);
	} else {
		LOG_INF("LTE RF requested, waiting for registration");
		events = k_event_wait(&modem_events, EVENT_REGISTERED | EVENT_FAILED, false,
				      K_SECONDS(CONFIG_SAMPLE_REGISTRATION_TIMEOUT_S));
		if ((events & EVENT_REGISTERED) != 0U) {
			events |= resolve_over_ppp(iface);
		} else if (events == 0U) {
			LOG_WRN("Not registered within %d s", CONFIG_SAMPLE_REGISTRATION_TIMEOUT_S);
		}

		k_sleep(K_SECONDS(2));

		/* The LTE RF is disabled again before the modem is powered down */
		(void)modem_cellular_lte_rf_release(modem);
	}

power_down:
	k_event_clear(&modem_events, EVENT_SUSPENDED);
	ret = pm_device_runtime_put(modem);
	if (ret != 0) {
		LOG_ERR("Failed to suspend the modem (%d)", ret);
		return;
	}

	wait_for_power_down(events);
}

/*
 * Phase 2: only bring the PPP interface up. The driver powers the modem up, enables the LTE RF
 * and dials the data call by itself, and undoes it all once the interface is taken down.
 */
static void run_network_phase(struct net_if *iface)
{
	uint32_t events;
	int ret;

	k_event_clear(&modem_events, UINT32_MAX);

	LOG_INF("Bringing the PPP interface up");
	ret = net_if_up(iface);
	if (ret != 0) {
		/* -ENODATA without an APN, which phase 1 set */
		LOG_ERR("Failed to bring the interface up (%d)", ret);
		return;
	}

	events = k_event_wait(&modem_events, EVENT_CONNECTED | EVENT_FAILED, false,
			      K_SECONDS(CONFIG_SAMPLE_CONNECT_TIMEOUT_S));
	if ((events & EVENT_CONNECTED) != 0U) {
		resolve_hostname();
		LOG_INF("Connected, staying up for %d s", CONFIG_SAMPLE_CONNECTED_TIME_S);
		events |= k_event_wait(&modem_events, EVENT_FAILED, false,
				       K_SECONDS(CONFIG_SAMPLE_CONNECTED_TIME_S));
	} else if (events == 0U) {
		LOG_WRN("Not connected within %d s", CONFIG_SAMPLE_CONNECT_TIMEOUT_S);
	}

	/* Taking the interface down hangs up and powers the modem down */
	LOG_INF("Taking the PPP interface down");
	k_event_clear(&modem_events, EVENT_SUSPENDED);
	ret = net_if_down(iface);
	if (ret != 0) {
		LOG_ERR("Failed to take the interface down (%d)", ret);
		return;
	}

	wait_for_power_down(events);
}

int main(void)
{
	const struct device *modem = DEVICE_DT_GET(DT_ALIAS(modem));
	struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
	int ret;

	if (!device_is_ready(modem)) {
		LOG_ERR("Modem %s not ready", modem->name);
		return 0;
	}

	if (iface == NULL) {
		LOG_ERR("No PPP interface");
		return 0;
	}

	net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);
	net_mgmt_init_event_callback(&conn_cb, conn_event_handler, NET_EVENT_CONN_IF_FATAL_ERROR);
	net_mgmt_add_event_callback(&conn_cb);
	net_mgmt_init_event_callback(&dns_cb, dns_server_event_handler,
				     NET_EVENT_DNS_SERVER_ADD | NET_EVENT_DNS_SERVERS_RECONFIGURED);
	net_mgmt_add_event_callback(&dns_cb);

	ret = cellular_set_callback(
		modem,
		CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED | CELLULAR_EVENT_SIM_STATUS_CHANGED |
			CELLULAR_EVENT_MODEM_SUSPENDED | CELLULAR_EVENT_MODEM_READY |
			CELLULAR_EVENT_MODEM_FAILED,
		modem_event_handler, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to set the event callback (%d)", ret);
		return 0;
	}

	LOG_INF("Phase 1: controlling the modem directly");
	run_controlled_phase(modem, iface);

	LOG_INF("Phase 2: bringing the network interface up");
	run_network_phase(iface);

	LOG_INF("Done");
	return 0;
}
