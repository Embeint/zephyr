/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_sntp_client_sample, LOG_LEVEL_DBG);

#include <zephyr/net/socket.h>
#include <zephyr/net/socket_service.h>
#include <zephyr/net/sntp.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "net_sample_common.h"

static K_SEM_DEFINE(sntp_async_received, 0, 1);
static void sntp_service_handler(struct net_socket_service_event *pev);
static struct k_work_delayable timeout_work;

NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(service_sntp_async, sntp_service_handler, 1);

int dns_query(const char *host, uint16_t port, int family, int socktype, struct sockaddr *addr,
	      socklen_t *addrlen)
{
	struct addrinfo hints = {
		.ai_family = family,
		.ai_socktype = socktype,
	};
	struct addrinfo *res = NULL;
	char addr_str[INET6_ADDRSTRLEN] = {0};
	int rv;

	/* Perform DNS query */
	rv = getaddrinfo(host, NULL, &hints, &res);
	if (rv < 0) {
		LOG_ERR("getaddrinfo failed (%d, errno %d)", rv, errno);
		return rv;
	}
	/* Store the first result */
	*addr = *res->ai_addr;
	*addrlen = res->ai_addrlen;
	/* Free the allocated memory */
	freeaddrinfo(res);
	/* Store the port */
	net_sin(addr)->sin_port = htons(port);
	/* Print the found address */
	inet_ntop(addr->sa_family, &net_sin(addr)->sin_addr, addr_str, sizeof(addr_str));
	LOG_INF("%s -> %s", host, addr_str);
	return 0;
}

static void sntp_timeout_handler(struct k_work *work)
{
	/* Close the service */
	LOG_WRN("SNTP query timed out, cancelling");
	sntp_close_async(&service_sntp_async);
}

static void sntp_service_handler(struct net_socket_service_event *pev)
{
	struct sntp_time s_time;
	int rc;

	/* Cancel timeout handler */
	k_work_cancel_delayable(&timeout_work);

	/* Read the response from the socket */
	rc = sntp_read_async(pev, &s_time);
	if (rc != 0) {
		LOG_ERR("Failed to read SNTP response (%d)", rc);
		return;
	}

	/* Close the service */
	sntp_close_async(&service_sntp_async);

	LOG_INF("SNTP Time: %llu (async)", s_time.seconds);
}

static void do_sntp(int family)
{
	char *family_str = family == AF_INET ? "IPv4" : "IPv6";
	struct sntp_ctx ctx;
	struct sockaddr addr;
	socklen_t addrlen;
	int rv;

	/* Get SNTP server */
	rv = dns_query(CONFIG_NET_SAMPLE_SNTP_SERVER_ADDRESS, CONFIG_NET_SAMPLE_SNTP_SERVER_PORT,
		       family, SOCK_DGRAM, &addr, &addrlen);
	if (rv != 0) {
		LOG_ERR("Failed to lookup %s SNTP server (%d)", family_str, rv);
		return;
	}

	for (int i = 0; i < 5; i++) {
		rv = sntp_init_async(&ctx, &addr, addrlen, &service_sntp_async);
		if (rv < 0) {
			LOG_ERR("Failed to initialise SNTP context (%d)", rv);
			goto end;
		}

		LOG_INF("Sending query");
		rv = sntp_send_async(&ctx);
		LOG_INF("Query sent");
		if (rv < 0) {
			LOG_ERR("Failed to send SNTP query (%d)", rv);
			goto end;
		}

		k_work_schedule(&timeout_work, i == 3 ? K_MSEC(1) : K_MSEC(500));
		k_sleep(K_SECONDS(1));
	}
	return;
end:
	sntp_close_async(&service_sntp_async);
}

int main(void)
{
	k_work_init_delayable(&timeout_work, sntp_timeout_handler);

	wait_for_network();

	do_sntp(AF_INET);
	return 0;
}
