/*
 * Copyright (c) 2026 OpenAI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/socket.h>
#include <zephyr/ztest.h>

#define DNS_FALLBACK_QUERY "example.com"
#define DNS_FALLBACK_TIMEOUT_MS 12000
#define DNS_FALLBACK_WAIT K_SECONDS(15)

struct dns_fallback_result {
	struct k_sem done;
	enum dns_resolve_status final_status;
	bool got_address;
	bool all_done;
};

static void dns_fallback_cb(enum dns_resolve_status status,
			    struct dns_addrinfo *info,
			    void *user_data)
{
	struct dns_fallback_result *result = user_data;

	if (status == DNS_EAI_INPROGRESS && info != NULL &&
	    info->ai_family == AF_INET) {
		result->got_address = true;
		return;
	}

	result->final_status = status;

	if (status == DNS_EAI_ALLDONE) {
		result->all_done = true;
		k_sem_give(&result->done);
	} else if (status != DNS_EAI_INPROGRESS) {
		k_sem_give(&result->done);
	}
}

ZTEST(dns_fallback_real, test_second_dns_server_answers)
{
	struct dns_fallback_result result = {
		.final_status = DNS_EAI_CANCELED,
	};
	uint16_t dns_id;
	int ret;

	k_sem_init(&result.done, 0, 1);

	ret = dns_get_addr_info(DNS_FALLBACK_QUERY,
				DNS_QUERY_TYPE_A,
				&dns_id,
				dns_fallback_cb,
				&result,
				DNS_FALLBACK_TIMEOUT_MS);
	zassert_equal(ret, 0, "Cannot start DNS query (%d)", ret);

	ret = k_sem_take(&result.done, DNS_FALLBACK_WAIT);
	zassert_equal(ret, 0, "DNS query timed out before fallback completed");

	zassert_true(result.got_address,
		     "No A record received from fallback DNS server");
	zassert_true(result.all_done,
		     "DNS query did not complete successfully, status %d",
		     result.final_status);
}

ZTEST_SUITE(dns_fallback_real, NULL, NULL, NULL, NULL, NULL);
