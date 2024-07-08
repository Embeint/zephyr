/*
 * Copyright (c) 2024 Embeint Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zbus/zbus.h>
#include <zephyr/ztest.h>
#include <zephyr/ztest_assert.h>

struct msg {
	int x;
};

ZBUS_CHAN_DEFINE(chan, struct msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

ZTEST(publish_stats, test_channel_metadata)
{
	struct msg *cval, val = {0};
	k_ticks_t pub_time;

	/* Application boot, no publishes */
	zassert_equal(0, zbus_chan_publish_count(&chan));
	zassert_equal(0, zbus_chan_publish_time(&chan));
	zassert_equal(0, zbus_chan_avg_publish_period(&chan));

	/* Should be no different after a second of runtime */
	k_sleep(K_SECONDS(1));
	zassert_equal(0, zbus_chan_publish_count(&chan));
	zassert_equal(0, zbus_chan_publish_time(&chan));
	zassert_equal(0, zbus_chan_avg_publish_period(&chan));

	/* Normal publish */
	zassert_equal(0, zbus_chan_pub(&chan, &val, K_NO_WAIT));
	zassert_equal(1, zbus_chan_publish_count(&chan));
	zassert_within(k_uptime_ticks(), zbus_chan_publish_time(&chan), 10);
	zassert_within(1000, zbus_chan_avg_publish_period(&chan), 10);

	/* Push 4 times in quick succession, wait for 2 second boundary */
	for (int i = 0; i < 4; i++) {
		zassert_equal(0, zbus_chan_pub(&chan, &val, K_NO_WAIT));
		pub_time = k_uptime_ticks();
	}
	k_sleep(K_TIMEOUT_ABS_MS(2000));
	zassert_equal(5, zbus_chan_publish_count(&chan));
	zassert_within(pub_time, zbus_chan_publish_time(&chan), 10);
	zassert_within(400, zbus_chan_avg_publish_period(&chan), 10);

	/* Channel claim and finish does not update metadata by default */
	zassert_equal(0, zbus_chan_claim(&chan, K_NO_WAIT));
	zassert_equal(0, zbus_chan_finish(&chan));

	zassert_equal(0, zbus_chan_claim(&chan, K_NO_WAIT));
	cval = zbus_chan_msg(&chan);
	cval->x = 1000;
	zassert_equal(0, zbus_chan_finish(&chan));
	zassert_equal(5, zbus_chan_publish_count(&chan));
	zassert_within(pub_time, zbus_chan_publish_time(&chan), 10);

	/* Manually update publish statistics with claim */
	zassert_equal(0, zbus_chan_claim(&chan, K_NO_WAIT));
	zbus_chan_update_publish_metadata(&chan);
	pub_time = k_uptime_ticks();
	zassert_equal(0, zbus_chan_finish(&chan));

	k_sleep(K_TIMEOUT_ABS_MS(3000));
	zassert_equal(6, zbus_chan_publish_count(&chan));
	zassert_within(pub_time, zbus_chan_publish_time(&chan), 10);
	zassert_within(500, zbus_chan_avg_publish_period(&chan), 10);
}

ZTEST_SUITE(publish_stats, NULL, NULL, NULL, NULL, NULL);
