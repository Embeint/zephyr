/*
 * Copyright (c) 2025 Embeint Pty Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT telit_le910cx_gnss

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/modem/at/user_pipe.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "gnss_nmea0183.h"
#include "gnss_parse.h"

LOG_MODULE_REGISTER(le910cx_gnss, CONFIG_GNSS_LOG_LEVEL);

#define PIPELINK_CONNECTED 0

struct le910cx_gnss_config {
	const struct device *modem;
	struct modem_pipelink *pipelink;
	const char *lna_req;
};

struct le910cx_gnss_data {
	struct modem_chat modem_chat_ctx;
	struct k_work_delayable location_poll;
	struct k_work pipe_release;
	struct k_sem pipe_connected;
	uint64_t next_poll;
	uint32_t fix_interval;
	atomic_t state;
	uint8_t gnss_receive_buf[256];
	uint8_t *gnss_argv_buf[16];
	uint32_t last_fix_info;
};

static void le910cx_gnss_on_gpsacp(struct modem_chat *chat, char **argv, uint16_t argc,
				   void *user_data);

static void le910cx_gnss_location_poll_result(struct modem_chat *chat,
					      enum modem_chat_script_result result,
					      void *user_data);

__maybe_unused static uint16_t gnss_lna_cmd(const uint8_t **request, void *user_data)
{
	const struct device *dev = user_data;
	const struct le910cx_gnss_config *config = dev->config;

	LOG_DBG("LNA request: %s", config->lna_req);
	*request = config->lna_req;
	return strlen(*request);
}

MODEM_CHAT_MATCH_DEFINE(ok_match, "OK", "", NULL);
MODEM_CHAT_MATCH_DEFINE(gpsacp_match, "$GPSACP: ", ",", le910cx_gnss_on_gpsacp);
MODEM_CHAT_MATCHES_DEFINE(abort_matches, MODEM_CHAT_MATCH("ERROR", "", NULL));

MODEM_CHAT_SCRIPT_CMDS_DEFINE(gnss_enable_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_FN(gnss_lna_cmd, ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSP=1", ok_match));
MODEM_CHAT_SCRIPT_CMDS_DEFINE(gnss_query_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSACP", gpsacp_match));
MODEM_CHAT_SCRIPT_CMDS_DEFINE(gnss_disable_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSP=0", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(gnss_enable_chat_script, gnss_enable_script_cmds, abort_matches, NULL, 2);
MODEM_CHAT_SCRIPT_DEFINE(gnss_query_chat_script, gnss_query_script_cmds, abort_matches,
			 le910cx_gnss_location_poll_result, 1);
MODEM_CHAT_SCRIPT_DEFINE(gnss_disable_chat_script, gnss_disable_script_cmds, abort_matches, NULL,
			 2);

static void le910cx_gnss_location_poll(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct le910cx_gnss_data *data =
		CONTAINER_OF(delayable, struct le910cx_gnss_data, location_poll);
	int rc;

	LOG_DBG("Polling acquired location");
	/* Schedule the next poll attempt */
	data->next_poll += data->fix_interval;
	k_work_schedule(&data->location_poll, K_TIMEOUT_ABS_MS(data->next_poll));

	rc = modem_at_user_pipe_claim(&data->modem_chat_ctx, K_NO_WAIT);
	if (rc < 0) {
		LOG_WRN("Failed to claim user pipe for poll");
		return;
	}

	rc = modem_chat_run_script_async(&data->modem_chat_ctx, &gnss_query_chat_script);
	if (rc < 0) {
		LOG_WRN("Failed to start query script");
		modem_at_user_pipe_release();
	}
}

static void le910cx_gnss_pipe_release(struct k_work *work)
{
	ARG_UNUSED(work);

	modem_at_user_pipe_release();
}

static void le910cx_gnss_parse_lat_lon(char *val, int64_t *ndeg, bool latitude)
{
	bool negative;
	/* While the number format matches NMEA0183, GPSACP places the cardinal
	 * direction at the end of the number, not as a separate parameter. e.g.
	 *      ,15259.3141E, insted of ,15259.3141,E,
	 * So before calling the NMEA parsing function, we need to pre-cache the
	 * last character of the string and delete it.
	 *
	 *  Latitude:  ddmm.mmmm
	 * Longitude: dddmm.mmmm
	 */
	if (latitude) {
		switch (val[9]) {
		case 'N':
			negative = false;
			break;
		case 'S':
			negative = true;
			break;
		default:
			LOG_WRN("Unexpected %s format %s", "latitude", val);
			return;
		}
		val[9] = '\0';
	} else {
		switch (val[10]) {
		case 'E':
			negative = false;
			break;
		case 'W':
			negative = true;
			break;
		default:
			LOG_WRN("Unexpected %s format %s", "longitude", val);
			return;
		}
		val[10] = '\0';
	}

	if (gnss_nmea0183_ddmm_mmmm_to_ndeg(val, ndeg) == 0) {
		if (negative) {
			*ndeg = -*ndeg;
		}
	}
}

static void le910cx_gnss_on_gpsacp(struct modem_chat *chat, char **argv, uint16_t argc,
				   void *user_data)
{
	const struct device *dev = user_data;
	struct le910cx_gnss_data *dev_data = dev->data;
	struct gnss_data data = {0};
	int64_t parsed_ll;
	int32_t fix_type;
	int rc;

	if (argc < 12) {
		LOG_DBG("Unexpected $GPSACP format (%d params)", argc);
		return;
	}

	/* ARGV indicies
	 *  [0] = Match ($GPSACP: )
	 *  [1] = UTC time (hhmmss.sss)
	 *  [2] = Latitude
	 *  [3] = Longitude
	 *  [4] = hdop
	 *  [5] = Altitude
	 *  [6] = Fix type
	 *  [7] = Heading (ddd.mm)
	 *  [8] = Speed (km/hr)
	 *  [9] = Speed (knots)
	 * [10] = UTC Date (ddmmyy)
	 * [11] = Number of satellites (GPS)
	 * [12] = Number of satellites (GLONASS)
	 */

	/* Check timestamp against last query */
	rc = strtol(argv[1], NULL, 10);
	if (rc == dev_data->last_fix_info) {
		/* GPSACP contains old data */
		LOG_DBG("Ignoring old data");
		data.info.fix_status = GNSS_FIX_STATUS_NO_FIX;
		data.info.fix_quality = GNSS_FIX_QUALITY_INVALID;
		goto publish;
	}
	dev_data->last_fix_info = rc;

	rc = gnss_parse_atoi(argv[6], 10, &fix_type);
	if ((rc != 0) || (fix_type < 2)) {
		/* No fix information */
		data.info.fix_status = GNSS_FIX_STATUS_NO_FIX;
		data.info.fix_quality = GNSS_FIX_QUALITY_INVALID;
		goto publish;
	} else if (fix_type == 2) {
		/* 2D fix */
		data.info.fix_status = GNSS_FIX_STATUS_ESTIMATED_FIX;
		data.info.fix_quality = GNSS_FIX_QUALITY_ESTIMATED;
	} else {
		/* 3D fix */
		data.info.fix_status = GNSS_FIX_STATUS_GNSS_FIX;
		data.info.fix_quality = GNSS_FIX_QUALITY_GNSS_SPS;
		if (gnss_parse_dec_to_milli(argv[5], &parsed_ll) == 0) {
			data.nav_data.altitude = (int32_t)parsed_ll;
		}
	}

	/* Parse UTC time */
	gnss_nmea0183_parse_hhmmss(argv[1], &data.utc);
	gnss_nmea0183_parse_ddmmyy(argv[10], &data.utc);

	/* Parse coordinates */
	le910cx_gnss_parse_lat_lon(argv[2], &data.nav_data.latitude, true);
	le910cx_gnss_parse_lat_lon(argv[3], &data.nav_data.longitude, false);

	/* Parse speed */
	if (gnss_nmea0183_knots_to_mms(argv[9], &parsed_ll) == 0) {
		data.nav_data.speed = (uint32_t)parsed_ll;
	}
	/* Ignore heading for now, it is not encoded in a NMEA standard form */
	(void)argv[7];

	/* Parse HDOP */
	if (gnss_parse_dec_to_milli(argv[4], &parsed_ll) == 0) {
		data.info.hdop = (uint16_t)parsed_ll;
	}

	/* Parse (total) number of satellites */
	data.info.satellites_cnt = strtol(argv[11], NULL, 10) + strtol(argv[12], NULL, 10);

publish:
	gnss_publish_data(dev, &data);
}

static void le910cx_gnss_location_poll_result(struct modem_chat *chat,
					      enum modem_chat_script_result result, void *user_data)
{
	const struct device *dev = user_data;
	struct le910cx_gnss_data *data = dev->data;

	LOG_DBG("Location poll script complete");
	k_work_submit(&data->pipe_release);
}

void le910cx_pipelink_callback(struct modem_pipelink *link, enum modem_pipelink_event event,
			       void *user_data)
{
	const struct device *dev = user_data;
	struct le910cx_gnss_data *data = dev->data;

	/* GNSS pipe (DLCI4) is currently ignored in favor of polling due to an inability
	 * to get data to be output on the DLCI channel.
	 */
	switch (event) {
	case MODEM_PIPELINK_EVENT_CONNECTED:
		LOG_DBG("GNSS pipe connected");
		atomic_set_bit(&data->state, PIPELINK_CONNECTED);
		break;

	case MODEM_PIPELINK_EVENT_DISCONNECTED:
		LOG_DBG("GNSS pipe disconnected");
		atomic_clear_bit(&data->state, PIPELINK_CONNECTED);
		break;

	default:
		break;
	}
}

static int le910cx_gnss_pm_control(const struct device *dev, enum pm_device_action action)
{
	const struct le910cx_gnss_config *config = dev->config;
	struct le910cx_gnss_data *data = dev->data;
	int rc = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		k_work_cancel_delayable(&data->location_poll);
		rc = modem_at_user_pipe_claim(&data->modem_chat_ctx, K_SECONDS(1));
		if (rc == 0) {
			rc = modem_chat_run_script(&data->modem_chat_ctx,
						   &gnss_disable_chat_script);
			if (rc < 0) {
				LOG_WRN("Failed to disable GNSS");
			}
			modem_at_user_pipe_release();
		} else {
			LOG_WRN("Failed to claim user pipe, cannot disable GNSS");
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Require the user GNSS pipe to be connected */
		if (!atomic_test_bit(&data->state, PIPELINK_CONNECTED)) {
			return -ENODEV;
		}
		/* Claim the AT user pipe */
		rc = modem_at_user_pipe_claim(&data->modem_chat_ctx, K_MSEC(100));
		if (rc < 0) {
			LOG_DBG("Failed to claim AT pipe");
			(void)pm_device_runtime_put(config->modem);
			return rc;
		}
		/* Enable the GNSS functionality */
		rc = modem_chat_run_script(&data->modem_chat_ctx, &gnss_enable_chat_script);
		modem_at_user_pipe_release();
		if (rc == 0) {
			data->next_poll = k_uptime_get() + data->fix_interval;
			k_work_schedule(&data->location_poll, K_TIMEOUT_ABS_MS(data->next_poll));
		} else {
			(void)pm_device_runtime_put(config->modem);
		}
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		break;
	}

	return rc;
}

static int le910cx_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
	struct le910cx_gnss_data *data = dev->data;

	data->fix_interval = fix_interval_ms;
	return 0;
}

static int le910cx_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
	struct le910cx_gnss_data *data = dev->data;

	*fix_interval_ms = data->fix_interval;
	return 0;
}

static int le910cx_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	*systems = (GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
		    GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_SBAS);
	return 0;
}

static int le910cx_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	*systems = (GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
		    GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_SBAS);
	return 0;
}

static int le910cx_gnss_init(const struct device *dev)
{
	const struct le910cx_gnss_config *config = dev->config;
	struct le910cx_gnss_data *data = dev->data;
	struct modem_chat_config chat_cfg = {
		.user_data = (void *)dev,
		.receive_buf = data->gnss_receive_buf,
		.receive_buf_size = sizeof(data->gnss_receive_buf),
		.delimiter = "\r",
		.delimiter_size = sizeof("\r") - 1,
		.filter = "\n",
		.filter_size = sizeof("\n") - 1,
		.argv = data->gnss_argv_buf,
		.argv_size = ARRAY_SIZE(data->gnss_argv_buf),
	};

	if (!device_is_ready(config->modem)) {
		LOG_DBG("Parent modem %s not ready", config->modem->name);
		return -ENODEV;
	}

	data->fix_interval = 1000;
	k_sem_init(&data->pipe_connected, 0, 1);
	k_work_init(&data->pipe_release, le910cx_gnss_pipe_release);
	k_work_init_delayable(&data->location_poll, le910cx_gnss_location_poll);
	modem_chat_init(&data->modem_chat_ctx, &chat_cfg);
	modem_pipelink_attach(config->pipelink, le910cx_pipelink_callback, (void *)dev);
	return pm_device_driver_init(dev, le910cx_gnss_pm_control);
}

static DEVICE_API(gnss, gnss_api) = {
	.set_fix_rate = le910cx_set_fix_rate,
	.get_fix_rate = le910cx_get_fix_rate,
	.get_enabled_systems = le910cx_get_enabled_systems,
	.get_supported_systems = le910cx_get_supported_systems,
};

#define LE910CX_GNSS(inst)                                                                         \
	MODEM_PIPELINK_DT_DECLARE(DT_INST_PARENT(inst), gnss_pipe);                                \
	static const struct le910cx_gnss_config le910cx_gnss_cfg_##inst = {                        \
		.modem = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                      \
		.pipelink = MODEM_PIPELINK_DT_GET(DT_INST_PARENT(inst), gnss_pipe),                \
		.lna_req = COND_CODE_1(DT_INST_PROP(inst, active_antenna), ("AT$GPSELNA=1"),       \
				       ("AT$GPSELNA=0")),                                          \
	};                                                                                         \
                                                                                                   \
	static struct le910cx_gnss_data le910cx_gnss_data_##inst;                                  \
	PM_DEVICE_DT_INST_DEFINE(inst, le910cx_gnss_pm_control);                                   \
	DEVICE_DT_INST_DEFINE(inst, le910cx_gnss_init, PM_DEVICE_DT_INST_GET(inst),                \
			      &le910cx_gnss_data_##inst, &le910cx_gnss_cfg_##inst, POST_KERNEL,    \
			      CONFIG_GNSS_INIT_PRIORITY, &gnss_api);

DT_INST_FOREACH_STATUS_OKAY(LE910CX_GNSS)
