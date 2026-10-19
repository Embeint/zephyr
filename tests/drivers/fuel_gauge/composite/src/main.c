/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

struct test_source_data {
	int32_t voltage_uv;
	int32_t current_ua;
};

static struct test_source_data source_data;
static struct test_source_data charging_source_data;
static enum charger_status test_charger_status = CHARGER_STATUS_NOT_CHARGING;

static int test_source_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	return 0;
}

static int test_source_channel_get(const struct device *dev, enum sensor_channel chan,
				   struct sensor_value *val)
{
	struct test_source_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		return sensor_value_from_micro(val, data->voltage_uv);
	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		return sensor_value_from_micro(val, data->current_ua);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, test_source_api) = {
	.sample_fetch = test_source_sample_fetch,
	.channel_get = test_source_channel_get,
};

static int test_charger_get_prop(const struct device *dev, charger_prop_t prop,
				 union charger_propval *val)
{
	ARG_UNUSED(dev);

	if (prop != CHARGER_PROP_STATUS) {
		return -ENOTSUP;
	}
	val->status = test_charger_status;
	return 0;
}

static DEVICE_API(charger, test_charger_api) = {
	.get_property = test_charger_get_prop,
};

DEVICE_DT_DEFINE(DT_NODELABEL(test_source), NULL, NULL, &source_data, NULL,
		 POST_KERNEL, 89, &test_source_api);
DEVICE_DT_DEFINE(DT_NODELABEL(test_source_charging), NULL, NULL, &charging_source_data, NULL,
		 POST_KERNEL, 89, &test_source_api);
DEVICE_DT_DEFINE(DT_NODELABEL(test_charger), NULL, NULL, NULL, NULL, POST_KERNEL, 89,
		 &test_charger_api);

static uint8_t get_soc(const struct device *gauge)
{
	union fuel_gauge_prop_val value;

	zassert_ok(fuel_gauge_get_prop(gauge, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT, &value));
	return value.relative_state_of_charge_pct;
}

/* Clear charging estimator history left by a previous test, settling at the given OCV */
static void reset_charging_gauge(const struct device *gauge, int32_t voltage_uv)
{
	charging_source_data.voltage_uv = voltage_uv;
	charging_source_data.current_ua = 0;
	test_charger_status = CHARGER_STATUS_FULL;
	(void)get_soc(gauge);
	/* Let the post-FULL relaxation complete */
	k_sleep(K_MSEC(150));
	test_charger_status = CHARGER_STATUS_NOT_CHARGING;
	(void)get_soc(gauge);
}

ZTEST(fuel_gauge_composite, test_ocv_tracks_loaded_voltage)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge));

	source_data.voltage_uv = 3920000;
	source_data.current_ua = 360000;
	zassert_equal(92, get_soc(gauge));
}

ZTEST(fuel_gauge_composite, test_charging_lifecycle)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge_charging));
	uint8_t soc;

	/* Initialise from OCV while not charging. */
	reset_charging_gauge(gauge, 3600000);
	zassert_equal(60, get_soc(gauge));

	/* Low charging current continues to use OCV. */
	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 10000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(92, get_soc(gauge));

	/* Refresh the OCV baseline, then integrate significant charging current. */
	charging_source_data.voltage_uv = 3600000;
	charging_source_data.current_ua = 0;
	test_charger_status = CHARGER_STATUS_NOT_CHARGING;
	zassert_equal(60, get_soc(gauge));

	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 360000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(60, get_soc(gauge));
	k_sleep(K_MSEC(120));
	soc = get_soc(gauge);
	zassert_true((soc >= 61) && (soc <= 62), "unexpected charged SoC: %u", soc);

	/* Discharge current is not integrated, but advances the OCV transition. */
	charging_source_data.voltage_uv = 3200000;
	charging_source_data.current_ua = -360000;
	test_charger_status = CHARGER_STATUS_NOT_CHARGING;
	soc = get_soc(gauge);
	zassert_true((soc >= 61) && (soc <= 62), "unexpected held SoC: %u", soc);
	k_sleep(K_MSEC(50));
	{
		uint8_t relaxing_soc = get_soc(gauge);

		zassert_true((relaxing_soc > 20) && (relaxing_soc < soc),
			     "unexpected relaxing SoC: %u", relaxing_soc);
	}
	k_sleep(K_MSEC(70));
	zassert_equal(20, get_soc(gauge));

	/* After relaxation, readings return to corrected OCV directly. */
	charging_source_data.voltage_uv = 3700000;
	charging_source_data.current_ua = 0;
	zassert_equal(70, get_soc(gauge));

	/* Charger termination is an authoritative full-charge calibration. */
	charging_source_data.voltage_uv = 3900000;
	test_charger_status = CHARGER_STATUS_FULL;
	zassert_equal(100, get_soc(gauge));
}

ZTEST(fuel_gauge_composite, test_charging_no_idle_integration)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge_charging));
	uint8_t soc;

	reset_charging_gauge(gauge, 3600000);

	/* Idle time before charging starts must not be integrated as charge. */
	k_sleep(K_MSEC(500));
	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 360000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(60, get_soc(gauge));

	/* Same for the gap between leaving relaxation and charging resuming. */
	k_sleep(K_MSEC(120));
	soc = get_soc(gauge);
	zassert_true((soc >= 61) && (soc <= 62), "unexpected charged SoC: %u", soc);
	charging_source_data.voltage_uv = 3600000;
	charging_source_data.current_ua = 0;
	test_charger_status = CHARGER_STATUS_NOT_CHARGING;
	(void)get_soc(gauge);
	k_sleep(K_MSEC(500));
	zassert_equal(60, get_soc(gauge));
	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 360000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(60, get_soc(gauge));
}

ZTEST(fuel_gauge_composite, test_charging_long_gap_restarts_integration)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge_charging));
	uint8_t soc;

	reset_charging_gauge(gauge, 3600000);

	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 360000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(60, get_soc(gauge));
	k_sleep(K_MSEC(120));
	soc = get_soc(gauge);
	zassert_true((soc >= 61) && (soc <= 62), "unexpected charged SoC: %u", soc);

	/* A gap longer than 5 minutes is not integrated, corrected OCV is used instead. */
	k_sleep(K_SECONDS(301));
	zassert_equal(92, get_soc(gauge));

	/* Integration resumes from the OCV estimate at the reading after the gap. */
	k_sleep(K_MSEC(120));
	zassert_equal(93, get_soc(gauge));
}

ZTEST(fuel_gauge_composite, test_charging_small_steps_accumulate)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge_charging));
	int64_t start;

	reset_charging_gauge(gauge, 3600000);

	/*
	 * 12 mA on 1000 uAh is 1/3 millipercent per ms, so each short poll step (k_sleep
	 * of 1 ms lasts 1-2 ms) truncates to zero unless the remainder is carried.
	 * Integrating for 3600 ms gives 1.2%.
	 */
	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 12000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(60, get_soc(gauge));
	start = k_uptime_get();
	while ((k_uptime_get() - start) < 3600) {
		k_sleep(K_MSEC(1));
		(void)get_soc(gauge);
	}
	zassert_equal(61, get_soc(gauge));
}

ZTEST(fuel_gauge_composite, test_charging_relaxation_from_charge_end)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge_charging));
	uint8_t soc;

	reset_charging_gauge(gauge, 3600000);

	charging_source_data.voltage_uv = 3920000;
	charging_source_data.current_ua = 360000;
	test_charger_status = CHARGER_STATUS_CHARGING;
	zassert_equal(60, get_soc(gauge));
	k_sleep(K_MSEC(120));
	soc = get_soc(gauge);
	zassert_true((soc >= 61) && (soc <= 62), "unexpected charged SoC: %u", soc);

	/* First reading after the relaxation time has passed since charging ended. */
	k_sleep(K_MSEC(150));
	charging_source_data.voltage_uv = 3200000;
	charging_source_data.current_ua = 0;
	test_charger_status = CHARGER_STATUS_NOT_CHARGING;
	zassert_equal(20, get_soc(gauge));
}

ZTEST(fuel_gauge_composite, test_charging_relaxation_after_full)
{
	const struct device *gauge = DEVICE_DT_GET(DT_NODELABEL(test_gauge_charging));
	uint8_t soc;

	reset_charging_gauge(gauge, 3600000);

	charging_source_data.voltage_uv = 3900000;
	test_charger_status = CHARGER_STATUS_FULL;
	zassert_equal(100, get_soc(gauge));

	/* Unplugging does not drop straight to the loaded OCV estimate. */
	charging_source_data.voltage_uv = 3700000;
	charging_source_data.current_ua = -100000;
	test_charger_status = CHARGER_STATUS_NOT_CHARGING;
	soc = get_soc(gauge);
	zassert_true(soc >= 99, "unexpected unplugged SoC: %u", soc);
	k_sleep(K_MSEC(50));
	soc = get_soc(gauge);
	zassert_true((soc > 70) && (soc < 99), "unexpected relaxing SoC: %u", soc);
	k_sleep(K_MSEC(70));
	zassert_equal(70, get_soc(gauge));
}

ZTEST_SUITE(fuel_gauge_composite, NULL, NULL, NULL, NULL, NULL);
