/*
 * Copyright (c) 2023 FTP Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT voltage_divider

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/voltage_divider.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/battery.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(voltage, CONFIG_SENSOR_LOG_LEVEL);

struct voltage_config {
	struct voltage_divider_dt_spec voltage;
	struct gpio_dt_spec gpio_power;
	uint32_t sample_delay_us;
	int32_t ocv_lookup_table[BATTERY_OCV_TABLE_LEN];
};

struct voltage_data {
	struct adc_sequence sequence;
	k_timeout_t earliest_sample;
	int32_t voltage_mv;
	uint32_t soc;
};

static int fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct voltage_config *config = dev->config;
	struct voltage_data *data = dev->data;
	int16_t adc_raw;
	int ret;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
	case SENSOR_CHAN_GAUGE_VOLTAGE:
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
	case SENSOR_CHAN_ALL:
		break;
	default:
		return -ENOTSUP;
	}

	data->sequence.buffer = &adc_raw;
	data->sequence.buffer_size = sizeof(adc_raw);

	/* Wait until sampling is valid */
	k_sleep(data->earliest_sample);

	/* Read the ADC */
	ret = adc_read(config->voltage.port.dev, &data->sequence);
	if (ret != 0) {
		LOG_ERR("adc_read: %d", ret);
	}

	/* Convert to a real voltage */
	data->voltage_mv = adc_raw;
	ret = adc_raw_to_millivolts_dt(&config->voltage.port, &data->voltage_mv);
	if (ret != 0) {
		LOG_ERR("raw_to_mv: %d", ret);
		return ret;
	}

	/* Scale according to the voltage divider */
	(void)voltage_divider_scale_dt(&config->voltage, &data->voltage_mv);

	/* Convert voltage to SoC if chemistry specified */
	if (config->ocv_lookup_table[0] != -1) {
		data->soc = battery_soc_lookup(config->ocv_lookup_table, 1000 * data->voltage_mv);
	}

	/* Print the reading and conversion */
	LOG_DBG("ADC: %d, Voltage: %dmV (%d %%)", adc_raw, data->voltage_mv, data->soc / 1000);
	return 0;
}

static int get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	const struct voltage_config *config = dev->config;
	struct voltage_data *data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = data->voltage_mv / 1000;
		val->val2 = (data->voltage_mv * 1000) % 1000000;
		break;
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		/* Can't convert to SoC without a lookup table */
		if (config->ocv_lookup_table[0] == -1) {
			return -ENOTSUP;
		}
		val->val1 = data->soc / 1000;
		val->val2 = (data->soc * 1000) % 1000000;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static const struct sensor_driver_api voltage_api = {
	.sample_fetch = fetch,
	.channel_get = get,
};

static int pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct voltage_config *config = dev->config;
	struct voltage_data *data = dev->data;
	int ret;

	if (config->gpio_power.port == NULL) {
		/* No work to do */
		return 0;
	}

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = gpio_pin_set_dt(&config->gpio_power, 1);
		if (ret != 0) {
			LOG_ERR("failed to set GPIO for PM resume");
		}
		data->earliest_sample = K_TIMEOUT_ABS_TICKS(
			k_uptime_ticks() + k_us_to_ticks_ceil32(config->sample_delay_us));
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = gpio_pin_set_dt(&config->gpio_power, 0);
		if (ret != 0) {
			LOG_ERR("failed to set GPIO for PM suspend");
		}
		break;
	case PM_DEVICE_ACTION_TURN_ON:
		ret = gpio_pin_configure_dt(&config->gpio_power, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("failed to configure GPIO for PM on");
		}
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static int voltage_init(const struct device *dev)
{
	const struct voltage_config *config = dev->config;
	struct voltage_data *data = dev->data;
	int ret;

	/* Default value to use if `power-gpios` does not exist */
	data->earliest_sample = K_TIMEOUT_ABS_TICKS(0);

	if (!adc_is_ready_dt(&config->voltage.port)) {
		LOG_ERR("ADC is not ready");
		return -ENODEV;
	}

	if (config->gpio_power.port != NULL) {
		if (!gpio_is_ready_dt(&config->gpio_power)) {
			LOG_ERR("Power GPIO is not ready");
			return -ENODEV;
		}
	}

	ret = adc_channel_setup_dt(&config->voltage.port);
	if (ret != 0) {
		LOG_ERR("setup: %d", ret);
		return ret;
	}

	ret = adc_sequence_init_dt(&config->voltage.port, &data->sequence);
	if (ret != 0) {
		LOG_ERR("sequence init: %d", ret);
		return ret;
	}

	return pm_device_driver_init(dev, pm_action);
}

#define VOLTAGE_INIT(inst)                                                                         \
	static struct voltage_data voltage_##inst##_data;                                          \
                                                                                                   \
	static const struct voltage_config voltage_##inst##_config = {                             \
		.voltage = VOLTAGE_DIVIDER_DT_SPEC_GET(DT_DRV_INST(inst)),                         \
		.gpio_power = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),                    \
		.sample_delay_us = DT_INST_PROP(inst, power_on_sample_delay_us),                   \
		.ocv_lookup_table =                                                                \
			BATTERY_OCV_TABLE_DT_GET(DT_DRV_INST(inst), ocv_capacity_table_0),         \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, pm_action);                                                 \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, &voltage_init, PM_DEVICE_DT_INST_GET(inst),             \
				     &voltage_##inst##_data, &voltage_##inst##_config,             \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &voltage_api);

DT_INST_FOREACH_STATUS_OKAY(VOLTAGE_INIT)
