/*
 * Copyright (c) 2025 Embeint Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/mfd/bq25190.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT ti_bq25190_sensor

struct sensor_bq25190_config {
	const struct device *mfd;
	uint8_t resolution;
	bool adcin_bias;
};

struct bs25190_adc_registers {
	/** Battery voltage */
	uint16_t v_bat;
	/** Thermistor */
	uint16_t ts;
	/** Battery current */
	uint16_t i_bat;
	/** ADCIN pin */
	uint16_t adc_in;
	/** IN pin (charger input) */
	uint16_t v_in;
	/** V_SYS voltage */
	uint16_t v_sys;
	/** IN pin current (charger input) */
	uint16_t i_in;
	/** Die temperature */
	uint16_t t_die;
};
BUILD_ASSERT(sizeof(struct bs25190_adc_registers) == 16);

struct sensor_bq25190_data {
	struct bs25190_adc_registers adc_regs;
	struct mfd_bq25190_cb int_cb;
	struct k_sem done;
	bool int_pin_available;
};

#define FLAG1_ADC_DONE 0x08

#define ADCCTRL0_ADC_EN              BIT(7)
#define ADCCTRL0_ADC_RATE_CONTINUOUS (0b00 << 5)
#define ADCCTRL0_ADC_RATE_ONE_SHOT   (0b01 << 5)
#define ADCCTRL0_ADC_RATE_1_SEC      (0b10 << 5)
#define ADCCTRL0_ADC_RATE_1_MIN      (0b11 << 5)
#define ADCCTRL0_ADC_SAMPLE_11BIT    (0b00 << 3)
#define ADCCTRL0_ADC_SAMPLE_10BIT    (0b01 << 3)
#define ADCCTRL0_ADC_SAMPLE_9BIT     (0b10 << 3)

#define ADCCTRL1_ADCIN_BIAS 0x02

#define ADC_DISABLE_TDIE  BIT(0)
#define ADC_DISABLE_ADCIN BIT(1)
#define ADC_DISABLE_TS    BIT(2)
#define ADC_DISABLE_VBAT  BIT(3)
#define ADC_DISABLE_VIN   BIT(4)
#define ADC_DISABLE_IBAT  BIT(5)
#define ADC_DISABLE_VSYS  BIT(6)
#define ADC_DISABLE_IIN   BIT(7)

LOG_MODULE_REGISTER(bq25190_sensor, CONFIG_SENSOR_BQ25190_LOG_LEVEL);

static void bq25190_interrupt(uint8_t flags[4], void *user_ctx)
{
	struct k_sem *done = user_ctx;

	if (flags[1] & FLAG1_ADC_DONE) {
		LOG_DBG("ADC_DONE interrupt");
		k_sem_give(done);
	}
}

int bq25190_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct sensor_bq25190_config *config = dev->config;
	struct sensor_bq25190_data *data = dev->data;
	bool sampled = false;
	uint8_t poll_ms = 5;
	uint16_t delay_ms;
	uint8_t val;
	int ret;

	/* Always read all ADC channels */
	ARG_UNUSED(chan);

	/* Measurement durations, expected first, measured in brackets:
	 *   Resolution 11 bits: 24ms (40ms)
	 *   Resolution 10 bits: 12ms (20ms)
	 *   Resolution  9 bits:  6ms (10ms)
	 *
	 * There are 3 channels enabled.
	 */
	switch (config->resolution) {
	case ADCCTRL0_ADC_SAMPLE_11BIT:
		delay_ms = 3 * 40;
		break;
	case ADCCTRL0_ADC_SAMPLE_10BIT:
		delay_ms = 3 * 20;
		break;
	default:
		delay_ms = 3 * 10;
		break;
	}

	/* Clear any pending interrupts */
	(void)k_sem_take(&data->done, K_NO_WAIT);

	/* Start the single shot sampling */
	LOG_DBG("Triggering single-shot ADC, resolution %02X, delay %d ms", config->resolution,
		delay_ms);
	val = ADCCTRL0_ADC_EN | ADCCTRL0_ADC_RATE_ONE_SHOT | config->resolution;
	ret = mfd_bq25190_reg_write(config->mfd, BQ25190_REG_ADCCTRL0, val);
	if (ret < 0) {
		return ret;
	}

	if (data->int_pin_available) {
		/* Wait for the interrupt to occur (same overall timeout as polling) */
		ret = k_sem_take(&data->done, K_MSEC(delay_ms + (10 * poll_ms)));
		sampled = ret == 0;
	} else {
		/* Wait until measurements should be complete */
		k_sleep(K_MSEC(delay_ms));

		/* Poll until ADC_DONE or timeout */
		for (int i = 0; i < 10; i++) {
			ret = mfd_bq25190_reg_read(config->mfd, BQ25190_REG_FLAG1, &val);
			if (ret < 0) {
				return ret;
			}
			if (val & FLAG1_ADC_DONE) {
				LOG_DBG("Sampling complete after %d ms", delay_ms + (i * poll_ms));
				sampled = true;
				break;
			}
			k_sleep(K_MSEC(poll_ms));
		}
	}

	if (!sampled) {
		LOG_WRN("ADC sampling timed out");
		return -EAGAIN;
	}

	/* Burst read all ADC results */
	ret = mfd_bq25190_reg_read_burst(config->mfd, BQ25190_REG_ADC_DATA_VBAT,
					 (void *)&data->adc_regs, sizeof(data->adc_regs));
	if (ret == 0) {
		LOG_HEXDUMP_DBG(&data->adc_regs, sizeof(data->adc_regs), "ADC Registers");
	}
	return ret;
}

int bq25190_channel_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	struct sensor_bq25190_data *data = dev->data;
	int32_t val_int;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		/* 1.25 mV step */
		val_int = 1250 * data->adc_regs.v_bat;
		break;
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_GAUGE_AVG_CURRENT:
		/* 1mA step */
		val_int = 1000 * (int16_t)data->adc_regs.i_bat;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		/* 0.5 degree step */
		val_int = 500000 * (int16_t)data->adc_regs.t_die;
		break;
	default:
		return -ENOTSUP;
	}

	return sensor_value_from_micro(val, val_int);
}

int sensor_bq25190_init(const struct device *dev)
{
	const struct sensor_bq25190_config *config = dev->config;
	struct sensor_bq25190_data *data = dev->data;
	uint8_t val;
	int ret;

	k_sem_init(&data->done, 0, 1);

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

	/* Register for callbacks if available */
	data->int_cb.interrupt = bq25190_interrupt;
	data->int_cb.user_ctx = &data->done;
	if (mfd_bq25190_register_callback(config->mfd, &data->int_cb) == 0) {
		data->int_pin_available = true;
	}

	/* Clear any previous flags (register is clear on read) */
	ret = mfd_bq25190_reg_read(config->mfd, BQ25190_REG_FLAG1, &val);
	if (ret < 0) {
		return ret;
	}

	/* Disable unused ADC channels to speed up sampling */
	val = ADC_DISABLE_ADCIN | ADC_DISABLE_TS | ADC_DISABLE_VIN | ADC_DISABLE_VSYS |
	      ADC_DISABLE_IIN;
	ret = mfd_bq25190_reg_write(config->mfd, BQ25190_REG_ADC_CHANNEL_DISABLE, val);
	if (ret < 0) {
		return ret;
	}

	/* Disable comparators, configure ADCIN bias */
	val = config->adcin_bias ? ADCCTRL1_ADCIN_BIAS : 0x00;
	return mfd_bq25190_reg_write(config->mfd, BQ25190_REG_ADCCTRL1, val);
}

static DEVICE_API(sensor, bq25190_driver_api) = {
	.sample_fetch = bq25190_sample_fetch,
	.channel_get = bq25190_channel_get,
};

#define SENSOR_BQ25190_DEFINE(n)                                                                   \
	static const struct sensor_bq25190_config sensor_bq25190_config##n = {                     \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(n)),                                           \
		.resolution = DT_INST_ENUM_IDX(n, adc_resolution) << 3,                            \
		.adcin_bias = DT_INST_PROP(n, adcin_bias),                                         \
	};                                                                                         \
	static struct sensor_bq25190_data sensor_bq25190_data##n;                                  \
	DEVICE_DT_INST_DEFINE(n, sensor_bq25190_init, NULL, &sensor_bq25190_data##n,               \
			      &sensor_bq25190_config##n, POST_KERNEL,                              \
			      CONFIG_SENSOR_BQ25190_INIT_PRIORITY, &bq25190_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_BQ25190_DEFINE)
