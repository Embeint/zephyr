/*
 * Copyright (c) 2025 Embeint Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/mfd/bq25190.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT ti_bq25190_gpio

#define BQ25190_GPIO_PINS 4

struct gpio_bq25190_config {
	struct gpio_driver_config common;
	const struct device *mfd;
};

struct gpio_bq25190_data {
	struct gpio_driver_data common;
	uint8_t open_drain_outputs;
};

#define STAT3_GPIO1_STAT BIT(7)
#define STAT3_GPIO2_STAT BIT(6)
#define STAT3_GPIO3_STAT BIT(5)
#define STAT3_GPIO4_STAT BIT(4)

static inline int gpio_bq25190_configure(const struct device *dev, gpio_pin_t pin,
					 gpio_flags_t flags)
{
	const struct gpio_bq25190_config *config = dev->config;
	struct gpio_bq25190_data *data = dev->data;
	uint8_t control_reg;
	uint8_t control_val;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (pin >= BQ25190_GPIO_PINS) {
		return -EINVAL;
	}
	control_reg = BQ25190_REG_GPIO1_CTRL + pin;

	if (flags & GPIO_INPUT) {
		if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
			/* Input doesn't support pull resistors */
			return -ENOTSUP;
		}
		control_val = BQ25190_GPIO_CTRL_INPUT_LEVEL_SENSITIVE;
	} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
		if (flags & GPIO_LINE_OPEN_DRAIN) {
			control_val = BQ25190_GPIO_CTRL_OPEN_DRAIN_HIGH;
			data->open_drain_outputs |= BIT(pin);
		} else {
			control_val = BQ25190_GPIO_CTRL_PUSH_PULL_HIGH;
			data->open_drain_outputs &= ~BIT(pin);
		}
	} else if (flags & GPIO_OUTPUT) {
		control_val = BQ25190_GPIO_CTRL_FORCED_LOW;
	} else {
		/* GPIO_DISCONNECTED: Closest we have is INPUT_LEVEL_SENSITIVE */
		control_val = BQ25190_GPIO_CTRL_INPUT_LEVEL_SENSITIVE;
	}

	return mfd_bq25190_reg_write(config->mfd, control_reg, control_val);
}

static int gpio_bq25190_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_bq25190_config *config = dev->config;
	uint8_t reg;
	int ret;

	/* Read status register */
	ret = mfd_bq25190_reg_read(config->mfd, BQ25190_REG_STAT3, &reg);

	/* Decode bitfields */
	*value = ((reg & STAT3_GPIO1_STAT) ? BIT(0) : 0) | ((reg & STAT3_GPIO2_STAT) ? BIT(1) : 0) |
		 ((reg & STAT3_GPIO3_STAT) ? BIT(2) : 0) | ((reg & STAT3_GPIO4_STAT) ? BIT(3) : 0);
	return ret;
}

static int gpio_bq25190_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					    gpio_port_value_t value)
{
	const struct gpio_bq25190_config *config = dev->config;
	struct gpio_bq25190_data *data = dev->data;
	uint8_t control_reg;
	uint8_t control_val;
	int ret;

	for (size_t idx = 0; idx < BQ25190_GPIO_PINS; idx++) {
		if ((mask & BIT(idx)) == 0U) {
			/* Not part of mask */
			continue;
		}

		/* Compute register address and control register value */
		control_reg = BQ25190_REG_GPIO1_CTRL + idx;
		if ((value & BIT(idx)) != 0U) {
			control_val = (data->open_drain_outputs & BIT(idx))
					      ? BQ25190_GPIO_CTRL_OPEN_DRAIN_HIGH
					      : BQ25190_GPIO_CTRL_PUSH_PULL_HIGH;
		} else {
			control_val = BQ25190_GPIO_CTRL_FORCED_LOW;
		}

		/* Write value to register*/
		ret = mfd_bq25190_reg_write(config->mfd, control_reg, control_val);
		if (ret != 0U) {
			return ret;
		}
	}

	return ret;
}

static int gpio_bq25190_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_bq25190_port_set_masked_raw(dev, pins, pins);
}

static int gpio_bq25190_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_bq25190_port_set_masked_raw(dev, pins, 0U);
}

static int gpio_bq25190_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_bq25190_config *config = dev->config;
	struct gpio_bq25190_data *data = dev->data;
	uint8_t control_reg;
	uint8_t control_val;
	int ret;

	for (size_t idx = 0; idx < BQ25190_GPIO_PINS; idx++) {
		if ((pins & BIT(idx)) == 0U) {
			/* Not part of requested toggle */
			continue;
		}
		/* Get the current output state */
		control_reg = BQ25190_REG_GPIO1_CTRL + idx;
		ret = mfd_bq25190_reg_read(config->mfd, control_reg, &control_val);
		if (ret != 0U) {
			return ret;
		}

		/* Invert the current output state */
		switch (control_val) {
		case BQ25190_GPIO_CTRL_FORCED_LOW:
			control_val = (data->open_drain_outputs & BIT(idx))
					      ? BQ25190_GPIO_CTRL_OPEN_DRAIN_HIGH
					      : BQ25190_GPIO_CTRL_PUSH_PULL_HIGH;
			break;
		case BQ25190_GPIO_CTRL_OPEN_DRAIN_HIGH:
		case BQ25190_GPIO_CTRL_PUSH_PULL_HIGH:
			control_val = BQ25190_GPIO_CTRL_FORCED_LOW;
			break;
		default:
			/* Not an output */
			continue;
		}

		/* Write the updated output value */
		ret = mfd_bq25190_reg_write(config->mfd, control_reg, control_val);
		if (ret != 0U) {
			return ret;
		}
	}
	return 0;
}

static DEVICE_API(gpio, gpio_bq25190_api) = {
	.pin_configure = gpio_bq25190_configure,
	.port_get_raw = gpio_bq25190_port_get_raw,
	.port_set_masked_raw = gpio_bq25190_port_set_masked_raw,
	.port_set_bits_raw = gpio_bq25190_port_set_bits_raw,
	.port_clear_bits_raw = gpio_bq25190_port_clear_bits_raw,
	.port_toggle_bits = gpio_bq25190_port_toggle_bits,
};

static int gpio_bq25190_init(const struct device *dev)
{
	const struct gpio_bq25190_config *config = dev->config;

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

	return 0;
}

#define GPIO_BQ25190_DEFINE(n)                                                                     \
	static const struct gpio_bq25190_config gpio_config##n = {                                 \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(n))};                                          \
                                                                                                   \
	static struct gpio_bq25190_data gpio_data##n;                                              \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_bq25190_init, NULL, &gpio_data##n, &gpio_config##n,          \
			      POST_KERNEL, CONFIG_GPIO_BQ25190_INIT_PRIORITY, &gpio_bq25190_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_BQ25190_DEFINE)
