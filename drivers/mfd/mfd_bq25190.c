/*
 * Copyright (c) 2025 Embeint Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq25190

#include <zephyr/devicetree.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/bq25190.h>

struct mfd_bq25190_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec enable_gpio;
	bool buck_gpio_controlled_voltage;
};

struct mfd_bq25190_data {
	struct k_mutex access;
};

#define BQ25190_SHIP_RST_SW_RST 0x80

#define BQ25190_PART_INFORMATION_EXPECTED 0x01

int mfd_bq25190_reg_read(const struct device *dev, uint8_t reg, uint8_t *data)
{
	const struct mfd_bq25190_config *config = dev->config;

	return i2c_reg_read_byte_dt(&config->i2c, reg, data);
}

int mfd_bq25190_reg_read_burst(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	const struct mfd_bq25190_config *config = dev->config;

	return i2c_write_read_dt(&config->i2c, &reg, sizeof(uint8_t), data, len);
}

int mfd_bq25190_reg_write(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct mfd_bq25190_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->i2c, reg, data);
}

int mfd_bq25190_reg_update(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t data)
{
	const struct mfd_bq25190_config *config = dev->config;
	struct mfd_bq25190_data *dev_data = dev->data;
	int ret;

	/* Mutex around "update_byte" since this is a MFD, and there is the potential for multiple
	 * child drivers to call this function on the same register at the same time.
	 */
	k_mutex_lock(&dev_data->access, K_FOREVER);
	ret = i2c_reg_update_byte_dt(&config->i2c, reg, mask, data);
	k_mutex_unlock(&dev_data->access);
	return ret;
}

static int mfd_bq25190_init(const struct device *dev)
{
	const struct mfd_bq25190_config *config = dev->config;
	struct mfd_bq25190_data *data = dev->data;
	uint8_t reg;
	int ret;

	k_mutex_init(&data->access);

	if (!i2c_is_ready_dt(&config->i2c)) {
		return -ENODEV;
	}

	/* Read and validate part information */
	ret = i2c_reg_read_byte_dt(&config->i2c, BQ25190_REG_PART_INFORMATION, &reg);
	if (ret || (reg != BQ25190_PART_INFORMATION_EXPECTED)) {
		return -EIO;
	}

	/* GPIO Configuration */
	if (config->enable_gpio.port) {
		gpio_pin_configure_dt(&config->enable_gpio, GPIO_OUTPUT_ACTIVE);
	}
	if (config->int_gpio.port) {
		gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	}

	/* Soft reset the device */
	ret = i2c_reg_write_byte_dt(&config->i2c, BQ25190_REG_SHIP_RST, BQ25190_SHIP_RST_SW_RST);
	if (ret || (reg != BQ25190_PART_INFORMATION_EXPECTED)) {
		return -EIO;
	}

	if (!config->buck_gpio_controlled_voltage) {
		/* Disable the default behaviour of setting the BUCK voltage from GPIO3 and
		 * GPIO4. We do this here instead of in the GPIO driver as the GPIO driver
		 * may not be compiled while the regulator driver is.
		 */
		ret = i2c_reg_write_byte_dt(&config->i2c, BQ25190_REG_GPIO3_CTRL,
					    BQ25190_GPIO_CTRL_INPUT_LEVEL_SENSITIVE);
		if (ret < 0) {
			return -EIO;
		}
		ret = i2c_reg_write_byte_dt(&config->i2c, BQ25190_REG_GPIO4_CTRL,
					    BQ25190_GPIO_CTRL_INPUT_LEVEL_SENSITIVE);
		if (ret < 0) {
			return -EIO;
		}
	}

	return ret;
}

#define TI_BQ25190_DEFINE(n)                                                                       \
	static struct mfd_bq25190_data mfd_bq25190_data##n;                                        \
                                                                                                   \
	static const struct mfd_bq25190_config mfd_bq25190_config##n = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),                           \
		.enable_gpio = GPIO_DT_SPEC_INST_GET_OR(n, enable_gpios, {0}),                     \
		.buck_gpio_controlled_voltage = DT_INST_PROP(n, buck_gpio_controlled_voltage),     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, mfd_bq25190_init, NULL, &mfd_bq25190_data##n,                     \
			      &mfd_bq25190_config##n, POST_KERNEL,                                 \
			      CONFIG_MFD_BQ25190_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TI_BQ25190_DEFINE);
