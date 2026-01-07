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
	bool buck_reset_force_1v8;
	bool buck_gpio_controlled_voltage;
};

struct mfd_bq25190_data {
	const struct mfd_bq25190_config *config;
	struct k_mutex access;
	struct gpio_callback int_cb;
	struct k_work int_work;
	sys_slist_t callback_list;
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

int mfd_bq25190_register_callback(const struct device *dev, struct mfd_bq25190_cb *cb)
{
	const struct mfd_bq25190_config *config = dev->config;
	struct mfd_bq25190_data *data = dev->data;

	if (config->int_gpio.port == NULL) {
		return -ENODEV;
	}

	sys_slist_append(&data->callback_list, &cb->node);
	return 0;
}

static void mfd_bq25190_int_handle(struct k_work *work)
{
	struct mfd_bq25190_data *data = CONTAINER_OF(work, struct mfd_bq25190_data, int_work);
	const uint8_t reg = BQ25190_REG_FLAG0;
	struct mfd_bq25190_cb *cb;
	uint8_t flags[4];
	int ret;

	/* Read the 4 flag registers */
	ret = i2c_write_read_dt(&data->config->i2c, &reg, sizeof(uint8_t), flags, sizeof(flags));
	if (ret < 0) {
		/* What to do here? */
		return;
	}

	/* Notify interested parties of interrupt flags */
	SYS_SLIST_FOR_EACH_CONTAINER(&data->callback_list, cb, node) {
		cb->interrupt(flags, cb->user_ctx);
	}
}

static void mfd_bq25190_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				      uint32_t pins)
{
	struct mfd_bq25190_data *data = CONTAINER_OF(cb, struct mfd_bq25190_data, int_cb);

	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Schedule work to handle the interrupt */
	k_work_submit(&data->int_work);
}

static int mfd_bq25190_init(const struct device *dev)
{
	const struct mfd_bq25190_config *config = dev->config;
	struct mfd_bq25190_data *data = dev->data;
	uint8_t reg;
	int ret;

	data->config = config;
	k_mutex_init(&data->access);
	sys_slist_init(&data->callback_list);
	k_work_init(&data->int_work, mfd_bq25190_int_handle);
	gpio_init_callback(&data->int_cb, mfd_bq25190_gpio_callback, BIT(config->int_gpio.pin));

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

	if (config->buck_reset_force_1v8) {
		/* Soft-resetting the device re-enables the default behaviour of setting the BUCK
		 * voltage from GPIO3 and GPIO4. If GPIO3 or GPIO4 were configured to be high
		 * outputs, the BUCK voltage could immediately change to any of 1.8V, 3.3V, 2.5V
		 * or 1.2V upon reset. Force GPIO3 and GPIO4 to the low state so that upon reset,
		 * the default value of Selection 1 is used (1.8V).
		 */
		ret = i2c_reg_write_byte_dt(&config->i2c, BQ25190_REG_GPIO3_CTRL,
					    BQ25190_GPIO_CTRL_FORCED_LOW);
		if (ret < 0) {
			return -EIO;
		}
		ret = i2c_reg_write_byte_dt(&config->i2c, BQ25190_REG_GPIO4_CTRL,
					    BQ25190_GPIO_CTRL_FORCED_LOW);
		if (ret < 0) {
			return -EIO;
		}
		/* Give some minimal time for the logic level to change */
		k_sleep(K_MSEC(1));
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

	if (config->int_gpio.port) {
		/* Configure the interrupts (if pin provided) after reset */
		ret = gpio_add_callback(config->int_gpio.port, &data->int_cb);
		if (ret < 0) {
			return ret;
		}
		ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
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
		.buck_reset_force_1v8 = DT_INST_PROP(n, buck_reset_force_1v8),                     \
		.buck_gpio_controlled_voltage = DT_INST_PROP(n, buck_gpio_controlled_voltage),     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, mfd_bq25190_init, NULL, &mfd_bq25190_data##n,                     \
			      &mfd_bq25190_config##n, POST_KERNEL,                                 \
			      CONFIG_MFD_BQ25190_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TI_BQ25190_DEFINE);
