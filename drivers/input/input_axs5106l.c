/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT chipsourcetek_axs5106l

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(axs5106l, CONFIG_INPUT_LOG_LEVEL);

#define AXS5106L_REG_TOUCH_DATA 0x01
#define AXS5106L_REG_ID         0x08

/* Touch data: gesture, point count, then 6 bytes per point (XH, XL, YH, YL, ...) */
#define AXS5106L_TOUCH_DATA_LEN 14
#define AXS5106L_POINTS_POS     1
#define AXS5106L_POINTS_MASK    0x0F
#define AXS5106L_P1_XH_POS      2
#define AXS5106L_P1_XL_POS      3
#define AXS5106L_P1_YH_POS      4
#define AXS5106L_P1_YL_POS      5
#define AXS5106L_COORD_H_MASK   0x0F

#define AXS5106L_ID_LEN 3

#define AXS5106L_RESET_ASSERT_MS 10
#define AXS5106L_RESET_DELAY_MS  50

struct axs5106l_config {
	struct input_touchscreen_common_config common;
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec reset_gpio;
};

INPUT_TOUCH_STRUCT_CHECK(struct axs5106l_config);

struct axs5106l_data {
	const struct device *dev;
	struct k_work work;
	struct gpio_callback int_gpio_cb;
	bool pressed;
};

static int axs5106l_read_reg(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	const struct axs5106l_config *config = dev->config;
	int ret;

	/* The controller expects a stop condition between the address write and the read */
	ret = i2c_write_dt(&config->i2c, &reg, sizeof(reg));
	if (ret < 0) {
		return ret;
	}

	return i2c_read_dt(&config->i2c, buf, len);
}

static int axs5106l_process(const struct device *dev)
{
	struct axs5106l_data *data = dev->data;
	uint8_t buf[AXS5106L_TOUCH_DATA_LEN];
	uint16_t x, y;
	uint8_t points;
	int ret;

	ret = axs5106l_read_reg(dev, AXS5106L_REG_TOUCH_DATA, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Could not read touch data: %d", ret);
		return ret;
	}

	LOG_HEXDUMP_DBG(buf, sizeof(buf), "touch data");

	points = buf[AXS5106L_POINTS_POS] & AXS5106L_POINTS_MASK;
	if (points == 0) {
		if (data->pressed) {
			input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
			data->pressed = false;
		}
		return 0;
	}

	x = ((buf[AXS5106L_P1_XH_POS] & AXS5106L_COORD_H_MASK) << 8) | buf[AXS5106L_P1_XL_POS];
	y = ((buf[AXS5106L_P1_YH_POS] & AXS5106L_COORD_H_MASK) << 8) | buf[AXS5106L_P1_YL_POS];

	input_touchscreen_report_pos(dev, x, y, K_FOREVER);
	input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
	data->pressed = true;

	return 0;
}

static void axs5106l_work_handler(struct k_work *work)
{
	struct axs5106l_data *data = CONTAINER_OF(work, struct axs5106l_data, work);

	axs5106l_process(data->dev);
}

static void axs5106l_isr_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct axs5106l_data *data = CONTAINER_OF(cb, struct axs5106l_data, int_gpio_cb);

	k_work_submit(&data->work);
}

static int axs5106l_init(const struct device *dev)
{
	const struct axs5106l_config *config = dev->config;
	struct axs5106l_data *data = dev->data;
	uint8_t id[AXS5106L_ID_LEN];
	int ret;

	data->dev = dev;
	k_work_init(&data->work, axs5106l_work_handler);

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR_DEVICE_NOT_READY(config->i2c.bus);
		return -ENODEV;
	}

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR_DEVICE_NOT_READY(config->reset_gpio.port);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO pin: %d", ret);
			return ret;
		}

		k_msleep(AXS5106L_RESET_ASSERT_MS);
		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_msleep(AXS5106L_RESET_DELAY_MS);
	}

	ret = axs5106l_read_reg(dev, AXS5106L_REG_ID, id, sizeof(id));
	if (ret < 0) {
		LOG_ERR("Could not read ID: %d", ret);
		return ret;
	}

	if (id[0] == 0) {
		LOG_ERR("Invalid ID %02x %02x %02x", id[0], id[1], id[2]);
		return -ENODEV;
	}

	LOG_DBG("ID %02x %02x %02x", id[0], id[1], id[2]);

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR_DEVICE_NOT_READY(config->int_gpio.port);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->int_gpio_cb, axs5106l_isr_handler, BIT(config->int_gpio.pin));

	ret = gpio_add_callback_dt(&config->int_gpio, &data->int_gpio_cb);
	if (ret < 0) {
		LOG_ERR("Could not set GPIO callback: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure GPIO interrupt: %d", ret);
		return ret;
	}

	return 0;
}

#define AXS5106L_DEFINE(inst)                                                                      \
	static const struct axs5106l_config axs5106l_config_##inst = {                             \
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(inst),                            \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
	};                                                                                         \
	static struct axs5106l_data axs5106l_data_##inst;                                          \
	DEVICE_DT_INST_DEFINE(inst, axs5106l_init, NULL, &axs5106l_data_##inst,                    \
			      &axs5106l_config_##inst, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,    \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(AXS5106L_DEFINE)
