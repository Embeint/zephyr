/*
 * Copyright (c) 2025 Embeint Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 *
 * BUCK registor control all assumes BUCK_HI_RANGE = 1, which is the POR value
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/mfd/bq25190.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/linear_range.h>

#define DT_DRV_COMPAT ti_bq25190_regulator

/* BQ25190 voltage sources */
enum bq25190_sources {
	BQ25190_SOURCE_BUCK,
	BQ25190_SOURCE_BUCK_BOOST,
	BQ25190_SOURCE_LDO1,
	BQ25190_SOURCE_LDO2,
};

enum bq25190_flags {
	BQ25190_LDO_SWITCH = BIT(0),
	BQ25190_GPIO_CONTROLLED = BIT(1),
};

struct regulator_bq25190_config {
	struct regulator_common_config common;
	struct gpio_dt_spec en_gpio;
	const struct device *mfd;
	uint8_t source;
	uint8_t flags;
};

struct regulator_bq25190_data {
	struct regulator_common_data common;
};

/* Common regulator CTRL1 defines */
#define REGULATOR_CTRL1_EN_SEQ_A      (0b000 << 5)
#define REGULATOR_CTRL1_EN_SEQ_B      (0b001 << 5)
#define REGULATOR_CTRL1_EN_SEQ_C      (0b010 << 5)
#define REGULATOR_CTRL1_EN_SEQ_D      (0b011 << 5)
#define REGULATOR_CTRL1_EN_DISABLED   (0b100 << 5)
#define REGULATOR_CTRL1_EN_ENABLED    (0b101 << 5)
#define REGULATOR_CTRL1_EN_GPIO       (0b110 << 5)
#define REGULATOR_CTRL1_POWER_GOOD_EN 0x80
#define REGULATOR_CTRL1_POWER_GOOD    0x02

#define REGULATOR_CTRL0_LDO_BYPASS 0x40

/* 25 mV for 0.4V-3.175V and 50 mV for 3.2V to 3.6V.
 * For simplicity, limit ourselves to the low range.
 */
static const struct linear_range buck_low_range = LINEAR_RANGE_INIT(400000, 25000, 0, 111);
static const struct linear_range buck_boost_range = LINEAR_RANGE_INIT(1700000, 50000, 0x00, 0x46);
static const struct linear_range ldo_range = LINEAR_RANGE_INIT(800000, 50000, 0x00, 0x38);

LOG_MODULE_REGISTER(bq25190_reg, CONFIG_REGULATOR_BQ25190_LOG_LEVEL);

/* Source to register information helper */
static int bq251890_voltage_info(uint8_t source, uint8_t is_switch, uint8_t *reg,
				 const struct linear_range **range)
{
	int ret = 0;

	switch (source) {
	case BQ25190_SOURCE_BUCK:
		*reg = BQ25190_REG_BUCK_VOUT;
		*range = &buck_low_range;
		break;
	case BQ25190_SOURCE_BUCK_BOOST:
		*reg = BQ25190_REG_BUBO_CTRL0;
		*range = &buck_boost_range;
		break;
	case BQ25190_SOURCE_LDO1:
	case BQ25190_SOURCE_LDO2:
		if (is_switch) {
			return -ENOSYS;
		}
		*reg = source == BQ25190_SOURCE_LDO1 ? BQ25190_REG_LDO1_CTRL0
						     : BQ25190_REG_LDO2_CTRL0;
		*range = &ldo_range;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int bq251890_control_info(uint8_t source, uint8_t *reg, uint8_t *gpio_reg)
{
	int ret = 0;

	switch (source) {
	case BQ25190_SOURCE_BUCK:
		*reg = BQ25190_REG_BUCK_CTRL1;
		*gpio_reg = BQ25190_REG_GPIO3_CTRL;
		break;
	case BQ25190_SOURCE_BUCK_BOOST:
		*reg = BQ25190_REG_BUBO_CTRL1;
		*gpio_reg = BQ25190_REG_GPIO2_CTRL;
		break;
	case BQ25190_SOURCE_LDO1:
		*reg = BQ25190_REG_LDO1_CTRL1;
		*gpio_reg = BQ25190_REG_GPIO4_CTRL;
		break;
	case BQ25190_SOURCE_LDO2:
		*reg = BQ25190_REG_LDO2_CTRL1;
		*gpio_reg = BQ25190_REG_GPIO1_CTRL;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

int get_enabled(const struct device *dev, bool *enabled)
{
	const struct regulator_bq25190_config *config = dev->config;
	uint8_t reg, gpio_reg, val;
	int ret;

	ret = bq251890_control_info(config->source, &reg, &gpio_reg);
	if (ret < 0) {
		return ret;
	}

	ret = mfd_bq25190_reg_read(config->mfd, reg, &val);
	if (ret < 0) {
		return ret;
	}
	*enabled = !!(val & REGULATOR_CTRL1_POWER_GOOD);
	return 0;
}

unsigned int regulator_bq25190_count_voltages(const struct device *dev)
{
	const struct regulator_bq25190_config *config = dev->config;
	const struct linear_range *range;
	uint8_t reg;
	int ret;

	ret = bq251890_voltage_info(config->source, config->flags & BQ25190_LDO_SWITCH, &reg,
				    &range);
	if (ret < 0) {
		return ret;
	}
	return linear_range_values_count(range);
}

int regulator_bq25190_list_voltage(const struct device *dev, unsigned int idx, int32_t *volt_uv)
{
	const struct regulator_bq25190_config *config = dev->config;
	const struct linear_range *range;
	uint8_t reg;
	int ret;

	ret = bq251890_voltage_info(config->source, config->flags & BQ25190_LDO_SWITCH, &reg,
				    &range);
	if (ret < 0) {
		return ret;
	}
	return linear_range_get_value(range, idx, volt_uv);
}

int regulator_bq25190_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_bq25190_config *config = dev->config;
	const struct linear_range *range;
	uint16_t idx;
	uint8_t reg;
	int ret;

	ret = bq251890_voltage_info(config->source, config->flags & BQ25190_LDO_SWITCH, &reg,
				    &range);
	if (ret < 0) {
		return ret;
	}

	/* Convert voltage to register value */
	ret = linear_range_get_win_index(range, min_uv, max_uv, &idx);
	if (ret < 0) {
		return ret;
	}

	/* Write the new register value.
	 * While some higher bits are defined in the LDO registers, they set the switch mode.
	 */
	LOG_DBG("%s: Voltage register: 0x%02X", dev->name, idx);
	return mfd_bq25190_reg_write(config->mfd, reg, idx);
}

int regulator_bq25190_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_bq25190_config *config = dev->config;
	const struct linear_range *range;
	uint8_t reg, val;
	int ret;

	ret = bq251890_voltage_info(config->source, config->flags & BQ25190_LDO_SWITCH, &reg,
				    &range);
	if (ret < 0) {
		return ret;
	}

	/* Read the appropriate register */
	ret = mfd_bq25190_reg_read(config->mfd, reg, &val);
	if (ret < 0) {
		return ret;
	}

	/* Convert to a voltage */
	ret = linear_range_get_value(range, val, volt_uv);
	LOG_DBG("%s: Voltage %d uV", dev->name, *volt_uv);
	return ret;
}

static int regulator_bq25190_control_common(const struct device *dev, uint8_t control)
{
	const struct regulator_bq25190_config *config = dev->config;
	uint8_t reg, gpio_reg;
	int ret;

	if (config->flags & BQ25190_GPIO_CONTROLLED) {
		/* Regulator is not controlled from this driver */
		return -EINVAL;
	}

	ret = bq251890_control_info(config->source, &reg, &gpio_reg);
	if (ret < 0) {
		return ret;
	}

	/* Always want the power good flag working */
	control |= REGULATOR_CTRL1_POWER_GOOD_EN;
	return mfd_bq25190_reg_write(config->mfd, reg, control);
}

int regulator_bq25190_enable(const struct device *dev)
{
	LOG_DBG("%s: enable", dev->name);

	return regulator_bq25190_control_common(dev, REGULATOR_CTRL1_EN_ENABLED);
}

int regulator_bq25190_disable(const struct device *dev)
{
	LOG_DBG("%s: disable", dev->name);

	return regulator_bq25190_control_common(dev, REGULATOR_CTRL1_EN_DISABLED);
}

int regulator_bq25190_init(const struct device *dev)
{
	const struct regulator_bq25190_config *config = dev->config;
	uint8_t reg, gpio_reg;
	bool enabled;
	int ret;

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

	if ((config->source == BQ25190_SOURCE_LDO1) || (config->source == BQ25190_SOURCE_LDO2)) {
		/* Base config register for the LDO */
		if (config->source == BQ25190_SOURCE_LDO1) {
			reg = BQ25190_REG_LDO1_CTRL0;
		} else if (config->source == BQ25190_SOURCE_LDO2) {
			reg = BQ25190_REG_LDO2_CTRL0;
		}

		/* Configuring the mode/voltage requires LDOs to be disabled (CTRL1 register).
		 * Don't go through regulator_bq25190_disable since that checks the
		 * `GPIO_CONTROLLED` flag.
		 */
		ret = mfd_bq25190_reg_write(config->mfd, reg + 1, REGULATOR_CTRL1_EN_DISABLED);
		if (ret < 0) {
			return ret;
		}

		/* If in switch mode, configure it, otherwise regulator_common_init will setup
		 * voltages and enable/disable.
		 */
		if (config->flags & BQ25190_LDO_SWITCH) {
			ret = mfd_bq25190_reg_write(config->mfd, reg, REGULATOR_CTRL0_LDO_BYPASS);
			if (ret < 0) {
				return ret;
			}
		}
	} else if (config->flags & BQ25190_LDO_SWITCH) {
		/* LDO switch mode only valid for LDOs */
		return -EINVAL;
	}

	ret = get_enabled(dev, &enabled);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("%s: %s at boot", dev->name, enabled ? "enabled" : "disabled");
	ret = regulator_common_init(dev, enabled);
	if (ret < 0) {
		return ret;
	}

	if (config->flags & BQ25190_GPIO_CONTROLLED) {
		ret = bq251890_control_info(config->source, &reg, &gpio_reg);
		if (ret < 0) {
			return ret;
		}

		LOG_DBG("%s: GPIO Controlled", dev->name);
		/* Configure regulator to be controlled by the GPIO */
		ret = mfd_bq25190_reg_write(
			config->mfd, reg, REGULATOR_CTRL1_POWER_GOOD_EN | REGULATOR_CTRL1_EN_GPIO);
		if (ret < 0) {
			return ret;
		}
		/* Configure GPIO to be an input */
		ret = mfd_bq25190_reg_write(config->mfd, gpio_reg,
					    BQ25190_GPIO_CTRL_INPUT_LEVEL_SENSITIVE);
		if (ret < 0) {
			return ret;
		}
	}

	return ret;
}

static DEVICE_API(regulator, api) = {
	.enable = regulator_bq25190_enable,
	.disable = regulator_bq25190_disable,
	.count_voltages = regulator_bq25190_count_voltages,
	.list_voltage = regulator_bq25190_list_voltage,
	.set_voltage = regulator_bq25190_set_voltage,
	.get_voltage = regulator_bq25190_get_voltage,
};

#define REGULATOR_BQ25190_DEFINE(node_id, id, _source)                                             \
	static struct regulator_bq25190_data regulator_bq25190_data_##id;                          \
                                                                                                   \
	static const struct regulator_bq25190_config regulator_bq25190_config_##id = {             \
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),                                \
		.en_gpio = GPIO_DT_SPEC_GET_OR(node_id, enable_gpios, {0}),                        \
		.mfd = DEVICE_DT_GET(DT_GPARENT(node_id)),                                         \
		.source = _source,                                                                 \
		.flags = COND_CODE_1(DT_PROP(node_id, ldo_mode_switch), (BQ25190_LDO_SWITCH),      \
				     (0)) |                                                        \
			 COND_CODE_1(DT_PROP(node_id, gpio_controlled), (BQ25190_GPIO_CONTROLLED), \
				     (0)),                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_DEFINE(node_id, regulator_bq25190_init, NULL, &regulator_bq25190_data_##id,      \
			 &regulator_bq25190_config_##id, POST_KERNEL,                              \
			 CONFIG_REGULATOR_BQ25190_INIT_PRIORITY, &api);

#define REGULATOR_BQ25190_DEFINE_COND(n, child, source)                                            \
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CHILD(n, child)),                                       \
		    (REGULATOR_BQ25190_DEFINE(DT_INST_CHILD(n, child), child##n, source)), ())

#define REGULATOR_BQ25190_DEFINE_ALL(n)                                                            \
	REGULATOR_BQ25190_DEFINE_COND(n, buck, BQ25190_SOURCE_BUCK)                                \
	REGULATOR_BQ25190_DEFINE_COND(n, buck_boost, BQ25190_SOURCE_BUCK_BOOST)                    \
	REGULATOR_BQ25190_DEFINE_COND(n, ldo1, BQ25190_SOURCE_LDO1)                                \
	REGULATOR_BQ25190_DEFINE_COND(n, ldo2, BQ25190_SOURCE_LDO2)

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_BQ25190_DEFINE_ALL)
