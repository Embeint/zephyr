/*
 * Copyright (c) 2025 Embeint Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/mfd/bq25190.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT ti_bq25190_charger

struct charger_bq25190_config {
	const struct device *mfd;
	uint32_t initial_current_microamp;
	uint32_t max_voltage_microvolt;
	uint8_t sys_reg;
	uint8_t tmr_ilim;
	uint8_t ntc_ctrl;
};

/* Charging current limits */
#define BQ25190_CURRENT_MIN_MA 5
#define BQ25190_CURRENT_MAX_MA 1000
#define BQ25190_VOLTAGE_MIN_MV 3500
#define BQ25190_VOLTAGE_MAX_MV 4650

#define BQ25190_FACTOR_VBAT_TO_MV 10

#define BQ25190_ICHG_CHG_DIS 0x80
#define BQ25190_ICHG_MSK     0x7F

#define BQ25190_TMR_ILIM_PB_LONG_PRESS_5S       (0b00 << 6)
#define BQ25190_TMR_ILIM_PB_LONG_PRESS_10S      (0b01 << 6)
#define BQ25190_TMR_ILIM_PB_LONG_PRESS_15S      (0b10 << 6)
#define BQ25190_TMR_ILIM_PB_LONG_PRESS_20S      (0b11 << 6)
#define BQ25190_TMR_ILIM_AUTO_WAKE_TIMER_500MS  (0b00 << 3)
#define BQ25190_TMR_ILIM_AUTO_WAKE_TIMER_1000MS (0b01 << 3)
#define BQ25190_TMR_ILIM_AUTO_WAKE_TIMER_2000MS (0b10 << 3)
#define BQ25190_TMR_ILIM_AUTO_WAKE_TIMER_4000MS (0b11 << 3)

#define BQ25190_STAT2_VIN_POWER_GOOD 0x02

#define BQ25190_STAT2_CHG_STAT_MASK     (0b11 << 6)
#define BQ25190_STAT2_CHG_ENABLED       (0b00 << 6)
#define BQ25190_STAT2_CHG_CONST_CURRENT (0b01 << 6)
#define BQ25190_STAT2_CHG_CONST_VOLTAGE (0b10 << 6)
#define BQ25190_STAT2_CHG_DONE          (0b11 << 6)

#define BQ25190_NTC_CTRL_TS_MONITOR_EN   BIT(7)
#define BQ25190_NTC_CTRL_TS_MONITOR_DIS  0x00
#define BQ25190_NTC_CTRL_TS_FAULT_BAT_EN BIT(6)
#define BQ25190_NTC_CTRL_TS_FAULT_VIN_EN BIT(5)

/*
 * For ICHG <= 35mA = ICHGCODE + 5mA
 * For ICHG > 35mA = 40 + ((ICHGCODE-31)*10)mA.
 * Maximum programmable current = 1000mA
 *
 * Return: value between 0 and 127, negative on error.
 */
static int bq25190_ma_to_ichg(uint32_t current_ma, uint8_t *ichg)
{
	current_ma = CLAMP(current_ma, BQ25190_CURRENT_MIN_MA, BQ25190_CURRENT_MAX_MA);

	if (current_ma <= 35) {
		*ichg = current_ma - 5;
		return 0;
	}

	*ichg = (current_ma - 40) / 10 + 31;

	return 0;
}

static uint32_t bq25190_ichg_to_ma(uint8_t ichg)
{
	ichg &= BQ25190_ICHG_MSK;

	if (ichg <= 30) {
		return (ichg + 5);
	}

	return (ichg - 31) * 10 + 40;
}

static int bq25190_mv_to_vbatreg(const struct charger_bq25190_config *cfg, uint32_t voltage_mv,
				 uint8_t *vbat)
{
	voltage_mv = CLAMP(voltage_mv, BQ25190_VOLTAGE_MIN_MV, BQ25190_VOLTAGE_MAX_MV);

	*vbat = (voltage_mv - BQ25190_VOLTAGE_MIN_MV) / BQ25190_FACTOR_VBAT_TO_MV;

	return 0;
}

static uint32_t bq25190_vbatreg_to_mv(uint8_t vbat)
{
	return (vbat * BQ25190_FACTOR_VBAT_TO_MV) + BQ25190_VOLTAGE_MIN_MV;
}

static int bq25190_charge_enable(const struct device *dev, const bool enable)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t val = enable ? 0 : BQ25190_ICHG_CHG_DIS;

	return mfd_bq25190_reg_update(cfg->mfd, BQ25190_REG_ICHG_CTRL, BQ25190_ICHG_CHG_DIS, val);
}

static int bq25190_set_charge_current(const struct device *dev, uint32_t const_charge_current_ua)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = bq25190_ma_to_ichg(const_charge_current_ua / 1000, &val);
	if (ret < 0) {
		return ret;
	}

	ret = mfd_bq25190_reg_update(cfg->mfd, BQ25190_REG_ICHG_CTRL, BQ25190_ICHG_MSK, val);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int bq25190_get_charge_current(const struct device *dev, uint32_t *const_charge_current_ua)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = mfd_bq25190_reg_read(cfg->mfd, BQ25190_REG_ICHG_CTRL, &val);
	if (ret < 0) {
		return ret;
	}

	*const_charge_current_ua = bq25190_ichg_to_ma(val) * 1000;

	return 0;
}

static int bq25190_set_charge_voltage(const struct device *dev, uint32_t const_charge_voltage_uv)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = bq25190_mv_to_vbatreg(cfg, const_charge_voltage_uv / 1000, &val);
	if (ret < 0) {
		return ret;
	}

	return mfd_bq25190_reg_write(cfg->mfd, BQ25190_REG_VBAT, val);
}

static int bq25190_get_charge_voltage(const struct device *dev, uint32_t *const_charge_voltage_uv)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = mfd_bq25190_reg_read(cfg->mfd, BQ25190_REG_VBAT, &val);
	if (ret < 0) {
		return ret;
	}

	*const_charge_voltage_uv = bq25190_vbatreg_to_mv(val) * 1000;

	return 0;
}

static int bq25190_get_online(const struct device *dev, enum charger_online *online)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = mfd_bq25190_reg_read(cfg->mfd, BQ25190_REG_STAT2, &val);
	if (ret < 0) {
		return ret;
	}

	if ((val & BQ25190_STAT2_VIN_POWER_GOOD) != 0x00) {
		*online = CHARGER_ONLINE_FIXED;
	} else {
		*online = CHARGER_ONLINE_OFFLINE;
	}

	return 0;
}

static int bq25190_get_status(const struct device *dev, enum charger_status *status)
{
	const struct charger_bq25190_config *cfg = dev->config;
	uint8_t stat2;
	int ret;

	ret = mfd_bq25190_reg_read(cfg->mfd, BQ25190_REG_STAT2, &stat2);
	if (ret < 0) {
		return ret;
	}

	if ((stat2 & BQ25190_STAT2_VIN_POWER_GOOD) == 0x00) {
		*status = CHARGER_STATUS_DISCHARGING;
		return 0;
	}

	switch (FIELD_GET(BQ25190_STAT2_CHG_STAT_MASK, stat2)) {
	case BQ25190_STAT2_CHG_ENABLED:
		*status = CHARGER_STATUS_NOT_CHARGING;
		break;
	case BQ25190_STAT2_CHG_CONST_CURRENT:
	case BQ25190_STAT2_CHG_CONST_VOLTAGE:
		*status = CHARGER_STATUS_CHARGING;
		break;
	case BQ25190_STAT2_CHG_DONE:
		*status = CHARGER_STATUS_FULL;
		break;
	}
	return 0;
}

static int bq25190_get_prop(const struct device *dev, charger_prop_t prop,
			    union charger_propval *val)
{
	switch (prop) {
	case CHARGER_PROP_ONLINE:
		return bq25190_get_online(dev, &val->online);
	case CHARGER_PROP_STATUS:
		return bq25190_get_status(dev, &val->status);
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return bq25190_get_charge_current(dev, &val->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return bq25190_get_charge_voltage(dev, &val->const_charge_voltage_uv);
	default:
		return -ENOTSUP;
	}
}

static int bq25190_set_prop(const struct device *dev, charger_prop_t prop,
			    const union charger_propval *val)
{
	switch (prop) {
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return bq25190_set_charge_current(dev, val->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return bq25190_set_charge_voltage(dev, val->const_charge_voltage_uv);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(charger, bq25190_api) = {
	.get_property = bq25190_get_prop,
	.set_property = bq25190_set_prop,
	.charge_enable = bq25190_charge_enable,
};

int charger_bq25190_init(const struct device *dev)
{
	const struct charger_bq25190_config *config = dev->config;
	union charger_propval val;
	int ret;

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

	/* Write initial configuration */
	ret = mfd_bq25190_reg_write(config->mfd, BQ25190_REG_SYS_REG, config->sys_reg);
	if (ret < 0) {
		return ret;
	}
	ret = mfd_bq25190_reg_write(config->mfd, BQ25190_REG_TMR_ILIM, config->tmr_ilim);
	if (ret < 0) {
		return ret;
	}
	ret = mfd_bq25190_reg_write(config->mfd, BQ25190_REG_NTC_CTRL, config->ntc_ctrl);
	if (ret < 0) {
		return ret;
	}
	val.const_charge_current_ua = config->initial_current_microamp;
	ret = bq25190_set_prop(dev, CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA, &val);
	if (ret < 0) {
		return ret;
	}
	val.const_charge_voltage_uv = config->max_voltage_microvolt;
	ret = bq25190_set_prop(dev, CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV, &val);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

#define CHARGER_BQ25190_DEFINE(n)                                                                  \
	static const struct charger_bq25190_config charger_bq25190_config##n = {                   \
		.mfd = DEVICE_DT_GET(DT_INST_PARENT(n)),                                           \
		.initial_current_microamp = DT_INST_PROP(n, constant_charge_current_max_microamp), \
		.max_voltage_microvolt = DT_INST_PROP(n, constant_charge_voltage_max_microvolt),   \
		.sys_reg = (DT_INST_ENUM_IDX(n, sys_regulation_voltage) << 5) |                    \
			   (DT_INST_ENUM_IDX(n, vin_overvoltage_protection_millivolt) << 2),       \
                                                                                                   \
		.tmr_ilim = (DT_INST_ENUM_IDX(n, input_current_limit_microamp) << 0) |             \
			    BQ25190_TMR_ILIM_PB_LONG_PRESS_10S |                                   \
			    BQ25190_TMR_ILIM_AUTO_WAKE_TIMER_2000MS, /* POR defaults */            \
		.ntc_ctrl = (DT_INST_PROP(n, ts_monitoring_disable)                                \
				     ? BQ25190_NTC_CTRL_TS_MONITOR_DIS                             \
				     : BQ25190_NTC_CTRL_TS_MONITOR_EN) |                           \
			    BQ25190_NTC_CTRL_TS_FAULT_VIN_EN,                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, charger_bq25190_init, NULL, NULL, &charger_bq25190_config##n,     \
			      POST_KERNEL, CONFIG_CHARGER_INIT_PRIORITY, &bq25190_api);

DT_INST_FOREACH_STATUS_OKAY(CHARGER_BQ25190_DEFINE)
