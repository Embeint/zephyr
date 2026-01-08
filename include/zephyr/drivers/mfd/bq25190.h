/*
 * Copyright (c) 2025 Embeint Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/sys/slist.h>

#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_BQ25190_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_BQ25190_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup mfd_interface_bq25190 MFD BQ25190 Interface
 * @ingroup mfd_interfaces
 * @{
 */

/** BQ25190 Registers */
enum mfd_bq25190_reg {
	BQ25190_REG_STAT0 = 0x00,
	BQ25190_REG_STAT1 = 0x01,
	BQ25190_REG_STAT2 = 0x02,
	BQ25190_REG_STAT3 = 0x03,
	BQ25190_REG_FLAG0 = 0x04,
	BQ25190_REG_FLAG1 = 0x05,
	BQ25190_REG_FLAG2 = 0x06,
	BQ25190_REG_FLAG3 = 0x07,
	BQ25190_REG_MASK0 = 0x08,
	BQ25190_REG_MASK1 = 0x09,
	BQ25190_REG_MASK2 = 0x0A,
	BQ25190_REG_MASK3 = 0x0B,
	BQ25190_REG_VBAT = 0x0C,
	BQ25190_REG_ICHG_CTRL = 0x0D,
	BQ25190_REG_CHARGECTRL0 = 0x0E,
	BQ25190_REG_CHARGECTRL1 = 0x0F,
	BQ25190_REG_IC_CTRL = 0x10,
	BQ25190_REG_TMR_ILIM = 0x11,
	BQ25190_REG_SHIP_RST = 0x12,
	BQ25190_REG_SYS_REG = 0x13,
	BQ25190_REG_ADCCTRL0 = 0x18,
	BQ25190_REG_ADCCTRL1 = 0x19,
	BQ25190_REG_ADCCTRL2 = 0x1A,
	BQ25190_REG_ADC_DATA_VBAT = 0x1B,
	BQ25190_REG_ADC_DATA_TS = 0x1D,
	BQ25190_REG_ADC_DATA_IBAT = 0x1F,
	BQ25190_REG_ADC_DATA_ADCIN = 0x21,
	BQ25190_REG_ADC_DATA_VIN = 0x23,
	BQ25190_REG_ADC_DATA_VSYS = 0x25,
	BQ25190_REG_ADC_DATA_IIN = 0x27,
	BQ25190_REG_ADC_DATA_TDIE = 0x29,
	BQ25190_REG_ADC_CHANNEL_DISABLE = 0x31,
	BQ25190_REG_BUCK_VOUT = 0x32,
	BQ25190_REG_BUCK_VOUT1 = 0x33,
	BQ25190_REG_BUCK_VOUT2 = 0x34,
	BQ25190_REG_BUCK_VOUT3 = 0x35,
	BQ25190_REG_BUCK_VOUT4 = 0x36,
	BQ25190_REG_BUCK_CTRL0 = 0x37,
	BQ25190_REG_BUCK_CTRL1 = 0x38,
	BQ25190_REG_BUBO_CTRL0 = 0x39,
	BQ25190_REG_BUBO_CTRL1 = 0x3A,
	BQ25190_REG_LDO1_CTRL0 = 0x3B,
	BQ25190_REG_LDO1_CTRL1 = 0x3C,
	BQ25190_REG_LDO2_CTRL0 = 0x3D,
	BQ25190_REG_LDO2_CTRL1 = 0x3E,
	BQ25190_REG_NTC_CTRL = 0x3F,
	BQ25190_REG_GPIO1_CTRL = 0x40,
	BQ25190_REG_GPIO2_CTRL = 0x41,
	BQ25190_REG_GPIO3_CTRL = 0x42,
	BQ25190_REG_GPIO4_CTRL = 0x43,
	BQ25190_REG_PART_INFORMATION = 0x44,
};

/** BQ25190 BQ25190_REG_GPIO{N}_CTRL values */
enum mfd_bq25190_gpio_cfg {
	/** Output: pulled up to VCC through resistor */
	BQ25190_GPIO_CTRL_OPEN_DRAIN_HIGH = (0b0000 << 4),
	/** Output: driven low */
	BQ25190_GPIO_CTRL_FORCED_LOW = (0b0001 << 4),
	BQ25190_GPIO_CTRL_LEVEL_SHIFTED_MR = (0b0010 << 4),
	/** Output: driven high  */
	BQ25190_GPIO_CTRL_PUSH_PULL_HIGH = (0b0011 << 4),
	/** Input: No pull */
	BQ25190_GPIO_CTRL_INPUT_LEVEL_SENSITIVE = (0b1000 << 4),
	BQ25190_GPIO_CTRL_INPUT_POSITIVE_EDGE_TRIGGER = (0b1010 << 4),
	BQ25190_GPIO_CTRL_INPUT_NEGATIVE_EDGE_TRIGGER = (0b1011 << 4),
};

/* Delay before the PMIC enters shipping mode after I2C command */
#define BQ25190_SHIP_MODE_ENTER_DELAY_MS 1000

/**
 * @brief Read single register from bq25190
 *
 * @param dev bq25190 MFD device
 * @param reg Register address
 * @param data address to read into
 *
 * @retval 0 If successful
 * @retval -errno In case of any bus error (see i2c_read_dt())
 */
int mfd_bq25190_reg_read(const struct device *dev, uint8_t reg, uint8_t *data);

/**
 * @brief Read single register from bq25190
 *
 * @param dev bq25190 MFD device
 * @param reg Register address
 * @param data Address to read into
 * @param len Number of bytes to read
 *
 * @retval 0 If successful
 * @retval -errno In case of any bus error (see i2c_read_dt())
 */
int mfd_bq25190_reg_read_burst(const struct device *dev, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Write single register to bq25190
 *
 * @param dev bq25190 MFD device
 * @param reg Register address
 * @param data Data to write
 *
 * @retval 0 If successful
 * @retval -errno In case of any bus error (see i2c_write_dt())
 */
int mfd_bq25190_reg_write(const struct device *dev, uint8_t reg, uint8_t data);

/**
 * @brief Update selected bits in bq25190 register
 *
 * @param dev bq25190 MFD device
 * @param reg Register address
 * @param mask mask of bits to be modified
 * @param data data to write
 *
 * @retval 0 If successful
 * @retval -errno In case of any bus error (see i2c_reg_update_byte())
 */
int mfd_bq25190_reg_update(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t data);

/** @brief BQ25190 MFD callback structure. */
struct mfd_bq25190_cb {
	/** @brief Common BQ25190 interrupt has occurred
	 *
	 * @param flags Value of the 4 interrupt flag registers
	 * @param user_ctx User context pointer
	 */
	void (*interrupt)(uint8_t flags[4], void *user_ctx);

	/* User provided context pointer */
	void *user_ctx;

	sys_snode_t node;
};

/**
 * @brief Register for notifications on the common IRQ
 *
 * @param dev bq25190 MFD device
 * @param cb Callback structure to register
 *
 * @retval 0 If successful
 * @retval -ENODEV If no interrupt pin available
 */
int mfd_bq25190_register_callback(const struct device *dev, struct mfd_bq25190_cb *cb);

/**
 * @brief Enter ship mode
 *
 * Ship mode is the lowest quiescent current state for the device with BATFET being turned off.
 * Ship mode is existed by Vin being applied or the MR pin being pulled low for over 1 second.
 *
 * @param dev bq25190 MFD device
 *
 * @retval 0 If successful
 * @retval -errno In case of any bus error (see i2c_reg_update_byte())
 */
int mfd_bq25190_ship_mode(const struct device *dev);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_BQ25190_H_ */
