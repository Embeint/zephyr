/**
 * @file
 * @copyright 2024 Embeint Inc
 * @author Jordan Yates <jordan@embeint.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <sys/types.h>

#include <Driver_Flash.h>
#include <RTE_Device.h>
#include <flash_layout.h>
#include <nrfx_spim.h>
#include <nrf-pinctrl.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>

#include "SPI_NOR.h"

#define MAX_FREQ_FLASH                    DT_PROP(FLASH_SPI_NOR, spi_max_frequency)
#define MAX_FREQ_BUS                      DT_PROP(FLASH_SPI_NOR_BUS, max_frequency)
#define SPIM_PIN_INIT(node_id, prop, idx) DT_PROP_BY_IDX(node_id, prop, idx),

/* Pinctrl definitions from nrf-pinctrl.h */
#define NRF_FUN_SPIM_SCK  4U
#define NRF_FUN_SPIM_MOSI 5U
#define NRF_FUN_SPIM_MISO 6U

#define NRFX_SPIM_INITIALIZER()                                                                    \
	{                                                                                          \
		.sck_pin = NRF_SPIM_PIN_NOT_CONNECTED,                                             \
		.mosi_pin = NRF_SPIM_PIN_NOT_CONNECTED,                                            \
		.miso_pin = NRF_SPIM_PIN_NOT_CONNECTED,                                            \
		.ss_pin = NRF_SPIM_PIN_NOT_CONNECTED,                                              \
		.ss_active_high = false,                                                           \
		.irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,                             \
		.orc = 0xFF,                                                                       \
		.frequency = 0,                                                                    \
		.mode = NRF_SPIM_MODE_0,                                                           \
		.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,                                         \
		.miso_pull = NRF_GPIO_PIN_NOPULL,                                                  \
	}

static const nrfx_spim_t spi_instance = {
	.p_reg = FLASH_SPI_NOR_BUS_REG,
	.drv_inst_idx = FLASH_SPI_NOR_BUS_INST,
};
static bool spi_nor_is_inited;

void spim_config_set_spim_pins(nrfx_spim_config_t *spim_config, const uint32_t spim_pins[],
			       size_t spim_pins_count)
{
	for (size_t i = 0; i < spim_pins_count; i++) {
		uint32_t psel = NRF_GET_PIN(spim_pins[i]);

		if (psel == NRF_PIN_DISCONNECTED) {
			psel = NRF_SPIM_PIN_NOT_CONNECTED;
		}

		switch (NRF_GET_FUN(spim_pins[i])) {
		case NRF_FUN_SPIM_SCK:
			spim_config->sck_pin = psel;
			break;
		case NRF_FUN_SPIM_MOSI:
			spim_config->mosi_pin = psel;
			break;
		case NRF_FUN_SPIM_MISO:
			spim_config->miso_pin = psel;
			break;
		}
	}
}

static inline uint32_t spim_config_max_frequency(uint32_t frequency)
{
	/* Get the highest supported frequency not exceeding the requested one.
	 */
	if (frequency >= MHZ(32) && (NRF_SPIM_HAS_32_MHZ_FREQ || NRF_SPIM_HAS_PRESCALER)) {
		return MHZ(32);
	} else if (frequency >= MHZ(16) && (NRF_SPIM_HAS_16_MHZ_FREQ || NRF_SPIM_HAS_PRESCALER)) {
		return MHZ(16);
	} else if (frequency >= MHZ(8)) {
		return MHZ(8);
	} else if (frequency >= MHZ(4)) {
		return MHZ(4);
	} else if (frequency >= MHZ(2)) {
		return MHZ(2);
	} else if (frequency >= MHZ(1)) {
		return MHZ(1);
	} else if (frequency >= KHZ(500)) {
		return KHZ(500);
	} else if (frequency >= KHZ(250)) {
		return KHZ(250);
	} else {
		return KHZ(125);
	}
}

static int spi_nor_access(uint8_t opcode, unsigned int access, off_t addr, void *data,
			  size_t length)
{
	bool is_addressed = (access & NOR_ACCESS_ADDRESSED) != 0U;
	bool is_write = (access & NOR_ACCESS_WRITE) != 0U;
	uint8_t buf[5] = {0};

	nrfx_spim_xfer_desc_t xfer_addr = NRFX_SPIM_XFER_TX(buf, 1);
	nrfx_spim_xfer_desc_t xfer_data;
	nrfx_err_t err;

	if (is_write) {
		xfer_data = (nrfx_spim_xfer_desc_t)NRFX_SPIM_XFER_TX(data, length);
	} else {
		xfer_data = (nrfx_spim_xfer_desc_t)NRFX_SPIM_XFER_RX(data, length);
	}

	buf[0] = opcode;
	if (is_addressed) {
		bool access_32bit = (access & NOR_ACCESS_32BIT_ADDR) != 0;
		union {
			uint32_t u32;
			uint8_t u8[4];
		} addr32 = {
			.u32 = sys_cpu_to_be32(addr),
		};

		if (access_32bit) {
			memcpy(&buf[1], &addr32.u8[0], 4);
			xfer_addr.tx_length += 4;
		} else {
			memcpy(&buf[1], &addr32.u8[1], 3);
			xfer_addr.tx_length += 3;
		}
	};

	/* Manually hold CS low across the transaction */
	nrfy_gpio_pin_clear(FLASH_SPI_NOR_CS_PIN);
	err = nrfx_spim_xfer(&spi_instance, &xfer_addr, 0);
	if (err == NRFX_SUCCESS && (length > 0)) {
		err = nrfx_spim_xfer(&spi_instance, &xfer_data, 0);
	}
	nrfy_gpio_pin_set(FLASH_SPI_NOR_CS_PIN);
	return err == NRFX_SUCCESS ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
}

#define spi_nor_cmd_read(opcode, dest, length) spi_nor_access(opcode, 0, 0, dest, length)
#define spi_nor_cmd_addr_read(opcode, addr, dest, length)                                          \
	spi_nor_access(opcode, NOR_ACCESS_ADDRESSED, addr, dest, length)
#define spi_nor_cmd_write(opcode) spi_nor_access(opcode, NOR_ACCESS_WRITE, 0, NULL, 0)
#define spi_nor_cmd_addr_write(opcode, addr, src, length)                                          \
	spi_nor_access(opcode, NOR_ACCESS_WRITE | NOR_ACCESS_ADDRESSED, addr, (void *)src, length)

static int spi_nor_wait_until_ready(void)
{
	uint8_t reg;
	int ret;

	while (true) {
		ret = spi_nor_cmd_read(SPI_NOR_CMD_RDSR, &reg, sizeof(reg));
		/* Exit on error or no longer WIP */
		if (ret || !(reg & SPI_NOR_WIP_BIT)) {
			break;
		}
	}
	return ret;
}

static const ARM_DRIVER_VERSION flash_driver_version = {
	ARM_FLASH_API_VERSION,
	ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0),
};

static const ARM_FLASH_CAPABILITIES flash_driver_capabilities = {
	.event_ready = 0,
	.data_width = 0,
	.erase_chip = 0,
};

static ARM_FLASH_INFO flash_info = {
	.sector_info = NULL, /* Uniform sector layout */
	.sector_count = SPI_FLASH_TOTAL_SIZE / SPI_FLASH_AREA_IMAGE_SECTOR_SIZE,
	.sector_size = SPI_FLASH_AREA_IMAGE_SECTOR_SIZE,
	.page_size = 256,
	.program_unit = sizeof(uint8_t),
	.erased_value = 0xFF,
};

static ARM_DRIVER_VERSION ARM_SPI_NOR_Flash_GetVersion(void)
{
	return flash_driver_version;
}

static ARM_FLASH_CAPABILITIES ARM_SPI_NOR_Flash_GetCapabilities(void)
{
	return flash_driver_capabilities;
}

static ARM_FLASH_INFO *ARM_SPI_NOR_Flash_GetInfo(void)
{
	return &flash_info;
}

static int32_t ARM_SPI_NOR_Flash_Initialize(ARM_Flash_SignalEvent_t cb_event)
{
	nrfx_spim_config_t spim_config = NRFX_SPIM_INITIALIZER();
	const uint32_t spim_pinctrl[] = {
		DT_FOREACH_CHILD_VARGS(DT_PINCTRL_BY_NAME(FLASH_SPI_NOR_BUS, default, 0),
				       DT_FOREACH_PROP_ELEM, psels, SPIM_PIN_INIT)};
	nrf_gpio_pin_drive_t drive = GPIO_PIN_CNF_DRIVE_H0H1;
	uint8_t reg;
	int rc;

	ARG_UNUSED(cb_event);

	spim_config.frequency = spim_config_max_frequency(MIN(MAX_FREQ_FLASH, MAX_FREQ_BUS));

	if (spi_nor_is_inited == false) {
		/* Hardware setup:
		 *   Configure CS GPIO
		 *   Extract SPIM pins from devicetree pinctrl encoding
		 *   Initialise SPIM instance in blocking mode
		 *   Force output pins to high drive mode (Cannot be specified in nrfx_spim_init)
		 */
		nrfy_gpio_cfg_output(FLASH_SPI_NOR_CS_PIN);
		nrfy_gpio_pin_set(FLASH_SPI_NOR_CS_PIN);
		spim_config_set_spim_pins(&spim_config, spim_pinctrl, ARRAY_SIZE(spim_pinctrl));
		if (nrfx_spim_init(&spi_instance, &spim_config, NULL, NULL) != NRFX_SUCCESS) {
			return ARM_DRIVER_ERROR_PARAMETER;
		}
		nrfy_gpio_reconfigure(FLASH_SPI_NOR_CS_PIN, NULL, NULL, NULL, &drive, NULL);
		nrfy_gpio_reconfigure(spim_config.mosi_pin, NULL, NULL, NULL, &drive, NULL);
		nrfy_gpio_reconfigure(spim_config.sck_pin, NULL, NULL, NULL, &drive, NULL);

		/* Flash chip setup:
		 *   Release from deep power down
		 *   Wait until the device is ready
		 */
		(void)spi_nor_cmd_write(SPI_NOR_CMD_RDPD);
		rc = spi_nor_cmd_read(SPI_NOR_CMD_RDSR, &reg, sizeof(reg));
		if (rc < 0) {
			return ARM_DRIVER_ERROR;
		} else if (reg & SPI_NOR_WIP_BIT) {
			rc = spi_nor_wait_until_ready();
			if (rc < 0) {
				return ARM_DRIVER_ERROR;
			}
		}
		spi_nor_is_inited = true;
	}

	return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_NOR_Flash_Uninitialize(void)
{
	if (spi_nor_is_inited) {
		/* Cleanup SPI instance and CS pin */
		nrfx_spim_uninit(&spi_instance);
		nrfy_gpio_input_disconnect(FLASH_SPI_NOR_CS_PIN);
		spi_nor_is_inited = false;
	}

	return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_NOR_Flash_PowerControl(ARM_POWER_STATE state)
{
	switch (state) {
	case ARM_POWER_FULL:
		/* Nothing to be done */
		return ARM_DRIVER_OK;
	case ARM_POWER_OFF:
	case ARM_POWER_LOW:
	default:
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
}

static int32_t ARM_SPI_NOR_Flash_ReadData(uint32_t addr, void *data, uint32_t cnt)
{
	int rc;

	/* Minimal input validation */
	if (addr + cnt > SPI_FLASH_TOTAL_SIZE) {
		return ARM_DRIVER_ERROR_PARAMETER;
	}
	rc = spi_nor_cmd_addr_read(SPI_NOR_CMD_READ, addr, data, cnt);
	return rc == 0 ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
}

static int32_t ARM_SPI_NOR_Flash_ProgramData(uint32_t addr, const void *data, uint32_t cnt)
{
	const uint8_t *out_ptr = data;
	size_t to_write;
	int rc = 0;

	/* Minimal input validation */
	if (addr + cnt > SPI_FLASH_TOTAL_SIZE) {
		return ARM_DRIVER_ERROR_PARAMETER;
	}

	while (cnt) {
		/* Don't write more than a page. */
		to_write = cnt;
		if (to_write >= flash_info.page_size) {
			to_write = flash_info.page_size;
		}
		/* Don't write across a page boundary */
		if (((addr + to_write - 1U) / flash_info.page_size) !=
		    (addr / flash_info.page_size)) {
			to_write = flash_info.page_size - (addr % flash_info.page_size);
		}

		/* Enable writing */
		rc = spi_nor_cmd_write(SPI_NOR_CMD_WREN);
		if (rc < 0) {
			break;
		}
		/* Write the data */
		rc = spi_nor_cmd_addr_write(SPI_NOR_CMD_READ, addr, out_ptr, to_write);
		if (rc < 0) {
			break;
		}
		/* Update bytes remaining */
		addr += to_write;
		out_ptr += to_write;
		cnt -= to_write;

		/* Wait until flash chip is ready again */
		rc = spi_nor_wait_until_ready();
		if (rc < 0) {
			return ARM_DRIVER_ERROR;
		}
	}
	/* Disable writing */
	(void)spi_nor_cmd_write(SPI_NOR_CMD_WRDI);

	return rc == 0 ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
}

static int32_t ARM_SPI_NOR_Flash_EraseSector(uint32_t addr)
{
	int rc;

	/* Minimal input validation, sector alignment */
	if (addr % flash_info.sector_size) {
		return ARM_DRIVER_ERROR_PARAMETER;
	}

	rc = spi_nor_cmd_write(SPI_NOR_CMD_WREN);
	if (rc < 0) {
		return ARM_DRIVER_ERROR;
	}
	rc = spi_nor_cmd_addr_write(SPI_NOR_CMD_SE, addr, NULL, 0);
	/* Wait until sector erase completes */
	if (spi_nor_wait_until_ready() < 0) {
		rc = -1;
	}
	(void)spi_nor_cmd_write(SPI_NOR_CMD_WRDI);
	return rc == 0 ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
}

static int32_t ARM_SPI_NOR_Flash_EraseChip(void)
{
	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static ARM_FLASH_STATUS ARM_SPI_NOR_Flash_GetStatus(void)
{
	ARM_FLASH_STATUS status = {.busy = false};
	/* Blocking driver implementation, therefore never busy */
	return status;
}

ARM_DRIVER_FLASH Driver_FLASH_SPI_NOR0 = {
	.GetVersion = ARM_SPI_NOR_Flash_GetVersion,
	.GetCapabilities = ARM_SPI_NOR_Flash_GetCapabilities,
	.Initialize = ARM_SPI_NOR_Flash_Initialize,
	.Uninitialize = ARM_SPI_NOR_Flash_Uninitialize,
	.PowerControl = ARM_SPI_NOR_Flash_PowerControl,
	.ReadData = ARM_SPI_NOR_Flash_ReadData,
	.ProgramData = ARM_SPI_NOR_Flash_ProgramData,
	.EraseSector = ARM_SPI_NOR_Flash_EraseSector,
	.EraseChip = ARM_SPI_NOR_Flash_EraseChip,
	.GetStatus = ARM_SPI_NOR_Flash_GetStatus,
	.GetInfo = ARM_SPI_NOR_Flash_GetInfo,
};
