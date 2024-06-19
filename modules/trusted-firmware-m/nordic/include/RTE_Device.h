/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

#include <zephyr/autoconf.h>

/* ARRAY_SIZE causes a conflict as it is defined both by TF-M and indirectly by devicetree.h */
#undef ARRAY_SIZE
#include <zephyr/devicetree.h>

#define UART_PIN_INIT(node_id, prop, idx) \
	DT_PROP_BY_IDX(node_id, prop, idx),

/* Configuration settings for Driver_USART0. */
#if DOMAIN_NS == 1U

#define RTE_USART0 1

#define RTE_USART0_PINS \
{ \
	DT_FOREACH_CHILD_VARGS( \
		DT_PINCTRL_BY_NAME(DT_NODELABEL(uart0), default, 0), \
		DT_FOREACH_PROP_ELEM, psels, UART_PIN_INIT \
	) \
}

#endif

/* Configuration settings for Driver_USART1. */
#if DT_PINCTRL_HAS_NAME(DT_NODELABEL(uart1), default) && DOMAIN_NS != 1U

#define RTE_USART1 1

#define RTE_USART1_PINS \
{ \
	DT_FOREACH_CHILD_VARGS( \
		DT_PINCTRL_BY_NAME(DT_NODELABEL(uart1), default, 0), \
		DT_FOREACH_PROP_ELEM, psels, UART_PIN_INIT \
	) \
}

#endif

/* Configuration settings for Driver_FLASH0. */
#define RTE_FLASH0 1

/* Configuration settings for Driver_FLASH_SPI_NOR0. */
#ifdef BL2_SPI_NOR_DRIVER_ENABLED

#define NRF_DT_GPIOS_TO_PSEL_BY_IDX(node_id, prop, idx)                                            \
	((DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, port) << 5) |                                 \
	 (DT_GPIO_PIN_BY_IDX(node_id, prop, idx) & 0x1F))

#define FLASH_SPI_NOR     DT_COMPAT_GET_ANY_STATUS_OKAY(jedec_spi_nor)
#define FLASH_SPI_NOR_BUS DT_BUS(FLASH_SPI_NOR)
#define FLASH_SPI_NOR_CS_PIN                                                                       \
	NRF_DT_GPIOS_TO_PSEL_BY_IDX(FLASH_SPI_NOR_BUS, cs_gpios, DT_REG_ADDR(FLASH_SPI_NOR))

#define NRFX_SPIM_ENABLED 1
#if DT_SAME_NODE(FLASH_SPI_NOR_BUS, DT_NODELABEL(spi0))
#define FLASH_SPI_NOR_BUS_REG  NRF_SPIM0
#define FLASH_SPI_NOR_BUS_INST NRFX_SPIM0_INST_IDX
#define NRFX_SPIM0_ENABLED     1
#elif DT_SAME_NODE(FLASH_SPI_NOR_BUS, DT_NODELABEL(spi1))
#define FLASH_SPI_NOR_BUS_REG  NRF_SPIM1
#define FLASH_SPI_NOR_BUS_INST NRFX_SPIM1_INST_IDX
#define NRFX_SPIM1_ENABLED     1
#elif DT_SAME_NODE(FLASH_SPI_NOR_BUS, DT_NODELABEL(spi2))
#define FLASH_SPI_NOR_BUS_REG  NRF_SPIM2
#define FLASH_SPI_NOR_BUS_INST NRFX_SPIM2_INST_IDX
#define NRFX_SPIM2_ENABLED     1
#elif DT_SAME_NODE(FLASH_SPI_NOR_BUS, DT_NODELABEL(spi3))
#define FLASH_SPI_NOR_BUS_REG  NRF_SPIM3
#define FLASH_SPI_NOR_BUS_INST NRFX_SPIM3_INST_IDX
#define NRFX_SPIM3_ENABLED     1
#elif DT_SAME_NODE(FLASH_SPI_NOR_BUS, DT_NODELABEL(spi4))
#define FLASH_SPI_NOR_BUS_REG  NRF_SPIM4
#define FLASH_SPI_NOR_BUS_INST NRFX_SPIM4_INST_IDX
#define NRFX_SPIM4_ENABLED     1
#else
#error Unknown SPI bus for jedec,spi-nor compatible
#endif

#endif /* BL2_SPI_NOR_DRIVER_ENABLED */

#endif /* __RTE_DEVICE_H */
