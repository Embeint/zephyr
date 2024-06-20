/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tfm_hal_defs.h"
#include "tfm_hal_platform_common.h"

#ifdef NRF_CC3XX_PLATFORM_LIB
#include "nrf_cc3xx_platform.h"
#endif

enum tfm_hal_status_t tfm_hal_platform_init(void)
{
#ifdef NRF_CC3XX_PLATFORM_LIB
	if (nrf_cc3xx_platform_init() != 0) {
		return TFM_HAL_ERROR_GENERIC;
	}
#endif
	return tfm_hal_platform_common_init();
}
