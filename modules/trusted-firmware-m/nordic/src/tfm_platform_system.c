/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "platform/include/tfm_platform_system.h"
#include "cmsis.h"
#include "tfm_platform_hal_ioctl.h"
#include "tfm_ioctl_core_api.h"

void tfm_platform_hal_system_reset(void)
{
	/* Reset the system */
	NVIC_SystemReset();
}

#ifndef PLATFORM_DEFAULT_SYSTEM_RESET_HALT

void tfm_hal_system_reset(void)
{
	tfm_platform_hal_system_reset();
}

void tfm_hal_system_halt(void)
{
	/*
	 * Disable IRQs to stop all threads, not just the thread that
	 * halted the system.
	 */
	__disable_irq();

	/*
	 * Enter sleep to reduce power consumption and do it in a loop in
	 * case a signal wakes up the CPU.
	 */
	while (1) {
		__WFE();
	}
}

#endif /* PLATFORM_DEFAULT_SYSTEM_RESET_HALT */

enum tfm_platform_err_t tfm_platform_hal_ioctl(tfm_platform_ioctl_req_t request,
					       psa_invec  *in_vec,
					       psa_outvec *out_vec)
{
	/* Core IOCTL services */
	switch (request) {
	case TFM_PLATFORM_IOCTL_READ_SERVICE:
		return tfm_platform_hal_read_service(in_vec, out_vec);
#if defined(GPIO_PIN_CNF_MCUSEL_Msk)
	case TFM_PLATFORM_IOCTL_GPIO_SERVICE:
		return tfm_platform_hal_gpio_service(in_vec, out_vec);
#endif /* defined(GPIO_PIN_CNF_MCUSEL_Msk) */


	/* Board specific IOCTL services */

	/* Not a supported IOCTL service.*/
	default:
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}
}
