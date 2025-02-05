/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "platform/include/tfm_platform_system.h"
#include "cmsis.h"
#include "tfm_platform_hal_ioctl.h"
#include "tfm_ioctl_api.h"

#include "exception_info.h"
#include "handle_attr.h"
#include "tfm_hal_isolation.h"

#ifdef TFM_PLATFORM_FAULT_INFO_QUERY
#define SECURE_FAULT_VALID_KEY 0xA89371E1
struct {
	uint32_t valid_key;
	struct exception_info_t info;
} secure_fault_info __NO_INIT;
#endif /* TFM_PLATFORM_FAULT_INFO_QUERY */

void tfm_platform_hal_system_reset(void)
{
#ifdef TFM_PLATFORM_FAULT_INFO_QUERY
	/* Preserve fault information */
	tfm_exception_info_get_context(&secure_fault_info.info);
	/* Presume data was populated if either of these values are not zero */
	if (secure_fault_info.info.EXC_RETURN != 0 || secure_fault_info.info.xPSR != 0) {
		secure_fault_info.valid_key = SECURE_FAULT_VALID_KEY;
	}
#endif /* TFM_PLATFORM_FAULT_INFO_QUERY */
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

#ifdef TFM_PLATFORM_FAULT_INFO_QUERY
enum tfm_platform_err_t tfm_platform_hal_fault_service(const psa_invec *in_vec,
						       const psa_outvec *out_vec)
{
	struct tfm_fault_info_service_args *args;
	struct tfm_fault_info_service_out *out;
	enum tfm_hal_status_t status;
	uintptr_t boundary = (1 << HANDLE_ATTR_NS_POS) & HANDLE_ATTR_NS_MASK;
	uint32_t attr = TFM_HAL_ACCESS_READWRITE;

	if (in_vec->len != sizeof(struct tfm_fault_info_service_args) ||
	    out_vec->len != sizeof(struct tfm_fault_info_service_out)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	args = (struct tfm_fault_info_service_args *)in_vec->base;
	out = (struct tfm_fault_info_service_out *)out_vec->base;

	/* Default failure */
	out->result = 0;

	if (args->destination == NULL) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	status = tfm_hal_memory_check(boundary, (uintptr_t)args->destination,
				      sizeof(args->destination), attr);
	if (status != TFM_HAL_SUCCESS) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	/* TF-M doesn't have build time assertions */
	if (sizeof(*args->destination) != sizeof(secure_fault_info.info)) {
		return TFM_PLATFORM_ERR_SYSTEM_ERROR;
	}

	if (secure_fault_info.valid_key == SECURE_FAULT_VALID_KEY) {
		/* Copy the fault info to nonsecure memory */
		memcpy(args->destination, &secure_fault_info.info, sizeof(*args->destination));
		out->result = sizeof(*args->destination);
		/* Clear the validity key */
		secure_fault_info.valid_key = 0x00;
	}

	return TFM_PLATFORM_ERR_SUCCESS;
}
#endif /* TFM_PLATFORM_FAULT_INFO_QUERY */

enum tfm_platform_err_t tfm_platform_hal_ioctl(tfm_platform_ioctl_req_t request, psa_invec *in_vec,
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
#ifdef TFM_PLATFORM_FAULT_INFO_QUERY
	case TFM_PLATFORM_IOCTL_FAULT_INFO_SERVICE:
		return tfm_platform_hal_fault_service(in_vec, out_vec);
#endif /* TFM_PLATFORM_FAULT_INFO_QUERY */

	/* Not a supported IOCTL service.*/
	default:
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}
}
