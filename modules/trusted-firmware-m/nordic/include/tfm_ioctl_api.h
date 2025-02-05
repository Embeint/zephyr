/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TFM_IOCTL_API_H__
#define TFM_IOCTL_API_H__

#include <limits.h>
#include <stdint.h>
#include <tfm_platform_api.h>

/* Include core IOCTL services */
#include <tfm_ioctl_core_api.h>

#ifdef __cplusplus
extern "C" {
#endif

enum tfm_platform_ioctl_extension_request_types_t {
	TFM_PLATFORM_IOCTL_FAULT_INFO_SERVICE = TFM_PLATFORM_IOCTL_CORE_LAST,
};

/** @brief Argument list for each platform fault service.
 */
struct tfm_fault_info_service_args {
	struct fault_exception_info_t *destination;
};

/** @brief Output list for each fault platform service
 */
struct tfm_fault_info_service_out {
	uint32_t result;
};

/** @brief Secure fault exception information */
struct fault_exception_info_t {
	uint32_t VECTACTIVE;           /* Active exception number. */
	uint32_t EXC_RETURN;           /* EXC_RETURN value in LR. */
	uint32_t MSP;                  /* (Secure) MSP. */
	uint32_t PSP;                  /* (Secure) PSP. */
	uint32_t *EXC_FRAME;           /* Exception frame on stack. */
	uint32_t EXC_FRAME_COPY[8];    /* Copy of the basic exception frame. */
	uint32_t CALLEE_SAVED_COPY[8]; /* Copy of the callee saved registers. */
	uint32_t xPSR;                 /* Program Status Registers. */
	uint32_t CFSR;                 /* Configurable Fault Status Register. */
	uint32_t HFSR;                 /* Hard Fault Status Register. */
	uint32_t BFAR;                 /* Bus Fault address register. */
	uint32_t BFARVALID;            /* Whether BFAR contains a valid address. */
	uint32_t MMFAR;                /* MemManage Fault address register. */
	uint32_t MMARVALID;            /* Whether MMFAR contains a valid address. */
	uint32_t SFSR;                 /* SecureFault Status Register. */
	uint32_t SFAR;                 /* SecureFault Address Register. */
	uint32_t SFARVALID;            /* Whether SFAR contains a valid address. */
} __PACKED;

/**
 * @brief Read a stored secure fault dump.
 *
 * @note After a successful read the fault dump is cleared
 *
 * @param destination        Pointer where the fault dump is stored

 * @param[out] result        Result of operation
 *
 * @return Returns values as specified by the tfm_platform_err_t
 */
enum tfm_platform_err_t tfm_platform_fault_info_read(struct fault_exception_info_t *destination,
						     uint32_t *result);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* TFM_IOCTL_API_H__ */
