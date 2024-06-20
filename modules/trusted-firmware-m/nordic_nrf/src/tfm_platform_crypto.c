/*
 * Copyright (c) 2024 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <psa/crypto.h>
#include <psa/crypto_platform.h>

/* Standard definition is hidden in the source tree */
int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen);

psa_status_t mbedtls_psa_external_get_random(void *context, uint8_t *output, size_t output_size,
					     size_t *output_length)
{
	int rc;

	(void)context;

	rc = mbedtls_hardware_poll(NULL, output, output_size, output_length);
	return rc == 0 ? PSA_SUCCESS : PSA_ERROR_HARDWARE_FAILURE;
}
