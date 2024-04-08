/**
 * @file
 * @brief MbedTLS PSA integration
 * @copyright 2024 Embeint Pty Ltd
 * @author Jordan Yates <jordan@embeint.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

#include <psa/crypto.h>

#ifdef CONFIG_MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG
int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen);

psa_status_t mbedtls_psa_external_get_random(mbedtls_psa_external_random_context_t *context,
					     uint8_t *output, size_t output_size,
					     size_t *output_length)
{
	int rc;

	ARG_UNUSED(context);

	rc = mbedtls_hardware_poll(NULL, output, output_size, output_length);
	return rc == 0 ? PSA_SUCCESS : PSA_ERROR_HARDWARE_FAILURE;
}
#endif /* CONFIG_MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG */
