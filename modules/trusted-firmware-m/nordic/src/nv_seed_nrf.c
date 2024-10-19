/**
 * @file
 * @copyright 2024 Embeint Inc
 * @author Jordan Yates <jordan@embeint.com>
 *
 * SPDX-License-Identifier: LicenseRef-Embeint
 *
 * Provide seed requests from the CryptoCell library
 */

#include "tfm_plat_crypto_nv_seed.h"

#include "nrf_cc3xx_platform_entropy.h"

int tfm_plat_crypto_provision_entropy_seed(void)
{
	return TFM_CRYPTO_NV_SEED_SUCCESS;
}

int tfm_plat_crypto_nv_seed_read(unsigned char *buf, size_t buf_len)
{
	size_t out_len;
	int rc;

	rc = nrf_cc3xx_platform_entropy_get(buf, buf_len, &out_len);
	if ((rc != 0) || (out_len != buf_len)) {
		return TFM_CRYPTO_NV_SEED_FAILED;
	}
	return TFM_CRYPTO_NV_SEED_SUCCESS;
}

int tfm_plat_crypto_nv_seed_write(const unsigned char *buf, size_t buf_len)
{
	/* No need to support */
	return TFM_CRYPTO_NV_SEED_SUCCESS;
}
