/**
 * @file
 * @copyright 2024 Embeint Inc
 * @author Jordan Yates <jordan@embeint.com>
 *
 * SPDX-License-Identifier: LicenseRef-Embeint
 */

#include "tfm_plat_crypto_keys.h"
#include "tfm_builtin_key_ids.h"
#include "tfm_builtin_key_loader.h"

#include <tfm_spm_log.h>

#include "nrf_cc3xx_platform_kmu.h"

#if defined(NRF91_SERIES)
#define NRF_CC310
#define HUK_SIZE_WORDS 4
#elif defined(NRF53_SERIES)
#define NRF_CC312
#define HUK_SIZE_WORDS 8
#else
#error Unknown nRF series
#endif

#define HUK_SIZE_BYTES (HUK_SIZE_WORDS * 4)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static enum tfm_plat_err_t tfm_plat_get_huk(uint8_t *buf, size_t buf_len, size_t *key_len,
					    psa_key_bits_t *key_bits, psa_algorithm_t *algorithm,
					    psa_key_type_t *type)
{
	int rc;

	if (buf_len < HUK_SIZE_BYTES) {
		return TFM_PLAT_ERR_SYSTEM_ERR;
	}

	/* Derive our HUK root from the KMU HUK */
	const char *label = "INFUSE_HUK";
	const char *context = "CTX";

	rc = nrf_cc3xx_platform_kmu_shadow_key_derive(0, HUK_SIZE_BYTES * 8, (uint8_t const *)label,
						      strlen(label), (uint8_t const *)context,
						      strlen(context), buf, buf_len);
	if (rc != 0) {
		SPMLOG_ERRMSGVAL("HUK derivation error ", rc);
		return TFM_PLAT_ERR_SYSTEM_ERR;
	}

	*key_len = HUK_SIZE_BYTES;
	*key_bits = HUK_SIZE_BYTES * 8;
	*algorithm = PSA_ALG_HKDF(PSA_ALG_SHA_256);
	*type = PSA_KEY_TYPE_DERIVE;

	return TFM_PLAT_ERR_SUCCESS;
}

static const tfm_plat_builtin_key_policy_t g_builtin_keys_policy[] = {
	{
		.key_id = TFM_BUILTIN_KEY_ID_HUK,
		.per_user_policy = 0,
		.usage = PSA_KEY_USAGE_DERIVE,
	},
};

static const tfm_plat_builtin_key_descriptor_t g_builtin_keys_desc[] = {
	{
		.key_id = TFM_BUILTIN_KEY_ID_HUK,
		.slot_number = TFM_BUILTIN_KEY_SLOT_HUK,
		.lifetime = TFM_BUILTIN_KEY_LOADER_LIFETIME,
		.loader_key_func = tfm_plat_get_huk,
	},
};

size_t tfm_plat_builtin_key_get_policy_table_ptr(const tfm_plat_builtin_key_policy_t *desc_ptr[])
{
	*desc_ptr = &g_builtin_keys_policy[0];
	return ARRAY_SIZE(g_builtin_keys_policy);
}

size_t tfm_plat_builtin_key_get_desc_table_ptr(const tfm_plat_builtin_key_descriptor_t *desc_ptr[])
{
	*desc_ptr = &g_builtin_keys_desc[0];
	return ARRAY_SIZE(g_builtin_keys_desc);
}
