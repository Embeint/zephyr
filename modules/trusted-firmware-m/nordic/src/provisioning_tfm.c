/**
 * @file
 * @copyright 2024 Embeint Inc
 * @author Jordan Yates <jordan@embeint.com>
 *
 * SPDX-License-Identifier: LicenseRef-Embeint
 */

#include "tfm_platform_system.h"
#include "tfm_plat_provisioning.h"
#include "tfm_plat_otp.h"

#include "nrf_cc3xx_platform_entropy.h"
#include "nrf_cc3xx_platform_kmu.h"
#include "nrfx_nvmc.h"
#include "nrfx.h"

#include <tfm_spm_log.h>

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

static bool kmu_slot_written(uint32_t idx)
{
	bool written = false;

	/* Key slots are 1 indexed */
	NRF_KMU->SELECTKEYSLOT = idx + 1;
	if (nrfx_nvmc_uicr_word_read(&NRF_UICR_S->KEYSLOT.CONFIG[idx].PERM) != 0xFFFFFFFF) {
		written = true;
		goto end;
	}
	if (nrfx_nvmc_uicr_word_read(&NRF_UICR_S->KEYSLOT.CONFIG[idx].DEST) != 0xFFFFFFFF) {
		written = true;
		goto end;
	}
	for (int i = 0; i < 4; i++) {
		if (nrfx_nvmc_uicr_word_read(&NRF_UICR_S->KEYSLOT.KEY[idx].VALUE[i]) !=
		    0xFFFFFFFF) {
			written = true;
			goto end;
		}
	}

end:
	NRF_KMU->SELECTKEYSLOT = 0;
	return written;
}

#ifdef TFM_LCS_SECURED_DISABLE_DEBUG_PORT
static enum tfm_plat_err_t disable_debugging(void)
{
	/* Configure the UICR such that upon the next reset, APPROTECT will be enabled */
	bool approt_writable;

	approt_writable = nrfx_nvmc_word_writable_check((uint32_t)&NRF_UICR_S->APPROTECT,
							UICR_APPROTECT_PALL_Protected);
	approt_writable &= nrfx_nvmc_word_writable_check((uint32_t)&NRF_UICR_S->SECUREAPPROTECT,
							 UICR_SECUREAPPROTECT_PALL_Protected);

	if (approt_writable) {
		nrfx_nvmc_word_write((uint32_t)&NRF_UICR_S->APPROTECT,
				     UICR_APPROTECT_PALL_Protected);
		nrfx_nvmc_word_write((uint32_t)&NRF_UICR_S->SECUREAPPROTECT,
				     UICR_SECUREAPPROTECT_PALL_Protected);
	} else {
		return TFM_PLAT_ERR_SYSTEM_ERR;
	}

	return TFM_PLAT_ERR_SUCCESS;
}
#endif

enum tfm_plat_err_t provision_assembly_and_test(void)
{
	uint8_t huk[MAX(HUK_SIZE_BYTES, 32)];
	enum tfm_plat_err_t err;
	uint32_t new_lcs;
	size_t len;
	int rc;

	SPMLOG_INFMSG("[INF] Assembly & Test\n");

	/* Key already written */
	if (kmu_slot_written(0)) {
		return TFM_PLAT_ERR_SUCCESS;
	}

	/* Get a random hardware unique key */
	rc = nrf_cc3xx_platform_entropy_get(huk, sizeof(huk), &len);
	if ((rc != 0) || (len != sizeof(huk))) {
		return TFM_PLAT_ERR_SYSTEM_ERR;
	}

	SPMLOG_INFMSG("[INF] Writing HUK to key slot 0\n");

	/* Write key to KMU */
	rc = nrf_cc3xx_platform_kmu_write_key_slot(0, NRF_CC3XX_PLATFORM_KMU_AES_ADDR,
						   NRF_CC3XX_PLATFORM_KMU_DEFAULT_PERMISSIONS, huk);
#ifdef NRF_CC312
	/* 2 part key required for CC312 */
	if (rc == 0) {
		rc = nrf_cc3xx_platform_kmu_write_key_slot(
			1, NRF_CC3XX_PLATFORM_KMU_AES_ADDR_2,
			NRF_CC3XX_PLATFORM_KMU_DEFAULT_PERMISSIONS, huk + (HUK_SIZE_BYTES / 2));
	}
#endif
	/* Clear sensitive memory buffer */
	memset(huk, 0x00, HUK_SIZE_BYTES);

	if (rc != 0) {
		return TFM_PLAT_ERR_SYSTEM_ERR;
	}

	new_lcs = PLAT_OTP_LCS_PSA_ROT_PROVISIONING;
	err = tfm_plat_otp_write(PLAT_OTP_ID_LCS, sizeof(new_lcs), (uint8_t *)&new_lcs);
	if (err != TFM_PLAT_ERR_SUCCESS) {
		return err;
	}

	return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t provision_psa_rot(void)
{
	enum tfm_plat_err_t err;
	uint32_t new_lcs;

	SPMLOG_INFMSG("[INF] PSA RoT\n");

	/* The Hardware Unique Keys should be already written */
	if (!kmu_slot_written(0)) {
		SPMLOG_ERRMSG("This device has not been provisioned with Hardware Unique Keys\n");
		return TFM_PLAT_ERR_SYSTEM_ERR;
	}

#ifdef TFM_LCS_SECURED_DISABLE_DEBUG_PORT
	err = disable_debugging();
	if (err != TFM_PLAT_ERR_SUCCESS) {
		return err;
	}
#else
	SPMLOG_INFMSG("\033[1;31mDebug port not disabled on entering SECURED LCS!"
		      " This device is \033[1;1mNOT SECURE\033[0m\n");
#endif

	new_lcs = PLAT_OTP_LCS_SECURED;
	err = tfm_plat_otp_write(PLAT_OTP_ID_LCS, sizeof(new_lcs), (uint8_t *)&new_lcs);
	if (err != TFM_PLAT_ERR_SUCCESS) {
		return err;
	}

	/* Perform a mandatory reset since we switch to an attestable LCS state */
	SPMLOG_INFMSG("[INF] Rebooting into attestable state\n");
	tfm_platform_hal_system_reset();

	/*
	 * We should never return from this function, a reset should be triggered
	 * before we reach this point. Returning an error to signal that something
	 * is wrong if we reached here.
	 */
	return TFM_PLAT_ERR_SYSTEM_ERR;
}

void tfm_plat_provisioning_check_for_dummy_keys(void)
{
	/* No support for dummy keys */
}

int tfm_plat_provisioning_is_required(void)
{
	enum tfm_plat_err_t err;
	enum plat_otp_lcs_t lcs;

	err = tfm_plat_otp_read(PLAT_OTP_ID_LCS, sizeof(lcs), (uint8_t *)&lcs);
	if (err != TFM_PLAT_ERR_SUCCESS) {
		return err;
	}

	return lcs == PLAT_OTP_LCS_ASSEMBLY_AND_TEST || lcs == PLAT_OTP_LCS_PSA_ROT_PROVISIONING;
}

enum tfm_plat_err_t tfm_plat_provisioning_perform(void)
{
	enum tfm_plat_err_t err;
	enum plat_otp_lcs_t lcs;

	err = tfm_plat_otp_read(PLAT_OTP_ID_LCS, sizeof(lcs), (uint8_t *)&lcs);
	if (err != TFM_PLAT_ERR_SUCCESS) {
		return err;
	}

	SPMLOG_INFMSG("[INF] Beginning TF-M provisioning (nRF)\n");

	if (lcs == PLAT_OTP_LCS_ASSEMBLY_AND_TEST) {
		err = provision_assembly_and_test();
		if (err != TFM_PLAT_ERR_SUCCESS) {
			return err;
		}
	}

	err = tfm_plat_otp_read(PLAT_OTP_ID_LCS, sizeof(lcs), (uint8_t *)&lcs);
	if (err != TFM_PLAT_ERR_SUCCESS) {
		return err;
	}

	if (lcs == PLAT_OTP_LCS_PSA_ROT_PROVISIONING) {
		err = provision_psa_rot();
		if (err != TFM_PLAT_ERR_SUCCESS) {
			return err;
		}
	}

	/* We should never get here, as PSA RoT Provisioning should have
	 * triggered a reboot into the SECURED (attestable) state.
	 */
	return TFM_PLAT_ERR_SYSTEM_ERR;
}
