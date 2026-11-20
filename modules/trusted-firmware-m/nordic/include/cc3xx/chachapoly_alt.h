/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ChaCha20-Poly1305 context for the CC3XX Cryptocell implementation.
 * Included by "mbedtls/chachapoly.h" when MBEDTLS_CHACHAPOLY_ALT is defined.
 */

#ifndef NRF_CC3XX_CHACHAPOLY_ALT_H
#define NRF_CC3XX_CHACHAPOLY_ALT_H

#include <stdint.h>

#include "mbedtls/chacha20.h"
#include "mbedtls/poly1305.h"

typedef struct mbedtls_chachapoly_context {
	/* Software state for multi-part operations and fallbacks */
	mbedtls_chacha20_context chacha20_ctx;
	mbedtls_poly1305_context poly1305_ctx;
	uint64_t aad_len;
	uint64_t ciphertext_len;
	int state;
	mbedtls_chachapoly_mode_t mode;
	/* Raw key for the Cryptocell one-shot operations */
	unsigned char key[32];
	int key_set;
} mbedtls_chachapoly_context;

#endif /* NRF_CC3XX_CHACHAPOLY_ALT_H */
