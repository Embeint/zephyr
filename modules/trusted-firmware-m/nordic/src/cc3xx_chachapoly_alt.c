/*
 * Copyright (c) 2026 Embeint Inc
 * Copyright The Mbed TLS Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ChaCha20-Poly1305 (RFC 8439) for the TF-M crypto partition.
 *
 * One-shot operations generate the ChaCha20 keystream on the CC3XX Cryptocell
 * through the nrfxlib PSA cipher driver, while Poly1305 is computed in
 * software. The Cryptocell Poly1305 implementation runs on the PKA engine and
 * has a large fixed cost per operation, which makes it slower than the
 * software implementation for typical message sizes.
 *
 * Multi-part operations (and one-shot operations that the Cryptocell cannot
 * process) use a software implementation derived from the Mbed TLS
 * library/chachapoly.c.
 */

#define MBEDTLS_ALLOW_PRIVATE_ACCESS

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "mbedtls/chachapoly.h"

#if defined(MBEDTLS_CHACHAPOLY_C) && defined(MBEDTLS_CHACHAPOLY_ALT)

#include "mbedtls/constant_time.h"
#include "mbedtls/error.h"
#include "mbedtls/platform_util.h"

#include "psa/crypto.h"
#include "cc3xx_psa_cipher.h"

#define CHACHAPOLY_STATE_INIT       (0)
#define CHACHAPOLY_STATE_AAD        (1)
#define CHACHAPOLY_STATE_CIPHERTEXT (2) /* Encrypting or decrypting */
#define CHACHAPOLY_STATE_FINISHED   (3)

#define CHACHAPOLY_KEY_LEN   32U
#define CHACHAPOLY_NONCE_LEN 12U
#define CHACHAPOLY_TAG_LEN   16U

/* Cryptocell DMA can only access RAM */
static bool dma_accessible(const void *ptr, size_t len)
{
	uintptr_t start = (uintptr_t)ptr;
	uintptr_t end;

	if (len == 0) {
		return true;
	}
	if ((ptr == NULL) || __builtin_add_overflow(start, len - 1, &end)) {
		return false;
	}
	return ((start & 0xE0000000UL) == 0x20000000UL) && ((end & 0xE0000000UL) == 0x20000000UL);
}

/* Buffers must either be identical (in-place) or not overlap at all */
static bool buffers_compatible(const unsigned char *in, const unsigned char *out, size_t len)
{
	if ((len == 0) || (in == out)) {
		return true;
	}
	return ((in + len) <= out) || ((out + len) <= in);
}

static bool hw_usable(const mbedtls_chachapoly_context *ctx, const unsigned char *nonce,
		      const unsigned char *input, unsigned char *output, size_t length)
{
	return ctx->key_set && dma_accessible(ctx->key, CHACHAPOLY_KEY_LEN) &&
	       dma_accessible(nonce, CHACHAPOLY_NONCE_LEN) && dma_accessible(input, length) &&
	       dma_accessible(output, length) && buffers_compatible(input, output, length);
}

static void hw_key_attributes(psa_key_attributes_t *attributes)
{
	*attributes = psa_key_attributes_init();
	psa_set_key_type(attributes, PSA_KEY_TYPE_CHACHA20);
	psa_set_key_bits(attributes, 8 * CHACHAPOLY_KEY_LEN);
}

/* Software implementation, derived from Mbed TLS library/chachapoly.c */

static int chachapoly_pad(mbedtls_chachapoly_context *ctx, uint64_t len)
{
	uint32_t partial_block_len = (uint32_t)(len % 16U);
	unsigned char zeroes[15];

	if (partial_block_len == 0U) {
		return 0;
	}
	memset(zeroes, 0, sizeof(zeroes));
	return mbedtls_poly1305_update(&ctx->poly1305_ctx, zeroes, 16U - partial_block_len);
}

static void put_uint64_le(uint64_t val, unsigned char *out)
{
	for (int i = 0; i < 8; i++) {
		out[i] = (unsigned char)(val >> (8 * i));
	}
}

void mbedtls_chachapoly_init(mbedtls_chachapoly_context *ctx)
{
	mbedtls_chacha20_init(&ctx->chacha20_ctx);
	mbedtls_poly1305_init(&ctx->poly1305_ctx);
	ctx->aad_len = 0U;
	ctx->ciphertext_len = 0U;
	ctx->state = CHACHAPOLY_STATE_INIT;
	ctx->mode = MBEDTLS_CHACHAPOLY_ENCRYPT;
	memset(ctx->key, 0, sizeof(ctx->key));
	ctx->key_set = 0;
}

void mbedtls_chachapoly_free(mbedtls_chachapoly_context *ctx)
{
	if (ctx == NULL) {
		return;
	}
	mbedtls_chacha20_free(&ctx->chacha20_ctx);
	mbedtls_poly1305_free(&ctx->poly1305_ctx);
	ctx->aad_len = 0U;
	ctx->ciphertext_len = 0U;
	ctx->state = CHACHAPOLY_STATE_INIT;
	ctx->mode = MBEDTLS_CHACHAPOLY_ENCRYPT;
	mbedtls_platform_zeroize(ctx->key, sizeof(ctx->key));
	ctx->key_set = 0;
}

int mbedtls_chachapoly_setkey(mbedtls_chachapoly_context *ctx, const unsigned char key[32])
{
	int ret;

	ret = mbedtls_chacha20_setkey(&ctx->chacha20_ctx, key);
	if (ret == 0) {
		memcpy(ctx->key, key, CHACHAPOLY_KEY_LEN);
		ctx->key_set = 1;
	}
	return ret;
}

int mbedtls_chachapoly_starts(mbedtls_chachapoly_context *ctx, const unsigned char nonce[12],
			      mbedtls_chachapoly_mode_t mode)
{
	int ret;
	unsigned char poly1305_key[64];

	/* Set counter = 0, will be update to 1 when generating Poly1305 key */
	ret = mbedtls_chacha20_starts(&ctx->chacha20_ctx, nonce, 0U);
	if (ret != 0) {
		goto cleanup;
	}

	/* Generate the Poly1305 key by getting the ChaCha20 keystream output with
	 * counter = 0. Only the first 256-bits (32 bytes) of the key is used.
	 */
	memset(poly1305_key, 0, sizeof(poly1305_key));
	ret = mbedtls_chacha20_update(&ctx->chacha20_ctx, sizeof(poly1305_key), poly1305_key,
				      poly1305_key);
	if (ret != 0) {
		goto cleanup;
	}

	ret = mbedtls_poly1305_starts(&ctx->poly1305_ctx, poly1305_key);
	if (ret == 0) {
		ctx->aad_len = 0U;
		ctx->ciphertext_len = 0U;
		ctx->state = CHACHAPOLY_STATE_AAD;
		ctx->mode = mode;
	}

cleanup:
	mbedtls_platform_zeroize(poly1305_key, sizeof(poly1305_key));
	return ret;
}

int mbedtls_chachapoly_update_aad(mbedtls_chachapoly_context *ctx, const unsigned char *aad,
				  size_t aad_len)
{
	if (ctx->state != CHACHAPOLY_STATE_AAD) {
		return MBEDTLS_ERR_CHACHAPOLY_BAD_STATE;
	}

	ctx->aad_len += aad_len;
	return mbedtls_poly1305_update(&ctx->poly1305_ctx, aad, aad_len);
}

int mbedtls_chachapoly_update(mbedtls_chachapoly_context *ctx, size_t len,
			      const unsigned char *input, unsigned char *output)
{
	int ret;

	if ((ctx->state != CHACHAPOLY_STATE_AAD) && (ctx->state != CHACHAPOLY_STATE_CIPHERTEXT)) {
		return MBEDTLS_ERR_CHACHAPOLY_BAD_STATE;
	}

	if (ctx->state == CHACHAPOLY_STATE_AAD) {
		ctx->state = CHACHAPOLY_STATE_CIPHERTEXT;
		ret = chachapoly_pad(ctx, ctx->aad_len);
		if (ret != 0) {
			return ret;
		}
	}

	ctx->ciphertext_len += len;

	if (ctx->mode == MBEDTLS_CHACHAPOLY_ENCRYPT) {
		ret = mbedtls_chacha20_update(&ctx->chacha20_ctx, len, input, output);
		if (ret != 0) {
			return ret;
		}
		ret = mbedtls_poly1305_update(&ctx->poly1305_ctx, output, len);
	} else {
		ret = mbedtls_poly1305_update(&ctx->poly1305_ctx, input, len);
		if (ret != 0) {
			return ret;
		}
		ret = mbedtls_chacha20_update(&ctx->chacha20_ctx, len, input, output);
	}
	return ret;
}

int mbedtls_chachapoly_finish(mbedtls_chachapoly_context *ctx, unsigned char mac[16])
{
	int ret;
	unsigned char len_block[16];

	if (ctx->state == CHACHAPOLY_STATE_INIT) {
		return MBEDTLS_ERR_CHACHAPOLY_BAD_STATE;
	}

	if (ctx->state == CHACHAPOLY_STATE_AAD) {
		ret = chachapoly_pad(ctx, ctx->aad_len);
	} else if (ctx->state == CHACHAPOLY_STATE_CIPHERTEXT) {
		ret = chachapoly_pad(ctx, ctx->ciphertext_len);
	} else {
		ret = 0;
	}
	if (ret != 0) {
		return ret;
	}

	ctx->state = CHACHAPOLY_STATE_FINISHED;

	/* The lengths of the AAD and ciphertext are processed by Poly1305 as the
	 * final 128-bit block, encoded as little-endian integers.
	 */
	put_uint64_le(ctx->aad_len, len_block);
	put_uint64_le(ctx->ciphertext_len, len_block + 8);

	ret = mbedtls_poly1305_update(&ctx->poly1305_ctx, len_block, 16U);
	if (ret != 0) {
		return ret;
	}
	return mbedtls_poly1305_finish(&ctx->poly1305_ctx, mac);
}

static int sw_crypt_and_tag(mbedtls_chachapoly_context *ctx, mbedtls_chachapoly_mode_t mode,
			    size_t length, const unsigned char nonce[12], const unsigned char *aad,
			    size_t aad_len, const unsigned char *input, unsigned char *output,
			    unsigned char tag[16])
{
	int ret;

	ret = mbedtls_chachapoly_starts(ctx, nonce, mode);
	if (ret != 0) {
		return ret;
	}
	ret = mbedtls_chachapoly_update_aad(ctx, aad, aad_len);
	if (ret != 0) {
		return ret;
	}
	ret = mbedtls_chachapoly_update(ctx, length, input, output);
	if (ret != 0) {
		return ret;
	}
	return mbedtls_chachapoly_finish(ctx, tag);
}

/* Hybrid implementation: ChaCha20 on the Cryptocell, Poly1305 in software */

static int poly1305_tag(const unsigned char poly1305_key[32], const unsigned char *aad,
			size_t aad_len, const unsigned char *ciphertext, size_t length,
			unsigned char tag[16])
{
	static const unsigned char zeroes[15];
	mbedtls_poly1305_context poly1305_ctx;
	unsigned char len_block[16];
	int ret;

	mbedtls_poly1305_init(&poly1305_ctx);

	ret = mbedtls_poly1305_starts(&poly1305_ctx, poly1305_key);
	if (ret != 0) {
		goto cleanup;
	}
	ret = mbedtls_poly1305_update(&poly1305_ctx, aad, aad_len);
	if ((ret == 0) && ((aad_len % 16U) != 0U)) {
		ret = mbedtls_poly1305_update(&poly1305_ctx, zeroes, 16U - (aad_len % 16U));
	}
	if (ret == 0) {
		ret = mbedtls_poly1305_update(&poly1305_ctx, ciphertext, length);
	}
	if ((ret == 0) && ((length % 16U) != 0U)) {
		ret = mbedtls_poly1305_update(&poly1305_ctx, zeroes, 16U - (length % 16U));
	}
	if (ret != 0) {
		goto cleanup;
	}

	put_uint64_le(aad_len, len_block);
	put_uint64_le(length, len_block + 8);
	ret = mbedtls_poly1305_update(&poly1305_ctx, len_block, sizeof(len_block));
	if (ret == 0) {
		ret = mbedtls_poly1305_finish(&poly1305_ctx, tag);
	}

cleanup:
	mbedtls_poly1305_free(&poly1305_ctx);
	return ret;
}

/* Start a Cryptocell ChaCha20 operation (counter 0) and output the first
 * keystream block, which is used as the Poly1305 key.
 */
static psa_status_t hw_chacha20_start(const mbedtls_chachapoly_context *ctx,
				      const unsigned char nonce[12], cc3xx_cipher_operation_t *op,
				      unsigned char poly1305_key[64])
{
	psa_key_attributes_t attributes;
	psa_status_t status;
	size_t out_len;

	hw_key_attributes(&attributes);
	memset(op, 0, sizeof(*op));

	status = cc3xx_cipher_encrypt_setup(op, &attributes, ctx->key, CHACHAPOLY_KEY_LEN,
					    PSA_ALG_STREAM_CIPHER);
	if (status == PSA_SUCCESS) {
		status = cc3xx_cipher_set_iv(op, nonce, CHACHAPOLY_NONCE_LEN);
	}
	if (status == PSA_SUCCESS) {
		memset(poly1305_key, 0, 64);
		status = cc3xx_cipher_update(op, poly1305_key, 64, poly1305_key, 64, &out_len);
		if ((status == PSA_SUCCESS) && (out_len != 64)) {
			status = PSA_ERROR_GENERIC_ERROR;
		}
	}
	if (status != PSA_SUCCESS) {
		(void)cc3xx_cipher_abort(op);
	}
	return status;
}

/* Process the message with the remaining keystream (counter 1 onwards) */
static psa_status_t hw_chacha20_finish(cc3xx_cipher_operation_t *op, const unsigned char *input,
				       unsigned char *output, size_t length)
{
	psa_status_t status = PSA_SUCCESS;
	size_t update_len = 0;
	size_t finish_len = 0;

	if (length > 0) {
		status = cc3xx_cipher_update(op, input, length, output, length, &update_len);
	}
	if (status == PSA_SUCCESS) {
		status = cc3xx_cipher_finish(op, output + update_len, length - update_len,
					     &finish_len);
	}
	if ((status == PSA_SUCCESS) && ((update_len + finish_len) != length)) {
		status = PSA_ERROR_GENERIC_ERROR;
	}
	if (status != PSA_SUCCESS) {
		(void)cc3xx_cipher_abort(op);
	}
	return status;
}

int mbedtls_chachapoly_encrypt_and_tag(mbedtls_chachapoly_context *ctx, size_t length,
				       const unsigned char nonce[12], const unsigned char *aad,
				       size_t aad_len, const unsigned char *input,
				       unsigned char *output, unsigned char tag[16])
{
	if (hw_usable(ctx, nonce, input, output, length)) {
		cc3xx_cipher_operation_t op;
		unsigned char poly1305_key[64];
		int ret;

		if (hw_chacha20_start(ctx, nonce, &op, poly1305_key) == PSA_SUCCESS) {
			if (hw_chacha20_finish(&op, input, output, length) == PSA_SUCCESS) {
				ret = poly1305_tag(poly1305_key, aad, aad_len, output, length, tag);
			} else {
				/* Input may already be modified for in-place operations */
				ret = MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
			}
			mbedtls_platform_zeroize(poly1305_key, sizeof(poly1305_key));
			return ret;
		}
		/* Nothing processed yet, fall back to the software implementation */
	}

	return sw_crypt_and_tag(ctx, MBEDTLS_CHACHAPOLY_ENCRYPT, length, nonce, aad, aad_len, input,
				output, tag);
}

int mbedtls_chachapoly_auth_decrypt(mbedtls_chachapoly_context *ctx, size_t length,
				    const unsigned char nonce[12], const unsigned char *aad,
				    size_t aad_len, const unsigned char tag[16],
				    const unsigned char *input, unsigned char *output)
{
	unsigned char check_tag[16];
	int ret;

	if (hw_usable(ctx, nonce, input, output, length)) {
		cc3xx_cipher_operation_t op;
		unsigned char poly1305_key[64];

		if (hw_chacha20_start(ctx, nonce, &op, poly1305_key) == PSA_SUCCESS) {
			/* Authenticate the ciphertext before decrypting it */
			ret = poly1305_tag(poly1305_key, aad, aad_len, input, length, check_tag);
			mbedtls_platform_zeroize(poly1305_key, sizeof(poly1305_key));
			if ((ret == 0) &&
			    (mbedtls_ct_memcmp(tag, check_tag, sizeof(check_tag)) != 0)) {
				ret = MBEDTLS_ERR_CHACHAPOLY_AUTH_FAILED;
			}
			if (ret != 0) {
				(void)cc3xx_cipher_abort(&op);
				return ret;
			}
			if (hw_chacha20_finish(&op, input, output, length) != PSA_SUCCESS) {
				return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
			}
			return 0;
		}
		/* Nothing processed yet, fall back to the software implementation */
	}

	ret = sw_crypt_and_tag(ctx, MBEDTLS_CHACHAPOLY_DECRYPT, length, nonce, aad, aad_len, input,
			       output, check_tag);
	if (ret != 0) {
		return ret;
	}

	/* Check tag in "constant-time" */
	if (mbedtls_ct_memcmp(tag, check_tag, sizeof(check_tag)) != 0) {
		mbedtls_platform_zeroize(output, length);
		return MBEDTLS_ERR_CHACHAPOLY_AUTH_FAILED;
	}
	return 0;
}

#endif /* MBEDTLS_CHACHAPOLY_C && MBEDTLS_CHACHAPOLY_ALT */
