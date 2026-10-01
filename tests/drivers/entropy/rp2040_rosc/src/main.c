/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/irq_offload.h>
#include <zephyr/ztest.h>

static const struct device *rng = DEVICE_DT_GET(DT_NODELABEL(test_rng));
static bool oscillator_ready = true;
static bool clock_ready = true;
static bool stuck;
static uint32_t samples;
static const uint8_t zeros[7];
static const uint8_t untouched[4] = {0xa5, 0xa5, 0xa5, 0xa5};
static int early_result;
static uint8_t early_bytes[3];

uint32_t sys_read32(uintptr_t address)
{
	if (address == 0x40060018) {
		return oscillator_ready ? BIT(12) | BIT(31) : 0;
	}
	if (address == 0x4006001c) {
		return stuck ? 0 : (samples++ & 1U);
	}
	if (address == 0x40008044) {
		return clock_ready ? BIT(1) : 0;
	}
	/* PLL_SYS is auxiliary source zero. */
	return 0;
}

static int early_call(void)
{
	early_result =
		entropy_get_entropy_isr(rng, early_bytes, sizeof(early_bytes), ENTROPY_BUSYWAIT);
	return 0;
}
/* Busy waits require the system timer, initialized at priority zero. */
SYS_INIT(early_call, PRE_KERNEL_2, 99);

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	oscillator_ready = true;
	clock_ready = true;
	stuck = false;
	samples = 0;
}

ZTEST(rosc, test_early_boot)
{
	zassert_equal(early_result, sizeof(early_bytes));
	zassert_mem_equal(early_bytes, zeros, sizeof(early_bytes));
}

ZTEST(rosc, test_partial_unaligned_and_canaries)
{
	uint8_t buffer[19];

	for (size_t length = 1; length < sizeof(buffer) - 1; length++) {
		memset(buffer, 0xa5, sizeof(buffer));
		zassert_ok(entropy_get_entropy(rng, buffer + 1, length));
		zassert_equal(buffer[0], 0xa5);
		zassert_equal(buffer[length + 1], 0xa5);
	}
	zassert_equal(samples, 16 * 153);
}

ZTEST(rosc, test_nonblocking_and_flags)
{
	uint8_t buffer[4] = {0xa5, 0xa5, 0xa5, 0xa5};

	zassert_equal(entropy_get_entropy_isr(rng, buffer, sizeof(buffer), 0), -EAGAIN);
	zassert_mem_equal(buffer, untouched, sizeof(buffer));
	zassert_equal(entropy_get_entropy_isr(rng, buffer, sizeof(buffer), BIT(7)), -EINVAL);
	zassert_ok(entropy_get_entropy_isr(rng, NULL, 0, 0));
	zassert_ok(entropy_get_entropy(rng, NULL, 0));
	zassert_equal(entropy_get_entropy(rng, NULL, 1), -EINVAL);
	zassert_equal(samples, 0);
}

ZTEST(rosc, test_oscillator_and_clock_failures)
{
	uint8_t buffer = 0xa5;

	oscillator_ready = false;
	zassert_equal(entropy_get_entropy(rng, &buffer, 1), -EIO);
	oscillator_ready = true;
	clock_ready = false;
	zassert_equal(entropy_get_entropy(rng, &buffer, 1), -ENOTSUP);
	zassert_equal(buffer, 0xa5);
	clock_ready = true;
	zassert_ok(entropy_get_entropy(rng, &buffer, 1));
}

ZTEST(rosc, test_stuck_source_is_bounded)
{
	uint8_t buffer = 0xa5;

	stuck = true;
	zassert_equal(entropy_get_entropy(rng, &buffer, 1), -EIO);
	zassert_equal(buffer, 0xa5);
}

static int interrupt_result;
static void interrupt_call(const void *unused)
{
	uint8_t buffer[3];

	ARG_UNUSED(unused);
	interrupt_result = entropy_get_entropy_isr(rng, buffer, sizeof(buffer), ENTROPY_BUSYWAIT);
}

ZTEST(rosc, test_interrupt)
{
	irq_offload(interrupt_call, NULL);
	zassert_equal(interrupt_result, 3);
	zassert_equal(samples, 48);
}

static K_THREAD_STACK_ARRAY_DEFINE(stacks, 2, 1024 + CONFIG_TEST_EXTRA_STACK_SIZE);
static struct k_thread threads[2];
static void worker(void *unused1, void *unused2, void *unused3)
{
	uint8_t buffer[7];

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);
	for (int i = 0; i < 20; i++) {
		zassert_ok(entropy_get_entropy(rng, buffer, sizeof(buffer)));
		zassert_mem_equal(buffer, zeros, sizeof(buffer));
	}
}

ZTEST(rosc, test_concurrent_callers)
{
	for (int i = 0; i < 2; i++) {
		k_thread_create(&threads[i], stacks[i], K_THREAD_STACK_SIZEOF(stacks[i]), worker,
				NULL, NULL, NULL, 0, 0, K_NO_WAIT);
	}
	for (int i = 0; i < 2; i++) {
		zassert_ok(k_thread_join(&threads[i], K_SECONDS(10)));
	}
	zassert_equal(samples, 2 * 20 * 7 * 16);
}

ZTEST_SUITE(rosc, NULL, NULL, before, NULL, NULL);
