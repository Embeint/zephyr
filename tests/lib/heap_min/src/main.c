/*
 * Copyright (c) 2025 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/sys_heap.h>
#include <inttypes.h>

#include "assert.h"

DEFINE_FFF_GLOBALS;

/* Align the test memory to the heap chunk size */
uint8_t heapmem[8192] __aligned(8);

ZTEST(lib_heap_min, test_heap_min_size_assert)
{
	struct sys_heap heap;

	expect_assert();
	sys_heap_init(&heap, (void *)heapmem, Z_HEAP_MIN_SIZE - 1);
	zassert_unreachable();
}

ZTEST(lib_heap_min, test_heap_min_size)
{
	struct sys_heap heap;
	void *mem;

	sys_heap_init(&heap, (void *)heapmem, Z_HEAP_MIN_SIZE);
	mem = sys_heap_alloc(&heap, 1);
	zassert_not_null(mem, "Could not allocate 1 byte from a Z_HEAP_MIN_SIZE heap");
	sys_heap_free(&heap, mem);
}

static int find_maximum_single_allocation(struct sys_heap *heap, int base_size)
{
	void *mem;

	for (int i = base_size + 16; i > 0; i--) {
		mem = sys_heap_alloc(heap, i);
		if (mem) {
			sys_heap_free(heap, mem);
			return i;
		}
	}
	return 0;
}

ZTEST(lib_heap_min, test_heap_overhead)
{
	int16_t base_heap_sizes[] = {64, 128, 256, 512, 1024, 2048};
	struct sys_heap heap;
	int base;
	int no_overhead;
	int min_overhead;
	int macro_overhead;

	TC_PRINT("Base |  Raw | Z_HEAP_MIN_SIZE | Z_HEAP_OVERHEAD\n");

	for (int i = 0; i < ARRAY_SIZE(base_heap_sizes); i++) {
		base = base_heap_sizes[i];

		sys_heap_init(&heap, heapmem, base);
		no_overhead = find_maximum_single_allocation(&heap, base);
		sys_heap_init(&heap, heapmem, base + Z_HEAP_MIN_SIZE);
		min_overhead = find_maximum_single_allocation(&heap, base);
		sys_heap_init(&heap, heapmem, base + Z_HEAP_OVERHEAD(base));
		macro_overhead = find_maximum_single_allocation(&heap, base);

		TC_PRINT("%4u | %4u | %15u | %15u\n", base, no_overhead, min_overhead,
			 macro_overhead);
	}
}

ZTEST_SUITE(lib_heap_min, NULL, NULL, NULL, NULL, NULL);
