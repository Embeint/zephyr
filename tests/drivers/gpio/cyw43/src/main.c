/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq_offload.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <whd.h>

static const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(test_gpio));
static int token;
static bool available;
static uint32_t result;
static uint32_t last_mask;
static uint32_t last_value;
static uint32_t hardware;
static unsigned int calls;

whd_interface_t airoc_wifi_get_whd_interface(void)
{
	return available ? &token : NULL;
}

uint32_t whd_wifi_set_iovar_buffer(whd_interface_t ifp, const char *name, void *buffer,
				   uint16_t length)
{
	zassert_equal(ifp, &token);
	zassert_equal(strcmp(name, "gpioout"), 0);
	zassert_equal(length, 8);
	last_mask = sys_get_le32(buffer);
	last_value = sys_get_le32((uint8_t *)buffer + 4);
	calls++;
	if (result == 0) {
		hardware = (hardware & ~last_mask) | (last_value & last_mask);
	}
	return result;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	available = true;
	result = 0;
	for (gpio_pin_t pin = 0; pin < 3; pin++) {
		zassert_ok(gpio_pin_configure(gpio, pin, GPIO_OUTPUT_LOW));
	}
	calls = 0;
}

ZTEST(gpio_cyw43, test_mask_and_value_preserve_other_pins)
{
	hardware = BIT(1) | BIT(2);
	zassert_ok(gpio_pin_set(gpio, 0, 1));
	zassert_equal(last_mask, BIT(0));
	zassert_equal(last_value, BIT(0));
	zassert_equal(hardware, 7);
	zassert_ok(gpio_pin_set(gpio, 0, 0));
	zassert_equal(hardware, BIT(1) | BIT(2));
	zassert_ok(gpio_pin_configure(gpio, 0, GPIO_OUTPUT_HIGH));
	zassert_equal(last_mask, BIT(0));
	zassert_equal(hardware, 7);
}

ZTEST(gpio_cyw43, test_errors_preserve_cached_state)
{
	gpio_port_value_t value;

	available = false;
	zassert_equal(gpio_pin_configure(gpio, 0, GPIO_OUTPUT_HIGH), -ENODEV);
	zassert_equal(gpio_pin_set(gpio, 0, 1), -ENODEV);
	zassert_equal(calls, 0);
	available = true;
	result = 1;
	zassert_equal(gpio_pin_set(gpio, 0, 1), -EIO);
	zassert_equal(gpio_port_toggle_bits(gpio, BIT(0)), -EIO);
	zassert_ok(gpio_port_get_raw(gpio, &value));
	zassert_equal(value, 0);
	result = 0;
	zassert_ok(gpio_port_toggle_bits(gpio, BIT(0)));
	zassert_equal(hardware, BIT(0));
}

ZTEST(gpio_cyw43, test_flags_and_masks)
{
	zassert_equal(gpio_pin_configure(gpio, 0, GPIO_INPUT), -ENOTSUP);
	zassert_equal(gpio_pin_configure(gpio, 0, GPIO_OUTPUT | GPIO_PULL_UP), -ENOTSUP);
	zassert_equal(gpio_pin_configure(gpio, 0, GPIO_OUTPUT | GPIO_OPEN_DRAIN), -ENOTSUP);
	zassert_equal(gpio_port_set_bits_raw(gpio, BIT(3)), -EINVAL);
	zassert_equal(calls, 0);
}

static int interrupt_result[4];
static void interrupt_call(const void *unused)
{
	gpio_port_value_t value;

	ARG_UNUSED(unused);
	interrupt_result[0] = gpio_pin_configure(gpio, 0, GPIO_OUTPUT_HIGH);
	interrupt_result[1] = gpio_pin_set(gpio, 0, 1);
	interrupt_result[2] = gpio_port_toggle_bits(gpio, BIT(0));
	interrupt_result[3] = gpio_port_get_raw(gpio, &value);
}

ZTEST(gpio_cyw43, test_interrupt_rejected)
{
	irq_offload(interrupt_call, NULL);
	for (size_t i = 0; i < ARRAY_SIZE(interrupt_result); i++) {
		zassert_equal(interrupt_result[i], -EWOULDBLOCK);
	}
	zassert_equal(calls, 0);
}

static K_THREAD_STACK_ARRAY_DEFINE(stacks, 2, 1024 + CONFIG_TEST_EXTRA_STACK_SIZE);
static struct k_thread threads[2];
static void toggle_worker(void *pin, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	for (int i = 0; i < 101; i++) {
		zassert_ok(gpio_port_toggle_bits(gpio, BIT((uintptr_t)pin)));
	}
}

ZTEST(gpio_cyw43, test_concurrent_updates)
{
	gpio_port_value_t value;

	for (uintptr_t i = 0; i < 2; i++) {
		k_thread_create(&threads[i], stacks[i], K_THREAD_STACK_SIZEOF(stacks[i]),
				toggle_worker, (void *)i, NULL, NULL, 0, 0, K_NO_WAIT);
	}
	for (int i = 0; i < 2; i++) {
		zassert_ok(k_thread_join(&threads[i], K_SECONDS(5)));
	}
	zassert_ok(gpio_port_get_raw(gpio, &value));
	zassert_equal(value, BIT(0) | BIT(1));
	zassert_equal(hardware, value);
	zassert_equal(calls, 202);
}

ZTEST_SUITE(gpio_cyw43, NULL, NULL, before, NULL, NULL);
