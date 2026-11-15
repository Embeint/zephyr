/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Helpers of the cellular modem driver, independent of its state machine */

#ifndef ZEPHYR_DRIVERS_MODEM_MODEM_CELLULAR_COMMON_H_
#define ZEPHYR_DRIVERS_MODEM_MODEM_CELLULAR_COMMON_H_

#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/modem/modem_cellular.h>

static inline bool modem_cellular_gpio_is_enabled(const struct gpio_dt_spec *gpio)
{
	return gpio->port != NULL;
}

/* Whether the status GPIO reports the modem as already powered on */
bool modem_cellular_modem_is_powered(const struct modem_cellular_config *config);

/* Switch the UART to @p desired_baudrate, returning the previous baudrate or 0 on failure */
uint32_t modem_cellular_baudrate_update(struct modem_cellular_data *data,
					uint32_t desired_baudrate);

void modem_cellular_notify_user_pipes_connected(struct modem_cellular_data *data);

void modem_cellular_notify_user_pipes_disconnected(struct modem_cellular_data *data);

/* Drop the cached network status, so that it reads as unknown until next reported */
void modem_cellular_invalidate_network_status(struct modem_cellular_data *data);

bool modem_cellular_status_is_registered(enum cellular_registration_status status);

/* Whether any access technology is registered, at home or roaming */
bool modem_cellular_is_registered(const struct modem_cellular_data *data);

/* Whether registered, or with the data context active per +CGEV */
bool modem_cellular_is_attached(const struct modem_cellular_data *data);

/*
 * Parse a +CGEV event into whether the data context is connected.
 *
 * @retval 0 Event for the data context, @p connected set.
 * @retval 1 Detach from the packet domain, which ends every context and the packet domain
 *         registration, @p connected set to false.
 * @retval -ENOENT Event for another context, or not about contexts.
 * @retval -EINVAL Not a +CGEV event.
 */
int modem_cellular_parse_cgev(const struct modem_cellular_data *data, char **argv, uint16_t argc,
			      bool *connected);

/* Forget the registration of every access technology */
void modem_cellular_clear_registration_status(struct modem_cellular_data *data);

void modem_cellular_emit_reg_state(struct modem_cellular_data *data,
				   enum cellular_registration_status status);

/*
 * Store the registration status reported by a +CREG, +CGREG, +CEREG or +C5GREG read command
 * answer or notification, along with any access technology it reports. @p status is set to
 * the reported status and @p previous to the one it replaced.
 *
 * @retval 0 Status stored.
 * @retval -EINVAL Malformed report, nothing stored.
 */
int modem_cellular_store_registration(struct modem_cellular_data *data, char **argv, uint16_t argc,
				      enum cellular_registration_status *status,
				      enum cellular_registration_status *previous);

/* Implementations of the cellular driver API */

int modem_cellular_get_registration_status(const struct device *dev,
					   enum cellular_access_technology tech,
					   enum cellular_registration_status *status);

int modem_cellular_get_modem_info(const struct device *dev, enum cellular_modem_info_type type,
				  char *info, size_t size);

int modem_cellular_get_network_status(const struct device *dev,
				      struct cellular_evt_network_status *status);

int modem_cellular_set_callback(const struct device *dev, cellular_event_mask_t mask,
				cellular_event_cb_t cb, void *user_data);

/* Whether an APN must be set before the data context can be configured */
bool modem_cellular_needs_apn(const struct modem_cellular_data *data);

/* Set the APN from CONFIG_MODEM_CELLULAR_APN and initialize the APN script */
void modem_cellular_init_apn(struct modem_cellular_data *data,
			     const struct modem_chat_match *abort_matches,
			     size_t abort_matches_size);

/* Build the APN script from the current APN */
void modem_cellular_build_apn_script(struct modem_cellular_data *data);

/*
 * Validate @p apn as a new APN: -EINVAL if NULL or too long, -EALREADY if unchanged. An empty
 * APN is valid, and has the network select the default APN of the subscription.
 */
int modem_cellular_check_apn(const struct modem_cellular_data *data, const char *apn);

/* Store @p apn, validated with modem_cellular_check_apn(), as the APN in use */
void modem_cellular_store_apn(struct modem_cellular_data *data, const char *apn);

#endif /* ZEPHYR_DRIVERS_MODEM_MODEM_CELLULAR_COMMON_H_ */
