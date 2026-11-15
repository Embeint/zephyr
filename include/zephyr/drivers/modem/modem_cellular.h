/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file modem_cellular.h
 * @brief Backend helpers for implementing cellular modem drivers.
 * @ingroup modem_cellular_backend
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CELLULAR_INTERNAL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CELLULAR_INTERNAL_H_

#include <zephyr/drivers/cellular.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/cmux.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/modem/ppp.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup modem_cellular_backend Cellular modem helpers
 * @brief Implement vendor-specific cellular modem drivers using common chat, CMUX, PPP, and
 * power-management support.
 * @in_driverbackendgroup{cellular_interface}
 * @{
 */

/** @cond INTERNAL_HIDDEN */

#define MODEM_CELLULAR_DATA_IMEI_LEN         (16)
#define MODEM_CELLULAR_DATA_SN_LEN           (CONFIG_MODEM_CELLULAR_SERIAL_NUMBER_MAX_SIZE + 1)
#define MODEM_CELLULAR_DATA_MODEL_ID_LEN     (65)
#define MODEM_CELLULAR_DATA_IMSI_LEN         (23)
#define MODEM_CELLULAR_DATA_ICCID_LEN        (22)
#define MODEM_CELLULAR_DATA_MANUFACTURER_LEN (65)
#define MODEM_CELLULAR_DATA_FW_VERSION_LEN   (65)
#define MODEM_CELLULAR_DATA_APN_LEN          (32)
#define MODEM_CELLULAR_MAX_APN_CMDS          (2)
#define MODEM_CELLULAR_APN_BUF_SIZE          (64)

struct modem_cellular_event_cb {
	cellular_event_mask_t mask;
	cellular_event_cb_t fn;
	void *user_data;
};

struct modem_cellular_event_pkg {
	uint8_t event;
	const void *ptr;
};

/** @endcond */

/** @cond INTERNAL_HIDDEN */

struct modem_cellular_user_pipe {
	struct modem_cmux_dlci dlci;
	uint8_t dlci_address;
	uint8_t *dlci_receive_buf;
	uint16_t dlci_receive_buf_size;
	struct modem_pipe *pipe;
	struct modem_pipelink *pipelink;
};

/** @endcond */

/**
 * @brief Chat scripts for cellular modem.
 *
 * Only the init and dial scripts are mandatory, other scripts are optional.
 *
 * With the original driver, if the network script is provided, it is used to wait for
 * network registration before issuing the dial script. If it is not provided, the dial script
 * is expected to configure the network registration and the modem waits for registration after
 * the dial script completes.
 *
 * With @kconfig{CONFIG_MODEM_CELLULAR_V2}, each script runs in a fixed step of the boot or of
 * the LTE RF control, as documented on each member. The scripts run with the instance's
 * @c modem_cellular_data as user data, and must be defined with
 * modem_cellular_chat_callback_handler() as their callback, through which the driver gets
 * their results. The init, dial and LTE RF scripts are required, which modem_cellular_init()
 * asserts.
 * After the init script, every script but the shutdown script runs on the AT command channel,
 * CMUX DLCI 1, except for the dial script, which runs on the PPP channel, CMUX DLCI 2. The
 * driver tracks the registration from +CREG, +CGREG, +CEREG or +C5GREG reports and the data
 * context from +CGEV reports on the AT command channel. Modems may keep the settings for these
 * reports per CMUX channel, so once the DLCI setup script has run the driver enables them on
 * the AT command channel itself, with AT+CEREG=3 and AT+CGEREP=1, after enabling numeric
 * +CME ERROR codes with AT+CMEE=1, which it reads the SIM status from. Scripts are not
 * expected to enable them, but may enable +CREG, +CGREG or +C5GREG reports where the modem
 * needs them.
 */
struct modem_cellular_config_scripts {
	/**
	 * Script that initializes the modem and enables CMUX. Must not be NULL.
	 *
	 * With @kconfig{CONFIG_MODEM_CELLULAR_V2}, run on the UART once the modem is powered,
	 * and expected to end by switching the modem to CMUX, e.g. with AT+CMUX=0. It is expected
	 * to leave the LTE RF disabled with the SIM powered, e.g. with AT+CFUN=4. A failure
	 * recovers the modem.
	 */
	const struct modem_chat_script *init;
	/**
	 * Optional script that waits for network registration before dialing.
	 *
	 * Not used with @kconfig{CONFIG_MODEM_CELLULAR_V2}, which registers once the LTE RF is
	 * enabled.
	 */
	const struct modem_chat_script *network;
	/**
	 * Script that starts the PPP data connection. Must not be NULL.
	 *
	 * With @kconfig{CONFIG_MODEM_CELLULAR_V2}, run by the PPP device on the PPP channel once
	 * the modem is attached, and expected to complete on CONNECT. A failure is retried after
	 * @kconfig{CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS}.
	 */
	const struct modem_chat_script *dial;
	/**
	 * Optional script that periodically polls modem state while registered.
	 *
	 * With @kconfig{CONFIG_MODEM_CELLULAR_V2}, run every
	 * @kconfig{CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS} while the AT command channel is
	 * ready, whether the LTE RF is enabled or not, and never while an LTE RF script runs.
	 * Commands that need the LTE RF can be made conditional on
	 * modem_cellular_run_check_lte_rf_enabled(). Its success reports the modem as
	 * responsive, and @kconfig{CONFIG_MODEM_CELLULAR_MAX_SCRIPT_FAILURES} consecutive failures
	 * recover the modem.
	 */
	const struct modem_chat_script *periodic;
	/**
	 * Optional script that prepares the modem for power-off.
	 *
	 * With @kconfig{CONFIG_MODEM_CELLULAR_V2}, run on the UART once CMUX is disconnected, or
	 * on the AT command channel if CMUX does not disconnect, and expected to complete once
	 * the modem is powering down. A failure falls back to the power key, if there is one.
	 */
	const struct modem_chat_script *shutdown;
	/**
	 * Optional script for configuring DLCI channels after opening.
	 *
	 * The original driver runs it on both the AT command channel and the PPP channel. With
	 * @kconfig{CONFIG_MODEM_CELLULAR_V2}, run on the AT command channel only, once it is
	 * open, before the driver enables the error codes and reports it relies on and checks
	 * the SIM. A failure
	 * is logged, and the modem brought up regardless.
	 */
	const struct modem_chat_script *dlci_setup;
#if defined(CONFIG_MODEM_CELLULAR_V2) || defined(__DOXYGEN__)
	/**
	 * Required script that enables the LTE RF, e.g. with AT+CFUN=1. Only available with
	 * @kconfig{CONFIG_MODEM_CELLULAR_V2}.
	 *
	 * Run on the AT command channel while the LTE RF is requested, after the data context is
	 * configured with the APN. The registration reports are already enabled by the driver.
	 * A failure is retried after
	 * @kconfig{CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS} while still requested.
	 */
	const struct modem_chat_script *lte_rf_enable;
	/**
	 * Required script that disables the LTE RF, leaving the SIM powered, e.g. with
	 * AT+CFUN=4. Only available with @kconfig{CONFIG_MODEM_CELLULAR_V2}.
	 *
	 * Run on the AT command channel once the last LTE RF request is released, and before
	 * the modem is powered down while the LTE RF is enabled. A failure is retried after
	 * @kconfig{CONFIG_MODEM_CELLULAR_LTE_RF_RETRY_MS} while still not requested, or
	 * ignored by a power down.
	 */
	const struct modem_chat_script *lte_rf_disable;
	/**
	 * Optional script run once AT+CPIN? reports the SIM ready after the modem starts, e.g. to
	 * read the IMSI and ICCID. Only available with @kconfig{CONFIG_MODEM_CELLULAR_V2}.
	 *
	 * Run on the AT command channel before any board-specific init script and before the
	 * LTE RF is first enabled, so that e.g. the APN can be selected from the IMSI. A failure
	 * is logged, and the modem brought up regardless.
	 */
	const struct modem_chat_script *sim_ready;
#endif
};

/**
 * @brief Vendor-specific common cellular modem configuration.
 *
 * Define one constant object for each modem variant. The referenced scripts and unsolicited-match
 * array must remain valid for the lifetime of every device instance that uses this configuration.
 *
 * This structure is for configuration that is common between all driver instances.
 */
struct modem_cellular_vendor_config_common {
	/** Chat scripts implementing the modem lifecycle. */
	struct modem_cellular_config_scripts scripts;
	/** Unsolicited modem response matches. */
	struct {
		/** Array of unsolicited response matches, or NULL when @c size is zero. */
		const struct modem_chat_match *matches;
		/** Number of elements in @c matches. */
		uint16_t size;
	} unsol_matches;
	/**
	 * Command delimiter used by the modem, as a NULL-terminated string.
	 *
	 * Must not be NULL and must remain valid for the lifetime of the device.
	 */
	const char *chat_delimiter;
	/**
	 * Characters filtered from modem responses, as a NULL-terminated string.
	 *
	 * May be NULL to disable filtering. A non-NULL string must remain valid for the lifetime of
	 * the device.
	 */
	const char *chat_filter;
	/** Time modem takes to change baudrate, in milliseconds */
	uint16_t baudrate_update_delay_ms;
	/** Delay between de-asserting the reset pin and pulsing the power pin, in milliseconds */
	uint16_t reset_power_on_delay_ms;
	/** Duration of the modem power-key pulse, in milliseconds. */
	uint16_t power_pulse_duration_ms;
	/** Duration of the modem reset pulse, in milliseconds. */
	uint16_t reset_pulse_duration_ms;
	/** Timeout for the modem to revert from CMUX to AT mode after a CMUX disconnect, in
	 * milliseconds. Defaults to @c reset_pulse_duration_ms if not specified (value == 0U) for
	 * legacy compatibility.
	 */
	uint16_t cmux_disconnect_timeout_ms;
	/** Maximum modem startup delay, in milliseconds. */
	uint16_t startup_time_ms;
	/** Maximum modem shutdown delay, in milliseconds. */
	uint16_t shutdown_time_ms;
	/** Force autostart regardless of the devicetree @c autostarts property. */
	bool force_autostart;
};

/**
 * @brief Vendor-inst-specific cellular modem configuration.
 *
 * Define one constant object for each modem instance. The referenced scripts must remain valid for
 * the lifetime of every device instance that uses this configuration.
 */
struct modem_cellular_vendor_config_inst {
	/**
	 * Optional script that configures the modem's UART baud rate.
	 *
	 * With @kconfig{CONFIG_MODEM_CELLULAR_V2}, run on the UART at the baud rate the modem
	 * powers up at, before the init script, which then runs at @a target_baudrate. A failure
	 * is taken to mean the modem already runs at the new baud rate.
	 */
	const struct modem_chat_script *set_baudrate;
	/** Integer baudrate associated with @a set_baudrate */
	uint32_t target_baudrate;
};

/**
 * @brief Modem configuration that varies per vendor and instance
 */
struct modem_cellular_vendor_config {
	/** Common configuration shared between all vendor instances */
	const struct modem_cellular_vendor_config_common *common;
	/** Optional configuration that varies per instance */
	const struct modem_cellular_vendor_config_inst *instance;
	/** Vendor specific instance configuration */
	const void *vendor_inst;
};

/** @cond INTERNAL_HIDDEN */

/* Initialise the members of struct modem_cellular_config shared by both implementations */
#define Z_MODEM_CELLULAR_CONFIG_INIT(inst, vendor_config)                                          \
	.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                                  \
	.vendor = vendor_config,                                                                   \
	.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                               \
	.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                         \
	.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                         \
	.wake_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_wake_gpios, {}),                           \
	.ring_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_ring_gpios, {}),                           \
	.dtr_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_dtr_gpios, {}),                             \
	.status_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_status_gpios, {}),                       \
	.autostarts = DT_INST_PROP(inst, autostarts),                                              \
	.hold_reset_on_suspend =                                                                   \
		DT_INST_ENUM_HAS_VALUE(inst, zephyr_mdm_reset_behavior, hold_on_suspend),          \
	.reset_on_resume =                                                                         \
		DT_INST_ENUM_HAS_VALUE(inst, zephyr_mdm_reset_behavior, toggle_on_resume),         \
	.reset_on_recovery =                                                                       \
		DT_INST_ENUM_HAS_VALUE(inst, zephyr_mdm_reset_behavior, toggle_on_recovery),       \
	.cmux_enable_runtime_power_save =                                                          \
		DT_INST_PROP_OR(inst, cmux_enable_runtime_power_save, 0),                          \
	.cmux_close_pipe_on_power_save = DT_INST_PROP_OR(inst, cmux_close_pipe_on_power_save, 0), \
	.cmux_no_powersave_handshake = DT_INST_PROP_OR(inst, cmux_no_powersave_handshake, 0),     \
	.use_default_pdp_context = DT_INST_PROP_OR(inst, zephyr_use_default_pdp_ctx, 0),           \
	.use_default_apn = DT_INST_PROP_OR(inst, zephyr_use_default_apn, 0),                       \
	.cmux_idle_timeout = K_MSEC(DT_INST_PROP_OR(inst, cmux_idle_timeout_ms, 0)),               \
	.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                         \
	.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),

#define Z_MODEM_CELLULAR_DEVICE_DEFINE(inst)                                                       \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL,                \
			      CONFIG_MODEM_CELLULAR_INIT_PRIORITY, &modem_cellular_api)

/** @endcond */

/* The state machine implementation, with its runtime data and configuration */
#if defined(CONFIG_MODEM_CELLULAR_V2)
#include <zephyr/drivers/modem/modem_cellular_v2.h>
#else
#include <zephyr/drivers/modem/modem_cellular_v1.h>
#endif

/** @cond INTERNAL_HIDDEN */

/** @brief Initialize the cellular modem
 * Called from the DEVICE_DT_INST_DEFINE macro for cellular modem drivers.
 */
int modem_cellular_init(const struct device *dev);

#if defined(CONFIG_ZTEST) && !defined(CONFIG_MODEM_BACKEND_UART)
/**
 * @brief Halt a cellular modem instance between tests.
 *
 * Stops the instance where it is, without running the shutdown sequence, so that a
 * following modem_cellular_init() boots it as if from reset. On return:
 *
 * - No timer, work item or event of the instance is pending or running.
 * - The chat, CMUX and PPP instances and every UART, DLCI and user pipe are released.
 * - The net_mgmt and ring GPIO callbacks registered by modem_cellular_init() are removed.
 * - State that would otherwise outlive a power cycle is cleared: the registration status,
 *   APN, recovery and script failure counters, original baudrate and periodic script pause.
 * - The PPP network interface is dormant, as after its initialisation.
 *
 * The callback set with cellular_set_callback() is kept. GPIOs are not driven and the UART
 * configuration is left untouched, so the next boot treats the baudrate it finds as the one
 * the modem powers up at.
 *
 * Calling it on an instance that is already halted has no effect. The caller ensures no
 * other thread calls into the instance while it runs.
 *
 * Only available with substitute pipe backends. The pipes are released, not closed, which
 * would leave a real UART backend receiving into the reinitialized instance.
 *
 * @param dev Cellular modem instance.
 */
void modem_cellular_ztest_halt(const struct device *dev);
#endif

int modem_cellular_pm_action(const struct device *dev, enum pm_device_action action);

extern const struct cellular_driver_api modem_cellular_api;

void modem_cellular_emit_event(struct modem_cellular_data *data, enum cellular_event evt,
			       const void *payload);

/*
 * Store the latest serving-cell status and emit CELLULAR_EVENT_NETWORK_STATUS_CHANGED
 * only when it changed, ignoring signal quality (rsrp/rsrq) so periodic polls do not
 * re-fire the event on signal fluctuation.
 */
void modem_cellular_emit_network_status(struct modem_cellular_data *data,
					const struct cellular_evt_network_status *status);

void modem_cellular_chat_on_imei(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_cgsn_sn(struct modem_chat *chat, char **argv, uint16_t argc,
				    void *user_data);
void modem_cellular_chat_on_cgsn_imei(struct modem_chat *chat, char **argv, uint16_t argc,
				      void *user_data);
void modem_cellular_chat_on_cgmm(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_csq(struct modem_chat *chat, char **argv, uint16_t argc,
				void *user_data);
void modem_cellular_chat_on_cesq(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_iccid(struct modem_chat *chat, char **argv, uint16_t argc,
				  void *user_data);
void modem_cellular_chat_on_imsi(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_cgmi(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_cgmr(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_cxreg(struct modem_chat *chat, char **argv, uint16_t argc,
				  void *user_data);
void modem_cellular_chat_on_cgev(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data);
void modem_cellular_chat_on_modem_ready(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data);

/** @endcond */

/**
 * @brief Handle completion of a modem chat script.
 *
 * Use this as the callback for lifecycle scripts executed by the cellular modem state machine. The
 * callback translates the chat result into the corresponding internal state-machine event.
 *
 * @param chat Chat instance that completed the script. Must not be NULL.
 * @param result Script completion result.
 * @param info Extra script completion information. Must not be NULL.
 * @param user_data Pointer to the associated @ref modem_cellular_data object. Must not be NULL.
 */
void modem_cellular_chat_callback_handler(struct modem_chat *chat,
					  enum modem_chat_script_result result,
					  const struct modem_chat_script_completion_info *info,
					  void *user_data);

/**
 * @brief Run check for script commands that need the LTE RF enabled.
 *
 * Pass it as the @c _run_check of @ref MODEM_CHAT_SCRIPT_CMD_RESP_COND and its variants in the
 * scripts of a @ref modem_cellular_vendor_config, which requires
 * @kconfig{CONFIG_MODEM_CHAT_COMMANDS_CONDITIONAL}. With @kconfig{CONFIG_MODEM_CELLULAR_V2}
 * the command runs while the driver has the LTE RF enabled through the @c lte_rf_enable
 * script. The original driver leaves the LTE RF to the vendor scripts, so the command always
 * runs.
 *
 * Changes to the LTE RF made outside of the driver, such as through a user pipe, are not
 * seen.
 *
 * @param user_data User data the driver runs its scripts with.
 *
 * @retval true LTE RF enabled, the command is run.
 * @retval false LTE RF disabled, the command is skipped.
 */
bool modem_cellular_run_check_lte_rf_enabled(void *user_data);

/**
 * @brief Run check for script commands that need the LTE RF disabled.
 *
 * The inverse of modem_cellular_run_check_lte_rf_enabled(), e.g. for a periodic script that
 * only checks the modem responds while the LTE RF is disabled. With the original driver the
 * command never runs.
 *
 * @param user_data User data the driver runs its scripts with.
 *
 * @retval true LTE RF disabled, the command is run.
 * @retval false LTE RF enabled, the command is skipped.
 */
bool modem_cellular_run_check_lte_rf_disabled(void *user_data);

/**
 * @brief Run check for script commands that need the modem registered.
 *
 * Pass it as the @c _run_check of @ref MODEM_CHAT_SCRIPT_CMD_RESP_COND and its variants in the
 * scripts of a @ref modem_cellular_vendor_config, which requires
 * @kconfig{CONFIG_MODEM_CHAT_COMMANDS_CONDITIONAL}. The command runs while the modem last
 * reported itself registered, at home or roaming, with any access technology.
 *
 * @param user_data User data the driver runs its scripts with.
 *
 * @retval true Registered, the command is run.
 * @retval false Not registered, the command is skipped.
 */
bool modem_cellular_run_check_registered(void *user_data);

/**
 * @defgroup modem_driver_macros Cellular modem driver definition macros
 * @brief Define device instances that use the cellular modem driver.
 * @ingroup modem_cellular_backend
 *
 * See the cellular modem documentation for a complete out-of-tree driver example.
 *
 * @{
 */

/**
 * @brief Generate a unique C identifier for a modem instance object.
 *
 * @param name Base object name.
 * @param inst Devicetree instance number.
 *
 * @return The generated C identifier.
 */
#define MODEM_CELLULAR_INST_NAME(name, inst) \
	CONCAT(name, _, DT_DRV_COMPAT, inst)

/** @cond INTERNAL_HIDDEN */

#define MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(inst, name, size)                                     \
	MODEM_PIPELINK_DT_INST_DEFINE(inst, name);                                                 \
	static uint8_t MODEM_CELLULAR_INST_NAME(name, inst)[size]                                  \

#define MODEM_CELLULAR_INIT_USER_PIPE(_inst, _name, _dlci_address)                                 \
	{                                                                                          \
		.dlci_address = _dlci_address,                                                     \
		.dlci_receive_buf = MODEM_CELLULAR_INST_NAME(_name, _inst),                        \
		.dlci_receive_buf_size = sizeof(MODEM_CELLULAR_INST_NAME(_name, _inst)),           \
		.pipelink = MODEM_PIPELINK_DT_INST_GET(_inst, _name),                              \
	}

#define MODEM_CELLULAR_DEFINE_USER_PIPES(inst, ...)                                                \
	static struct modem_cellular_user_pipe MODEM_CELLULAR_INST_NAME(user_pipes, inst)[] = {    \
		__VA_ARGS__                                                                        \
	}

#define MODEM_CELLULAR_GET_USER_PIPES(inst) \
	MODEM_CELLULAR_INST_NAME(user_pipes, inst)

/* Extract the first argument (pipe name) from a pair */
#define MODEM_CELLULAR_GET_PIPE_NAME_ARG(arg1, ...) arg1

/* Extract the second argument (DLCI address) from a pair */
#define MODEM_CELLULAR_GET_DLCI_ADDRESS_ARG(arg1, arg2, ...) arg2

/* Define user pipe data using instance and extracted pipe name */
#define MODEM_CELLULAR_DEFINE_USER_PIPE_DATA_HELPER(_args, inst)                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(inst,                                                 \
					     MODEM_CELLULAR_GET_PIPE_NAME_ARG _args,               \
					     CONFIG_MODEM_CELLULAR_USER_PIPE_BUFFER_SIZES)

/* Initialize user pipe using instance, extracted pipe name, and DLCI address */
#define MODEM_CELLULAR_INIT_USER_PIPE_HELPER(_args, inst)                                          \
	MODEM_CELLULAR_INIT_USER_PIPE(inst,                                                        \
				      MODEM_CELLULAR_GET_PIPE_NAME_ARG _args,                      \
				      MODEM_CELLULAR_GET_DLCI_ADDRESS_ARG _args)

/** @endcond */

/**
 * @brief Define additional CMUX user pipes for a modem instance.
 *
 * Each variadic argument is a parenthesized @c (name, dlci_address) pair. The macro defines the
 * receive buffers, pipe links, and user-pipe array consumed by @ref MODEM_CELLULAR_DEFINE_INSTANCE.
 * Invoke it once for each modem instance before invoking @ref MODEM_CELLULAR_DEFINE_INSTANCE.
 *
 * @param inst Devicetree instance number.
 * @param ... One or more <tt>(name, dlci_address)</tt> pairs.
 */
#define MODEM_CELLULAR_DEFINE_AND_INIT_USER_PIPES(inst, ...)                                       \
	FOR_EACH_FIXED_ARG(MODEM_CELLULAR_DEFINE_USER_PIPE_DATA_HELPER,                            \
			   (;), inst, __VA_ARGS__);                                                \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		FOR_EACH_FIXED_ARG(MODEM_CELLULAR_INIT_USER_PIPE_HELPER,                           \
				   (,), inst, __VA_ARGS__)                                         \
	);

/**
 * @brief Define a multiple match object that expects a single response followed by an "OK"
 *
 * Replaces a MODEM_CHAT_MATCH_DEFINE followed by MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match)
 *
 * @param _sym Name of the matches array object
 * @param _match Match string per @ref MODEM_CHAT_MATCH_DEFINE
 * @param _separators Separators string per @ref MODEM_CHAT_MATCH_DEFINE
 * @param _callback Callback on match per @ref MODEM_CHAT_MATCH_DEFINE
 */
#define MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(_sym, _match, _separators, _callback)                  \
	MODEM_CHAT_MATCHES_DEFINE(_sym,                                                            \
		MODEM_CHAT_MATCH_INITIALIZER(_match, _separators, _callback, false, true),         \
		MODEM_CHAT_MATCH("OK", "", NULL))

/**
 * @brief Construct AT+IPR command from devicetree node
 *
 * @param inst Devicetree instance number.
 */
#define _MODEM_CELLULAR_IPR_CMD(inst) "AT+IPR=" STRINGIFY(DT_INST_PROP_OR(inst, target_baudrate, 0))

/* Expands the script symbol before MODEM_CHAT_SCRIPT_DEFINE() stringifies it as the name */
#define _MODEM_CELLULAR_CHAT_SCRIPT_DEFINE(...) MODEM_CHAT_SCRIPT_DEFINE(__VA_ARGS__)

/**
 * @brief Define the standard script for updating the modem baudrate
 *
 * The script is not required to be used if defined.
 *
 * @param inst Devicetree instance number.
 */
#define MODEM_CELLULAR_COMMON_BAUDRATE_CHAT_SCRIPT(inst)                                           \
	MODEM_CHAT_SCRIPT_CMDS_DEFINE(                                                             \
		MODEM_CELLULAR_INST_NAME(baudrate_cmds, inst),                                     \
		MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),                                      \
		MODEM_CHAT_SCRIPT_CMD_RESP(_MODEM_CELLULAR_IPR_CMD(inst), ok_match));              \
                                                                                                   \
	/* Not every instance uses the script, mark it unused outside the stringified name */      \
	static const struct modem_chat_script __maybe_unused                                       \
		MODEM_CELLULAR_INST_NAME(baudrate_script, inst);                                   \
	_MODEM_CELLULAR_CHAT_SCRIPT_DEFINE(MODEM_CELLULAR_INST_NAME(baudrate_script, inst),        \
					   MODEM_CELLULAR_INST_NAME(baudrate_cmds, inst),          \
					   abort_matches, modem_cellular_chat_callback_handler, 1);

/**
 * @brief Helper macro for populating @ref modem_cellular_vendor_config_inst baudrate fields
 *
 * @param inst Devicetree instance number.
 */
#define MODEM_CELLULAR_VENDOR_CONFIG_INST_TARGET_BAUDRATE(inst)                                    \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, target_baudrate),                                  \
		(.set_baudrate = &MODEM_CELLULAR_INST_NAME(baudrate_script, inst),                 \
		.target_baudrate = DT_INST_PROP(inst, target_baudrate)),                           \
		(.set_baudrate = NULL))

/**
 * @brief Define the standard scripts for controlling the LTE RF
 *
 * Defines a script that enables the LTE RF with AT+CFUN=1, and one that disables it with
 * AT+CFUN=4, leaving the SIM powered. Point the @c lte_rf_enable and @c lte_rf_disable members
 * of @ref modem_cellular_config_scripts at them with @ref MODEM_CELLULAR_COMMON_LTE_RF_SCRIPTS.
 * Requires the matches defined by @ref MODEM_CELLULAR_COMMON_CHAT_MATCHES.
 *
 * @param timeout_s Timeout of each script, in seconds.
 */
#define MODEM_CELLULAR_COMMON_LTE_RF_CHAT_SCRIPTS(timeout_s)                                       \
	MODEM_CHAT_SCRIPT_CMDS_DEFINE(modem_cellular_lte_rf_enable_cmds,                           \
				      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match));          \
	MODEM_CHAT_SCRIPT_DEFINE(modem_cellular_lte_rf_enable_script,                              \
				 modem_cellular_lte_rf_enable_cmds, abort_matches,                 \
				 modem_cellular_chat_callback_handler, timeout_s);                 \
	MODEM_CHAT_SCRIPT_CMDS_DEFINE(modem_cellular_lte_rf_disable_cmds,                          \
				      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match));          \
	MODEM_CHAT_SCRIPT_DEFINE(modem_cellular_lte_rf_disable_script,                             \
				 modem_cellular_lte_rf_disable_cmds, abort_matches,                \
				 modem_cellular_chat_callback_handler, timeout_s)

/**
 * @brief Helper macro for populating the LTE RF members of @ref modem_cellular_config_scripts
 *
 * Points them at the scripts defined by @ref MODEM_CELLULAR_COMMON_LTE_RF_CHAT_SCRIPTS.
 */
#define MODEM_CELLULAR_COMMON_LTE_RF_SCRIPTS                                                       \
	.lte_rf_enable = &modem_cellular_lte_rf_enable_script,                                     \
	.lte_rf_disable = &modem_cellular_lte_rf_disable_script

/**
 * @brief Define common chat matches used by cellular modem scripts.
 *
 * Invoke this macro once at file scope. It defines matches for successful commands, common command
 * failures, modem identity and signal queries, and PPP dial responses. The generated identifiers
 * are intended for use in the modem's @ref modem_cellular_config_scripts.
 */
#define MODEM_CELLULAR_COMMON_CHAT_MATCHES()							   \
	MODEM_CHAT_MATCH_DEFINE(ok_match, "OK", "", NULL);					   \
	MODEM_CHAT_MATCHES_DEFINE(__maybe_unused allow_match,					   \
				  MODEM_CHAT_MATCH("OK", "", NULL),				   \
				  MODEM_CHAT_MATCH("ERROR", "", NULL));				   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused imei_match,				   \
				"", "", modem_cellular_chat_on_imei);				   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cgsn_sn_match,			   \
				"", "", modem_cellular_chat_on_cgsn_sn);			   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cgsn_imei_match,			   \
				"+CGSN: ", "", modem_cellular_chat_on_cgsn_imei);		   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cgmm_match,				   \
				"", "", modem_cellular_chat_on_cgmm);				   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused csq_match,				   \
				"+CSQ: ", ",", modem_cellular_chat_on_csq);			   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cesq_match,				   \
				"+CESQ: ", ",", modem_cellular_chat_on_cesq);			   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused qccid_match,				   \
				"+QCCID: ", "", modem_cellular_chat_on_iccid);			   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused iccid_match,				   \
				"+ICCID: ", "", modem_cellular_chat_on_iccid);			   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused ccid_match,				   \
				"+CCID: ", "", modem_cellular_chat_on_iccid);			   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cimi_match,				   \
				"", "", modem_cellular_chat_on_imsi);				   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cgmi_match,				   \
				"", "", modem_cellular_chat_on_cgmi);				   \
	MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(__maybe_unused cgmr_match,				   \
				"", "", modem_cellular_chat_on_cgmr);				   \
	MODEM_CHAT_MATCH_DEFINE(connect_match __maybe_unused,					   \
				"CONNECT", "", NULL);						   \
	MODEM_CHAT_MATCHES_DEFINE(__maybe_unused abort_matches,					   \
				  MODEM_CHAT_MATCH("ERROR", "", NULL));				   \
	MODEM_CHAT_MATCHES_DEFINE(__maybe_unused dial_abort_matches,				   \
				  MODEM_CHAT_MATCH("ERROR", "", NULL),				   \
				  MODEM_CHAT_MATCH("BUSY", "", NULL),				   \
				  MODEM_CHAT_MATCH("NO ANSWER", "", NULL),			   \
				  MODEM_CHAT_MATCH("NO CARRIER", "", NULL),			   \
				  MODEM_CHAT_MATCH("NO DIALTONE", "", NULL))

/**
 * @brief Expand to the common unsolicited response matches.
 *
 * Use this macro in the entry list passed to MODEM_CHAT_MATCHES_DEFINE().
 * Vendor drivers may add additional entries after this expansion.
 */
#define MODEM_CELLULAR_COMMON_UNSOL_MATCHES							   \
	MODEM_CHAT_MATCH("+CREG: ", ",", modem_cellular_chat_on_cxreg),				   \
	MODEM_CHAT_MATCH("+CEREG: ", ",", modem_cellular_chat_on_cxreg),			   \
	MODEM_CHAT_MATCH("+CGREG: ", ",", modem_cellular_chat_on_cxreg),			   \
	MODEM_CHAT_MATCH("+CGEV: ", ",", modem_cellular_chat_on_cgev),				   \
	MODEM_CHAT_MATCH("APP RDY", "", modem_cellular_chat_on_modem_ready),			   \
	MODEM_CHAT_MATCH("Ready", "", modem_cellular_chat_on_modem_ready)

/**
 * @brief Define the PPP network interface of a cellular modem instance.
 *
 * Defines the @c modem_ppp object used by @ref MODEM_CELLULAR_DEFINE_INSTANCE. With
 * @kconfig{CONFIG_MODEM_CELLULAR_V2} it also defines the instance's PPP device, which the
 * network interface takes a runtime PM reference on while it is up. Invoke it before
 * @ref MODEM_CELLULAR_DEFINE_INSTANCE.
 *
 * @param inst Devicetree instance number.
 * @param _init_iface Hook for the PPP L2 network interface init function, or NULL.
 * @param _mtu Max size of net_pkt data sent and received on the PPP L2 network interface.
 * @param _buf_size Size of the partial PPP frame transmit and receive buffers.
 */
#define MODEM_CELLULAR_PPP_DEFINE(inst, _init_iface, _mtu, _buf_size)                              \
	Z_MODEM_CELLULAR_PPP_DEFINE(inst, _init_iface, _mtu, _buf_size)

/**
 * @brief Define a cellular modem device instance.
 *
 * This macro creates the immutable device configuration, power-management object, and Zephyr device
 * for a devicetree instance. Before invoking it, define the instance's PPP object, initialize a
 * @ref modem_cellular_data object, and invoke @ref MODEM_CELLULAR_DEFINE_AND_INIT_USER_PIPES().
 *
 * @param inst Devicetree instance number.
 * @param vendor_config Pointer to a constant @ref modem_cellular_vendor_config object. Must not be
 *        NULL and must remain valid for the lifetime of the device.
 */
#define MODEM_CELLULAR_DEFINE_INSTANCE(inst, vendor_config)                                        \
	BUILD_ASSERT(vendor_config != NULL, "vendor_config must be non-NULL");                     \
	Z_MODEM_CELLULAR_DEFINE_INSTANCE(inst, vendor_config)

/**
 * @brief Descriptor binding a board-supplied init script to a modem instance.
 *
 * Registered with MODEM_CELLULAR_BOARD_INIT_DEFINE.
 */
struct modem_cellular_board_init {
	const struct device *dev;
	const struct modem_chat_script *script;
};

/**
 * @brief Register a board-specific init script for a modem instance.
 *
 * The script runs over the AT control channel after CMUX is established and
 * before APN and network configuration. The driver supplies the completion
 * callback that advances the connect sequence, so the script's own callback
 * is unused.
 *
 * @param node_id Devicetree node identifier of the modem instance.
 * @param _script Pointer to a modem_chat_script defined with
 *                MODEM_CHAT_SCRIPT_DEFINE.
 */
#if defined(CONFIG_MODEM_CELLULAR)
#define MODEM_CELLULAR_BOARD_INIT_DEFINE(node_id, _script)                                         \
	static const STRUCT_SECTION_ITERABLE(                                                      \
		modem_cellular_board_init,                                                         \
		CONCAT(modem_cellular_board_init_, DT_DEP_ORD(node_id))) = {                       \
		.dev = DEVICE_DT_GET(node_id),                                                     \
		.script = (_script),                                                               \
	}
#else
#define MODEM_CELLULAR_BOARD_INIT_DEFINE(node_id, _script)
#endif

/** @} */

/** @} */

/**
 * @addtogroup cellular_interface
 * @{
 */

/**
 * @brief Pause the cellular_modem driver's periodic chat script.
 *
 * Scheduled periodic-script runs are suppressed until
 * cellular_modem_resume_periodic_script() is called. An in-flight script
 * invocation at the time of this call is allowed to complete; suppression
 * takes effect from the next scheduled run.
 *
 * @param dev Cellular device created with @ref MODEM_CELLULAR_DEFINE_INSTANCE(). Must not be NULL.
 *
 * @retval 0 Success.
 * @retval -ENOTSUP Device has no periodic chat script configured.
 * @retval -EINVAL Periodic script is already paused.
 *
 * @see cellular_modem_resume_periodic_script
 */
int cellular_modem_pause_periodic_script(const struct device *dev);

/**
 * @brief Resume the cellular_modem driver's periodic chat script.
 *
 * Re-enables the periodic script. If at least one scheduled run was
 * skipped while paused, the script fires immediately; otherwise the
 * periodic timer restarts at the configured interval.
 *
 * @param dev Cellular device created with @ref MODEM_CELLULAR_DEFINE_INSTANCE(). Must not be NULL.
 *
 * @retval 0 Success.
 * @retval -ENOTSUP Device has no periodic chat script configured.
 * @retval -EINVAL Periodic script is not currently paused.
 *
 * @see cellular_modem_pause_periodic_script
 */
int cellular_modem_resume_periodic_script(const struct device *dev);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CELLULAR_INTERNAL_H_ */
