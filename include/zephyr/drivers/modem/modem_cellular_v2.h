/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file modem_cellular_v2.h
 * @brief State Machine Framework based cellular modem driver implementation.
 *
 * Included by modem_cellular.h when @kconfig{CONFIG_MODEM_CELLULAR_V2} is enabled.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_V2_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_V2_H_

#ifndef ZEPHYR_INCLUDE_DRIVERS_CELLULAR_INTERNAL_H_
#error "Include <zephyr/drivers/modem/modem_cellular.h> instead"
#endif

#include <zephyr/net/net_mgmt.h>
#include <zephyr/smf.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup modem_cellular_backend
 * @{
 */

/** @cond INTERNAL_HIDDEN */

/* Runtime data of an instance, see the documentation of the original implementation */
struct modem_cellular_data {
	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[CONFIG_MODEM_CELLULAR_UART_BUFFER_SIZES];
	uint8_t uart_backend_transmit_buf[CONFIG_MODEM_CELLULAR_UART_BUFFER_SIZES];
	uint32_t original_baudrate;

	/* CMUX, with the AT command channel on DLCI 1 and the PPP channel on DLCI 2 */
	struct modem_cmux cmux;
	uint8_t cmux_receive_buf[MODEM_CMUX_WORK_BUFFER_SIZE];
	uint8_t cmux_transmit_buf[MODEM_CMUX_WORK_BUFFER_SIZE];
	struct modem_cmux_dlci dlci1;
	struct modem_cmux_dlci dlci2;
	struct modem_pipe *dlci1_pipe;
	struct modem_pipe *dlci2_pipe;
	/* Pipe the shutdown script runs on, NULL while there is none */
	struct modem_pipe *cmd_pipe;
	uint8_t dlci1_receive_buf[MODEM_CMUX_WORK_BUFFER_SIZE];
	uint8_t dlci2_receive_buf[MODEM_CMUX_WORK_BUFFER_SIZE];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CONFIG_MODEM_CELLULAR_CHAT_BUFFER_SIZE];
	uint8_t *chat_argv[32];
	uint8_t recovery_count;
	struct modem_chat_script board_init_script;

	/* Status */
	enum cellular_registration_status registration_status_gsm;
	enum cellular_registration_status registration_status_gprs;
	enum cellular_registration_status registration_status_lte;
	enum cellular_registration_status registration_status_5g;
	/* The data context is active per +CGEV, which may stand in for registration */
	bool pdn_active;
	enum cellular_access_technology access_tech;
	uint8_t rssi;
	uint8_t rsrp;
	uint8_t rsrq;
	struct cellular_evt_network_status network_status;
	bool network_status_valid;
	uint8_t imei[MODEM_CELLULAR_DATA_IMEI_LEN];
	uint8_t sn[MODEM_CELLULAR_DATA_SN_LEN];
	uint8_t model_id[MODEM_CELLULAR_DATA_MODEL_ID_LEN];
	uint8_t imsi[MODEM_CELLULAR_DATA_IMSI_LEN];
	uint8_t iccid[MODEM_CELLULAR_DATA_ICCID_LEN];
	uint8_t manufacturer[MODEM_CELLULAR_DATA_MANUFACTURER_LEN];
	uint8_t fw_version[MODEM_CELLULAR_DATA_FW_VERSION_LEN];

	/* State machine, and the event it is running for */
	struct smf_ctx smf;
	uint8_t smf_event;
	/* Chat script the current state started, NULL until it starts one */
	const struct modem_chat_script *script_running;
	const struct device *dev;
	struct k_work_delayable timeout_work;
	/* Moved on by every start of the timer and every transition, so that a timeout that
	 * expired before either can be told apart from the current one
	 */
	uint32_t timer_gen;

	/* Number of LTE RF requests held */
	atomic_t lte_rf_refs;

	/* Periodic script, run while ready for commands */
	struct k_work_delayable periodic_work;
	uint8_t script_failure_counter;
	atomic_t periodic_paused;
	/* Set when a run was skipped while paused, which then runs on resume */
	bool periodic_skipped;
	/* Set when a run became due while an LTE RF script ran, which then runs after it */
	bool periodic_pending;
	/* Whether the LTE RF was last left enabled */
	bool lte_rf_enabled;

	/* SIM status from the last AT+CPIN? query, when it gave one */
	enum cellular_sim_status sim_status;
	bool sim_status_known;
	/* When the SIM wait gives up */
	k_timepoint_t sim_deadline;

	/* Data context APN, configured before each LTE RF enable */
	uint8_t apn[MODEM_CELLULAR_DATA_APN_LEN];
	/* An APN has been set, which may be empty to use the default APN of the subscription */
	bool apn_set;
	struct modem_chat_script_chat apn_chats[MODEM_CELLULAR_MAX_APN_CMDS];
	struct modem_chat_script apn_script;
	char apn_buf[MODEM_CELLULAR_MAX_APN_CMDS][MODEM_CELLULAR_APN_BUF_SIZE];
	/* Set when the APN changes, until the data context is configured with it */
	atomic_t apn_changed;

	/* Power management */
	struct k_sem suspended_sem;

	/* Event dispatcher, for the events in the queue and the level signals pending */
	struct k_work event_dispatch_work;
	struct k_msgq event_queue;
	struct modem_cellular_event_pkg event_buf[8];
	atomic_t signals;
	/* Whether the device is to be resumed, as last requested */
	atomic_t pm_resume;

	struct k_mutex api_lock;
	struct modem_cellular_event_cb cb;
	/* Callback of the PPP device, alongside the one set with cellular_set_callback() */
	struct modem_cellular_event_cb ppp_cb;

	/* Set when the power-on pulse was skipped because the status GPIO reported the modem
	 * already powered, the init script then confirms it responds.
	 */
	bool power_on_skipped;
	/* Set when suspension during the init script leaves the powered modem at the new
	 * baudrate, which a resume must retain until the modem is power-cycled.
	 */
	bool baudrate_kept;
	/* Set when a suspend waits for the current step to settle */
	bool suspend_pending;
	/* Set when the CMUX connect attempt has failed, a suspend is then acted on at once */
	bool cmux_connect_failed;
	/* Set once the UART has opened for the CMUX connect attempt */
	bool cmux_bus_opened;
	/* Set while recovery turns a running modem off with the power key, before turning it
	 * back on.
	 */
	bool power_cycle;
	/* Reason reported with CELLULAR_EVENT_MODEM_FAILED, 0 until the modem fails and again
	 * once it is resumed.
	 */
	int failure_reason;
};

struct modem_cellular_config {
	const struct device *uart;
	const struct modem_cellular_vendor_config *vendor;
	struct modem_ppp *ppp;
	struct gpio_dt_spec power_gpio;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec wake_gpio;
	struct gpio_dt_spec ring_gpio;
	struct gpio_dt_spec dtr_gpio;
	struct gpio_dt_spec status_gpio;
	bool autostarts;
	bool hold_reset_on_suspend;
	bool reset_on_resume;
	bool reset_on_recovery;
	bool cmux_enable_runtime_power_save;
	bool cmux_close_pipe_on_power_save;
	bool cmux_no_powersave_handshake;
	bool use_default_pdp_context;
	bool use_default_apn;
	k_timeout_t cmux_idle_timeout;
	struct modem_cellular_user_pipe *user_pipes;
	uint8_t user_pipes_size;
	struct modem_pipelink *ppp_pipelink;
};

struct modem_cellular_ppp_config {
	/* Cellular modem whose PPP channel the device dials */
	const struct device *core;
	struct modem_ppp *ppp;
};

struct modem_cellular_ppp_data {
	const struct device *dev;
	struct smf_ctx smf;
	uint8_t smf_event;

	/* Event dispatcher, for the events in the queue and the level signals pending */
	struct k_work event_dispatch_work;
	struct k_msgq event_queue;
	struct modem_cellular_event_pkg event_buf[8];
	atomic_t signals;
	/* Whether the device is to be resumed, as last requested */
	atomic_t pm_resume;
	struct k_work_delayable timeout_work;
	/* Moved on by every start and stop of the timer, so that a timeout that expired before
	 * the timer was stopped or restarted can be told apart from the current one
	 */
	uint32_t timer_gen;
	struct k_sem suspended_sem;
	/* Set from closing the PPP channel until the close completes */
	bool pipe_closing;
	/* Set when the data context was lost while the dial script ran */
	bool dial_call_ended;
	/* Set once the hangup interval has passed */
	bool hangup_elapsed;
	/* Reports the PPP link dead, e.g. on NO CARRIER */
	struct net_mgmt_event_callback ppp_dead_cb;
	/* Reason the core gave for the modem failing */
	int failure_reason;

	/* Chat for the dial script, run on the PPP channel */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CONFIG_MODEM_CELLULAR_CHAT_BUFFER_SIZE];
	uint8_t *chat_argv[8];
	struct modem_chat_script dial_script;
	const struct modem_chat_script *script_running;
};

int modem_cellular_ppp_init(const struct device *dev);

int modem_cellular_ppp_pm_action(const struct device *dev, enum pm_device_action action);

#define MODEM_CELLULAR_PPP_DEV_ID(inst) CONCAT(modem_cellular_ppp_, DT_INST_DEP_ORD(inst))

/* Expands the arguments, which MODEM_DEV_PPP_DEFINE stringifies */
#define MODEM_CELLULAR_DEV_PPP_DEFINE(...) MODEM_DEV_PPP_DEFINE(__VA_ARGS__)

#define Z_MODEM_CELLULAR_PPP_DEFINE(inst, _init_iface, _mtu, _buf_size)                            \
	static struct modem_ppp MODEM_CELLULAR_INST_NAME(ppp, inst);                               \
	static struct modem_cellular_ppp_data MODEM_CELLULAR_INST_NAME(ppp_data, inst);            \
	static const struct modem_cellular_ppp_config MODEM_CELLULAR_INST_NAME(ppp_config,         \
									       inst) = {   \
		.core = DEVICE_DT_INST_GET(inst),                                                  \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
	PM_DEVICE_DEFINE(MODEM_CELLULAR_PPP_DEV_ID(inst), modem_cellular_ppp_pm_action);           \
	DEVICE_DEFINE(MODEM_CELLULAR_PPP_DEV_ID(inst),                                             \
		      MODEM_CELLULAR_PPP_DEVICE_NAME(DT_DRV_INST(inst)), modem_cellular_ppp_init,  \
		      PM_DEVICE_GET(MODEM_CELLULAR_PPP_DEV_ID(inst)),                              \
		      &MODEM_CELLULAR_INST_NAME(ppp_data, inst),                                   \
		      &MODEM_CELLULAR_INST_NAME(ppp_config, inst), POST_KERNEL,                    \
		      CONFIG_MODEM_CELLULAR_INIT_PRIORITY, NULL);                                  \
	MODEM_CELLULAR_DEV_PPP_DEFINE(DEVICE_GET(MODEM_CELLULAR_PPP_DEV_ID(inst)),                 \
				      MODEM_CELLULAR_INST_NAME(ppp, inst), _init_iface, _mtu,      \
				      _buf_size)

#define Z_MODEM_CELLULAR_DEFINE_INSTANCE(inst, vendor_config)                                      \
	MODEM_PIPELINK_DT_INST_DEFINE(inst, MODEM_CELLULAR_PIPELINK_PPP);                           \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		Z_MODEM_CELLULAR_CONFIG_INIT(inst, vendor_config)                                  \
		.ppp_pipelink = MODEM_PIPELINK_DT_INST_GET(inst, MODEM_CELLULAR_PIPELINK_PPP),       \
	};                                                                                         \
	Z_MODEM_CELLULAR_DEVICE_DEFINE(inst);

/** @endcond */

/**
 * @brief Name of the pipelink to the PPP channel of a modem.
 *
 * The pipelink carries CMUX DLCI 2. It is connected once the CMUX is up and the AT command
 * channel on DLCI 1, which is kept by the driver, is set up, and disconnected along with the
 * CMUX. User pipes are connected and disconnected at the same time. Pass it as the name to
 * @ref MODEM_PIPELINK_DT_DECLARE and @ref MODEM_PIPELINK_DT_GET along with the modem's
 * devicetree node.
 */
#define MODEM_CELLULAR_PIPELINK_PPP ppp_pipe

/**
 * @brief Name of the PPP device of a modem.
 *
 * Each modem has a PPP device, which dials the data call on the PPP channel once the modem is
 * attached and attaches the PPP network interface to it. The network interface holds a runtime
 * PM reference on the PPP device while it is up, which in turn holds one on the modem. Pass the
 * name to @c device_get_binding to get the device.
 *
 * @param node_id Devicetree node of the modem.
 */
#define MODEM_CELLULAR_PPP_DEVICE_NAME(node_id)                                                    \
	STRINGIFY(CONCAT(modem_cellular_ppp_, DT_DEP_ORD(node_id)))

/** @} */

/**
 * @addtogroup cellular_interface
 * @{
 */

/**
 * @brief Request the LTE RF of a cellular modem to be enabled.
 *
 * Requests are counted, and the LTE RF is enabled while at least one is held. The modem
 * is left with its LTE RF disabled once it is ready for commands, and enables it again
 * whenever it becomes ready with a request held, so requests are kept across the modem
 * being restarted by recovery. Each request holds a runtime PM reference on the modem, which
 * powers it up, and keeps it from being suspended until the request is released.
 *
 * Unless the modem uses its default PDP context, the data context is configured with the APN
 * before each enable. The APN, from @kconfig{CONFIG_MODEM_CELLULAR_APN} or cellular_set_apn(),
 * cannot be changed while a request is held.
 *
 * @param dev Cellular device created with @ref MODEM_CELLULAR_DEFINE_INSTANCE(). Must not be NULL.
 *
 * @retval 0 Request held. The LTE RF is enabled once the modem is ready for commands.
 * @retval -ENODATA No APN is available for the data context.
 * @retval <0 The modem could not be resumed, see pm_device_runtime_get().
 *
 * @see modem_cellular_lte_rf_release
 */
int modem_cellular_lte_rf_request(const struct device *dev);

/**
 * @brief Release a request for the LTE RF of a cellular modem.
 *
 * The LTE RF is disabled once the last request is released, before the modem is suspended
 * if no other runtime PM reference is held on it. The reference is released asynchronously.
 *
 * @param dev Cellular device created with @ref MODEM_CELLULAR_DEFINE_INSTANCE(). Must not be NULL.
 *
 * @retval 0 Request released.
 * @retval -EALREADY No request is held.
 * @retval <0 The reference could not be released, see pm_device_runtime_put_async().
 *
 * @see modem_cellular_lte_rf_request
 */
int modem_cellular_lte_rf_release(const struct device *dev);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_V2_H_ */
