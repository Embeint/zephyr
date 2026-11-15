/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file modem_cellular_v1.h
 * @brief Original cellular modem driver implementation.
 *
 * Included by modem_cellular.h unless @kconfig{CONFIG_MODEM_CELLULAR_V2} is enabled.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_V1_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_V1_H_

#ifndef ZEPHYR_INCLUDE_DRIVERS_CELLULAR_INTERNAL_H_
#error "Include <zephyr/drivers/modem/modem_cellular.h> instead"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup modem_cellular_backend
 * @{
 */

/** @cond INTERNAL_HIDDEN */

/* Zephyr networking interface states:
 *    NET_IF_LOWER_UP: Carrier is on in AWAIT_REGISTERED and REGISTERED
 *     NET_IF_DORMANT: Interface is dormant in every state except REGISTERED
 */
enum modem_cellular_state {
	MODEM_CELLULAR_STATE_IDLE = 0,
	MODEM_CELLULAR_STATE_RECOVERY,
	MODEM_CELLULAR_STATE_RESET_PULSE,
	MODEM_CELLULAR_STATE_AWAIT_RESET,
	MODEM_CELLULAR_STATE_POWER_ON_PULSE,
	MODEM_CELLULAR_STATE_AWAIT_POWER_ON,
	MODEM_CELLULAR_STATE_SET_BAUDRATE,
	MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT,
	MODEM_CELLULAR_STATE_CONNECT_CMUX,
	MODEM_CELLULAR_STATE_OPEN_DLCI1,
	MODEM_CELLULAR_STATE_OPEN_DLCI2,
	MODEM_CELLULAR_STATE_RUN_BOARD_INIT_SCRIPT,
	MODEM_CELLULAR_STATE_WAIT_FOR_APN,
	MODEM_CELLULAR_STATE_RUN_APN_SCRIPT,
	MODEM_CELLULAR_STATE_RUN_NETWORK_SCRIPT,
	MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT,
	MODEM_CELLULAR_STATE_AWAIT_REGISTERED,
	MODEM_CELLULAR_STATE_REGISTERED,
	MODEM_CELLULAR_STATE_AWAIT_PPP_DEAD,
	MODEM_CELLULAR_STATE_INIT_POWER_OFF,
	MODEM_CELLULAR_STATE_RUN_SHUTDOWN_SCRIPT,
	MODEM_CELLULAR_STATE_POWER_OFF_PULSE,
	MODEM_CELLULAR_STATE_AWAIT_POWER_OFF,
	MODEM_CELLULAR_STATE_AWAIT_DIAL,
};

enum modem_cellular_event {
	MODEM_CELLULAR_EVENT_RESUME = 0,
	MODEM_CELLULAR_EVENT_SUSPEND,
	MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS,
	MODEM_CELLULAR_EVENT_SCRIPT_FAILED,
	MODEM_CELLULAR_EVENT_CMUX_CONNECTED,
	MODEM_CELLULAR_EVENT_CMUX_DISCONNECTED,
	MODEM_CELLULAR_EVENT_DLCI1_OPENED,
	MODEM_CELLULAR_EVENT_DLCI2_OPENED,
	MODEM_CELLULAR_EVENT_TIMEOUT,
	MODEM_CELLULAR_EVENT_REGISTERED,
	MODEM_CELLULAR_EVENT_DEREGISTERED,
	MODEM_CELLULAR_EVENT_BUS_OPENED,
	MODEM_CELLULAR_EVENT_BUS_CLOSED,
	MODEM_CELLULAR_EVENT_PPP_DEAD,
	MODEM_CELLULAR_EVENT_MODEM_READY,
	MODEM_CELLULAR_EVENT_APN_SET,
	MODEM_CELLULAR_EVENT_RING,
	MODEM_CELLULAR_EVENT_PERIODIC_KICK,
	MODEM_CELLULAR_EVENT_DIAL,
	MODEM_CELLULAR_EVENT_HANGUP,
} __packed;

/** @endcond */

/**
 * @brief Runtime data for a cellular modem driver instance.
 *
 * Define one zero-initialized object per device instance. The object and all referenced
 * configuration data must remain valid for the lifetime of the device. Set the documented
 * configuration members before passing the object to @ref MODEM_CELLULAR_DEFINE_INSTANCE().
 * The cellular modem implementation owns and updates all other members after device initialization.
 */
struct modem_cellular_data {
	/** @cond INTERNAL_HIDDEN */
	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[CONFIG_MODEM_CELLULAR_UART_BUFFER_SIZES];
	uint8_t uart_backend_transmit_buf[CONFIG_MODEM_CELLULAR_UART_BUFFER_SIZES];
	uint32_t original_baudrate;

	/* CMUX */
	struct modem_cmux cmux;
	uint8_t cmux_receive_buf[MODEM_CMUX_WORK_BUFFER_SIZE];
	uint8_t cmux_transmit_buf[MODEM_CMUX_WORK_BUFFER_SIZE];

	struct modem_cmux_dlci dlci1;
	struct modem_cmux_dlci dlci2;
	struct modem_pipe *dlci1_pipe;
	struct modem_pipe *dlci2_pipe;
	/* Points to dlci1_pipe or NULL. Used for shutdown script if not NULL */
	struct modem_pipe *cmd_pipe;
	uint8_t dlci1_receive_buf[MODEM_CMUX_WORK_BUFFER_SIZE];
	/* DLCI 2 is only used for chat scripts. */
	uint8_t dlci2_receive_buf[MODEM_CMUX_WORK_BUFFER_SIZE];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CONFIG_MODEM_CELLULAR_CHAT_BUFFER_SIZE];
	uint8_t *chat_argv[32];
	uint8_t script_failure_counter;
	uint8_t recovery_count;

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
	uint8_t apn[MODEM_CELLULAR_DATA_APN_LEN];
	/* An APN has been set, which may be empty to use the default APN of the subscription */
	bool apn_set;

	struct modem_chat_script_chat apn_chats[MODEM_CELLULAR_MAX_APN_CMDS];
	struct modem_chat_script apn_script;
	char apn_buf[MODEM_CELLULAR_MAX_APN_CMDS][MODEM_CELLULAR_APN_BUF_SIZE];

	struct modem_chat_script board_init_script;

	struct net_mgmt_event_callback net_mgmt_event_callback;
#if defined(CONFIG_MODEM_CELLULAR_ON_DEMAND_CONNECT)
	struct net_mgmt_event_callback if_event_callback;
#endif

	enum modem_cellular_state state;
	/** Chat script the current state started, NULL until it starts one */
	const struct modem_chat_script *script_running;
	const struct device *dev;
	struct k_work_delayable timeout_work;

	/* Power management */
	struct k_sem suspended_sem;

	/* Event dispatcher */
	struct k_work event_dispatch_work;
	struct k_msgq event_queue;
	struct modem_cellular_event_pkg event_buf[8];
	const void *event_ptr;

	struct k_mutex api_lock;
	struct modem_cellular_event_cb cb;

	/* Ring interrupt */
	struct gpio_callback ring_gpio_cb;

	/** Set when the periodic chat script is paused. */
	atomic_t periodic_paused;
	/** Set when a TIMEOUT is swallowed while paused; cleared on KICK. */
	bool periodic_timeout_skipped;
	/** Set when the power-on pulse was skipped because the status GPIO reported
	 * the modem already powered; the init script then confirms it responds.
	 */
	bool power_on_skipped;
	/** Set when suspended during the init script, which leaves the modem powered at the
	 * UART's current baudrate; a resume that finds it still powered keeps that baudrate.
	 */
	bool baudrate_kept;
	/** Set when the init script failed, so the modem did not respond at the current
	 * baudrate; a suspend that follows, when recoveries are exhausted, does not keep it.
	 */
	bool init_script_failed;
	/** Set when a suspend arrives while CMUX is connecting; acted on once it settles. */
	bool suspend_pending;
	/** Set when the CMUX connect attempt has failed; a suspend is then acted on at once. */
	bool cmux_connect_failed;
	/** Set once the UART has opened for the CMUX connect attempt. */
	bool cmux_bus_opened;

#if defined(CONFIG_MODEM_CELLULAR_STATS)
	/** Operational statistics, exposed via cellular_get_stats(). */
	struct cellular_stats stats;
	/** k_uptime (ms) when registration was last lost; 0 while registered. */
	uint32_t stats_outage_start_ms;
#endif
	/** @endcond */
};

/** @cond INTERNAL_HIDDEN */

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
};

#define Z_MODEM_CELLULAR_PPP_DEFINE(inst, _init_iface, _mtu, _buf_size)                            \
	MODEM_DT_INST_PPP_DEFINE(inst, MODEM_CELLULAR_INST_NAME(ppp, inst), _init_iface, _mtu,     \
				 _buf_size)

#define Z_MODEM_CELLULAR_DEFINE_INSTANCE(inst, vendor_config)                                      \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		Z_MODEM_CELLULAR_CONFIG_INIT(inst, vendor_config)                                  \
	};                                                                                         \
	Z_MODEM_CELLULAR_DEVICE_DEFINE(inst);

/** @endcond */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_V1_H_ */
