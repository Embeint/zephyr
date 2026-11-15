/*
 * Copyright (c) 2023 Bjarki Arge Andreasen
 * Copyright (c) 2026 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Chat parsers and helpers of the cellular modem driver, independent of its state machine */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/modem/modem_cellular.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipelink.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(modem_cellular, CONFIG_MODEM_LOG_LEVEL);

#include <stdlib.h>
#include <string.h>

#include "modem_cellular_common.h"

void modem_cellular_emit_event(struct modem_cellular_data *data,
			       enum cellular_event evt, const void *payload)
{
	struct modem_cellular_event_cb cb;

	/* Take a consistent snapshot, then invoke outside the lock so callbacks may call the API. */
	k_mutex_lock(&data->api_lock, K_FOREVER);
	cb = data->cb;
	k_mutex_unlock(&data->api_lock);

	if ((cb.fn != NULL) && ((cb.mask & evt) != 0U)) {
		cb.fn(data->dev, evt, payload, cb.user_data);
	}
}

/*
 * Compares the fields that constitute a network-status change. Signal quality
 * (rsrp/rsrq) is deliberately excluded: it fluctuates on nearly every poll and
 * would otherwise re-fire the event continuously.
 */
static bool modem_cellular_network_status_equal(const struct cellular_evt_network_status *a,
						const struct cellular_evt_network_status *b)
{
	if (a->status != b->status || a->access_tech != b->access_tech) {
		return false;
	}

	/* Cell identity */
	if (a->cell.lte.mcc != b->cell.lte.mcc || a->cell.lte.mnc != b->cell.lte.mnc ||
	    a->cell.lte.tac != b->cell.lte.tac || a->cell.lte.gci != b->cell.lte.gci) {
		return false;
	}

	/* Radio channel */
	if (a->cell.lte.earfcn != b->cell.lte.earfcn || a->cell.lte.band != b->cell.lte.band ||
	    a->cell.lte.phys_cell_id != b->cell.lte.phys_cell_id) {
		return false;
	}

	return true;
}

void modem_cellular_emit_network_status(struct modem_cellular_data *data,
					const struct cellular_evt_network_status *status)
{
	bool changed;

	/* api_lock serialises the cache against get_network_status(); emitters run
	 * on the modem work queue and must not already hold it. The cache always
	 * takes the latest value (so the getter reports current signal), but the
	 * change test ignores signal quality.
	 */
	k_mutex_lock(&data->api_lock, K_FOREVER);
	changed = !data->network_status_valid ||
		  !modem_cellular_network_status_equal(&data->network_status, status);
	data->network_status = *status;
	data->network_status_valid = true;
	k_mutex_unlock(&data->api_lock);

	/* Emit outside the lock, and only on a real change so periodic pollers do
	 * not re-fire the event every cycle.
	 */
	if (changed) {
		modem_cellular_emit_event(data, CELLULAR_EVENT_NETWORK_STATUS_CHANGED, status);
	}
}

void modem_cellular_invalidate_network_status(struct modem_cellular_data *data)
{
	k_mutex_lock(&data->api_lock, K_FOREVER);
	data->network_status_valid = false;
	k_mutex_unlock(&data->api_lock);
}

static void modem_cellular_emit_modem_info(struct modem_cellular_data *data,
					   enum cellular_modem_info_type field)
{
	struct cellular_evt_modem_info evt = {
		.field = field,
	};

	modem_cellular_emit_event(data, CELLULAR_EVENT_MODEM_INFO_CHANGED, &evt);
}

bool modem_cellular_modem_is_powered(const struct modem_cellular_config *config)
{
	int ret;

	/* A status GPIO reads asserted while the modem is already powered on, e.g.
	 * after an MCU-only reset that left the modem running. Without a status GPIO,
	 * report the modem as off so the normal power-on path runs unchanged.
	 */
	if (!modem_cellular_gpio_is_enabled(&config->status_gpio)) {
		return false;
	}

	ret = gpio_pin_get_dt(&config->status_gpio);
	if (ret < 0) {
		LOG_WRN("Failed to read modem status GPIO (%d)", ret);
		return false;
	}

	return ret == 1;
}

void modem_cellular_notify_user_pipes_connected(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = data->dev->config;
	struct modem_cellular_user_pipe *user_pipe;
	struct modem_pipelink *pipelink;

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		user_pipe = &config->user_pipes[i];
		pipelink = user_pipe->pipelink;
		modem_pipelink_notify_connected(pipelink);
	}
}

void modem_cellular_notify_user_pipes_disconnected(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = data->dev->config;
	struct modem_cellular_user_pipe *user_pipe;
	struct modem_pipelink *pipelink;

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		user_pipe = &config->user_pipes[i];
		pipelink = user_pipe->pipelink;
		modem_pipelink_notify_disconnected(pipelink);
	}
}

void modem_cellular_chat_on_imei(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	/* Any modem using this callback is making the assumption that IMEI == SN,
	 * as the documented response to 'AT+CGSN' is the SN, not the IMEI.
	 */
	strncpy(data->imei, argv[1], sizeof(data->imei) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_IMEI);
	strncpy(data->sn, argv[1], sizeof(data->sn) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_SERIAL_NUMBER);
}

void modem_cellular_chat_on_cgsn_sn(struct modem_chat *chat, char **argv, uint16_t argc,
				    void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->sn, argv[1], sizeof(data->sn) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_SERIAL_NUMBER);
}

void modem_cellular_chat_on_cgsn_imei(struct modem_chat *chat, char **argv, uint16_t argc,
				      void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	const char *rsp;
	size_t rsp_len;

	if (argc != 2) {
		return;
	}

	/* 3GPP specifies 15 digit string type in decimal format */
	rsp = argv[1];
	rsp_len = strlen(rsp);
	if ((rsp_len != 17) || (rsp[0] != '"') || (rsp[16] != '"')) {
		LOG_WRN("Invalid CGSN response: %s", rsp);
		return;
	}

	/* IMEI from AT+CGSN is string quoted.
	 * 3GPP specifies the length as exactly 15 digits.
	 * Start from offset 1 to skip first quote character.
	 */
	memcpy(data->imei, rsp + 1, 15);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_IMEI);
}

void modem_cellular_chat_on_cgmm(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->model_id, argv[1], sizeof(data->model_id) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_MODEL_ID);
}

void modem_cellular_chat_on_cgmi(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->manufacturer, argv[1], sizeof(data->manufacturer) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_MANUFACTURER);
}

void modem_cellular_chat_on_cgmr(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->fw_version, argv[1], sizeof(data->fw_version) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_FW_VERSION);
}

void modem_cellular_chat_on_csq(struct modem_chat *chat, char **argv, uint16_t argc,
				void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 3) {
		return;
	}

	data->rssi = (uint8_t)atoi(argv[1]);
}

void modem_cellular_chat_on_cesq(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 7) {
		return;
	}

	data->rsrq = (uint8_t)atoi(argv[5]);
	data->rsrp = (uint8_t)atoi(argv[6]);
}

void modem_cellular_chat_on_iccid(struct modem_chat *chat, char **argv, uint16_t argc,
				  void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->iccid, argv[1], sizeof(data->iccid) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_SIM_ICCID);
}

void modem_cellular_chat_on_imsi(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->imsi, argv[1], sizeof(data->imsi) - 1);
	modem_cellular_emit_modem_info(data, CELLULAR_MODEM_INFO_SIM_IMSI);
}

uint32_t modem_cellular_baudrate_update(struct modem_cellular_data *data,
					uint32_t desired_baudrate)
{
	const struct modem_cellular_config *config = data->dev->config;
	struct uart_config cfg = {0};
	uint32_t original_baudrate;
	int ret;

	ret = uart_config_get(config->uart, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to get UART configuration (%d)", ret);
		return 0;
	}
	original_baudrate = cfg.baudrate;
	cfg.baudrate = desired_baudrate;
	ret = uart_configure(config->uart, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to set new baudrate (%d)", ret);
		return 0;
	}
	return original_baudrate;
}

int modem_cellular_get_modem_info(const struct device *dev, enum cellular_modem_info_type type,
				  char *info, size_t size)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;
	const char *info_str;

	if (size <= 1) {
		return -EINVAL;
	}

	switch (type) {
	case CELLULAR_MODEM_INFO_IMEI:
		info_str = &data->imei[0];
		break;
	case CELLULAR_MODEM_INFO_SIM_IMSI:
		info_str = &data->imsi[0];
		break;
	case CELLULAR_MODEM_INFO_MANUFACTURER:
		info_str = &data->manufacturer[0];
		break;
	case CELLULAR_MODEM_INFO_FW_VERSION:
		info_str = &data->fw_version[0];
		break;
	case CELLULAR_MODEM_INFO_MODEL_ID:
		info_str = &data->model_id[0];
		break;
	case CELLULAR_MODEM_INFO_SIM_ICCID:
		info_str = &data->iccid[0];
		break;
	case CELLULAR_MODEM_INFO_SERIAL_NUMBER:
		info_str = &data->sn[0];
		break;
	default:
		return -ENODATA;
	}

	/* All internal copies of modem info are NUL terminated.
	 * Copy at most `size - 1` bytes of the info to the output.
	 * Manually NUL terminate the output.
	 */
	strncpy(info, info_str, size - 1);
	info[size - 1] = '\0';
	return 0;
}

int modem_cellular_get_network_status(const struct device *dev,
				      struct cellular_evt_network_status *status)
{
	struct modem_cellular_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->api_lock, K_FOREVER);

	if (data->network_status_valid) {
		*status = data->network_status;
	} else {
		ret = -ENODATA;
	}

	k_mutex_unlock(&data->api_lock);

	return ret;
}

int modem_cellular_set_callback(const struct device *dev, cellular_event_mask_t mask,
				cellular_event_cb_t cb, void *user_data)
{
	struct modem_cellular_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->api_lock, K_FOREVER);

	if (cb == NULL) {
		data->cb.fn = NULL;
		data->cb.mask = 0;
		data->cb.user_data = NULL;
	} else {
		data->cb.fn = cb;
		data->cb.mask = mask;
		data->cb.user_data = user_data;
	}

	k_mutex_unlock(&data->api_lock);
	return ret;
}

void modem_cellular_emit_reg_state(struct modem_cellular_data *data,
				   enum cellular_registration_status status)
{
	struct cellular_evt_registration_status evt = {
		.status = status,
	};

	modem_cellular_emit_event(data, CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED, &evt);
}

bool modem_cellular_status_is_registered(enum cellular_registration_status status)
{
	return (status == CELLULAR_REGISTRATION_REGISTERED_HOME) ||
	       (status == CELLULAR_REGISTRATION_REGISTERED_ROAMING);
}

bool modem_cellular_is_registered(const struct modem_cellular_data *data)
{
	return modem_cellular_status_is_registered(data->registration_status_gsm) ||
	       modem_cellular_status_is_registered(data->registration_status_gprs) ||
	       modem_cellular_status_is_registered(data->registration_status_lte) ||
	       modem_cellular_status_is_registered(data->registration_status_5g);
}

void modem_cellular_clear_registration_status(struct modem_cellular_data *data)
{
	data->registration_status_gsm = CELLULAR_REGISTRATION_NOT_REGISTERED;
	data->registration_status_gprs = CELLULAR_REGISTRATION_NOT_REGISTERED;
	data->registration_status_lte = CELLULAR_REGISTRATION_NOT_REGISTERED;
	data->registration_status_5g = CELLULAR_REGISTRATION_NOT_REGISTERED;
	data->pdn_active = false;
}

int modem_cellular_get_registration_status(const struct device *dev,
					   enum cellular_access_technology tech,
					   enum cellular_registration_status *status)
{
	int ret = 0;
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;

	switch (tech) {
	case CELLULAR_ACCESS_TECHNOLOGY_GSM:
	case CELLULAR_ACCESS_TECHNOLOGY_GSM_COMPACT:
		*status = data->registration_status_gsm;
		break;
	case CELLULAR_ACCESS_TECHNOLOGY_GSM_EGPRS:
	case CELLULAR_ACCESS_TECHNOLOGY_EC_GSM_IOT:
	case CELLULAR_ACCESS_TECHNOLOGY_UTRAN:
	case CELLULAR_ACCESS_TECHNOLOGY_UTRAN_HSDPA:
	case CELLULAR_ACCESS_TECHNOLOGY_UTRAN_HSUPA:
	case CELLULAR_ACCESS_TECHNOLOGY_UTRAN_HSDPA_HSUPA:
		*status = data->registration_status_gprs;
		break;
	case CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN:
	case CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN_NB_S1:
	case CELLULAR_ACCESS_TECHNOLOGY_E_UTRA_NR_DUAL:
	case CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN_NB_S1_SAT:
	case CELLULAR_ACCESS_TECHNOLOGY_E_UTRAN_WB_S1_SAT:
	case CELLULAR_ACCESS_TECHNOLOGY_NG_RAN_SAT:
		*status = data->registration_status_lte;
		break;
	case CELLULAR_ACCESS_TECHNOLOGY_E_UTRA_5G_CN:
	case CELLULAR_ACCESS_TECHNOLOGY_NR_5G_CN:
	case CELLULAR_ACCESS_TECHNOLOGY_NG_RAN:
		*status = data->registration_status_5g;
		break;
	default:
		ret = -ENODATA;
		break;
	}

	return ret;
}

int modem_cellular_store_registration(struct modem_cellular_data *data, char **argv, uint16_t argc,
				      enum cellular_registration_status *status,
				      enum cellular_registration_status *previous)
{
	enum cellular_registration_status *stored;
	uint8_t num_args;
	uint8_t base;

	/* This receives both +C*REG? read command answers and unsolicited notifications.
	 * Their syntax differs in that the former has one more parameter, <n>, which is first.
	 * A notification's <stat> is followed by a quoted area code, or by an empty one when a
	 * later parameter such as the EMM cause is given without the location.
	 */
	if (argc >= 3 && argv[2][0] != '"' && argv[2][0] != '\0') {
		/* +CEREG: <n>,<stat>[,<tac>[...]] */
		base = 2;
	} else if (argc >= 2) {
		/* +CEREG: <stat>[,<tac>[...]] */
		base = 1;
	} else {
		return -EINVAL;
	}
	/* Long form of the various CXREG options:
	 *   +CREG: <stat>[,<lac>,<ci>[,<AcT>]]
	 *   +CGREG:<stat>[,<lac>,<ci>[,<AcT>,<rac>]]
	 *   +CEREG: <stat>[,[<tac>],[<ci>],[<AcT>]]
	 *   +C5GREG: <stat>[,[<tac>],[<ci>],[<AcT>],[<Allowed_NSSAI_length>],[<Allowed_NSSAI>]]
	 */
	num_args = argc - base;
	*status = atoi(argv[base]);
	if ((num_args >= 4) && (argv[base + 3][0] != '\0')) {
		data->access_tech = strtol(argv[base + 3], NULL, 10);
	}
	LOG_DBG("REG %d AcT %d", *status, data->access_tech);

	if (strcmp(argv[0], "+CREG: ") == 0) {
		stored = &data->registration_status_gsm;
	} else if (strcmp(argv[0], "+CGREG: ") == 0) {
		stored = &data->registration_status_gprs;
	} else if (strcmp(argv[0], "+C5GREG: ") == 0) {
		stored = &data->registration_status_5g;
	} else { /* CEREG */
		stored = &data->registration_status_lte;
	}

	*previous = *stored;
	*stored = *status;
	return 0;
}

/* Whether a data call can be made: registered, or with the data context active per +CGEV */
bool modem_cellular_is_attached(const struct modem_cellular_data *data)
{
	return modem_cellular_is_registered(data) || data->pdn_active;
}

/* Context ID in a +CGEV parameter, or -1 when the parameter is empty or not numeric */
static int modem_cellular_cgev_parse_cid(const char *param)
{
	char *end;
	long cid = strtol(param, &end, 10);

	return ((end == param) || (cid < 0)) ? -1 : (int)cid;
}

/* Context ID of a +CGEV ACT or DEACT event, or -1 when the event does not carry one */
static int modem_cellular_cgev_cid(char **argv, uint16_t argc)
{
	/* The last word of the event is <cid>, <p_cid> or <PDP_type> */
	const char *param = strrchr(argv[1], ' ');

	if (param == NULL) {
		return -1;
	}
	param++;

	if ((*param < '0') || (*param > '9')) {
		/* DEACT <PDP_type>, <PDP_addr>, [<cid>], the <cid> only given if known to the MT */
		return (argc > 3) ? modem_cellular_cgev_parse_cid(argv[3]) : -1;
	}
	if (strstr(argv[1], "PDN") != NULL) {
		/* PDN ACT <cid> or PDN DEACT <cid> */
		return modem_cellular_cgev_parse_cid(param);
	}
	/* ACT <p_cid>, <cid>, <event_type> or DEACT <p_cid>, <cid>, <event_type> */
	return (argc > 2) ? modem_cellular_cgev_parse_cid(argv[2]) : -1;
}

int modem_cellular_parse_cgev(const struct modem_cellular_data *data, char **argv, uint16_t argc,
			      bool *connected)
{
	const struct modem_cellular_config *config = data->dev->config;
	/* The modem dials the default context itself, otherwise the driver dials its own */
	const int data_cid =
		config->use_default_pdp_context ? 0 : CONFIG_MODEM_CELLULAR_PDP_CONTEXT_ID;
	int cid;

	if ((argc < 2) || (strcmp(argv[0], "+CGEV: ") != 0)) {
		return -EINVAL;
	}
	/* See 3GPP TS 27.007 for +CGEV syntax
	 *
	 * Any of the following means PDN connected:
	 * +CGEV: NW PDN ACT <cid>
	 * +CGEV: ME PDN ACT <cid>[,<reason>[,<cid_other>]]
	 * +CGEV: NW ACT <p_cid>, <cid>, <event_type>
	 * +CGEV: ME ACT <p_cid>, <cid>, <event_type>
	 *
	 * Any of the following means PDN disconnected:
	 * +CGEV: NW DETACH
	 * +CGEV: ME DETACH
	 * +CGEV: NW DEACT <PDP_type>, <PDP_addr>, [<cid>]
	 * +CGEV: ME DEACT <PDP_type>, <PDP_addr>, [<cid>]
	 * +CGEV: NW PDN DEACT <cid>
	 * +CGEV: ME PDN DEACT <cid>
	 * +CGEV: NW DEACT <p_cid>, <cid>, <event_type>
	 * +CGEV: ME DEACT <p_cid>, <cid>, <event_type>
	 */

	/* argv[0] is the match, the event up to the first ',' follows in argv[1] */
	if (strstr(argv[1], "DETACH") != NULL) {
		/* Detaching from the packet domain ends every context */
		*connected = false;
		return 1;
	}

	if ((strstr(argv[1], " ACT") != NULL) || (strstr(argv[1], "DEACT") != NULL)) {
		cid = modem_cellular_cgev_cid(argv, argc);
		/* Events for other contexts do not affect the data call. An event without a
		 * <cid> may be for the data call, so it is assumed to be.
		 */
		if ((cid >= 0) && (cid != data_cid)) {
			return -ENOENT;
		}
		*connected = (strstr(argv[1], " ACT") != NULL);
		return 0;
	}

	return -ENOENT;
}

#ifdef CONFIG_MODEM_CELLULAR_APN
BUILD_ASSERT(sizeof(CONFIG_MODEM_CELLULAR_APN) - 1 < MODEM_CELLULAR_DATA_APN_LEN,
	     "CONFIG_MODEM_CELLULAR_APN too long for data->apn");
#endif

MODEM_CHAT_MATCH_DEFINE(apn_ok_match, "OK", "", NULL);

static int append_apn_cmd(struct modem_cellular_data *data, uint8_t *steps, const char *fmt,
			  const char *apn_value)
{
	int n;

	if (*steps >= MODEM_CELLULAR_MAX_APN_CMDS) {
		return -ENOMEM;
	}

	n = snprintk(data->apn_buf[*steps], MODEM_CELLULAR_APN_BUF_SIZE, fmt, apn_value);

	if (n < 0 || n >= MODEM_CELLULAR_APN_BUF_SIZE) {
		return -ENOMEM;
	}

	/* Fill the matching chat entry */
	modem_chat_script_chat_init(&data->apn_chats[*steps]);
	modem_chat_script_chat_set_request(&data->apn_chats[*steps], data->apn_buf[*steps]);
	modem_chat_script_chat_set_response_matches(&data->apn_chats[*steps], &apn_ok_match, 1);

	(*steps)++;
	return 0;
}

void modem_cellular_build_apn_script(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = data->dev->config;
	uint8_t steps = 0;
	const char *apn_value = data->apn;

	if (config->use_default_apn) {
		/* Omit the APN name AT+CGDCONT=<cid>,"IP","" */
		apn_value = "";
	}
	append_apn_cmd(data, &steps,
		       "AT+CGDCONT=" STRINGIFY(CONFIG_MODEM_CELLULAR_PDP_CONTEXT_ID)
		       ",\"IP\",\"%s\"",
		       apn_value);

	/* Vendor‑specific extras */
#if DT_HAS_COMPAT_STATUS_OKAY(swir_hl7800)
	append_apn_cmd(data, &steps,
		       "AT+KCNXCFG=" STRINGIFY(CONFIG_MODEM_CELLULAR_PDP_CONTEXT_ID)
		       ",\"GPRS\",\"%s\",,,\"IPV4\"",
		       apn_value);
#endif

	/* Glue the array into the script object */
	modem_chat_script_set_script_chats(&data->apn_script, data->apn_chats, steps);
}

bool modem_cellular_needs_apn(const struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config = data->dev->config;

	if (config->use_default_apn) {
		return false;
	}
	return data->apn[0] == '\0';
}

void modem_cellular_init_apn(struct modem_cellular_data *data,
			     const struct modem_chat_match *abort_matches,
			     size_t abort_matches_size)
{
#ifdef CONFIG_MODEM_CELLULAR_APN
	if (strlen(CONFIG_MODEM_CELLULAR_APN) > 0) {
		strncpy(data->apn, CONFIG_MODEM_CELLULAR_APN, sizeof(data->apn) - 1);
		data->apn[sizeof(data->apn) - 1] = '\0';
	}
#endif

	modem_chat_script_init(&data->apn_script);
	modem_chat_script_set_name(&data->apn_script, "modem_cellular_set_apn");
	modem_chat_script_set_abort_matches(&data->apn_script, abort_matches, abort_matches_size);
	modem_chat_script_set_timeout(&data->apn_script, 1);
	modem_chat_script_set_callback(&data->apn_script,
				       modem_cellular_chat_callback_handler);
}

int modem_cellular_check_apn(const struct modem_cellular_data *data, const char *apn)
{
	if ((apn == NULL) || (*apn == '\0')) {
		return -EINVAL;
	}

	if (strlen(apn) >= MODEM_CELLULAR_DATA_APN_LEN) {
		return -EINVAL;
	}

	if (strcmp(apn, data->apn) == 0) {
		return -EALREADY;
	}

	return 0;
}
