/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/modem/modem_cellular.h>

#include <stdlib.h>

#define DT_DRV_COMPAT telit_le910c1tx

/* Order must match the `sim-detect` devicetree enum */
enum le910c1tx_sim_detect {
	LE910C1TX_SIM_DETECT_ACTIVE_LOW,
	LE910C1TX_SIM_DETECT_ACTIVE_HIGH,
	LE910C1TX_SIM_DETECT_FORCE_PRESENT,
	LE910C1TX_SIM_DETECT_UNCHANGED,
};

/* AT#SIMDET modes */
#define LE910C1TX_SIMDET_FORCE_PRESENT 1
#define LE910C1TX_SIMDET_SIMIN_PIN     2

struct le910c1tx_modem_cellular_config {
	/** SIM detection configuration */
	enum le910c1tx_sim_detect sim_detect;
};

struct le910c1tx_modem_cellular_data {
	/** Common modem data */
	struct modem_cellular_data data;
	/** Vendor specific data */

	/** Current AT#SIMDET mode reported by the modem */
	int simdet_mode;
};
BUILD_ASSERT(offsetof(struct le910c1tx_modem_cellular_data, data) == 0,
	     "Common data must be at start of struct");

static void le910c1tx_on_simdet(struct modem_chat *chat, char **argv, uint16_t argc,
				void *user_data);
static bool le910c1tx_sim_detect_active_low(void *user_data);
static bool le910c1tx_sim_detect_active_high(void *user_data);
static bool le910c1tx_sim_detect_configured(void *user_data);
static bool le910c1tx_simdet_pin_required(void *user_data);
static bool le910c1tx_simdet_force_present_required(void *user_data);
static bool le910c1tx_simdet_required(void *user_data);

MODEM_CELLULAR_COMMON_CHAT_MATCHES();

MODEM_CHAT_MATCHES_DEFINE(telit_le910c1tx_unsol, MODEM_CELLULAR_COMMON_UNSOL_MATCHES);

MODEM_CELLULAR_OK_CHAT_MATCH_DEFINE(le910c1tx_simdet_match, "#SIMDET: ", ",", le910c1tx_on_simdet);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(
	telit_le910c1tx_init_chat_script_cmds, MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100), MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100), MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#CFLO=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#STIA=0", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_COND("AT#SIMINCFG=0,0", ok_match,
					le910c1tx_sim_detect_active_low),
	MODEM_CHAT_SCRIPT_CMD_RESP_COND("AT#SIMINCFG=0,1", ok_match,
					le910c1tx_sim_detect_active_high),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT_COND("AT#SIMDET?", le910c1tx_simdet_match,
					     le910c1tx_sim_detect_configured),
	MODEM_CHAT_SCRIPT_CMD_RESP_COND("AT#SIMDET=2", ok_match, le910c1tx_simdet_pin_required),
	MODEM_CHAT_SCRIPT_CMD_RESP_COND("AT#SIMDET=1", ok_match,
					le910c1tx_simdet_force_present_required),
	/* Delay required for changes to AT#SIMDET */
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE_COND("", 5000, le910c1tx_simdet_required),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+ICCID", iccid_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CIMI", cimi_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=2", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=2", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=2", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGSN", imei_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+IPR=115200", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGMM", cgmm_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGMI", cgmi_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGMR", cgmr_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 5000),
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CMUX=0,0,5," STRINGIFY(CONFIG_MODEM_CMUX_MTU), 1000));

/* Timeout accounts for the optional AT#SIMDET delay */
MODEM_CHAT_SCRIPT_DEFINE(telit_le910c1tx_init_chat_script, telit_le910c1tx_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 15);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_le910c1tx_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE(
				"ATD*99***" STRINGIFY(CONFIG_MODEM_CELLULAR_PDP_CONTEXT_ID) "#",
				0));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910c1tx_dial_chat_script, telit_le910c1tx_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_le910c1tx_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910c1tx_periodic_chat_script,
			 telit_le910c1tx_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);

static enum le910c1tx_sim_detect le910c1tx_sim_detect(void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	const struct modem_cellular_config *config = data->dev->config;
	const struct le910c1tx_modem_cellular_config *vendor_config = config->vendor->vendor_inst;

	return vendor_config->sim_detect;
}

static bool le910c1tx_sim_detect_active_low(void *user_data)
{
	return le910c1tx_sim_detect(user_data) == LE910C1TX_SIM_DETECT_ACTIVE_LOW;
}

static bool le910c1tx_sim_detect_active_high(void *user_data)
{
	return le910c1tx_sim_detect(user_data) == LE910C1TX_SIM_DETECT_ACTIVE_HIGH;
}

static bool le910c1tx_sim_detect_configured(void *user_data)
{
	return le910c1tx_sim_detect(user_data) != LE910C1TX_SIM_DETECT_UNCHANGED;
}

static void le910c1tx_on_simdet(struct modem_chat *chat, char **argv, uint16_t argc,
				void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	struct le910c1tx_modem_cellular_data *vendor_data =
		CONTAINER_OF(data, struct le910c1tx_modem_cellular_data, data);

	/* Response format is #SIMDET: <mode>,<simin> */
	vendor_data->simdet_mode = (argc >= 2) ? strtol(argv[1], NULL, 10) : -1;
}

static int le910c1tx_simdet_target(void *user_data)
{
	switch (le910c1tx_sim_detect(user_data)) {
	case LE910C1TX_SIM_DETECT_ACTIVE_LOW:
	case LE910C1TX_SIM_DETECT_ACTIVE_HIGH:
		return LE910C1TX_SIMDET_SIMIN_PIN;
	case LE910C1TX_SIM_DETECT_FORCE_PRESENT:
		return LE910C1TX_SIMDET_FORCE_PRESENT;
	default:
		return -1;
	}
}

static bool le910c1tx_simdet_required(void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	struct le910c1tx_modem_cellular_data *vendor_data =
		CONTAINER_OF(data, struct le910c1tx_modem_cellular_data, data);
	int target = le910c1tx_simdet_target(user_data);

	/* AT#SIMDET should only be sent if the mode is not already correct */
	return (target >= 0) && (vendor_data->simdet_mode != target);
}

static bool le910c1tx_simdet_pin_required(void *user_data)
{
	return (le910c1tx_simdet_target(user_data) == LE910C1TX_SIMDET_SIMIN_PIN) &&
	       le910c1tx_simdet_required(user_data);
}

static bool le910c1tx_simdet_force_present_required(void *user_data)
{
	return (le910c1tx_simdet_target(user_data) == LE910C1TX_SIMDET_FORCE_PRESENT) &&
	       le910c1tx_simdet_required(user_data);
}

static const struct modem_cellular_vendor_config_common telit_le910c1tx_vendor = {
	/* clang-format off */
	.scripts = {
		.init = &telit_le910c1tx_init_chat_script,
		.dial = &telit_le910c1tx_dial_chat_script,
		.periodic = &telit_le910c1tx_periodic_chat_script,
	},
	.unsol_matches = {
		.matches = telit_le910c1tx_unsol,
		.size = ARRAY_SIZE(telit_le910c1tx_unsol),
	},
	/* clang-format on */
	.chat_delimiter = "\r",
	.chat_filter = "\n",
	.baudrate_update_delay_ms = 300,
	.reset_power_on_delay_ms = 500,
	.power_pulse_duration_ms = 1500,
	.reset_pulse_duration_ms = 250,
	.startup_time_ms = 30000,
	.shutdown_time_ms = 5000,
};

#define MODEM_CELLULAR_DEVICE_TELIT_LE910C1TX(inst)                                                \
	MODEM_DT_INST_PPP_DEFINE(inst, MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 1500, 64);       \
                                                                                                   \
	static const struct le910c1tx_modem_cellular_config le910c1tx_vendor_cfg##inst = {         \
		.sim_detect =                                                                      \
			DT_INST_ENUM_IDX_OR(inst, sim_detect, LE910C1TX_SIM_DETECT_UNCHANGED),     \
	};                                                                                         \
                                                                                                   \
	static const struct modem_cellular_vendor_config MODEM_CELLULAR_INST_NAME(vendor_config,   \
										  inst) = {        \
		.common = &telit_le910c1tx_vendor,                                                 \
		.vendor_inst = &le910c1tx_vendor_cfg##inst,                                        \
	};                                                                                         \
                                                                                                   \
	static struct le910c1tx_modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst);          \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_AND_INIT_USER_PIPES(inst, (user_pipe_0, 3))                          \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_INSTANCE(inst, &MODEM_CELLULAR_INST_NAME(vendor_config, inst))

DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_TELIT_LE910C1TX)
