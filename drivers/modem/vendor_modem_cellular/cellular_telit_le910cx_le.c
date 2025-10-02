/*
 * Copyright (c) 2026 Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/modem/modem_cellular.h>

#include <stdlib.h>

#define DT_DRV_COMPAT telit_le910cx_le

static void le910cx_le_on_rfsts(struct modem_chat *chat, char **argv, uint16_t argc,
				void *user_data);

MODEM_CELLULAR_COMMON_CHAT_MATCHES();

MODEM_CHAT_MATCHES_DEFINE(telit_le910cx_le_unsol, MODEM_CELLULAR_COMMON_UNSOL_MATCHES,
			  MODEM_CHAT_MATCH("#RFSTS", "", le910cx_le_on_rfsts));

MODEM_CHAT_SCRIPT_CMDS_DEFINE(
	telit_le910cx_le_baudrate_chat_script_cmds, MODEM_CHAT_SCRIPT_CMD_RESP("AT", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+IPR=" STRINGIFY(CONFIG_MODEM_CELLULAR_NEW_BAUDRATE),
						       ok_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910cx_le_baudrate_chat_script,
			 telit_le910cx_le_baudrate_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 1);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(
	telit_le910cx_le_init_chat_script_cmds, MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT", allow_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
	/* Disable SIM URC messages, unhandled and causes GNSS stability issues */
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#STIA=0", ok_match),
#ifdef CONFIG_MODEM_CELLULAR_SIM_DETECTION_ACTIVE_LOW
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#SIMINCFG=0,0", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#SIMDET=2", ok_match),
#endif /* CONFIG_MODEM_CELLULAR_SIM_DETECTION_ACTIVE_LOW */
#ifdef CONFIG_MODEM_CELLULAR_SIM_DETECTION_ACTIVE_HIGH
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#SIMINCFG=0,1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#SIMDET=2", ok_match),
#endif /* CONFIG_MODEM_CELLULAR_SIM_DETECTION_ACTIVE_HIGH */
#ifdef CONFIG_MODEM_CELLULAR_SIM_DETECTION_FORCE_PRESENT
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#SIMDET=1", ok_match),
#endif /* CONFIG_MODEM_CELLULAR_SIM_DETECTION_FORCE_PRESENT */
	/* Delay required for changes to AT#SIMDET */
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 5000),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+ICCID", iccid_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match), MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match), MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match), MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match), MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match), MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match), MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	/* Put into active mode here even though it resets later since
	 *  it is required for the APN configuration.
	 */
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#CMUXMODE=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#CFLO=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMUX=0,0,4,1500", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910cx_le_init_chat_script, telit_le910cx_le_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_le910cx_le_network_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=2", ok_match),
			      /* V.24 RTS/DTR control can reset the CFUN state back to 4 */
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910cx_le_network_chat_script,
			 telit_le910cx_le_network_chat_script_cmds, dial_abort_matches,
			 modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_le910cx_le_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATD*99***1#\r", connect_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910cx_le_dial_chat_script, telit_le910cx_le_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_le910cx_le_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      /* RFSTS can be ignored by the modem if a GPSACP command is executing
			       * on a different DLCI channel.
			       */
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT#RFSTS", 500));

MODEM_CHAT_SCRIPT_DEFINE(telit_le910cx_le_periodic_chat_script,
			 telit_le910cx_le_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);

static const struct modem_cellular_vendor_config telit_le910cx_le_vendor = {
	/* clang-format off */
	.scripts = {
		.init = &telit_le910cx_le_init_chat_script,
		.network = &telit_le910cx_le_network_chat_script,
		.dial = &telit_le910cx_le_dial_chat_script,
		.periodic = &telit_le910cx_le_periodic_chat_script,
		.set_baudrate = &telit_le910cx_le_baudrate_chat_script,
	},
	.unsol_matches = {
		.matches = telit_le910cx_le_unsol,
		.size = ARRAY_SIZE(telit_le910cx_le_unsol),
	},
	/* clang-format on */
	.power_pulse_duration_ms = 1500,
	.reset_pulse_duration_ms = 250,
	.startup_time_ms = 30000,
	.shutdown_time_ms = 5000,
};

#define MODEM_CELLULAR_DEVICE_TELIT_LE910CX_LE(inst)                                               \
	MODEM_DT_INST_PPP_DEFINE(inst, MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 128);  \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_AND_INIT_USER_PIPES(inst, (user_pipe_0, 3))                          \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_INSTANCE(inst, &telit_le910cx_le_vendor)

DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_TELIT_LE910CX_LE)
