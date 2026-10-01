/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdint.h>
typedef void *whd_interface_t;
uint32_t whd_wifi_set_iovar_buffer(whd_interface_t ifp, const char *name, void *buffer,
				   uint16_t length);
