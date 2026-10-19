/*
 * Copyright (c) 2024, Embeint Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_fuel_gauge_composite

#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/battery.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Only build the charging SoC estimator when at least one instance enables it */
#define CHARGING_SOC_ESTIMATOR_ENABLED DT_ANY_INST_HAS_BOOL_STATUS_OKAY(charging_soc_estimator)

struct composite_config {
	const struct device *source_primary;
	const struct device *source_secondary;
	const struct device *charger;
	int32_t ocv_lookup_table[BATTERY_OCV_TABLE_LEN];
	uint32_t charge_capacity_microamp_hours;
	int32_t factory_internal_resistance_micro_ohms;
	enum battery_chemistry chemistry;
	bool secondary_fallback;
	bool fg_channels;
#if CHARGING_SOC_ESTIMATOR_ENABLED
	uint32_t charging_current_threshold_microamp;
	uint32_t ocv_relaxation_time_ms;
	bool charging_soc_estimator;
#endif /* CHARGING_SOC_ESTIMATOR_ENABLED */
};

struct composite_data {
	int64_t next_reading;
	bool primary_success;
#if CHARGING_SOC_ESTIMATOR_ENABLED
	k_ticks_t soc_update_time;
	k_ticks_t relaxation_start_time;
	int64_t estimated_soc_millipercent;
	int64_t relaxation_start_soc_millipercent;
	/* Integrated charge not yet applied to the estimate, in uA * ms */
	int64_t charge_residual;
	bool soc_initialized;
	bool integrating;
	bool relaxation_pending;
	bool relaxation_anchored;
#endif /* CHARGING_SOC_ESTIMATOR_ENABLED */
};

LOG_MODULE_REGISTER(composite_fg);

static int composite_fetch(const struct device *dev)
{
	int rc, rc2;

	rc = pm_device_runtime_get(dev);
	if (rc < 0) {
		return rc;
	}
	rc = sensor_sample_fetch(dev);
	/* Unconditionally release the PM constraint */
	rc2 = pm_device_runtime_put(dev);
	if (rc == 0) {
		rc = rc2;
	}
	return rc;
}

static int composite_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	const struct composite_config *config = dev->config;
	struct composite_data *data = dev->data;
	int rc = -ENOTSUP;

	if (data->primary_success) {
		/* Primary sensor sampled successfully */
		rc = sensor_channel_get(config->source_primary, chan, val);
	}
	if ((rc == -ENOTSUP) && config->source_secondary) {
		/* Primary sensor doesn't support the channel OR failed to sample.
		 * If the primary sensor failed to sample, the only API compliant way `get` can be
		 * called is if `source-secondary-fallback` exists and the secondary measurement
		 * succeeded.
		 */
		rc = sensor_channel_get(config->source_secondary, chan, val);
	}
	return rc;
}

#if CHARGING_SOC_ESTIMATOR_ENABLED

#define SOC_MILLIPERCENT_FULL           100000
/* Longer gaps between charging readings are not integrated */
#define SOC_INTEGRATION_MAX_INTERVAL_MS (5 * 60 * MSEC_PER_SEC)

/*
 * Update the stateful SoC estimate from an already resistance-corrected OCV estimate.
 *
 * With no history, corrected OCV provides the initial estimate. While positive charging
 * current exceeds the configured threshold, charge is integrated against the design
 * capacity and each reading restarts the pending post-charge relaxation. Only the interval
 * between consecutive charging readings is integrated (after a gap longer than
 * SOC_INTEGRATION_MAX_INTERVAL_MS, corrected OCV is used and integration restarts), and
 * sub-millipercent remainders are carried between readings. Once significant charging ends,
 * the estimate transitions linearly from its last integrated value toward the corrected OCV
 * estimate over the configured relaxation time, measured from the last charging reading.
 * All non-charging readings, including large discharges, advance this transition; their
 * current is not integrated. After relaxation completes, corrected OCV is used directly
 * until significant charging resumes. A charger FULL indication immediately calibrates the
 * estimate to 100%, and leaving FULL relaxes from 100% toward corrected OCV in the same way
 * as after charging.
 */
static uint8_t composite_charging_soc_update(const struct composite_config *config,
					     struct composite_data *data, int64_t ocv_soc,
					     int64_t current, bool current_valid,
					     enum charger_status charger_status)
{
	k_ticks_t now = k_uptime_ticks();
	int64_t interval_ms = k_ticks_to_ms_floor64(now - data->soc_update_time);
	bool charging = current_valid && (current > config->charging_current_threshold_microamp);

	if (charger_status == CHARGER_STATUS_FULL) {
		LOG_DBG("Charger full, calibrating to 100%%");
		data->estimated_soc_millipercent = SOC_MILLIPERCENT_FULL;
		data->soc_initialized = true;
		/* Terminal voltage is elevated after charge termination, relax once FULL ends */
		data->relaxation_pending = true;
		data->relaxation_anchored = false;
	} else if (charging) {
		if (!data->soc_initialized) {
			/* With no history, corrected OCV is the best available initial estimate. */
			LOG_DBG("Charging with no history, initialising from OCV");
			data->estimated_soc_millipercent = ocv_soc;
			data->soc_initialized = true;
		} else if (data->integrating && (interval_ms <= SOC_INTEGRATION_MAX_INTERVAL_MS)) {
			int64_t capacity_scale = 36LL * config->charge_capacity_microamp_hours;
			int64_t charge = data->charge_residual + current * interval_ms;

			/*
			 * Current is positive while charging. In millipercent:
			 *
			 *   delta = current_uA * interval_ms / (36 * capacity_uAh)
			 *
			 * The remainder is carried so that small steps are not truncated away.
			 */
			data->estimated_soc_millipercent += charge / capacity_scale;
			data->charge_residual = charge % capacity_scale;
			LOG_DBG("Integrated %lld uA over %lld ms: %+lld m%%", current, interval_ms,
				charge / capacity_scale);
		} else {
			/*
			 * First charging reading, or the previous one is too old for its current to
			 * represent the interval: integrate from now, not from the last reading.
			 */
			if (data->integrating) {
				/* Charge over the gap is unknown, fall back to corrected OCV */
				LOG_DBG("%lld ms since last charging reading, restarting "
					"integration from OCV",
					interval_ms);
				data->estimated_soc_millipercent = ocv_soc;
			} else {
				LOG_DBG("Charging started, integrating from now");
			}
			data->charge_residual = 0;
		}
		data->relaxation_pending = true;
		data->relaxation_anchored = false;
	} else if (data->relaxation_pending) {
		int64_t elapsed_ms;

		if (!data->relaxation_anchored) {
			/* Charging ended at the last charging or FULL reading, not at this one. */
			data->relaxation_start_time = data->soc_update_time;
			data->relaxation_start_soc_millipercent = data->estimated_soc_millipercent;
			data->relaxation_anchored = true;
			LOG_DBG("Charging ended, relaxing from %lld m%% toward OCV",
				data->relaxation_start_soc_millipercent);
		}

		elapsed_ms = k_ticks_to_ms_floor64(now - data->relaxation_start_time);
		if (elapsed_ms >= config->ocv_relaxation_time_ms) {
			LOG_DBG("Relaxation complete after %lld ms", elapsed_ms);
			data->estimated_soc_millipercent = ocv_soc;
			data->relaxation_pending = false;
			data->relaxation_anchored = false;
		} else {
			int64_t soc_delta = ocv_soc - data->relaxation_start_soc_millipercent;

			data->estimated_soc_millipercent =
				data->relaxation_start_soc_millipercent +
				(soc_delta * elapsed_ms) / config->ocv_relaxation_time_ms;
			LOG_DBG("Relaxing: %lld/%u ms", elapsed_ms, config->ocv_relaxation_time_ms);
		}
	} else {
		/* Outside significant charging, use corrected OCV and refresh the baseline. */
		data->estimated_soc_millipercent = ocv_soc;
		data->soc_initialized = true;
	}

	data->integrating = charging && (charger_status != CHARGER_STATUS_FULL);

	data->soc_update_time = now;

	data->estimated_soc_millipercent =
		CLAMP(data->estimated_soc_millipercent, 0, SOC_MILLIPERCENT_FULL);

	LOG_DBG("OCV: %6lld m%% Estimate: %6lld m%% Current: %8lld uA%s Status: %d", ocv_soc,
		data->estimated_soc_millipercent, current, current_valid ? "" : " (invalid)",
		charger_status);

	return data->estimated_soc_millipercent / 1000;
}

#endif /* CHARGING_SOC_ESTIMATOR_ENABLED */

static bool composite_charging_soc_estimator(const struct composite_config *config)
{
#if CHARGING_SOC_ESTIMATOR_ENABLED
	return config->charging_soc_estimator;
#else
	ARG_UNUSED(config);
	return false;
#endif /* CHARGING_SOC_ESTIMATOR_ENABLED */
}

static int composite_get_prop_state_of_charge(const struct device *dev,
					      union fuel_gauge_prop_val *val)
{
	const struct composite_config *config = dev->config;
	struct composite_data *data = dev->data;
	enum sensor_channel sensor_chan;
	struct sensor_value sensor_val;
	union charger_propval propval;
	enum charger_status charger_status = CHARGER_STATUS_UNKNOWN;
	int64_t current = 0;
	int64_t ocv_soc;
	int32_t voltage;
	uint8_t soc;
	bool current_valid = false;
	int rc;

	/* Attempt to query the state of charge directly */
	rc = composite_channel_get(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &sensor_val);
	if (rc == 0) {
#if CHARGING_SOC_ESTIMATOR_ENABLED
		/* Estimator history is not maintained across direct readings, reset integration */
		data->integrating = false;
#endif /* CHARGING_SOC_ESTIMATOR_ENABLED */
		val->absolute_state_of_charge_pct = sensor_val.val1;
		return 0;

	} else if (rc != -ENOTSUP) {
		/* Error other than an unsupported channel */
		return rc;
	}

	if (config->ocv_lookup_table[0] == -1) {
		/* No OCV table to support estimation */
		return -ENOTSUP;
	}

	/* Unsupported channel, compute from OCV table */
	sensor_chan = config->fg_channels ? SENSOR_CHAN_GAUGE_VOLTAGE : SENSOR_CHAN_VOLTAGE;
	rc = composite_channel_get(dev, sensor_chan, &sensor_val);
	if (rc != 0) {
		/* Failed to retrieve voltage, no way to compute SoC */
		return rc;
	}
	voltage = sensor_value_to_micro(&sensor_val);

	/* Read current when required for resistance correction or stateful estimation. */
	if ((config->factory_internal_resistance_micro_ohms > 0) ||
	    composite_charging_soc_estimator(config)) {
		sensor_chan =
			config->fg_channels ? SENSOR_CHAN_GAUGE_AVG_CURRENT : SENSOR_CHAN_CURRENT;
		rc = composite_channel_get(dev, sensor_chan, &sensor_val);
		if (rc == 0) {
			/* Failure to measure current is not a failure to measure SoC. */
			current_valid = true;
			current = sensor_value_to_micro(&sensor_val);

			if (config->factory_internal_resistance_micro_ohms > 0) {
				int32_t voltage_drop;

				/* Drop across the internal resistance.
				 * V,I,R in micro units, V=IR must be divided by 10^6.
				 */
				voltage_drop =
					(current * config->factory_internal_resistance_micro_ohms) /
					1000000;
				LOG_DBG("Raw Voltage: %7d uV Current: %8lld uA Drop: %5d uV",
					voltage, current, voltage_drop);
				/* Adjust voltage according to drop */
				voltage -= voltage_drop;
			}
		}
	}

	/* Convert voltage to state of charge */
	ocv_soc = battery_soc_lookup(config->ocv_lookup_table, voltage);
	soc = ocv_soc / 1000;

	/* Query charging state for estimate constraints and full-charge calibration. */
	if ((config->charger != NULL) && device_is_ready(config->charger) &&
	    (charger_get_prop(config->charger, CHARGER_PROP_STATUS, &propval) == 0)) {
		charger_status = propval.status;
	}

#if CHARGING_SOC_ESTIMATOR_ENABLED
	if (config->charging_soc_estimator) {
		soc = composite_charging_soc_update(config, data, ocv_soc, current, current_valid,
						    charger_status);
	}
#else
	ARG_UNUSED(data);
	ARG_UNUSED(current_valid);
#endif /* CHARGING_SOC_ESTIMATOR_ENABLED */

	/* Limit according to charging state */
	if (config->charger != NULL) {
		if ((soc >= 100) && (charger_status == CHARGER_STATUS_CHARGING)) {
			/* OCV says at least 100%, but charger reports still charging */
			LOG_DBG("Clamping SoC to 99%%");
			soc = 99;
		}
		if ((soc < 100) && (charger_status == CHARGER_STATUS_FULL)) {
			/* OCV reports less than 100%, but charger reports full */
			LOG_DBG("Forcing SoC to 100%%");
			soc = 100;
		}
	}

	val->relative_state_of_charge_pct = soc;
	return 0;
}

static int composite_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
			      union fuel_gauge_prop_val *val)
{
	const struct composite_config *config = dev->config;
	struct composite_data *data = dev->data;
	const k_ticks_t validity_ticks =
		k_ms_to_ticks_near64(CONFIG_FUEL_GAUGE_COMPOSITE_DATA_VALIDITY_MS);
	enum sensor_channel sensor_chan;
	struct sensor_value sensor_val;
	int rc = 0;

	/* Validate at build time that equivalent channel output fields still match */
	BUILD_ASSERT(sizeof(val->absolute_state_of_charge_pct) ==
		     sizeof(val->relative_state_of_charge_pct));
	BUILD_ASSERT(offsetof(union fuel_gauge_prop_val, absolute_state_of_charge_pct) ==
		     offsetof(union fuel_gauge_prop_val, relative_state_of_charge_pct));
	BUILD_ASSERT(sizeof(val->current_ua) == sizeof(val->avg_current_ua));
	BUILD_ASSERT(offsetof(union fuel_gauge_prop_val, current_ua) ==
		     offsetof(union fuel_gauge_prop_val, avg_current_ua));

	if (k_uptime_ticks() >= data->next_reading) {
		/* Trigger a sample on the input devices */
		rc = composite_fetch(config->source_primary);
		data->primary_success = rc == 0;
		/* Run the secondary source if it exists and either:
		 *     It is a fallback source and the primary source failed
		 *     It is not a fallback source and the primary succeeded
		 */
		if (config->source_secondary &&
		    (data->primary_success ^ config->secondary_fallback)) {
			rc = composite_fetch(config->source_secondary);
		}
		if (rc != 0) {
			return rc;
		}
		/* Update timestamp for next reading */
		data->next_reading = k_uptime_ticks() + validity_ticks;
	}

	switch (prop) {
	case FUEL_GAUGE_FULL_CHARGE_CAPACITY_UAH:
		rc = composite_channel_get(dev, SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY, &sensor_val);
		if (rc == 0) {
			val->full_charge_capacity_uah = sensor_value_to_micro(&sensor_val);
		} else if (rc == -ENOTSUP) {
			if (config->charge_capacity_microamp_hours == 0) {
				return -ENOTSUP;
			}
			val->full_charge_capacity_uah = config->charge_capacity_microamp_hours;
			rc = 0;
		}
		break;
	case FUEL_GAUGE_DESIGN_CAPACITY:
		rc = composite_channel_get(dev, SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY,
					   &sensor_val);
		if (rc == 0) {
			val->design_cap = sensor_val.val1;
		} else if (rc == -ENOTSUP) {
			if (config->charge_capacity_microamp_hours == 0) {
				return -ENOTSUP;
			}
			val->design_cap = config->charge_capacity_microamp_hours / 1000;
			rc = 0;
		}
		break;
	case FUEL_GAUGE_VOLTAGE_UV:
		sensor_chan = config->fg_channels ? SENSOR_CHAN_GAUGE_VOLTAGE : SENSOR_CHAN_VOLTAGE;
		rc = composite_channel_get(dev, sensor_chan, &sensor_val);
		val->voltage_uv = sensor_value_to_micro(&sensor_val);
		break;
	case FUEL_GAUGE_ABSOLUTE_STATE_OF_CHARGE_PCT:
	case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE_PCT:
		rc = composite_get_prop_state_of_charge(dev, val);
		break;
	case FUEL_GAUGE_CURRENT_UA:
	case FUEL_GAUGE_AVG_CURRENT_UA:
		sensor_chan =
			config->fg_channels ? SENSOR_CHAN_GAUGE_AVG_CURRENT : SENSOR_CHAN_CURRENT;
		rc = composite_channel_get(dev, sensor_chan, &sensor_val);
		val->current_ua = sensor_value_to_micro(&sensor_val);
		break;
	case FUEL_GAUGE_TEMPERATURE_DK:
		sensor_chan = config->fg_channels ? SENSOR_CHAN_GAUGE_TEMP : SENSOR_CHAN_DIE_TEMP;
		rc = composite_channel_get(dev, sensor_chan, &sensor_val);
		/* Output unit = 0.1K (10x increase + 273.0) */
		val->temperature_dk = sensor_value_to_deci(&sensor_val) + 2730;
		break;
	default:
		return -ENOTSUP;
	}

	return rc;
}

static int fuel_gauge_composite_init(const struct device *dev)
{
	const struct composite_config *config = dev->config;

	/* Validate sources are ready */
	if (!device_is_ready(config->source_primary)) {
		return -ENODEV;
	}
	if (config->source_secondary && !device_is_ready(config->source_secondary)) {
		return -ENODEV;
	}
	return 0;
}

static DEVICE_API(fuel_gauge, composite_api) = {
	.get_property = composite_get_prop,
};

#define BATTERY_RESISTANCE(inst) DT_INST_PROP_OR(inst, factory_internal_resistance_micro_ohms, 0)

#define CHARGING_SOC_ESTIMATOR(inst) DT_INST_PROP(inst, charging_soc_estimator)

#define COMPOSITE_INIT(inst)                                                                       \
	BUILD_ASSERT(BATTERY_RESISTANCE(inst) >= 0);                                               \
	BUILD_ASSERT(DT_INST_PROP(inst, charging_current_threshold_microamp) >= 0);                \
	BUILD_ASSERT(DT_INST_PROP(inst, ocv_relaxation_time_ms) >= 0);                             \
	BUILD_ASSERT(!CHARGING_SOC_ESTIMATOR(inst) ||                                              \
			     (DT_INST_PROP_OR(inst, charge_full_design_microamp_hours, 0) > 0),    \
		     "charging-soc-estimator requires charge-full-design-microamp-hours");         \
	static const struct composite_config composite_##inst##_config = {                         \
		.source_primary = DEVICE_DT_GET(DT_INST_PROP(inst, source_primary)),               \
		.source_secondary = DEVICE_DT_GET_OR_NULL(DT_INST_PROP(inst, source_secondary)),   \
		.charger = DEVICE_DT_GET_OR_NULL(DT_INST_PROP(inst, charger_status)),              \
		.ocv_lookup_table =                                                                \
			BATTERY_OCV_TABLE_DT_GET(DT_DRV_INST(inst), ocv_capacity_table_0),         \
		.charge_capacity_microamp_hours =                                                  \
			DT_INST_PROP_OR(inst, charge_full_design_microamp_hours, 0),               \
		.factory_internal_resistance_micro_ohms = BATTERY_RESISTANCE(inst),                \
		.chemistry = BATTERY_CHEMISTRY_DT_GET(inst),                                       \
		.secondary_fallback = DT_INST_PROP(inst, source_secondary_fallback),               \
		.fg_channels = DT_INST_PROP(inst, fuel_gauge_channels),                            \
		IF_ENABLED(CHARGING_SOC_ESTIMATOR_ENABLED,                                         \
			   (.charging_current_threshold_microamp =                                 \
				    DT_INST_PROP(inst, charging_current_threshold_microamp),       \
			    .ocv_relaxation_time_ms = DT_INST_PROP(inst, ocv_relaxation_time_ms),  \
			    .charging_soc_estimator = CHARGING_SOC_ESTIMATOR(inst), EMPTY))};      \
	static struct composite_data composite_##inst##_data;                                      \
	DEVICE_DT_INST_DEFINE(inst, fuel_gauge_composite_init, NULL, &composite_##inst##_data,     \
			      &composite_##inst##_config, POST_KERNEL,                             \
			      CONFIG_SENSOR_INIT_PRIORITY, &composite_api);

DT_INST_FOREACH_STATUS_OKAY(COMPOSITE_INIT)
