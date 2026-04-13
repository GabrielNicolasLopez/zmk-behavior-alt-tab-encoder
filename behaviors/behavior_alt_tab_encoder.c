/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_alt_tab_encoder

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/keymap.h>
#include <zmk/sensors.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_alt_tab_encoder_config {
    struct zmk_behavior_binding hold_binding;
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    uint32_t release_after_ms;
    int tap_ms;
};

struct behavior_alt_tab_encoder_data {
    struct sensor_value remainder[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int triggers[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];

    struct k_work_delayable release_work;
    struct zmk_behavior_binding_event last_event;

    const struct device *dev;
    bool active;
};

static void alt_tab_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct behavior_alt_tab_encoder_data *data =
        CONTAINER_OF(dwork, struct behavior_alt_tab_encoder_data, release_work);

    if (!data->active || data->dev == NULL) {
        return;
    }

    const struct behavior_alt_tab_encoder_config *cfg = data->dev->config;

    struct zmk_behavior_binding_event event = data->last_event;
    event.timestamp = k_uptime_get();

    LOG_DBG("Alt-Tab timeout elapsed, releasing hold binding");

    zmk_behavior_invoke_binding((struct zmk_behavior_binding *)&cfg->hold_binding, event, false);
    data->active = false;
}

static void alt_tab_start_if_needed(const struct device *dev,
                                    struct zmk_behavior_binding_event event) {
    struct behavior_alt_tab_encoder_data *data = dev->data;
    const struct behavior_alt_tab_encoder_config *cfg = dev->config;

    if (data->active) {
        return;
    }

    LOG_DBG("Starting Alt-Tab session");

    zmk_behavior_invoke_binding((struct zmk_behavior_binding *)&cfg->hold_binding, event, true);
    data->active = true;
}

static void alt_tab_refresh_timeout(const struct device *dev) {
    struct behavior_alt_tab_encoder_data *data = dev->data;
    const struct behavior_alt_tab_encoder_config *cfg = dev->config;

    k_work_reschedule(&data->release_work, K_MSEC(cfg->release_after_ms));
}

static int alt_tab_encoder_accept_data(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event,
                                       const struct zmk_sensor_config *sensor_config,
                                       size_t channel_data_size,
                                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_alt_tab_encoder_data *data = dev->data;

    const struct sensor_value value = channel_data[0].value;
    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    int triggers = 0;

    /* Compatibility path for older EC11 reporting in val2 only. */
    if (value.val1 == 0) {
        triggers = value.val2;
    } else {
        struct sensor_value remainder = data->remainder[sensor_index][event.layer];

        remainder.val1 += value.val1;
        remainder.val2 += value.val2;

        if (abs(remainder.val2) >= 1000000) {
            remainder.val1 += remainder.val2 / 1000000;
            remainder.val2 %= 1000000;
        }

        int trigger_degrees = 360 / sensor_config->triggers_per_rotation;
        triggers = remainder.val1 / trigger_degrees;
        remainder.val1 %= trigger_degrees;

        data->remainder[sensor_index][event.layer] = remainder;
    }

    data->triggers[sensor_index][event.layer] = triggers;

    LOG_DBG("sensor_index=%d layer=%d triggers=%d", sensor_index, event.layer, triggers);

    return 0;
}

static int alt_tab_encoder_process(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event,
                                   enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_alt_tab_encoder_config *cfg = dev->config;
    struct behavior_alt_tab_encoder_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->triggers[sensor_index][event.layer];
    if (triggers == 0) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    /* Force handling on central, matching stock sensor behavior patterns. */
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    data->last_event = event;

    alt_tab_start_if_needed(dev, event);

    struct zmk_behavior_binding triggered_binding;
    if (triggers > 0) {
        triggered_binding = cfg->cw_binding;
    } else {
        triggers = -triggers;
        triggered_binding = cfg->ccw_binding;
    }

    for (int i = 0; i < triggers; i++) {
        zmk_behavior_queue_add(&event, triggered_binding, true, cfg->tap_ms);
        zmk_behavior_queue_add(&event, triggered_binding, false, 0);
    }

    /* This is the key part you wanted:
     * any new rotation, in either direction, refreshes the same timeout.
     */
    alt_tab_refresh_timeout(dev);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_alt_tab_encoder_init(const struct device *dev) {
    struct behavior_alt_tab_encoder_data *data = dev->data;

    data->dev = dev;
    data->active = false;
    k_work_init_delayable(&data->release_work, alt_tab_release_work_handler);

    return 0;
}

static const struct behavior_driver_api behavior_alt_tab_encoder_driver_api = {
    .sensor_binding_accept_data = alt_tab_encoder_accept_data,
    .sensor_binding_process = alt_tab_encoder_process,
};

#define _TRANSFORM_ENTRY(idx, node)                                                                \
    {                                                                                              \
        .behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(node, bindings, idx)),              \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param1), (0),      \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param1))),                  \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param2), (0),      \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param2))),                  \
    }

#define ALT_TAB_ENCODER_INST(n)                                                                    \
    static struct behavior_alt_tab_encoder_config behavior_alt_tab_encoder_config_##n = {          \
        .hold_binding = _TRANSFORM_ENTRY(0, n),                                                    \
        .cw_binding = _TRANSFORM_ENTRY(1, n),                                                      \
        .ccw_binding = _TRANSFORM_ENTRY(2, n),                                                     \
        .tap_ms = DT_INST_PROP_OR(n, tap_ms, 5),                                                   \
        .release_after_ms = DT_INST_PROP_OR(n, release_after_ms, 1000),                           \
    };                                                                                             \
    static struct behavior_alt_tab_encoder_data behavior_alt_tab_encoder_data_##n = {};            \
    BEHAVIOR_DT_INST_DEFINE(n,                                                                     \
                            behavior_alt_tab_encoder_init,                                          \
                            NULL,                                                                   \
                            &behavior_alt_tab_encoder_data_##n,                                    \
                            &behavior_alt_tab_encoder_config_##n,                                  \
                            POST_KERNEL,                                                           \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_alt_tab_encoder_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ALT_TAB_ENCODER_INST)