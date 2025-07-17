#include "RTC_manager.h"
#include <string.h>
#include <stdlib.h>
#include "log_thread.h"
#include "esp_sntp.h"

static const char *TAG = "RTC_manager";

/**
 * @note Vurder å gjere heile modulen trådtrygg med mutexa - evt ein felles mutex
 */

// === RTC-variablar ===
RTC_DATA_ATTR static SensorBatch        rtc_sensor_batch;
RTC_DATA_ATTR static sheared_config_t   rtc_hub_config;
RTC_DATA_ATTR static buoye_config_t     rtc_buoye_config;
RTC_DATA_ATTR static sheared_status_t   rtc_sheared_status;
RTC_DATA_ATTR static state_t rtc_next_wakeup_state;
RTC_DATA_ATTR static poll_config_t rtc_poll_config;
RTC_DATA_ATTR static size_t rtc_static_points_count;
// Dynamisk, fordi `mesure_points` er fleksibel:
RTC_DATA_ATTR static int rtc_static_points[64]; // juster storleik!
RTC_DATA_ATTR static motorcontroller_pkg_t rtc_motctrl_pkg_lowering;
RTC_DATA_ATTR static motorcontroller_pkg_t rtc_motctrl_pkg_rising;

// === Intern gyldigheitsindikatorar ===
RTC_DATA_ATTR static uint32_t rtc_magic_sensorbatch     = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_hub_config      = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_buoye_config    = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_sensor_config   = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_status          = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_next_wakeup = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_poll_config = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_motctrl_lowering = 0;
RTC_DATA_ATTR static uint32_t rtc_magic_motctrl_rising = 0;

#define MAGIC_SENSORBATCH       0xABCDEF01
#define MAGIC_HUB_CONFIG        0xABCDEF01
#define MAGIC_BUOYE_CONFIG      0xABCDEF03
#define MAGIC_SENSOR_CONFIG     0xABCDEF04
#define MAGIC_STATUS            0xABCDEF05
#define MAGIC_NEXT_WAKEUP_STATE 0xABCDEF06
#define MAGIC_POLL_CONFIG       0xABCDEF07
#define MAGIC_MOTCTRL_LOWERING  0xABCDEF08
#define MAGIC_MOTCTRL_RISING    0xABCDEF09

// === CLEAR ===
void RTC_manager_clear_all(void) {
    rtc_magic_sensorbatch = 0;
    rtc_magic_buoye_config = 0;
    rtc_magic_sensor_config = 0;
    rtc_magic_status = 0;
    rtc_magic_motctrl_lowering = 0;
    rtc_magic_motctrl_rising = 0;
}

// === SENSORBATCH ===
esp_err_t rtc_save_sensor_batch(const SensorBatch *batch) {
    rtc_sensor_batch = *batch;
    rtc_magic_sensorbatch = MAGIC_SENSORBATCH;
    return ESP_OK;
}

esp_err_t rtc_load_sensor_batch(SensorBatch *batch) {
    if (rtc_magic_sensorbatch != MAGIC_SENSORBATCH) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig sensorbatch i RTC");
        return ESP_FAIL;
    }
    *batch = rtc_sensor_batch;
    return ESP_OK;
}

// === HUB Config ===
esp_err_t rtc_save_hub_config(const sheared_config_t *config) {
    rtc_hub_config = *config;
    rtc_magic_hub_config = MAGIC_HUB_CONFIG;
    return ESP_OK;
}

esp_err_t rtc_load_hub_config(sheared_config_t *config) {
    if (rtc_magic_hub_config != MAGIC_HUB_CONFIG) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig buoye config i RTC");
        return ESP_FAIL;
    }
    *config = rtc_hub_config;
    return ESP_OK;
}

// === BUOYE CONFIG ===
esp_err_t rtc_save_buoye_config(const buoye_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "rtc_save: invalid config-ptr");
        return ESP_ERR_INVALID_ARG;
    }
    rtc_buoye_config = *config;
    rtc_magic_buoye_config = MAGIC_BUOYE_CONFIG;
    return ESP_OK;
}

esp_err_t rtc_load_buoye_config(buoye_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "rtc_load: invalid config-ptr");
        return ESP_ERR_INVALID_ARG;
    }
    if (rtc_magic_buoye_config != MAGIC_BUOYE_CONFIG) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig buoye config i RTC");
        return ESP_FAIL;
    }
    *config = rtc_buoye_config;
    return ESP_OK;
}

// === poll_config_t ===
esp_err_t rtc_save_poll_config(const poll_config_t *config, size_t static_points_count) {
    if (!config) return ESP_ERR_INVALID_ARG;

    if (static_points_count > 64) {
        ESP_LOGE(TAG, "For mange målepunkt for RTC! Max er 64.");
        return ESP_FAIL;
    }

    rtc_poll_config = *config;
    rtc_static_points_count = static_points_count;

    for (size_t i = 0; i < static_points_count; i++) {
        rtc_static_points[i] = config->static_points[i];
    }

    rtc_magic_poll_config = MAGIC_POLL_CONFIG;
    return ESP_OK;
}

esp_err_t rtc_load_poll_config(poll_config_t *config_out, size_t *static_points_count_out) {
    if (!config_out || !static_points_count_out) return ESP_ERR_INVALID_ARG;

    if (rtc_magic_poll_config != MAGIC_POLL_CONFIG) {
        ESP_LOGW(TAG, "Ingen gyldig poll_config i RTC");
        return ESP_FAIL;
    }

    *config_out = rtc_poll_config;

    size_t count = rtc_static_points_count;
    if (count > 20) count = 20; // Avgrens til struct-feltet

    for (size_t i = 0; i < count; i++) {
        config_out->static_points[i] = rtc_static_points[i];
    }

    *static_points_count_out = count;
    return ESP_OK;
}


// === STATUS ===
esp_err_t rtc_save_sheared_status(const sheared_status_t *status) {
    rtc_sheared_status = *status;
    rtc_magic_status = MAGIC_STATUS;
    return ESP_OK;
}

esp_err_t rtc_load_sheared_status(sheared_status_t *status) {
    if (rtc_magic_status != MAGIC_STATUS) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig sheared status i RTC");
        return ESP_FAIL;
    }
    *status = rtc_sheared_status;
    return ESP_OK;
}

esp_err_t rtc_save_wakeup_state(state_t state) {
    rtc_next_wakeup_state = state;
    rtc_magic_next_wakeup = MAGIC_NEXT_WAKEUP_STATE;
    return ESP_OK;
}

esp_err_t rtc_load_wakeup_state(state_t *state) {
    if (rtc_magic_next_wakeup != MAGIC_NEXT_WAKEUP_STATE) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig wakeup state i RTC");
        *state = INIT;
        return ESP_FAIL;
    }
    *state = rtc_next_wakeup_state;
    return ESP_OK;
}

esp_err_t rtc_save_motorcontroller_pkg_lowering(const motorcontroller_pkg_t *pkg) {
    if (!pkg) return ESP_ERR_INVALID_ARG;
    rtc_motctrl_pkg_lowering = *pkg;
    rtc_magic_motctrl_lowering = MAGIC_MOTCTRL_LOWERING;
    return ESP_OK;
}

esp_err_t rtc_load_motorcontroller_pkg_lowering(motorcontroller_pkg_t *pkg) {
    if (!pkg) return ESP_ERR_INVALID_ARG;
    if (rtc_magic_motctrl_lowering != MAGIC_MOTCTRL_LOWERING) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig lowering motorcontroller pkg i RTC");
        return ESP_FAIL;
    }
    *pkg = rtc_motctrl_pkg_lowering;
    return ESP_OK;
}

esp_err_t rtc_save_motorcontroller_pkg_rising(const motorcontroller_pkg_t *pkg) {
    if (!pkg) return ESP_ERR_INVALID_ARG;
    rtc_motctrl_pkg_rising = *pkg;
    rtc_magic_motctrl_rising = MAGIC_MOTCTRL_RISING;
    return ESP_OK;
}

esp_err_t rtc_load_motorcontroller_pkg_rising(motorcontroller_pkg_t *pkg) {
    if (!pkg) return ESP_ERR_INVALID_ARG;
    if (rtc_magic_motctrl_rising != MAGIC_MOTCTRL_RISING) {
        ESP_LOGW_THREAD(TAG, "Ingen gyldig rising motorcontroller pkg i RTC");
        return ESP_FAIL;
    }
    *pkg = rtc_motctrl_pkg_rising;
    return ESP_OK;
}