#pragma once

#include "esp_err.h"
#include "buoye_structs.h"

// === Init / Clear ===
void RTC_manager_clear_all(void);

// === SensorBatch ===
esp_err_t rtc_save_sensor_batch(const SensorBatch *batch);
esp_err_t rtc_load_sensor_batch(SensorBatch *batch);

// === HUB Config ===
esp_err_t rtc_save_hub_config(const sheared_config_t *config);
esp_err_t rtc_load_hub_config(sheared_config_t *config);

// === Buoye Config ===
esp_err_t rtc_save_buoye_config(const buoye_config_t *config);
esp_err_t rtc_load_buoye_config(buoye_config_t *config);

// === Sensor Config ===
esp_err_t rtc_save_poll_config(const poll_config_t *config, size_t static_points_count);
esp_err_t rtc_load_poll_config(poll_config_t *config_out, size_t *static_points_count_out);

// === Status ===
esp_err_t rtc_save_sheared_status(const sheared_status_t *status);
esp_err_t rtc_load_sheared_status(sheared_status_t *status);

// === Next Wakeup State ===
esp_err_t rtc_save_wakeup_state(state_t state);
esp_err_t rtc_load_wakeup_state(state_t *state);
