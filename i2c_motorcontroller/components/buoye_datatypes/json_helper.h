
#pragma once
#include "buoye_structs.h"
#include "esp_err.h"
#include "sdkconfig.h"

// Bygge om til at alle tar inn cahr og endrer den i staden for
// å retunere esp_err_t i staden slik at vi slepper å bruke free(json)
// sheared_config_t

esp_err_t sheared_config_to_json(const sheared_config_t *s, char *buf, size_t bufsize);
esp_err_t sheared_config_to_json_alloc(const sheared_config_t *s, char **out_json, size_t *out_len);
esp_err_t json_to_sheared_config(const char* json_str, sheared_config_t* s);

// buoye_config_t
esp_err_t buoye_config_to_json(const buoye_config_t *s, char *buf, size_t bufsize);
esp_err_t buoye_config_to_json_alloc(const buoye_config_t *s, char **out_json, size_t *out_len);
esp_err_t json_to_buoye_config(const char *json_str, buoye_config_t *s);

// sheared_status_t
esp_err_t sheared_status_to_json(const sheared_status_t *s, char *buf, size_t bufsize);
esp_err_t sheared_status_to_json_alloc(const sheared_status_t *s, char **out_json, size_t *out_len);
esp_err_t json_to_sheared_status(const char *json_str, sheared_status_t *s);

#if CONFIG_USE_TROLL_DATA

esp_err_t sensor_reading_to_json(const SensorReading *s, char *buf, size_t bufsize);
esp_err_t sensor_reading_to_json_alloc(const SensorReading *s, char **out_json, size_t *out_len);
esp_err_t json_to_sensor_reading(const char *json_str, SensorReading *s);

esp_err_t sensor_batch_to_json(const SensorBatch *s, char *buf, size_t bufsize);
esp_err_t sensor_batch_to_json_alloc(const SensorBatch *s, char **out_buf);
esp_err_t json_to_sensor_batch(const char *json_str, SensorBatch *s);

#endif