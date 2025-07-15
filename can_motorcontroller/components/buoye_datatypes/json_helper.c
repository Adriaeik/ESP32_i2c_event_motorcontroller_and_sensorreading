#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "cJSON.h"
#include "esp_log.h"
#include "log_thread.h"
#include "json_helper.h"

static cJSON *poll_config_to_json(const poll_config_t *pc) {
    if (!pc) return NULL;
    cJSON *obj = cJSON_CreateObject();
    if (!obj) return NULL;

    cJSON_AddNumberToObject(obj, "pt", pc->poll_type);
    cJSON_AddNumberToObject(obj, "to", (int64_t)pc->timeout);
    cJSON_AddNumberToObject(obj, "sd", pc->start_depth);
    cJSON_AddNumberToObject(obj, "ed", pc->end_depth);
    cJSON *mp = cJSON_CreateArray();
        for (int i = 0; i < MAX_POINTS; i++) {
            cJSON_AddItemToArray(mp, cJSON_CreateNumber((int)(pc->static_points[i])));
        }
    if (mp) cJSON_AddItemToObject(obj, "mp", mp);
    cJSON_AddNumberToObject(obj, "sc", pc->samples);
    cJSON_AddNumberToObject(obj, "al", pc->alpha_u8);
    cJSON_AddNumberToObject(obj, "sp", pc->static_poll_s);
    return obj;
}

static esp_err_t json_to_poll_config(cJSON *json, poll_config_t *pc) {
    if (!json || !pc) return ESP_ERR_INVALID_ARG;

    pc->poll_type = cJSON_GetObjectItem(json, "pt")->valueint;
    pc->timeout = (time_t)cJSON_GetObjectItem(json, "to")->valuedouble;
    pc->start_depth = cJSON_GetObjectItem(json, "sd")->valueint;
    pc->end_depth = cJSON_GetObjectItem(json, "ed")->valueint;

    cJSON *mp = cJSON_GetObjectItem(json, "mp");
    int len = cJSON_GetArraySize(mp);
    for (int i = 0; i < len && i < MAX_POINTS; i++) {
        pc->static_points[i] = cJSON_GetArrayItem(mp, i)->valueint;
    }

    pc->samples = cJSON_GetObjectItem(json, "sc")->valueint;
    pc->alpha_u8 = cJSON_GetObjectItem(json, "al")->valueint;
    pc->static_poll_s = cJSON_GetObjectItem(json, "sp")->valueint;
    return ESP_OK;
}


esp_err_t sheared_config_to_json(const sheared_config_t *s, char *buf, size_t bufsize) {
    if (!s || !buf || !bufsize) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "p", s->periodic_time_sec);
    cJSON_AddBoolToObject(root, "f", s->service_flag);
    cJSON_AddNumberToObject(root, "t", (int64_t)s->service_time);
    cJSON *pc = poll_config_to_json(&s->poll_config);
    if (!pc) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "pc", pc);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    if (strlen(tmp) >= bufsize) {
        free(tmp);
        return ESP_ERR_NO_MEM;
    }

    strcpy(buf, tmp);
    free(tmp);
    return ESP_OK;
}

/**
 * @brief Konverterer sheared_config_t til JSON med malloc-allokert streng.
 * @param s         Peikar til konfig-struktur
 * @param out_json  Peikar til peikar som blir sett til JSON-streng (må frigjerast)
 * @param out_len   Blir sett til lengda på JSON-strengen (utan nullbyte)
 * @return          ESP_OK ved suksess, elles feilkode
 */
esp_err_t sheared_config_to_json_alloc(const sheared_config_t *s, char **out_json, size_t *out_len) {
    if (!s || !out_json || !out_len) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "p", s->periodic_time_sec);
    cJSON_AddNumberToObject(root, "pl", s->periodic_time_lowering_sec);
    cJSON_AddNumberToObject(root, "pr", s->periodic_time_rising_sec);
    cJSON_AddBoolToObject(root, "f", s->service_flag);
    cJSON_AddNumberToObject(root, "t", (int64_t)s->service_time);
    cJSON *pc = poll_config_to_json(&s->poll_config);
    if (!pc) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "pc", pc);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    *out_json = tmp;
    *out_len = strlen(tmp);
    return ESP_OK;
}


esp_err_t json_to_sheared_config(const char *json_str, sheared_config_t *s) {
    if (!json_str || !s) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) return ESP_ERR_INVALID_STATE;

    s->periodic_time_sec = cJSON_GetObjectItem(root, "p")->valueint;
    s->periodic_time_lowering_sec = cJSON_GetObjectItem(root, "pl")->valueint;
    s->periodic_time_rising_sec = cJSON_GetObjectItem(root, "pr")->valueint;
    s->service_flag = cJSON_GetObjectItem(root, "f")->valueint;
    s->service_time = (time_t)cJSON_GetObjectItem(root, "t")->valuedouble;

    cJSON *pc = cJSON_GetObjectItem(root, "pc");
    if (!pc) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t res = json_to_poll_config(pc, &s->poll_config);
    cJSON_Delete(root);
    return res;
}


esp_err_t buoye_config_to_json(const buoye_config_t *s, char *buf, size_t bufsize) {
    if (!s || !buf || !bufsize) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "p", s->sheared_config.periodic_time_sec);
    cJSON_AddNumberToObject(root, "pl", s->sheared_config.periodic_time_lowering_sec);
    cJSON_AddNumberToObject(root, "pr", s->sheared_config.periodic_time_rising_sec);
    cJSON_AddBoolToObject(root,   "f", s->sheared_config.service_flag);
    cJSON_AddNumberToObject(root, "t", (int64_t)s->sheared_config.service_time);
    
    cJSON *pc = poll_config_to_json(&s->sheared_config.poll_config);
    if (!pc) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "pc", pc);

    cJSON_AddNumberToObject(root, "lt", (int64_t)s->current_time_to_start_work);
    cJSON_AddNumberToObject(root, "nt", (int64_t)s->time_to_start_next_work);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    if (strlen(tmp) >= bufsize) {
        free(tmp);
        return ESP_ERR_NO_MEM;
    }

    strcpy(buf, tmp);
    free(tmp);
    return ESP_OK;
}

/**
 * @brief Konverterer buoye_config_t til JSON med malloc-allokert streng.
 * @param s         Peikar til buøye-konfigurasjon
 * @param out_json  Peikar til peikar som blir sett til malloc-streng (må frigjerast)
 * @param out_len   Blir sett til lengda på JSON-strengen (utan nullbyte)
 * @return          ESP_OK ved suksess, elles feilkode
 * 
 * EXEMPLE:
buoye_config_t config = buoye_config_default();
char *json_str = NULL;
size_t json_len = 0;
if (buoye_config_to_json_alloc(&config, &json_str, &json_len) == ESP_OK) {
    printf("Konfig JSON: %s\n", json_str);
    free(json_str);
} 
 */
esp_err_t buoye_config_to_json_alloc(const buoye_config_t *s, char **out_json, size_t *out_len) {
    if (!s || !out_json || !out_len) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "p", s->sheared_config.periodic_time_sec);
    cJSON_AddNumberToObject(root, "pl", s->sheared_config.periodic_time_lowering_sec);
    cJSON_AddNumberToObject(root, "pr", s->sheared_config.periodic_time_rising_sec);
    cJSON_AddBoolToObject(root,   "f", s->sheared_config.service_flag);
    cJSON_AddNumberToObject(root, "t", (int64_t)s->sheared_config.service_time);
    
    cJSON *pc = poll_config_to_json(&s->sheared_config.poll_config);
    if (!pc) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "pc", pc);

    cJSON_AddNumberToObject(root, "lt", (int64_t)s->current_time_to_start_work);
    cJSON_AddNumberToObject(root, "nt", (int64_t)s->time_to_start_next_work);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    *out_json = tmp;
    *out_len = strlen(tmp);
    return ESP_OK;
}

esp_err_t json_to_buoye_config(const char *json_str, buoye_config_t *s) {
    if (!json_str || !s) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) return ESP_ERR_INVALID_STATE;

    s->sheared_config.periodic_time_sec = cJSON_GetObjectItem(root, "p")->valueint;
    s->sheared_config.service_flag      = cJSON_GetObjectItem(root, "f")->valueint;
    s->sheared_config.service_time      = (time_t)cJSON_GetObjectItem(root, "t")->valuedouble;

    cJSON *pc = cJSON_GetObjectItem(root, "pc");
    if (!pc) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = json_to_poll_config(pc, &s->sheared_config.poll_config);
    if (err != ESP_OK) {
        cJSON_Delete(root);
        return err;
    }

    s->current_time_to_start_work = (time_t)cJSON_GetObjectItem(root, "lt")->valuedouble;
    s->time_to_start_next_work    = (time_t)cJSON_GetObjectItem(root, "nt")->valuedouble;

    cJSON_Delete(root);
    return ESP_OK;
}


esp_err_t sheared_status_to_json(const sheared_status_t *s, char *buf, size_t bufsize) {
    if (!s || !buf || !bufsize) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "b", s->battery_level);
    cJSON_AddNumberToObject(root, "e", s->clock_error_s);
    cJSON_AddNumberToObject(root, "z", s->estimated_depth);
    cJSON_AddNumberToObject(root, "s", (int64_t)s->time_to_start_cycle);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    if (strlen(tmp) >= bufsize) {
        free(tmp);
        return ESP_ERR_NO_MEM;
    }

    strcpy(buf, tmp);
    free(tmp);
    return ESP_OK;
}
/**
 * @brief Konverterer sheared_status_t til JSON og returnerer malloc-allokert streng.
 * @param s         Peikar til struct med statusdata
 * @param out_json  Peikar til peikar som blir sett til malloc-streng (må frigjerast av kallande kode)
 * @param out_len   Blir sett til lengda på JSON-strengen (utan nullbyte)
 * @return          ESP_OK ved suksess, elles feil
 */
esp_err_t sheared_status_to_json_alloc(const sheared_status_t *s, char **out_json, size_t *out_len) {
    if (!s || !out_json || !out_len) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "b", s->battery_level);
    cJSON_AddNumberToObject(root, "e", s->clock_error_s);
    cJSON_AddNumberToObject(root, "z", s->estimated_depth);
    cJSON_AddNumberToObject(root, "s", (int64_t)s->time_to_start_cycle);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!tmp) return ESP_ERR_NO_MEM;

    *out_json = tmp;
    *out_len = strlen(tmp);
    return ESP_OK;
}


esp_err_t json_to_sheared_status(const char *json_str, sheared_status_t *s) {
    if (!json_str || !s) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) return ESP_ERR_INVALID_STATE;

    s->battery_level = cJSON_GetObjectItem(root, "b")->valueint;
    s->clock_error_s = cJSON_GetObjectItem(root, "e")->valueint;
    s->estimated_depth = cJSON_GetObjectItem(root, "z")->valueint;
    s->time_to_start_cycle = (time_t)cJSON_GetObjectItem(root, "s")->valuedouble;

    cJSON_Delete(root);
    return ESP_OK;
}

#if CONFIG_USE_TROLL_DATA

// SensorReading serialization
esp_err_t sensor_reading_to_json(const SensorReading *s, char *buf, size_t bufsize) {
    if (!s || !buf || !bufsize) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "id", s->sensor_id);
    cJSON_AddNumberToObject(root, "v", s->value);
    cJSON_AddNumberToObject(root, "q", s->quality);
    cJSON_AddNumberToObject(root, "u", s->unit);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    size_t needed = strlen(tmp) + 1;
    if (needed > bufsize) {
        free(tmp);
        return ESP_ERR_NO_MEM;
    }

    memcpy(buf, tmp, needed);
    free(tmp);
    return ESP_OK;
}
/**
 * @brief Convert a SensorReading to a JSON string (allocated on the heap).
 * Caller must free the returned string.
 */
esp_err_t sensor_reading_to_json_alloc(const SensorReading *s, char **out_json, size_t *out_len) {
    if (!s || !out_json || !out_len) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "id", s->sensor_id);
    cJSON_AddNumberToObject(root, "v", s->value);
    cJSON_AddNumberToObject(root, "q", s->quality);
    cJSON_AddNumberToObject(root, "u", s->unit);

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    *out_json = tmp;
    *out_len = strlen(tmp);
    return ESP_OK;
}


// SensorReading deserialization
esp_err_t json_to_sensor_reading(const char *json_str, SensorReading *s) {
    if (!json_str || !s) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) return ESP_ERR_INVALID_STATE;

    cJSON *item;
    if (!(item = cJSON_GetObjectItem(root, "id")) || !cJSON_IsNumber(item)) goto invalid;
    s->sensor_id = item->valueint;

    if (!(item = cJSON_GetObjectItem(root, "v")) || !cJSON_IsNumber(item)) goto invalid;
    s->value = item->valuedouble;

    if (!(item = cJSON_GetObjectItem(root, "q")) || !cJSON_IsNumber(item)) goto invalid;
    s->quality = item->valueint;

    if (!(item = cJSON_GetObjectItem(root, "u")) || !cJSON_IsNumber(item)) goto invalid;
    s->unit = item->valueint;

    cJSON_Delete(root);
    return ESP_OK;

invalid:
    cJSON_Delete(root);
    return ESP_ERR_INVALID_STATE;
}

// SensorBatch serialization
esp_err_t sensor_batch_to_json(const SensorBatch *s, char *buf, size_t bufsize) {
    if (!s || !buf || !bufsize) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "st", s->STATE);
    cJSON_AddNumberToObject(root, "c", s->count);
    cJSON_AddNumberToObject(root, "ts", (int64_t)s->timestamp);
    cJSON_AddNumberToObject(root, "d", s->depth);

    cJSON *readings = cJSON_CreateArray();
    if (!readings) goto no_mem;
    cJSON_AddItemToObject(root, "r", readings);

    for (uint16_t i = 0; i < s->count; i++) {
        cJSON *reading = cJSON_CreateObject();
        if (!reading) goto no_mem;
        
        cJSON_AddNumberToObject(reading, "id", s->readings[i].sensor_id);
        cJSON_AddNumberToObject(reading, "v", s->readings[i].value);
        cJSON_AddNumberToObject(reading, "q", s->readings[i].quality);
        cJSON_AddNumberToObject(reading, "u", s->readings[i].unit);
        
        cJSON_AddItemToArray(readings, reading);
    }

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    size_t needed = strlen(tmp) + 1;
    if (needed > bufsize) {
        free(tmp);
        return ESP_ERR_NO_MEM;
    }

    memcpy(buf, tmp, needed);
    free(tmp);
    return ESP_OK;

no_mem:
    cJSON_Delete(root);
    return ESP_ERR_NO_MEM;
}

// Bruker heap i staden: 
esp_err_t sensor_batch_to_json_alloc(const SensorBatch *s, char **out_buf) {
    if (!s || !out_buf) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "st", s->STATE);
    cJSON_AddNumberToObject(root, "c", s->count);
    cJSON_AddNumberToObject(root, "ts", (int64_t)s->timestamp);
    cJSON_AddNumberToObject(root, "d", s->depth);

    cJSON *readings = cJSON_CreateArray();
    if (!readings) goto no_mem;
    cJSON_AddItemToObject(root, "r", readings);

    for (uint16_t i = 0; i < s->count; i++) {
        cJSON *reading = cJSON_CreateObject();
        if (!reading) goto no_mem;
        
        cJSON_AddNumberToObject(reading, "id", s->readings[i].sensor_id);
        cJSON_AddNumberToObject(reading, "v", s->readings[i].value);
        cJSON_AddNumberToObject(reading, "q", s->readings[i].quality);
        cJSON_AddNumberToObject(reading, "u", s->readings[i].unit);
        
        cJSON_AddItemToArray(readings, reading);
    }

    char *tmp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!tmp) return ESP_ERR_NO_MEM;

    *out_buf = tmp;
    return ESP_OK;

no_mem:
    cJSON_Delete(root);
    return ESP_ERR_NO_MEM;
}


// SensorBatch deserialization
esp_err_t json_to_sensor_batch(const char *json_str, SensorBatch *s) {
    if (!json_str || !s) return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) return ESP_ERR_INVALID_STATE;

    cJSON *item;
    if (!(item = cJSON_GetObjectItem(root, "st")) || !cJSON_IsNumber(item)) goto invalid;
    s->STATE = item->valueint;

    if (!(item = cJSON_GetObjectItem(root, "c")) || !cJSON_IsNumber(item)) goto invalid;
    s->count = item->valueint;

    if (s->count > SENSOR_BATCH_MAX_COUNT) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!(item = cJSON_GetObjectItem(root, "ts")) || !cJSON_IsNumber(item)) goto invalid;
    s->timestamp = (time_t)item->valuedouble;

    if (!(item = cJSON_GetObjectItem(root, "d")) || !cJSON_IsNumber(item)) goto invalid;
    s->depth = item->valuedouble;

    cJSON *readings = cJSON_GetObjectItem(root, "r");
    if (!readings || !cJSON_IsArray(readings) || cJSON_GetArraySize(readings) < s->count) {
        goto invalid;
    }

    for (uint16_t i = 0; i < s->count; i++) {
        cJSON *elem = cJSON_GetArrayItem(readings, i);
        if (!elem) goto invalid;

        if (!(item = cJSON_GetObjectItem(elem, "id")) || !cJSON_IsNumber(item)) goto invalid;
        s->readings[i].sensor_id = item->valueint;

        if (!(item = cJSON_GetObjectItem(elem, "v")) || !cJSON_IsNumber(item)) goto invalid;
        s->readings[i].value = item->valuedouble;

        if (!(item = cJSON_GetObjectItem(elem, "q")) || !cJSON_IsNumber(item)) goto invalid;
        s->readings[i].quality = item->valueint;

        if (!(item = cJSON_GetObjectItem(elem, "u")) || !cJSON_IsNumber(item)) goto invalid;
        s->readings[i].unit = item->valueint;
    }

    cJSON_Delete(root);
    return ESP_OK;

invalid:
    cJSON_Delete(root);
    return ESP_ERR_INVALID_STATE;
}


#endif