#include "serde_helper.h"
#include "esp_crc.h"

// Use ESP-IDF's CRC16 implementation
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg) {
    return esp_crc16_le(UINT16_MAX, (const uint8_t*)pkg, sizeof(motorcontroller_pkg_t));
}

uint16_t calculate_resp_crc(const motorcontroller_response_t *resp) {
    return esp_crc16_le(UINT16_MAX, (const uint8_t*)resp, sizeof(motorcontroller_response_t));
}


esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len, uint16_t crc) {
    if (!pkg || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    
    pkg_wire_t wire_pkg = {
        .pkg = *pkg,
        .crc = crc
    };
    
    memcpy(buf, &wire_pkg, WIRE_PKG_SIZE);
    *out_len = WIRE_PKG_SIZE;
    return ESP_OK;
}

esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc) {
    if (!buf || !pkg || !crc) return ESP_ERR_INVALID_ARG;
    if (len < WIRE_PKG_SIZE) return ESP_ERR_INVALID_SIZE;
    
    pkg_wire_t wire_pkg;
    memcpy(&wire_pkg, buf, WIRE_PKG_SIZE);
    *pkg = wire_pkg.pkg;
    *crc = wire_pkg.crc;
    
    return ESP_OK;
}

esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len, uint16_t crc) {
    if (!resp || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    
    resp_wire_t wire_resp = {
        .resp = *resp,
        .crc = crc
    };
    
    memcpy(buf, &wire_resp, WIRE_RESP_SIZE);
    *out_len = WIRE_RESP_SIZE;
    return ESP_OK;
}

esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp, uint16_t *crc) {
    if (!buf || !resp || !crc) return ESP_ERR_INVALID_ARG;
    if (len < WIRE_RESP_SIZE) return ESP_ERR_INVALID_SIZE;
    
    resp_wire_t wire_resp;
    memcpy(&wire_resp, buf, WIRE_RESP_SIZE);
    *resp = wire_resp.resp;
    *crc = wire_resp.crc;
    
    return ESP_OK;
}