#pragma once

#include "buoye_structs.h"
#include "esp_err.h"
#include <string.h>
#include "esp_crc.h"

#pragma pack(push,1)
typedef struct { 
    motorcontroller_pkg_t pkg;
    uint16_t crc;
} pkg_wire_t;

typedef struct { 
    motorcontroller_response_t resp;
    uint16_t crc;
} resp_wire_t;

// Unified response with status header
typedef struct {
    uint8_t status;          // Response status byte
    motorcontroller_response_t resp;
    uint16_t crc;            // CRC of status + resp
} unified_response_wire_t;

#pragma pack(pop)

#define WIRE_PKG_SIZE        sizeof(pkg_wire_t)
#define WIRE_RESP_SIZE       sizeof(resp_wire_t)
#define UNIFIED_RESP_SIZE    sizeof(unified_response_wire_t)

// CRC calculation helpers
uint16_t calculate_unified_crc(const unified_response_wire_t *resp);

/**
 * @brief Calculate CRC for motor controller package
 * @param pkg Package to calculate CRC for
 * @return Calculated CRC
 */
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg);

/**
 * @brief Calculate CRC for motor controller response
 * @param resp Response to calculate CRC for
 * @return Calculated CRC
 */
uint16_t calculate_resp_crc(const motorcontroller_response_t *resp);

/**
 * @brief Serialize motor controller package
 * @param pkg Package to serialize
 * @param buffer Output buffer
 * @param len Output length
 * @param crc CRC to include
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buffer, size_t *len, uint16_t crc);

/**
 * @brief Deserialize motor controller package
 * @param buffer Input buffer
 * @param len Input length
 * @param pkg Output package
 * @param crc Output CRC
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t deserialize_pkg(const uint8_t *buffer, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc);

/**
 * @brief Serialize motor controller response
 * @param resp Response to serialize
 * @param buffer Output buffer
 * @param len Output length
 * @param crc CRC to include
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buffer, size_t *len, uint16_t crc);

/**
 * @brief Deserialize motor controller response
 * @param buffer Input buffer
 * @param len Input length
 * @param resp Output response
 * @param crc Output CRC
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t deserialize_resp(const uint8_t *buffer, size_t len, motorcontroller_response_t *resp, uint16_t *crc);
