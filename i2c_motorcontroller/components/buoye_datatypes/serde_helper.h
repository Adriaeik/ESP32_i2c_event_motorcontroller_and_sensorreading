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
#pragma pack(pop)

#define WIRE_PKG_SIZE   sizeof(pkg_wire_t)
#define WIRE_RESP_SIZE  sizeof(resp_wire_t)

// CRC calculation helpers
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg);
uint16_t calculate_resp_crc(const motorcontroller_response_t *resp);

esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len, uint16_t crc);
esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc);
esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len, uint16_t crc);
esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp, uint16_t *crc);