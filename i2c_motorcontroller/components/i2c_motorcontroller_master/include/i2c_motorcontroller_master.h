#pragma once

#include "esp_err.h"
#include "buoye_structs.h"
#include "serde_helper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

// Event group bits for motor controller operations
#define MOTCTRL_MASTER_PKG_SENT_BIT        (1 << 0)
#define MOTCTRL_MASTER_RESP_RECEIVED_BIT   (1 << 1)
#define MOTCTRL_MASTER_TIMEOUT_BIT         (1 << 2)
#define MOTCTRL_MASTER_ERROR_BIT           (1 << 3)

typedef enum {
    I2C_SLAVE_STATE_IDLE,
    I2C_SLAVE_STATE_BUSY,
    I2C_SLAVE_STATE_RESP_READY
} i2c_slave_state_t;

#define SLAVE_STATUS_IDLE        0x00
#define SLAVE_STATUS_BUSY        0x01
#define SLAVE_STATUS_RESP_READY  0x02

/**
 * @brief Initialize I2C master for motor controller communication
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_init(void);

/**
 * @brief Deinitialize I2C master
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_deinit(void);

/**
 * @brief Send package to motor controller slave
 * 
 * @param pkg Package to send
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_send_pkg(const motorcontroller_pkg_t *pkg, int timeout_sec);

/**
 * @brief Wait for response from motor controller slave
 * 
 * @param resp Response structure to fill
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_wait_response(motorcontroller_response_t *resp, int timeout_sec);

/**
 * @brief Wake up motor controller slave
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_wake_up_motorcontroller(void);

/**
 * @brief Calculate expected wait time for motor controller operation
 * 
 * @param state Operation state (LOWERING or RISING)
 * @param prev_estimated_cm_per_s Previous speed estimate
 * @param rising_timeout_percent Timeout percentage for rising
 * @return Expected wait time in seconds
 */
int motctrl_master_calculate_wait_time(state_t state, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent);

/**
 * @brief Check if motor controller is ready for new operation
 * 
 * @return true if ready, false otherwise
 */
bool i2c_motctrl_master_is_ready(void);

/**
 * @brief Cancel current operation
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_cancel_operation(void);

#ifdef __cplusplus
}
#endif