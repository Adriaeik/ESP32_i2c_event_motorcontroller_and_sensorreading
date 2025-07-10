#pragma once

#include "esp_err.h"
#include "driver/i2c_slave.h"
#include "buoye_structs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C slave event types
 */
typedef enum {
    I2C_SLAVE_EVT_PKG_RECEIVED,
    I2C_SLAVE_EVT_RESP_REQUESTED,
    I2C_SLAVE_EVT_RESP_SENT,
    I2C_SLAVE_EVT_ERROR
} i2c_slave_event_t;

/**
 * @brief I2C slave state
 */
typedef enum {
    I2C_SLAVE_STATE_IDLE,
    I2C_SLAVE_STATE_PKG_RECEIVED,
    I2C_SLAVE_STATE_WORKING,
    I2C_SLAVE_STATE_RESP_READY,
    I2C_SLAVE_STATE_RESP_SENT
} i2c_slave_state_t;

/**
 * @brief I2C slave event data
 */
typedef struct {
    i2c_slave_event_t event;
    esp_err_t error_code;
    union {
        struct {
            motorcontroller_pkg_t pkg;
            bool crc_valid;
        } pkg_data;
        struct {
            motorcontroller_response_t resp;
        } resp_data;
    };
} i2c_slave_event_data_t;

/**
 * @brief I2C slave context structure
 */
typedef struct {
    i2c_slave_dev_handle_t dev_handle;
    QueueHandle_t event_queue;
    TaskHandle_t slave_task_handle;
    EventGroupHandle_t operation_event_group;
    
    // Current state
    i2c_slave_state_t state;
    motorcontroller_pkg_t received_pkg;
    motorcontroller_response_t response_to_send;
    bool pkg_ready;
    bool resp_ready;
    
    // Buffers for communication
    uint8_t rx_buffer[256];
    uint8_t tx_buffer[256];
    size_t rx_len;
    size_t tx_len;
    
    // Status byte for master polling
    uint8_t status_byte;
} i2c_motctrl_slave_ctx_t;

// Event group bits
#define MOTCTRL_SLAVE_PKG_RECEIVED_BIT    (1 << 0)
#define MOTCTRL_SLAVE_RESP_READY_BIT      (1 << 1)
#define MOTCTRL_SLAVE_RESP_SENT_BIT       (1 << 2)
#define MOTCTRL_SLAVE_ERROR_BIT           (1 << 3)

// Status byte values
#define SLAVE_STATUS_IDLE               0x00
#define SLAVE_STATUS_READY              0x01
#define SLAVE_STATUS_WORKING            0x02
#define SLAVE_STATUS_RESP_READY         0x03
#define SLAVE_STATUS_ERROR              0xFF

/**
 * @brief Install and configure I2C slave driver
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_innstall_slave_driver_cnfig(void);

/**
 * @brief Deinitialize I2C slave
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_slave_deinit(void);

/**
 * @brief Wait for package from master
 * 
 * @param pkg Package structure to fill
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_slave_wait_pkg(motorcontroller_pkg_t *pkg, int timeout_sec);

/**
 * @brief Send response to master
 * 
 * @param resp Response to send
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_slave_send_response(const motorcontroller_response_t *resp, int timeout_sec);

/**
 * @brief Set slave working state (called when motor controller starts working)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_slave_set_working(void);

/**
 * @brief Set slave response ready state (called when motor controller finishes)
 * 
 * @param resp Response data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_slave_set_response_ready(const motorcontroller_response_t *resp);

/**
 * @brief Get current slave state
 * 
 * @return Current slave state
 */
i2c_slave_state_t i2c_motctrl_slave_get_state(void);

/**
 * @brief Check if slave is ready to receive new package
 * 
 * @return true if ready, false otherwise
 */
bool i2c_motctrl_slave_is_ready(void);

#ifdef __cplusplus
}
#endif