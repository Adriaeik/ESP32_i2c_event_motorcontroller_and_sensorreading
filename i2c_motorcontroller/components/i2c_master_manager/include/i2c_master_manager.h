#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C device types supported by the master
 */
typedef enum {
    I2C_DEVICE_TYPE_MOTOR_CONTROLLER,
    I2C_DEVICE_TYPE_LSM6DS032TR,
    I2C_DEVICE_TYPE_CUSTOM,
    I2C_DEVICE_TYPE_MAX
} i2c_device_type_t;

/**
 * @brief I2C device configuration (renamed to avoid conflicts)
 */
typedef struct {
    char name[32];                    // Device name/identifier
    uint8_t address;                  // I2C address
    i2c_device_type_t type;          // Device type
    uint32_t speed_hz;               // Device-specific speed
    bool enabled;                    // Is device enabled
    
    // Device-specific config
    union {
        struct {
            int wakeup_pin;          // Wake-up pin for motor controller
        } motor_ctrl;
        struct {
            uint16_t sample_rate;    // Sample rate for IMU
        } lsm6ds032tr;
        struct {
            void *custom_config;     // Pointer to custom config
        } custom;
    };
} i2c_mgr_device_config_t;  // Renamed to avoid conflicts

/**
 * @brief I2C operation types
 */
typedef enum {
    I2C_OP_TYPE_MOTOR_CTRL_SEND_PKG,
    I2C_OP_TYPE_MOTOR_CTRL_GET_RESP,
    I2C_OP_TYPE_SENSOR_READ,
    I2C_OP_TYPE_SENSOR_WRITE,
    I2C_OP_TYPE_CUSTOM
} i2c_operation_type_t;

/**
 * @brief I2C operation data
 */
typedef struct {
    i2c_operation_type_t op_type;
    uint8_t device_addr;
    uint8_t *data;
    size_t data_len;
    uint32_t timeout_ms;
    
    // Operation-specific data
    union {
        struct {
            void *pkg;      // Generic pointer to avoid including buoye_structs.h
            void *resp;     // Generic pointer to avoid including buoye_structs.h
        } motor_ctrl;
        struct {
            uint8_t reg_addr;
            uint8_t *read_buffer;
            size_t read_len;
        } sensor;
        struct {
            void *custom_data;
        } custom;
    };
} i2c_operation_t;

/**
 * @brief I2C master event types (renamed to avoid conflicts)
 */
typedef enum {
    I2C_MGR_EVT_OPERATION_COMPLETE,
    I2C_MGR_EVT_OPERATION_TIMEOUT,
    I2C_MGR_EVT_OPERATION_ERROR,
    I2C_MGR_EVT_DEVICE_READY
} i2c_mgr_event_type_t;

/**
 * @brief I2C master event data (renamed to avoid conflicts)
 */
typedef struct {
    i2c_mgr_event_type_t event_type;
    uint8_t device_addr;
    esp_err_t error_code;
    i2c_operation_t *operation;
    void *result_data;
    size_t result_len;
} i2c_mgr_event_data_t;

/**
 * @brief I2C master callback function type (renamed to avoid conflicts)
 */
typedef void (*i2c_mgr_callback_t)(const i2c_mgr_event_data_t *event_data, void *user_data);

/**
 * @brief I2C master context (renamed to avoid conflicts)
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t *device_handles;
    i2c_mgr_device_config_t *device_configs;
    uint8_t num_devices;
    uint8_t max_devices;
    
    QueueHandle_t operation_queue;
    TaskHandle_t manager_task_handle;
    EventGroupHandle_t event_group;
    
    i2c_mgr_callback_t callback;
    void *callback_user_data;
    
    // Current operation tracking
    i2c_operation_t *current_operation;
    TickType_t operation_start_time;
} i2c_mgr_ctx_t;

// Event group bits
#define I2C_MASTER_INIT_DONE_BIT           (1 << 0)
#define I2C_MASTER_OPERATION_DONE_BIT      (1 << 1)
#define I2C_MASTER_ERROR_BIT               (1 << 2)

/**
 * @brief Initialize I2C master manager
 * 
 * @param callback Callback function for events
 * @param user_data User data for callback
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_manager_init(i2c_mgr_callback_t callback, void *user_data);

/**
 * @brief Deinitialize I2C master manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_manager_deinit(void);

/**
 * @brief Add device to I2C master
 * 
 * @param config Device configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_add_device(const i2c_mgr_device_config_t *config);

/**
 * @brief Remove device from I2C master
 * 
 * @param device_addr Device address to remove
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_remove_device(uint8_t device_addr);

/**
 * @brief Queue an I2C operation
 * 
 * @param operation Operation to queue
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_queue_operation(const i2c_operation_t *operation);

/**
 * @brief Read from sensor register (convenience function)
 * 
 * @param device_addr Device address
 * @param reg_addr Register address
 * @param data Buffer for read data
 * @param len Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_sensor_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Write to sensor register (convenience function)
 * 
 * @param device_addr Device address
 * @param reg_addr Register address
 * @param data Data to write
 * @param len Number of bytes to write
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_sensor_write_reg(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Check if device is available on bus
 * 
 * @param device_addr Device address to check
 * @return true if device responds, false otherwise
 */
bool i2c_master_device_is_available(uint8_t device_addr);

/**
 * @brief Get device configuration by address
 * 
 * @param device_addr Device address
 * @return Device config pointer or NULL if not found
 */
const i2c_mgr_device_config_t* i2c_master_get_device_config(uint8_t device_addr);

#ifdef __cplusplus
}
#endif