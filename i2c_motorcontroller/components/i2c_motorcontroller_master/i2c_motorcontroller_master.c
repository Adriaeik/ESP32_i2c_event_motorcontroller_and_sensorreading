#include "i2c_motorcontroller_master.h"
#include "i2c_master_manager.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "serde_helper.h"

static const char *TAG = "i2c_motctrl_master";

static EventGroupHandle_t s_motctrl_event_group = NULL;
static motorcontroller_response_t s_received_response;
static bool s_manager_initialized = false;
static bool s_operation_in_progress = false;

// Internal function prototypes
static esp_err_t execute_motor_ctrl_send_pkg(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms);
static esp_err_t execute_motor_ctrl_get_resp(motorcontroller_response_t *resp, uint32_t timeout_ms);

// Callback for I2C manager events
static void motctrl_master_event_callback(const i2c_mgr_event_data_t *event_data, void *user_data)
{
    if (event_data->device_addr != CONFIG_MOTCTRL_I2C_ADDR) {
        return; // Not our device
    }
    
    switch (event_data->event_type) {
        case I2C_MGR_EVT_OPERATION_COMPLETE:
            if (event_data->operation->op_type == I2C_OP_TYPE_CUSTOM) {
                ESP_LOGI(TAG, "Motor controller operation completed successfully");
                xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MASTER_PKG_SENT_BIT);
            }
            break;
            
        case I2C_MGR_EVT_OPERATION_ERROR:
            ESP_LOGE(TAG, "Motor controller operation failed: %s", esp_err_to_name(event_data->error_code));
            xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MASTER_ERROR_BIT);
            break;
            
        case I2C_MGR_EVT_OPERATION_TIMEOUT:
            ESP_LOGW(TAG, "Motor controller operation timeout");
            xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MASTER_TIMEOUT_BIT);
            break;
            
        default:
            break;
    }
}

esp_err_t i2c_motctrl_master_init(void)
{
    if (s_manager_initialized) {
        ESP_LOGW(TAG, "Motor controller master already initialized");
        return ESP_OK;
    }

    // Create event group
    s_motctrl_event_group = xEventGroupCreate();
    if (s_motctrl_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // Initialize I2C manager (this will auto-configure the motor controller device)
    esp_err_t ret = i2c_master_manager_init(motctrl_master_event_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C manager: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
        return ret;
    }

    s_manager_initialized = true;
    ESP_LOGI(TAG, "Motor controller master initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_motctrl_master_deinit(void)
{
    if (!s_manager_initialized) {
        return ESP_OK;
    }

    // Deinitialize I2C manager
    i2c_master_manager_deinit();

    // Clean up event group
    if (s_motctrl_event_group != NULL) {
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
    }

    s_manager_initialized = false;
    s_operation_in_progress = false;
    ESP_LOGI(TAG, "Motor controller master deinitialized");
    return ESP_OK;
}

esp_err_t i2c_motctrl_master_send_pkg(const motorcontroller_pkg_t *pkg, int timeout_sec)
{
    if (!s_manager_initialized) {
        ESP_LOGE(TAG, "Motor controller master not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_operation_in_progress) {
        ESP_LOGE(TAG, "Operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear event group
    xEventGroupClearBits(s_motctrl_event_group, 0xFF);
    s_operation_in_progress = true;

    // Execute motor controller package send
    esp_err_t ret = execute_motor_ctrl_send_pkg(pkg, timeout_sec * 1000);
    if (ret != ESP_OK) {
        s_operation_in_progress = false;
        ESP_LOGE(TAG, "Failed to send motor controller package: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for completion
    EventBits_t bits = xEventGroupWaitBits(
        s_motctrl_event_group,
        MOTCTRL_MASTER_PKG_SENT_BIT | MOTCTRL_MASTER_ERROR_BIT | MOTCTRL_MASTER_TIMEOUT_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    s_operation_in_progress = false;

    if (bits & MOTCTRL_MASTER_ERROR_BIT) {
        return ESP_FAIL;
    }

    if (bits & MOTCTRL_MASTER_TIMEOUT_BIT) {
        ESP_LOGE(TAG, "Package send timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (!(bits & MOTCTRL_MASTER_PKG_SENT_BIT)) {
        ESP_LOGE(TAG, "Unexpected timeout waiting for package send");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t i2c_motctrl_master_wait_response(motorcontroller_response_t *resp, int timeout_sec)
{
    if (!s_manager_initialized) {
        ESP_LOGE(TAG, "Motor controller master not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_operation_in_progress) {
        ESP_LOGE(TAG, "Operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear event group
    xEventGroupClearBits(s_motctrl_event_group, 0xFF);
    s_operation_in_progress = true;

    // Execute motor controller response wait
    esp_err_t ret = execute_motor_ctrl_get_resp(&s_received_response, timeout_sec * 1000);
    if (ret != ESP_OK) {
        s_operation_in_progress = false;
        ESP_LOGE(TAG, "Failed to get motor controller response: %s", esp_err_to_name(ret));
        return ret;
    }

    s_operation_in_progress = false;
    *resp = s_received_response;
    return ESP_OK;
}

esp_err_t i2c_motctrl_master_wake_up_motorcontroller(void)
{
#ifdef CONFIG_MOTCTRL_WAKEUP_PIN
    if (CONFIG_MOTCTRL_WAKEUP_PIN >= 0) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << CONFIG_MOTCTRL_WAKEUP_PIN),
            .pull_down_en = 0,
            .pull_up_en = 0,
        };
        gpio_config(&io_conf);
        
        gpio_set_level(CONFIG_MOTCTRL_WAKEUP_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(CONFIG_MOTCTRL_WAKEUP_PIN, 0);
        
        ESP_LOGI(TAG, "Wake-up signal sent to motor controller");
    }
#endif
    return ESP_OK;
}

int motctrl_master_calculate_wait_time(state_t state, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent)
{
    int base_time = 60; // Default 1 minute
    
    if (prev_estimated_cm_per_s > 0) {
        // Estimate based on previous speed (assuming end_depth is available)
        // For now, use a reasonable default since we don't have pkg context here
        int estimated_depth = 100; // cm - you might want to store this in a static variable
        base_time = (estimated_depth * 100) / prev_estimated_cm_per_s;
    }
    
    if (state == RISING) {
        // Apply rising timeout percentage
        base_time = (base_time * (100 + rising_timeout_percent)) / 100;
    }
    
    // Add 10% buffer
    base_time = (base_time * 110) / 100;
    
    // Minimum 30 seconds, maximum 30 minutes
    if (base_time < 30) base_time = 30;
    if (base_time > 1800) base_time = 1800;
    
    return base_time;
}

bool i2c_motctrl_master_is_ready(void)
{
    return (s_manager_initialized && !s_operation_in_progress);
}

esp_err_t i2c_motctrl_master_cancel_operation(void)
{
    if (!s_operation_in_progress) {
        return ESP_OK;
    }

    s_operation_in_progress = false;
    xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MASTER_ERROR_BIT);

    ESP_LOGI(TAG, "Operation cancelled");
    return ESP_OK;
}

// Internal implementation functions
static esp_err_t execute_motor_ctrl_send_pkg(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms)
{
    // Check if motor controller device is available
    const i2c_mgr_device_config_t *device_config = i2c_master_get_device_config(CONFIG_MOTCTRL_I2C_ADDR);
    if (device_config == NULL) {
        ESP_LOGE(TAG, "Motor controller device not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Get device handle
    i2c_master_dev_handle_t device_handle = i2c_master_get_device_handle(CONFIG_MOTCTRL_I2C_ADDR);
    if (device_handle == NULL) {
        ESP_LOGE(TAG, "Motor controller device handle not found");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Serialize motor controller package
    uint8_t tx_buffer[WIRE_PKG_SIZE + 10]; // Extra space for safety
    size_t tx_len;
    uint16_t crc = calculate_pkg_crc(pkg);
    ESP_LOGI(TAG, "calculated crc: 0x%04X", crc);
    
    esp_err_t ret = serialize_pkg(pkg, tx_buffer, &tx_len, crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize motor controller package");
        return ret;
    }
    
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, tx_buffer, tx_len, ESP_LOG_INFO);
    
    // Send package directly using device handle
    ret = i2c_master_transmit(device_handle, tx_buffer, tx_len, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit motor controller package: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Signal completion
    xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MASTER_PKG_SENT_BIT);
    
    ESP_LOGI(TAG, "Motor controller package sent successfully");
    return ESP_OK;
}

static esp_err_t execute_motor_ctrl_get_resp(motorcontroller_response_t *resp, uint32_t timeout_ms)
{
    // Check if motor controller device is available
    const i2c_mgr_device_config_t *device_config = i2c_master_get_device_config(CONFIG_MOTCTRL_I2C_ADDR);
    if (device_config == NULL) {
        ESP_LOGE(TAG, "Motor controller device not found");
        return ESP_ERR_NOT_FOUND;
    }

    // Get device handle
    i2c_master_dev_handle_t device_handle = i2c_master_get_device_handle(CONFIG_MOTCTRL_I2C_ADDR);
    if (device_handle == NULL) {
        ESP_LOGE(TAG, "Motor controller device handle not found");
        return ESP_ERR_NOT_FOUND;
    }

    const int poll_interval_ms = 100;
    const int max_retries = (timeout_ms + poll_interval_ms - 1) / poll_interval_ms;
    int retry_count = 0;
    uint8_t status_byte = 0;
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "Polling for motor controller response...");

    while (retry_count < max_retries) {
        // Read status byte from slave
        err = i2c_master_receive(device_handle, &status_byte, 1, poll_interval_ms);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, &status_byte, 1, ESP_LOG_INFO);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "Status read failed: %s (retry %d/%d)", 
                esp_err_to_name(err), retry_count+1, max_retries);
            } else {
                ESP_LOGD(TAG, "Status byte: 0x%02x", status_byte);
                
                // Handle response states
                if (status_byte == SLAVE_STATUS_RESP_READY) {
                    uint8_t response_buffer[WIRE_RESP_SIZE];
                    
                    // Read full response
                    // vTaskDelay(pdMS_TO_TICKS(4000));
                    err = i2c_master_receive(device_handle, response_buffer, sizeof(response_buffer), 1000);
                    
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Response read failed: %s", esp_err_to_name(err));
                        return err;
                    }

                    // Deserialize and validate response
                    uint16_t received_crc;
                    err = deserialize_resp(response_buffer, sizeof(response_buffer), resp, &received_crc);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Deserialization failed: %s", esp_err_to_name(err));
                        return err;
                    }
                    ESP_LOG_BUFFER_HEX_LEVEL(TAG, response_buffer, sizeof(response_buffer), ESP_LOG_INFO);

                    // Calculate CRC for received response
                    uint16_t calculated_crc = calculate_resp_crc(resp);
                    if (received_crc != calculated_crc) {
                        ESP_LOGE(TAG, "CRC mismatch: received 0x%04X vs calculated 0x%04X", 
                                received_crc, calculated_crc);
                        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
                        continue;
                        return ESP_ERR_INVALID_CRC;
                    }

                    ESP_LOGI(TAG, "Response received successfully");
                    return ESP_OK;
                }
            else if (status_byte == SLAVE_STATUS_BUSY) {
                ESP_LOGD(TAG, "Slave busy, waiting...");
            }
            else if (status_byte == SLAVE_STATUS_IDLE) {
                ESP_LOGD(TAG, "Slave idle, waiting for response...");
            }
            else {
                ESP_LOGW(TAG, "Unexpected status: 0x%02x", status_byte);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
        retry_count++;
    }

    if (retry_count >= max_retries) {
        ESP_LOGE(TAG, "Response polling timeout after %dms", timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}