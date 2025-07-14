#include "i2c_master_manager.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "i2c_master_mgr";

static i2c_mgr_ctx_t s_master_ctx = {0};

// Task for managing I2C operations
static void i2c_master_manager_task(void *pvParameters);

// Internal helper functions
static esp_err_t i2c_master_execute_operation(i2c_operation_t *operation);
static esp_err_t i2c_master_auto_configure_devices(void);
static esp_err_t i2c_master_create_device_handle(const i2c_mgr_device_config_t *config);

esp_err_t i2c_master_manager_init(i2c_mgr_callback_t callback, void *user_data)
{
    if (s_master_ctx.bus_handle != NULL) {
        ESP_LOGW(TAG, "Master manager already initialized");
        return ESP_OK;
    }

    // Initialize master context
    memset(&s_master_ctx, 0, sizeof(s_master_ctx));
    s_master_ctx.callback = callback;
    s_master_ctx.callback_user_data = user_data;
    s_master_ctx.max_devices = 8; // Maximum number of devices

    // Allocate device arrays
    s_master_ctx.device_handles = calloc(s_master_ctx.max_devices, sizeof(i2c_master_dev_handle_t));
    s_master_ctx.device_configs = calloc(s_master_ctx.max_devices, sizeof(i2c_mgr_device_config_t));
    
    if (!s_master_ctx.device_handles || !s_master_ctx.device_configs) {
        ESP_LOGE(TAG, "Failed to allocate device arrays");
        return ESP_ERR_NO_MEM;
    }

    // Initialize I2C master bus - FIXED configuration
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_I2C_MASTER_I2C_NUM,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_GPIO,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  // FIXED: Use external pull-ups
        .trans_queue_depth = 10,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &s_master_ctx.bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Create operation queue
    s_master_ctx.operation_queue = xQueueCreate(16, sizeof(i2c_operation_t));
    if (s_master_ctx.operation_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create operation queue");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // Create event group
    s_master_ctx.event_group = xEventGroupCreate();
    if (s_master_ctx.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // Auto-configure devices from Kconfig
    ret = i2c_master_auto_configure_devices();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to auto-configure devices");
        goto cleanup;
    }

    // Create manager task
    BaseType_t task_ret = xTaskCreate(
        i2c_master_manager_task,
        "i2c_mgr",
        4096,
        NULL,
        6,
        &s_master_ctx.manager_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create manager task");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    xEventGroupSetBits(s_master_ctx.event_group, I2C_MASTER_INIT_DONE_BIT);
    ESP_LOGI(TAG, "I2C master manager initialized with %d devices", s_master_ctx.num_devices);
    return ESP_OK;

cleanup:
    i2c_master_manager_deinit();
    return ret;
}

esp_err_t i2c_master_manager_deinit(void)
{
    // Stop manager task
    if (s_master_ctx.manager_task_handle != NULL) {
        vTaskDelete(s_master_ctx.manager_task_handle);
        s_master_ctx.manager_task_handle = NULL;
    }

    // Remove all devices
    for (int i = 0; i < s_master_ctx.num_devices; i++) {
        if (s_master_ctx.device_handles[i] != NULL) {
            i2c_master_bus_rm_device(s_master_ctx.device_handles[i]);
        }
    }

    // Clean up resources
    if (s_master_ctx.event_group != NULL) {
        vEventGroupDelete(s_master_ctx.event_group);
        s_master_ctx.event_group = NULL;
    }

    if (s_master_ctx.operation_queue != NULL) {
        vQueueDelete(s_master_ctx.operation_queue);
        s_master_ctx.operation_queue = NULL;
    }

    if (s_master_ctx.bus_handle != NULL) {
        i2c_del_master_bus(s_master_ctx.bus_handle);
        s_master_ctx.bus_handle = NULL;
    }

    if (s_master_ctx.device_handles) {
        free(s_master_ctx.device_handles);
        s_master_ctx.device_handles = NULL;
    }

    if (s_master_ctx.device_configs) {
        free(s_master_ctx.device_configs);
        s_master_ctx.device_configs = NULL;
    }

    ESP_LOGI(TAG, "I2C master manager deinitialized");
    return ESP_OK;
}

esp_err_t i2c_master_add_device(const i2c_mgr_device_config_t *config)
{
    if (s_master_ctx.num_devices >= s_master_ctx.max_devices) {
        ESP_LOGE(TAG, "Maximum number of devices reached");
        return ESP_ERR_NO_MEM;
    }

    // Check if device already exists
    for (int i = 0; i < s_master_ctx.num_devices; i++) {
        if (s_master_ctx.device_configs[i].address == config->address) {
            ESP_LOGW(TAG, "Device with address 0x%02X already exists", config->address);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Copy configuration
    s_master_ctx.device_configs[s_master_ctx.num_devices] = *config;

    // Create device handle
    esp_err_t ret = i2c_master_create_device_handle(&s_master_ctx.device_configs[s_master_ctx.num_devices]);
    if (ret != ESP_OK) {
        return ret;
    }

    s_master_ctx.num_devices++;
    ESP_LOGI(TAG, "Added device '%s' at address 0x%02X", config->name, config->address);
    return ESP_OK;
}

esp_err_t i2c_master_queue_operation(const i2c_operation_t *operation)
{
    if (s_master_ctx.operation_queue == NULL) {
        ESP_LOGE(TAG, "Manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(s_master_ctx.operation_queue, operation, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue operation");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t i2c_master_sensor_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len, uint32_t timeout_ms)
{
    i2c_operation_t operation = {
        .op_type = I2C_OP_TYPE_SENSOR_READ,
        .device_addr = device_addr,
        .timeout_ms = timeout_ms,
        .sensor.reg_addr = reg_addr,
        .sensor.read_buffer = data,
        .sensor.read_len = len
    };

    return i2c_master_queue_operation(&operation);
}

esp_err_t i2c_master_motor_ctrl_wake_up(void)
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

// Internal helper functions
static esp_err_t i2c_master_auto_configure_devices(void)
{
    esp_err_t ret = ESP_OK;

    // Add motor controller device (CRITICAL FIX)
    i2c_mgr_device_config_t motctrl_config = {
        .name = "Motor Controller",
        .address = CONFIG_MOTCTRL_I2C_ADDR,  // Should match slave address
        .type = I2C_DEVICE_TYPE_MOTOR_CONTROLLER,
        .speed_hz = CONFIG_I2C_MASTER_CLK_SPEED,
        .enabled = true
    };

    ret = i2c_master_add_device(&motctrl_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add motor controller device");
        return ret;
    }

#ifdef CONFIG_LSM6DS032TR_DEVICE_ENABLED
    // Add LSM6DS032TR sensor
    i2c_mgr_device_config_t imu_config = {
        .name = "LSM6DS032TR",
        .address = CONFIG_LSM6DS032TR_I2C_ADDR,
        .type = I2C_DEVICE_TYPE_LSM6DS032TR,
        .speed_hz = CONFIG_I2C_MASTER_CLK_SPEED,
        .enabled = true,
        .lsm6ds032tr.sample_rate = CONFIG_LSM6DS032TR_SAMPLE_RATE
    };

    ret = i2c_master_add_device(&imu_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add LSM6DS032TR device");
        return ret;
    }
#endif

#ifdef CONFIG_CUSTOM_DEVICE_1_ENABLED
    // Add custom device 1
    i2c_mgr_device_config_t custom1_config = {
        .name = CONFIG_CUSTOM_DEVICE_1_NAME,
        .address = CONFIG_CUSTOM_DEVICE_1_ADDR,
        .type = I2C_DEVICE_TYPE_CUSTOM,
        .speed_hz = CONFIG_I2C_MASTER_CLK_SPEED,
        .enabled = true
    };

    ret = i2c_master_add_device(&custom1_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add custom device 1");
        return ret;
    }
#endif

#ifdef CONFIG_CUSTOM_DEVICE_2_ENABLED
    // Add custom device 2
    i2c_mgr_device_config_t custom2_config = {
        .name = CONFIG_CUSTOM_DEVICE_2_NAME,
        .address = CONFIG_CUSTOM_DEVICE_2_ADDR,
        .type = I2C_DEVICE_TYPE_CUSTOM,
        .speed_hz = CONFIG_I2C_MASTER_CLK_SPEED,
        .enabled = true
    };

    ret = i2c_master_add_device(&custom2_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add custom device 2");
        return ret;
    }
#endif

    return ESP_OK;
}

static esp_err_t i2c_master_create_device_handle(const i2c_mgr_device_config_t *config)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->address,
        .scl_speed_hz = config->speed_hz,
    };
    
    ESP_LOGD(TAG, "Creating device handle for %s", config->name);
    esp_err_t ret = i2c_master_bus_add_device(
        s_master_ctx.bus_handle, 
        &dev_cfg, 
        &s_master_ctx.device_handles[s_master_ctx.num_devices]
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device handle for %s: %s", config->name, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Created device handle for %s at 0x%02X", 
                config->name, config->address);
    }

    return ret;
}

static void i2c_master_manager_task(void *pvParameters)
{
    i2c_operation_t operation;
    ESP_LOGI(TAG, "I2C master manager task started");

    while (true) {
        if (xQueueReceive(s_master_ctx.operation_queue, &operation, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Executing operation type %d for device 0x%02X", operation.op_type, operation.device_addr);
            
            s_master_ctx.current_operation = &operation;
            s_master_ctx.operation_start_time = xTaskGetTickCount();
            
            esp_err_t result = i2c_master_execute_operation(&operation);
            
            // Notify callback if present
            if (s_master_ctx.callback) {
                i2c_mgr_event_data_t event_data = {
                    .event_type = (result == ESP_OK) ? I2C_MGR_EVT_OPERATION_COMPLETE : I2C_MGR_EVT_OPERATION_ERROR,
                    .device_addr = operation.device_addr,
                    .error_code = result,
                    .operation = &operation,
                    .result_data = operation.data,
                    .result_len = operation.data_len
                };
                
                s_master_ctx.callback(&event_data, s_master_ctx.callback_user_data);
            }
            
            s_master_ctx.current_operation = NULL;
            
            if (result == ESP_OK) {
                xEventGroupSetBits(s_master_ctx.event_group, I2C_MASTER_OPERATION_DONE_BIT);
            } else {
                xEventGroupSetBits(s_master_ctx.event_group, I2C_MASTER_ERROR_BIT);
            }
        }
    }
}

static esp_err_t i2c_master_execute_operation(i2c_operation_t *operation)
{
    // Find device handle
    i2c_master_dev_handle_t device_handle = NULL;
    for (int i = 0; i < s_master_ctx.num_devices; i++) {
        if (s_master_ctx.device_configs[i].address == operation->device_addr) {
            device_handle = s_master_ctx.device_handles[i];
            break;
        }
    }
    
    if (device_handle == NULL) {
        ESP_LOGE(TAG, "Device handle not found for address 0x%02X", operation->device_addr);
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t ret = ESP_OK;
    
    switch (operation->op_type) {
        case I2C_OP_TYPE_MOTOR_CTRL_SEND_PKG:
        case I2C_OP_TYPE_MOTOR_CTRL_GET_RESP:
        {
            ESP_LOGW(TAG, "Motor controller operations should be handled by wrapper component");
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
        }
        
        case I2C_OP_TYPE_SENSOR_READ:
        {
            // Read from sensor register
            ret = i2c_master_transmit_receive(
                device_handle,
                &operation->sensor.reg_addr, 1,
                operation->sensor.read_buffer, operation->sensor.read_len,
                pdMS_TO_TICKS(operation->timeout_ms)
            );
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read from sensor register 0x%02X: %s", 
                        operation->sensor.reg_addr, esp_err_to_name(ret));
            } else {
                ESP_LOGD(TAG, "Read %d bytes from sensor register 0x%02X", 
                        operation->sensor.read_len, operation->sensor.reg_addr);
            }
            break;
        }
        
        case I2C_OP_TYPE_SENSOR_WRITE:
        {
            // Write to sensor register
            uint8_t write_buffer[operation->data_len + 1];
            write_buffer[0] = operation->sensor.reg_addr;
            memcpy(&write_buffer[1], operation->data, operation->data_len);
            
            ret = i2c_master_transmit(device_handle, write_buffer, operation->data_len + 1, 
                                     pdMS_TO_TICKS(operation->timeout_ms));
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write to sensor register 0x%02X: %s", 
                        operation->sensor.reg_addr, esp_err_to_name(ret));
            } else {
                ESP_LOGD(TAG, "Wrote %d bytes to sensor register 0x%02X", 
                        operation->data_len, operation->sensor.reg_addr);
            }
            break;
        }
        
        case I2C_OP_TYPE_CUSTOM:
        {
            // Custom operation - transmit data if provided
            if (operation->data && operation->data_len > 0) {
                ESP_LOGD(TAG, "Transmitting %d bytes to device 0x%02X", operation->data_len, operation->device_addr);
                ret = i2c_master_transmit(device_handle, operation->data, operation->data_len, 
                                         pdMS_TO_TICKS(operation->timeout_ms));
            } else {
                // FIXED: For device probing with no data, just try to address the device
                ESP_LOGD(TAG, "Probing device 0x%02X", operation->device_addr);
                uint8_t dummy = 0;
                ret = i2c_master_transmit(device_handle, &dummy, 0, pdMS_TO_TICKS(operation->timeout_ms));
            }
            break;
        }
        
        default:
            ESP_LOGE(TAG, "Unknown operation type: %d", operation->op_type);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }
    
    return ret;
}

esp_err_t i2c_master_remove_device(uint8_t device_addr)
{
    for (int i = 0; i < s_master_ctx.num_devices; i++) {
        if (s_master_ctx.device_configs[i].address == device_addr) {
            // Remove device handle
            if (s_master_ctx.device_handles[i] != NULL) {
                i2c_master_bus_rm_device(s_master_ctx.device_handles[i]);
            }
            
            // Shift remaining devices
            for (int j = i; j < s_master_ctx.num_devices - 1; j++) {
                s_master_ctx.device_configs[j] = s_master_ctx.device_configs[j + 1];
                s_master_ctx.device_handles[j] = s_master_ctx.device_handles[j + 1];
            }
            
            s_master_ctx.num_devices--;
            ESP_LOGI(TAG, "Removed device at address 0x%02X", device_addr);
            return ESP_OK;
        }
    }
    
    ESP_LOGW(TAG, "Device at address 0x%02X not found", device_addr);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t i2c_master_sensor_write_reg(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    i2c_operation_t operation = {
        .op_type = I2C_OP_TYPE_SENSOR_WRITE,
        .device_addr = device_addr,
        .data = (uint8_t*)data,
        .data_len = len,
        .timeout_ms = timeout_ms,
        .sensor.reg_addr = reg_addr
    };

    return i2c_master_queue_operation(&operation);
}

bool i2c_master_device_is_available(uint8_t device_addr)
{
    // Find device handle
    i2c_master_dev_handle_t device_handle = NULL;
    for (int i = 0; i < s_master_ctx.num_devices; i++) {
        if (s_master_ctx.device_configs[i].address == device_addr) {
            device_handle = s_master_ctx.device_handles[i];
            break;
        }
    }
    
    if (device_handle == NULL) {
        return false;
    }
    
    // FIXED: Try to probe the device with proper buffer
    uint8_t dummy = 0;
    esp_err_t ret = i2c_master_transmit(device_handle, &dummy, 0, pdMS_TO_TICKS(100));
    return (ret == ESP_OK);
}

const i2c_mgr_device_config_t* i2c_master_get_device_config(uint8_t device_addr)
{
    for (int i = 0; i < s_master_ctx.num_devices; i++) {
        if (s_master_ctx.device_configs[i].address == device_addr) {
            return &s_master_ctx.device_configs[i];
        }
    }
    return NULL;
}

void i2c_scan_physical_bus() {
    ESP_LOGI(TAG, "Scanning physical I2C bus...");
    
    bool found_any = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // FIXED: Create temporary device handle for each address during scan
        i2c_device_config_t temp_dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = CONFIG_I2C_MASTER_CLK_SPEED,  // Use configured speed
        };
        
        i2c_master_dev_handle_t temp_handle = NULL;
        esp_err_t ret = i2c_master_bus_add_device(s_master_ctx.bus_handle, &temp_dev_cfg, &temp_handle);
        
        if (ret == ESP_OK) {
            // FIXED: Use a dummy byte for probing instead of NULL/0
            uint8_t dummy_data = 0x00;
            esp_err_t probe_ret = i2c_master_transmit(temp_handle, &dummy_data, 0, pdMS_TO_TICKS(50));
            
            if (probe_ret == ESP_OK) {
                ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
                found_any = true;
                
                // Check if this is a configured device
                const i2c_mgr_device_config_t *config = i2c_master_get_device_config(addr);
                if (config) {
                    ESP_LOGI(TAG, "  -> Configured as: %s", config->name);
                } else {
                    ESP_LOGI(TAG, "  -> Unconfigured device");
                }
            } else {
                // Only log at debug level for failed probes to reduce noise
                ESP_LOGD(TAG, "No response from 0x%02X: %s", addr, esp_err_to_name(probe_ret));
            }
            
            // Clean up temporary handle
            i2c_master_bus_rm_device(temp_handle);
        }
    }
    
    if (!found_any) {
        ESP_LOGW(TAG, "No devices responded during scan - check wiring and pull-ups");
    }
}