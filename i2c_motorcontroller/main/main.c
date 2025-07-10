#include <stdio.h>
#include "buoye_structs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "freertos/event_groups.h"

// Comment out to test as SLAVE, uncomment for MASTER
#define TEST_AS_MASTER

#ifdef TEST_AS_MASTER
#include "i2c_motorcontroller_master.h"  // Use wrapper instead of manager
#include "i2c_master_manager.h"          // For sensor functions
#else
#include "i2c_motorcontroller_slave.h"
#endif

static const char *TAG = "I2C_FLEX_TEST";
static EventGroupHandle_t test_event_group;

#define TEST_COMPLETE_BIT   (1 << 0)
#define TEST_ERROR_BIT      (1 << 1)

// Mock functions for print_pkg and print_resp (replace with your actual implementations)
void print_pkg(const motorcontroller_pkg_t *pkg) {
    ESP_LOGI(TAG, "Package: STATE=%d, end_depth=%d, samples=%d, alpha=%.2f, beta=%.2f", 
             pkg->STATE, pkg->end_depth, pkg->samples, pkg->alpha, pkg->beta);
}

void print_resp(const motorcontroller_response_t *resp) {
    ESP_LOGI(TAG, "Response: STATE=%d, result=%s, working_time=%d, speed=%d", 
             resp->STATE, esp_err_to_name(resp->result), resp->working_time, resp->estimated_cm_per_s);
}

#ifdef TEST_AS_MASTER
// Master callback for I2C events (for sensors)
static void i2c_master_event_callback(const i2c_mgr_event_data_t *event_data, void *user_data)
{
    // Only handle sensor events here, motor controller events handled by wrapper
    if (event_data->device_addr == CONFIG_MOTCTRL_I2C_ADDR) {
        return; // Motor controller events handled by wrapper
    }
    
    switch (event_data->event_type) {
        case I2C_MGR_EVT_OPERATION_COMPLETE:
            ESP_LOGI(TAG, "Sensor I2C operation completed for device 0x%02X", event_data->device_addr);
            xEventGroupSetBits(test_event_group, TEST_COMPLETE_BIT);
            break;
            
        case I2C_MGR_EVT_OPERATION_ERROR:
            ESP_LOGE(TAG, "Sensor I2C operation failed for device 0x%02X: %s", 
                     event_data->device_addr, esp_err_to_name(event_data->error_code));
            xEventGroupSetBits(test_event_group, TEST_ERROR_BIT);
            break;
            
        case I2C_MGR_EVT_OPERATION_TIMEOUT:
            ESP_LOGW(TAG, "Sensor I2C operation timeout for device 0x%02X", event_data->device_addr);
            xEventGroupSetBits(test_event_group, TEST_ERROR_BIT);
            break;
            
        default:
            ESP_LOGD(TAG, "Sensor I2C event: %d", event_data->event_type);
            break;
    }
}

// Test motor controller functionality
static void test_motor_controller(void)
{
    ESP_LOGI(TAG, "=== Testing Motor Controller ===");
    
    // Wake up motor controller
    ESP_LOGI(TAG, "Waking up motor controller...");
    i2c_motctrl_master_wake_up_motorcontroller();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Create test package
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    pkg.STATE = LOWERING;
    pkg.end_depth = 100;
    pkg.samples = 5;
    pkg.alpha = 0.7;
    pkg.beta = 0.3;
    pkg.prev_estimated_cm_per_s = 5000;
    pkg.rising_timeout_percent = 20;
    
    ESP_LOGI(TAG, "Sending package:");
    print_pkg(&pkg);
    
    // Send package using wrapper (blocking call)
    esp_err_t err = i2c_motctrl_master_send_pkg(&pkg, 10); // 10 second timeout
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor controller package send failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Motor controller package sent successfully!");
    
    // Calculate wait time and get response
    int wait_time = motctrl_master_calculate_wait_time(
        pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    ESP_LOGI(TAG, "Waiting for response (estimated: %d seconds)...", wait_time);
    
    motorcontroller_response_t resp;
    err = i2c_motctrl_master_wait_response(&resp, wait_time);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor controller response failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Received response:");
    print_resp(&resp);
}

// Test sensor functionality (if enabled)
static void test_sensors(void)
{
    ESP_LOGI(TAG, "=== Testing Sensors ===");
    
#ifdef CONFIG_LSM6DS032TR_DEVICE_ENABLED
    ESP_LOGI(TAG, "Testing LSM6DS032TR sensor...");
    
    // Check if device is available
    if (!i2c_master_device_is_available(CONFIG_LSM6DS032TR_I2C_ADDR)) {
        ESP_LOGW(TAG, "LSM6DS032TR sensor not detected");
    } else {
        ESP_LOGI(TAG, "LSM6DS032TR sensor detected!");
        
        // Read WHO_AM_I register (should return 0x6C for LSM6DS032TR)
        uint8_t who_am_i = 0;
        xEventGroupClearBits(test_event_group, TEST_COMPLETE_BIT | TEST_ERROR_BIT);
        
        esp_err_t err = i2c_master_sensor_read_reg(CONFIG_LSM6DS032TR_I2C_ADDR, 0x0F, &who_am_i, 1, 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to queue sensor read: %s", esp_err_to_name(err));
        } else {
            // Wait for completion
            EventBits_t bits = xEventGroupWaitBits(test_event_group, TEST_COMPLETE_BIT | TEST_ERROR_BIT, 
                                                  pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
            
            if (bits & TEST_COMPLETE_BIT) {
                ESP_LOGI(TAG, "LSM6DS032TR WHO_AM_I: 0x%02X (expected: 0x6C)", who_am_i);
            } else {
                ESP_LOGE(TAG, "Failed to read from LSM6DS032TR sensor");
            }
        }
    }
#else
    ESP_LOGI(TAG, "LSM6DS032TR sensor not enabled in menuconfig");
#endif

    // Test custom devices if enabled
#ifdef CONFIG_CUSTOM_DEVICE_1_ENABLED
    ESP_LOGI(TAG, "Testing custom device 1 (%s)...", CONFIG_CUSTOM_DEVICE_1_NAME);
    
    if (i2c_master_device_is_available(CONFIG_CUSTOM_DEVICE_1_ADDR)) {
        ESP_LOGI(TAG, "Custom device 1 detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_1_ADDR);
    } else {
        ESP_LOGW(TAG, "Custom device 1 not detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_1_ADDR);
    }
#else
    ESP_LOGI(TAG, "Custom device 1 not enabled in menuconfig");
#endif

#ifdef CONFIG_CUSTOM_DEVICE_2_ENABLED
    ESP_LOGI(TAG, "Testing custom device 2 (%s)...", CONFIG_CUSTOM_DEVICE_2_NAME);
    
    if (i2c_master_device_is_available(CONFIG_CUSTOM_DEVICE_2_ADDR)) {
        ESP_LOGI(TAG, "Custom device 2 detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_2_ADDR);
    } else {
        ESP_LOGW(TAG, "Custom device 2 not detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_2_ADDR);
    }
#else
    ESP_LOGI(TAG, "Custom device 2 not enabled in menuconfig");
#endif
}

#endif

void app_main(void) {
    ESP_LOGI(TAG, "=== I2C Motor Controller & Sensor Test ===");
    
    // Create test event group
    test_event_group = xEventGroupCreate();
    if (test_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create test event group");
        return;
    }
    
    #ifdef TEST_AS_MASTER
    /********** MASTER TEST **********/
    ESP_LOGI(TAG, "Starting I2C MASTER test");
    
    // Initialize motor controller (this also initializes the I2C manager)
    esp_err_t err = i2c_motctrl_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor controller master initialization failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Motor controller master initialized successfully!");
    
    // Scan for devices
    ESP_LOGI(TAG, "Scanning for I2C devices...");
    bool found_any = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_device_is_available(addr)) {
            const i2c_mgr_device_config_t *config = i2c_master_get_device_config(addr);
            if (config) {
                ESP_LOGI(TAG, "Found configured device: %s at address 0x%02X", config->name, addr);
            } else {
                ESP_LOGI(TAG, "Found unconfigured device at address 0x%02X", addr);
            }
            found_any = true;
        }
    }
    
    if (!found_any) {
        ESP_LOGW(TAG, "No I2C devices found! Check wiring and pull-up resistors.");
    }
    
    // Test motor controller
    ESP_LOGI(TAG, "Testing motor controller communication...");
    test_motor_controller();
    
    // Test sensors
    test_sensors();
    
    ESP_LOGI(TAG, "MASTER test completed!");
    
    // Cleanup
    i2c_motctrl_master_deinit();
    
    #else
    /********** SLAVE TEST **********/
    ESP_LOGI(TAG, "Starting I2C SLAVE test");
    
    // Install slave driver
    esp_err_t err = i2c_innstall_slave_driver_cnfig();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Slave driver install failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Slave initialized, ready to receive commands");
    
    // Main slave loop
    while (true) {
        motorcontroller_pkg_t pkg;
        int timeout_sec = 30;
        ESP_LOGI(TAG, "Waiting for package (timeout: %d seconds)...", timeout_sec);
        
        err = i2c_motctrl_slave_wait_pkg(&pkg, timeout_sec);
        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "No package received, continuing to wait...");
            continue;
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Package wait failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        ESP_LOGI(TAG, "Received package:");
        print_pkg(&pkg);
        
        i2c_motctrl_slave_set_working();
        
        // Simulate work
        int work_time_ms = (pkg.prev_estimated_cm_per_s > 0) ? 
            (pkg.end_depth * 1000) / (pkg.prev_estimated_cm_per_s / 100) : 5000;
        if (work_time_ms > 10000) work_time_ms = 10000;
        if (work_time_ms < 2000) work_time_ms = 2000;
        
        ESP_LOGI(TAG, "Simulating motor work for %d ms...", work_time_ms);
        vTaskDelay(pdMS_TO_TICKS(work_time_ms));
        
        // Prepare response
        motorcontroller_response_t resp;
        motorcontroller_response_init_default(&resp);
        resp.STATE = pkg.STATE;
        resp.result = ESP_OK;
        resp.working_time = work_time_ms / 1000;
        resp.estimated_cm_per_s = pkg.prev_estimated_cm_per_s;
        
        ESP_LOGI(TAG, "Work completed! Sending response:");
        print_resp(&resp);
        
        // Send response with retry
        bool response_sent = false;
        for (int i = 0; i < 5; ++i) {
            err = i2c_motctrl_slave_send_response(&resp, 15);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Response sent successfully on attempt %d", i + 1);
                response_sent = true;
                break;
            }
            ESP_LOGW(TAG, "Attempt %d failed: %s", i + 1, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        if (!response_sent) {
            ESP_LOGE(TAG, "Failed to send response after 5 attempts");
        }
        
        ESP_LOGI(TAG, "=== Transaction complete, ready for next command ===\n");
    }
    
    #endif
    
    ESP_LOGI(TAG, "Test finished");
}