#include <stdio.h>
#include "buoye_structs.h"
#include "log_thread.h"
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

// Debug print functions for structs
void print_pkg(const motorcontroller_pkg_t *pkg) {
    ESP_LOGI_THREAD(TAG, "=== Motor Controller Package ===");
    ESP_LOGI_THREAD(TAG, "  STATE: %s", state_to_string(pkg->STATE));
    ESP_LOGI_THREAD(TAG, "  prev_working_time: %d sec", pkg->prev_working_time);
    ESP_LOGI_THREAD(TAG, "  rising_timeout_percent: %d%%", pkg->rising_timeout_percent);
    ESP_LOGI_THREAD(TAG, "  prev_reported_depth: %u cm", pkg->prev_reported_depth);
    ESP_LOGI_THREAD(TAG, "  prev_end_depth: %u cm", pkg->prev_end_depth);
    ESP_LOGI_THREAD(TAG, "  prev_estimated_cm_per_s: %u (%.2f cm/s)", 
                    pkg->prev_estimated_cm_per_s, pkg->prev_estimated_cm_per_s / 100.0f);
    ESP_LOGI_THREAD(TAG, "  poll_type: %d", pkg->poll_type);
    ESP_LOGI_THREAD(TAG, "  end_depth: %u cm", pkg->end_depth);
    ESP_LOGI_THREAD(TAG, "  samples: %u", pkg->samples);
    ESP_LOGI_THREAD(TAG, "  static_poll_interval_s: %u sec", pkg->static_poll_interval_s);
    ESP_LOGI_THREAD(TAG, "  alpha: %.2f", pkg->alpha);
    ESP_LOGI_THREAD(TAG, "  beta: %.2f", pkg->beta);
    
    // Print static points if any
    ESP_LOGI_THREAD(TAG, "  static_points: ");
    for (int i = 0; i < 10 && pkg->static_points[i] != 0; i++) {
        ESP_LOGI_THREAD(TAG, "    [%d]: %u cm", i, pkg->static_points[i]);
    }
}

void print_resp(const motorcontroller_response_t *resp) {
    ESP_LOGI_THREAD(TAG, "=== Motor Controller Response ===");
    ESP_LOGI_THREAD(TAG, "  STATE: %s", state_to_string(resp->STATE));
    ESP_LOGI_THREAD(TAG, "  result: %s", esp_err_to_name(resp->result));
    ESP_LOGI_THREAD(TAG, "  working_time: %d sec", resp->working_time);
    ESP_LOGI_THREAD(TAG, "  estimated_cm_per_s: %u (%.2f cm/s)", 
                    resp->estimated_cm_per_s, resp->estimated_cm_per_s / 100.0f);
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
            ESP_LOGI_THREAD(TAG, "Sensor I2C operation completed for device 0x%02X", event_data->device_addr);
            xEventGroupSetBits(test_event_group, TEST_COMPLETE_BIT);
            break;
            
        case I2C_MGR_EVT_OPERATION_ERROR:
            ESP_LOGE_THREAD(TAG, "Sensor I2C operation failed for device 0x%02X: %s", 
                     event_data->device_addr, esp_err_to_name(event_data->error_code));
            xEventGroupSetBits(test_event_group, TEST_ERROR_BIT);
            break;
            
        case I2C_MGR_EVT_OPERATION_TIMEOUT:
            ESP_LOGW_THREAD(TAG, "Sensor I2C operation timeout for device 0x%02X", event_data->device_addr);
            xEventGroupSetBits(test_event_group, TEST_ERROR_BIT);
            break;
            
        default:
            ESP_LOGD_THREAD(TAG, "Sensor I2C event: %d", event_data->event_type);
            break;
    }
}

// Test motor controller functionality
static void test_motor_controller(void)
{
    ESP_LOGI_THREAD(TAG, "=== Testing Motor Controller ===");
    
    // Wake up motor controller
    ESP_LOGI_THREAD(TAG, "Waking up motor controller...");
    i2c_motctrl_master_wake_up_motorcontroller();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Create test package
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    pkg.STATE = LOWERING;
    pkg.end_depth = 100;
    pkg.samples = 235;
    pkg.alpha = 34323.7;
    pkg.beta = 5634.3;
    pkg.prev_estimated_cm_per_s = 5000; // 50.00 cm/s
    pkg.rising_timeout_percent = 20;
    
    // Add some static points for testing
    pkg.static_points[0] = 100;
    pkg.static_points[1] = 200;
    pkg.static_points[2] = 400;
    for(int i = 0; i< 79; ++i) pkg.static_points[i] = 10*i;
    
    ESP_LOGI_THREAD(TAG, "Sending package:");
    print_pkg(&pkg);
    
    // Send package using wrapper (blocking call)
    esp_err_t err = i2c_motctrl_master_send_pkg(&pkg, 10); // 10 second timeout
    if (err != ESP_OK) {
        ESP_LOGE_THREAD(TAG, "Motor controller package send failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI_THREAD(TAG, "Motor controller package sent successfully!");
    
    // Calculate wait time and get response
    int wait_time = motctrl_master_calculate_wait_time(
        pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    ESP_LOGI_THREAD(TAG, "Waiting for response (estimated: %d seconds)...", wait_time);
    
    motorcontroller_response_t resp;
    // vTaskDelay(pdMS_TO_TICKS(6000));
    err = i2c_motctrl_master_wait_response(&resp, wait_time);
    if (err != ESP_OK) {
        ESP_LOGE_THREAD(TAG, "Motor controller response failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI_THREAD(TAG, "Received response:");
    print_resp(&resp);
}

// Test sensor functionality (if enabled)
static void test_sensors(void)
{
    ESP_LOGI_THREAD(TAG, "=== Testing Sensors ===");
    
#ifdef CONFIG_LSM6DS032TR_DEVICE_ENABLED
    ESP_LOGI_THREAD(TAG, "Testing LSM6DS032TR sensor...");
    
    // Check if device is available
    if (!i2c_master_device_is_available(CONFIG_LSM6DS032TR_I2C_ADDR)) {
        ESP_LOGW_THREAD(TAG, "LSM6DS032TR sensor not detected");
    } else {
        ESP_LOGI_THREAD(TAG, "LSM6DS032TR sensor detected!");
        
        // Read WHO_AM_I register (should return 0x6C for LSM6DS032TR)
        uint8_t who_am_i = 0;
        xEventGroupClearBits(test_event_group, TEST_COMPLETE_BIT | TEST_ERROR_BIT);
        
       uint8_t raw[12];
    esp_err_t err = i2c_master_sensor_read_reg(
        CONFIG_LSM6DS032TR_I2C_ADDR,
        0x22,   // start på OUTX_L_G med autoincrement
        raw,           // buffer for dei 12 bytea
        12,            // tal byte
        1000           // timeout i ms
    );

    // Slå saman LSB/MSB til int16_t
    int16_t gyro[3], accel[3];
    for (int i = 0; i < 3; i++) {
        gyro[i]  = (int16_t)(raw[2*i]   | (raw[2*i+1]   << 8));
        accel[i] = (int16_t)(raw[6+2*i] | (raw[6+2*i+1] << 8));
    }

    // Skriv ut
    ESP_LOGI(TAG, "Gyro  (dps): X=%d, Y=%d, Z=%d", gyro[0],  gyro[1],  gyro[2]);
    ESP_LOGI(TAG, "Accel (mg) : X=%d, Y=%d, Z=%d", accel[0], accel[1], accel[2]);

            // Wait for completion
            EventBits_t bits = xEventGroupWaitBits(test_event_group, TEST_COMPLETE_BIT | TEST_ERROR_BIT, 
                                                  pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
            
            if (bits & TEST_COMPLETE_BIT) {
                ESP_LOGI_THREAD(TAG, "LSM6DS032TR WHO_AM_I: 0x%02X (expected: 0x6C)", who_am_i);
            } else {
                ESP_LOGE_THREAD(TAG, "Failed to read from LSM6DS032TR sensor");
            }
        
    }
#else
    ESP_LOGI_THREAD(TAG, "LSM6DS032TR sensor not enabled in menuconfig");
#endif

    // Test custom devices if enabled
#ifdef CONFIG_CUSTOM_DEVICE_1_ENABLED
    ESP_LOGI_THREAD(TAG, "Testing custom device 1 (%s)...", CONFIG_CUSTOM_DEVICE_1_NAME);
    
    if (i2c_master_device_is_available(CONFIG_CUSTOM_DEVICE_1_ADDR)) {
        ESP_LOGI_THREAD(TAG, "Custom device 1 detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_1_ADDR);
    } else {
        ESP_LOGW_THREAD(TAG, "Custom device 1 not detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_1_ADDR);
    }
#else
    ESP_LOGI_THREAD(TAG, "Custom device 1 not enabled in menuconfig");
#endif

#ifdef CONFIG_CUSTOM_DEVICE_2_ENABLED
    ESP_LOGI_THREAD(TAG, "Testing custom device 2 (%s)...", CONFIG_CUSTOM_DEVICE_2_NAME);
    
    if (i2c_master_device_is_available(CONFIG_CUSTOM_DEVICE_2_ADDR)) {
        ESP_LOGI_THREAD(TAG, "Custom device 2 detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_2_ADDR);
    } else {
        ESP_LOGW_THREAD(TAG, "Custom device 2 not detected at address 0x%02X", CONFIG_CUSTOM_DEVICE_2_ADDR);
    }
#else
    ESP_LOGI_THREAD(TAG, "Custom device 2 not enabled in menuconfig");
#endif
}

#endif // TEST_AS_MASTER

void app_main(void) {
    ESP_LOGI_THREAD(TAG, "=== I2C Motor Controller & Sensor Test ===");
    ESP_LOGI_THREAD(TAG, "Thread: %s", pcTaskGetName(NULL));
    ESP_LOGI_THREAD(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Create test event group
    test_event_group = xEventGroupCreate();
    if (test_event_group == NULL) {
        ESP_LOGE_THREAD(TAG, "Failed to create test event group");
        return;
    }
    
    #ifdef TEST_AS_MASTER
    /********** MASTER TEST **********/
    ESP_LOGI_THREAD(TAG, "Starting I2C MASTER test");
    ESP_LOGI_THREAD(TAG, "Master will communicate with slave at address 0x%02X", CONFIG_MOTCTRL_I2C_ADDR);
    
    // Initialize motor controller (this also initializes the I2C manager)
    esp_err_t err = i2c_motctrl_master_init();
    if (err != ESP_OK) {
        ESP_LOGE_THREAD(TAG, "Motor controller master initialization failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI_THREAD(TAG, "Motor controller master initialized successfully!");
    
    // Give slave some time to initialize
    ESP_LOGI_THREAD(TAG, "Waiting for slave to be ready...");

    // Scan for devices
    ESP_LOGI_THREAD(TAG, "Scanning for I2C devices...");
    i2c_scan_physical_bus();
    
    bool found_any = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_device_is_available(addr)) {
            const i2c_mgr_device_config_t *config = i2c_master_get_device_config(addr);
            if (config) {
                ESP_LOGI_THREAD(TAG, "Found configured device: %s at address 0x%02X", config->name, addr);
            } else {
                ESP_LOGI_THREAD(TAG, "Found unconfigured device at address 0x%02X", addr);
            }
            found_any = true;
        }
    }
    
    if (!found_any) {
        ESP_LOGW_THREAD(TAG, "No I2C devices found! Check wiring and pull-up resistors.");
        ESP_LOGW_THREAD(TAG, "Make sure the slave device is running and at address 0x%02X", CONFIG_MOTCTRL_I2C_ADDR);
    }
    
    // Test motor controller communication
    ESP_LOGI_THREAD(TAG, "Testing motor controller communication...");
    test_motor_controller();
    
    // Test sensors
    test_sensors();
    
    ESP_LOGI_THREAD(TAG, "MASTER test completed!");
    
    // Keep running for monitoring
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI_THREAD(TAG, "Master still running... Checking devices...");
        
        // Periodic device check
        if (i2c_master_device_is_available(CONFIG_MOTCTRL_I2C_ADDR)) {
            ESP_LOGI_THREAD(TAG, "Motor controller still available at 0x%02X", CONFIG_MOTCTRL_I2C_ADDR);
        } else {
            ESP_LOGW_THREAD(TAG, "Motor controller not responding at 0x%02X", CONFIG_MOTCTRL_I2C_ADDR);
        }
    }
    
    #else
    /********** SLAVE TEST **********/
    ESP_LOGI_THREAD(TAG, "Starting I2C SLAVE test");
    ESP_LOGI_THREAD(TAG, "Slave will listen at address 0x%02X", CONFIG_MOTCTRL_I2C_ADDR);
    
    // Install slave driver
    esp_err_t err = i2c_install_slave_driver_config();
    if (err != ESP_OK) {
        ESP_LOGE_THREAD(TAG, "Slave driver install failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI_THREAD(TAG, "Slave initialized, ready to receive commands");
    
    // Main slave loop
    int transaction_count = 0;
    while (true) {
        motorcontroller_pkg_t pkg;
        int timeout_sec = 30;
        ESP_LOGI_THREAD(TAG, "Waiting for package #%d (timeout: %d seconds)...", 
                       transaction_count + 1, timeout_sec);
        
        err = i2c_motctrl_slave_wait_pkg(&pkg, timeout_sec);
        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW_THREAD(TAG, "No package received, continuing to wait...");
            continue;
        } else if (err != ESP_OK) {
            ESP_LOGE_THREAD(TAG, "Package wait failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        transaction_count++;
        ESP_LOGI_THREAD(TAG, "=== Transaction #%d ===", transaction_count);
        ESP_LOGI_THREAD(TAG, "Received package:");
        print_pkg(&pkg);
        
        // Set working state
        i2c_motctrl_slave_set_working();
        
        // Simulate work based on package parameters
        int work_time_ms = (pkg.prev_estimated_cm_per_s > 0) ? 
            (pkg.end_depth * 1000) / (pkg.prev_estimated_cm_per_s / 100) : 5000;
        
        // Clamp work time to reasonable limits
        if (work_time_ms > 15000) work_time_ms = 15000;  // Max 15 seconds
        if (work_time_ms < 2000) work_time_ms = 2000;    // Min 2 seconds
        
        ESP_LOGI_THREAD(TAG, "Simulating motor work for %d ms...", work_time_ms);
        
        // Simulate work with periodic status updates
        int progress_updates = work_time_ms / 1000;
        for (int i = 0; i < progress_updates; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI_THREAD(TAG, "Work progress: %d/%d seconds", i + 1, progress_updates);
        }
        
        // Prepare response
        motorcontroller_response_t resp;
        motorcontroller_response_init_default(&resp);
        resp.STATE = pkg.STATE;
        resp.result = ESP_OK;
        resp.working_time = work_time_ms / 1000;
        resp.estimated_cm_per_s = pkg.prev_estimated_cm_per_s; // For simulation, keep same speed
        
        ESP_LOGI_THREAD(TAG, "Work completed! Sending response:");
        print_resp(&resp);
        
        // Send response with retry
        bool response_sent = false;
        for (int i = 0; i < 5; ++i) {
            err = i2c_motctrl_slave_send_response(&resp, 15);
            if (err == ESP_OK) {
                ESP_LOGI_THREAD(TAG, "Response sent successfully on attempt %d", i + 1);
                response_sent = true;
                break;
            }
            ESP_LOGW_THREAD(TAG, "Response send attempt %d failed: %s", i + 1, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        if (!response_sent) {
            ESP_LOGE_THREAD(TAG, "Failed to send response after 5 attempts");
        }
        
        ESP_LOGI_THREAD(TAG, "=== Transaction #%d complete, ready for next command ===\n", transaction_count);
    }
    
    #endif
    
    ESP_LOGI_THREAD(TAG, "Test finished");
}