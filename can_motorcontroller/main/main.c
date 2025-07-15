/**
 * @file main.c
 * @brief CAN Motor Controller Main Application
 * 
 * Set DEVICE_ROLE to select what this device should do:
 * - DEVICE_ROLE_MANAGER: Acts as main controller (sends commands)
 * - DEVICE_ROLE_WORKER: Acts as motor controller (receives commands)
 * - DEVICE_ROLE_TEST: Runs test suite
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Component includes
#include "can_motctrl_common.h"

// Role selection - CHANGE THIS LINE to set device role
#define DEVICE_ROLE_MANAGER     1
#define DEVICE_ROLE_WORKER      2
#define DEVICE_ROLE_TEST        3

#define DEVICE_ROLE             DEVICE_ROLE_WORKER    // <-- CHANGE THIS

// Conditional includes based on role
#if DEVICE_ROLE == DEVICE_ROLE_MANAGER
    #include "can_motctrl_manager.h"
#elif DEVICE_ROLE == DEVICE_ROLE_WORKER
    #include "can_motctrl_worker.h"
#elif DEVICE_ROLE == DEVICE_ROLE_TEST
    // Test functions (defined in can_motctrl_test.c)
    esp_err_t run_can_motctrl_tests(void);
#endif

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== CAN Motor Controller System ===");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if DEVICE_ROLE == DEVICE_ROLE_MANAGER
    ESP_LOGI(TAG, "Role: MANAGER (Main Controller)");
    
    // Initialize manager
    ret = can_motctrl_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manager init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Wait for worker to be ready
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Example operation
    while (true) {
        motorcontroller_pkg_t pkg;
        motorcontroller_pkg_init_default(&pkg);
        
        pkg.STATE = LOWERING;
        pkg.end_depth = 200;  // 2 meters
        pkg.samples = 3;
        pkg.static_points[0] = 50;   // 50cm
        pkg.static_points[1] = 100;  // 1m  
        pkg.static_points[2] = 150;  // 1.5m
        pkg.static_points[3] = 0;    // End marker
        
        ESP_LOGI(TAG, "Sending dive command to %d cm...", pkg.end_depth);
        
        ret = can_motctrl_manager_send_pkg(&pkg, 10);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Package sent, waiting for response...");
            
            motorcontroller_response_t response;
            int timeout = calculate_operation_timeout(pkg.STATE, pkg.prev_estimated_cm_per_s,
                                                     pkg.rising_timeout_percent, pkg.end_depth,
                                                     pkg.static_points, pkg.samples, 
                                                     pkg.static_poll_interval_s);
            
            ret = can_motctrl_manager_wait_response(&response, timeout);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Operation completed successfully!");
                ESP_LOGI(TAG, "  Working time: %d seconds", response.working_time);
                ESP_LOGI(TAG, "  Estimated speed: %d cm/s", response.estimated_cm_per_s);
            } else {
                ESP_LOGE(TAG, "Failed to get response: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to send package: %s", esp_err_to_name(ret));
        }
        
        // Wait before next operation
        vTaskDelay(pdMS_TO_TICKS(15000));
    }

#elif DEVICE_ROLE == DEVICE_ROLE_WORKER
    ESP_LOGI(TAG, "Role: WORKER (Motor Controller)");
    
    // Initialize worker
    ret = can_motctrl_worker_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Worker init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Worker main loop
    while (true) {
        ESP_LOGI(TAG, "Waiting for command...");
        
        motorcontroller_pkg_t pkg;
        ret = can_motctrl_worker_wait_pkg(&pkg, 30);  // 30 second timeout
        
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "No command received, continuing...");
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error receiving package: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        ESP_LOGI(TAG, "Command received: %s to %d cm", 
                get_state_string(pkg.STATE), pkg.end_depth);
        
        // Set working state
        can_motctrl_worker_set_working();
        
        // Simulate motor operation
        ESP_LOGI(TAG, "Simulating motor operation...");
        int work_time = 5 + (pkg.end_depth / 50);  // Rough simulation
        
        for (int i = 0; i < work_time; i++) {
            ESP_LOGI(TAG, "Working... %d/%d seconds", i + 1, work_time);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // Prepare response
        motorcontroller_response_t response;
        motorcontroller_response_init_default(&response);
        response.STATE = pkg.STATE;
        response.result = ESP_OK;
        response.working_time = work_time;
        response.estimated_cm_per_s = 45;  // Simulated
        
        ESP_LOGI(TAG, "Sending response...");
        ret = can_motctrl_worker_send_response(&response, 10);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Response sent successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(ret));
        }
    }

#elif DEVICE_ROLE == DEVICE_ROLE_TEST
    ESP_LOGI(TAG, "Role: TEST MODE");
    
    // Run tests
    ret = run_can_motctrl_tests();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "🎉 ALL TESTS PASSED! 🎉");
    } else {
        ESP_LOGE(TAG, "❌ SOME TESTS FAILED ❌");
    }
    
    // Keep running to see results
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

#else
    #error "Invalid DEVICE_ROLE configuration. Set DEVICE_ROLE to DEVICE_ROLE_MANAGER, DEVICE_ROLE_WORKER, or DEVICE_ROLE_TEST"
#endif
}