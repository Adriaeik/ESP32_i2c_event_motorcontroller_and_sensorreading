#include "can_bus_manager.h"
#include "can_motctrl_manager.h"
#include "can_motctrl_worker.h"
#include "motorcontroller_worker.h"
#include "winch_controller.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/task.h"
#include "RTC_manager.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// Uncomment for manager, comment for worker
// #define ROLE_MANAGER         // <------------


static const char *TAG = "MAIN";


#ifdef ROLE_MANAGER
#include "motorcontroller_manager.h"

void manager_main_task(void *arg) {
    ESP_LOGI(TAG, "Manager task started");
    can_bus_manager_deinit();
    
    while (1) {
        // Initialize CAN bus for everyone to use
        ESP_ERROR_CHECK(can_bus_manager_init());
        
        // Test LOWERING cycle
        ESP_LOGI(TAG, "=== Starting LOWERING cycle ===");
        
        // Load system package for LOWERING
        motorcontroller_pkg_t pkg;
        esp_err_t result = load_system_motorcontroller_pkg(&pkg, LOWERING);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load system package: %s", esp_err_to_name(result));
            goto cleanup;
        }
        
        // Start manager worker task
        result = motorcontroller_manager_start_worker();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start manager worker: %s", esp_err_to_name(result));
            goto cleanup;
        }
        
        // Wait for completion (60 second timeout)
        result = motorcontroller_manager_wait_completion(60000);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "LOWERING cycle completed successfully");
        } else {
            ESP_LOGE(TAG, "LOWERING cycle failed: %s", esp_err_to_name(result));
        }
        
        // Wait a bit before next cycle
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Test RISING cycle
        ESP_LOGI(TAG, "=== Starting RISING cycle ===");
        
        result = load_system_motorcontroller_pkg(&pkg, RISING);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load system package: %s", esp_err_to_name(result));
            goto cleanup;
        }
        
        result = motorcontroller_manager_start_worker();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start manager worker: %s", esp_err_to_name(result));
            goto cleanup;
        }
        
        result = motorcontroller_manager_wait_completion(60000);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "RISING cycle completed successfully");
        } else {
            ESP_LOGE(TAG, "RISING cycle failed: %s", esp_err_to_name(result));
        }
        
cleanup:
        // Clean up CAN bus
        can_bus_manager_deinit();
        
        // Wait before next full cycle
        ESP_LOGI(TAG, "Cycle complete, waiting 10 seconds before next cycle");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

#else
#include "motorcontroller_worker.h"

void worker_main_task(void *arg) {
    ESP_LOGI(TAG, "Worker task started");
    
    while (1) {
        // Check wakeup reason
        if (wakeup_reason_is_ext1()) {
            ESP_LOGI(TAG, "EXT1 wakeup detected - going to home position");
            
            // Initialize hardware
            ESP_ERROR_CHECK(motorcontroller_worker_init_hardware());
            
            // Go to home position
            esp_err_t result = winch_go_to_home_position();
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "Home position reached successfully");
            } else {
                ESP_LOGE(TAG, "Failed to reach home position: %s", esp_err_to_name(result));
            }
            
            goto sleep;
        }
        
        // Normal work cycle
        ESP_LOGI(TAG, "Starting normal work cycle");
        can_bus_manager_init();
        

        esp_err_t ret = can_subscribe_set(CAN_MTOTCTRL_WORKER);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to subscribe to package channels");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        ESP_LOGI(TAG, "Worker ready to receive work");
        // Wait for work package
        motorcontroller_pkg_t pkg;
        ret = receive_work_package(&pkg, portMAX_DELAY);
        print_motorcontroller_pkg_info(&pkg, TAG);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive work package: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        
        ESP_LOGI(TAG, "Work package received - State: %s", state_to_string(pkg.STATE));
        
        // Do the work
        motorcontroller_response_t resp;
        ret = do_work(&pkg, &resp);
        
        // Send response
        esp_err_t send_result = send_work_response(&resp, 5000);
        if (send_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(send_result));
        } else {
            ESP_LOGI(TAG, "Response sent successfully");
        }
        
cleanup:
        // Clean up CAN
        can_unsubscribe_set(CAN_MTOTCTRL_WORKER);
        can_bus_manager_deinit();
        
sleep:
        ESP_LOGI(TAG, "Work cycle complete, entering deep sleep");
        
        // Configure wakeup sources
        esp_sleep_enable_timer_wakeup(30 * 1000000); // 30 seconds for testing
        //TODO:: fix pins
        esp_sleep_enable_ext1_wakeup(GPIO_NUM_1, ESP_EXT1_WAKEUP_ANY_HIGH); // Boot button
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, ESP_EXT1_WAKEUP_ANY_HIGH); // Boot by manager
        
        // Enter deep sleep
        // esp_deep_sleep_start();
    }
}
#endif //ROLE_MANAGER


void app_main() {
    // Initialize CAN bus manager
    ESP_ERROR_CHECK(can_bus_manager_init());    
    // Create role-specific task
#ifdef ROLE_MANAGER
    xTaskCreate(manager_main_task, "manager_main", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started as MANAGER");
#else
    xTaskCreate(worker_main_task, "worker_main", 8192, NULL,5, NULL);
    ESP_LOGI(TAG, "Started as WORKER");
#endif
    // keepalive
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}




















































