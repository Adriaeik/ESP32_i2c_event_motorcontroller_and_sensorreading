// #include "can_bus_manager.h"
// #include "can_motctrl_manager.h"
// #include "can_motctrl_worker.h"
// #include "winch_controller.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "esp_sleep.h"
// #include "freertos/task.h"

// // Uncomment for manager, comment for worker
// // #define ROLE_MANAGER         // <------------

// static const char *TAG = "MAIN";

// #ifdef ROLE_MANAGER
// #include "motorcontroller_manager.h"

// void manager_main_task(void *arg) {
//     ESP_LOGI(TAG, "Manager task started");
//     can_bus_manager_deinit();
    
//     while (1) {
//         // Initialize CAN bus for everyone to use
//         ESP_ERROR_CHECK(can_bus_manager_init());
        
//         // Test LOWERING cycle
//         ESP_LOGI(TAG, "=== Starting LOWERING cycle ===");
        
//         // Load system package for LOWERING
//         motorcontroller_pkg_t pkg;
//         esp_err_t result = load_system_motorcontroller_pkg(&pkg, LOWERING);
//         if (result != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to load system package: %s", esp_err_to_name(result));
//             goto cleanup;
//         }
        
//         // Start manager worker task
//         result = motorcontroller_manager_start_worker();
//         if (result != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to start manager worker: %s", esp_err_to_name(result));
//             goto cleanup;
//         }
        
//         // Wait for completion (60 second timeout)
//         result = motorcontroller_manager_wait_completion(60000);
//         if (result == ESP_OK) {
//             ESP_LOGI(TAG, "LOWERING cycle completed successfully");
//         } else {
//             ESP_LOGE(TAG, "LOWERING cycle failed: %s", esp_err_to_name(result));
//         }
        
//         // Wait a bit before next cycle
//         vTaskDelay(pdMS_TO_TICKS(5000));
        
//         // Test RISING cycle
//         ESP_LOGI(TAG, "=== Starting RISING cycle ===");
        
//         result = load_system_motorcontroller_pkg(&pkg, RISING);
//         if (result != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to load system package: %s", esp_err_to_name(result));
//             goto cleanup;
//         }
        
//         result = motorcontroller_manager_start_worker();
//         if (result != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to start manager worker: %s", esp_err_to_name(result));
//             goto cleanup;
//         }
        
//         result = motorcontroller_manager_wait_completion(60000);
//         if (result == ESP_OK) {
//             ESP_LOGI(TAG, "RISING cycle completed successfully");
//         } else {
//             ESP_LOGE(TAG, "RISING cycle failed: %s", esp_err_to_name(result));
//         }
        
// cleanup:
//         // Clean up CAN bus
//         can_bus_manager_deinit();
        
//         // Wait before next full cycle
//         ESP_LOGI(TAG, "Cycle complete, waiting 10 seconds before next cycle");
//         vTaskDelay(pdMS_TO_TICKS(10000));
//     }
// }

// #else
// #include "motorcontroller_worker.h"
// void worker_main_task(void *arg) {
//     ESP_LOGI(TAG, "Worker task started");
    
//     while (1) {
//         // Subscribe only when ready to receive work
//         esp_err_t ret = can_subscribe_set(CAN_MTOTCTRL_WORKER);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to subscribe to package channels");
//             vTaskDelay(pdMS_TO_TICKS(5000));
//             continue;
//         }
        
//         ESP_LOGI(TAG, "Worker ready to receive work");
        
//         // Wait for work package
//         motorcontroller_pkg_t pkg;
//         ret = receive_work_package(&pkg, portMAX_DELAY);
        
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "Package received - State: %s", get_state_string(pkg.STATE));
            
//             // Just simulate work for now
//             vTaskDelay(pdMS_TO_TICKS(2000));
            
//             // Create and send response
//             motorcontroller_response_t resp;
//             motorcontroller_response_init_default(&resp);
//             resp.STATE = pkg.STATE;
//             resp.result = ESP_OK;
//             resp.working_time = 2;
//             resp.estimated_cm_per_s = pkg.prev_estimated_cm_per_s;
            
//             send_work_response(&resp, 5000);
//         } else {
//             ESP_LOGE(TAG, "Failed to receive package: %s", esp_err_to_name(ret));
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
        
//         can_unsubscribe_set(CAN_MTOTCTRL_WORKER);
        
//         // NO cleanup, NO sleep - just loop forever like working version
//     }
// }

// // void worker_main_task(void *arg) {
// //     ESP_LOGI(TAG, "Worker task started");
    
// //     while (1) {
// //         // Check wakeup reason
// //         // if (wakeup_reason_is_ext1()) {
// //         //     ESP_LOGI(TAG, "EXT1 wakeup detected - going to home position");
            
// //         //     // Initialize hardware
// //         //     ESP_ERROR_CHECK(motorcontroller_worker_init_hardware());
            
// //         //     // Go to home position
// //         //     esp_err_t result = winch_go_to_home_position();
// //         //     if (result == ESP_OK) {
// //         //         ESP_LOGI(TAG, "Home position reached successfully");
// //         //     } else {
// //         //         ESP_LOGE(TAG, "Failed to reach home position: %s", esp_err_to_name(result));
// //         //     }
            
// //         //     goto sleep;
// //         // }
        
// //         // Normal work cycle
// //         ESP_LOGI(TAG, "Starting normal work cycle");
        
// //         // Initialize hardware
// //         // ESP_ERROR_CHECK(motorcontroller_worker_init_hardware());
        

// //         esp_err_t ret = can_subscribe_set(CAN_MTOTCTRL_WORKER);
// //         if (ret != ESP_OK) {
// //             ESP_LOGE(TAG, "Failed to subscribe to package channels");
// //             vTaskDelay(pdMS_TO_TICKS(5000));
// //             continue;
// //         }
// //         ESP_LOGI(TAG, "Worker ready to receive work");
// //         // vTaskDelay(pdMS_TO_TICKS(1000));
// //         // Wait for work package
// //         motorcontroller_pkg_t pkg;
// //         ret = receive_work_package(&pkg, portMAX_DELAY);
// //         if (ret != ESP_OK) {
// //             ESP_LOGE(TAG, "Failed to receive work package: %s", esp_err_to_name(ret));
// //             goto cleanup;
// //         }
        
// //         ESP_LOGI(TAG, "Work package received - State: %s", get_state_string(pkg.STATE));
        
// //         // Do the work
// //         motorcontroller_response_t resp;
// //         ret = do_work(&pkg, &resp);
        
// //         // Send response
// //         esp_err_t send_result = send_work_response(&resp, 5000);
// //         if (send_result != ESP_OK) {
// //             ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(send_result));
// //         } else {
// //             ESP_LOGI(TAG, "Response sent successfully");
// //         }
        
// // cleanup:
// //         // Clean up CAN
// //         can_unsubscribe_set(CAN_MTOTCTRL_WORKER);
// //         can_bus_manager_deinit();
        
// // // sleep:
// //         ESP_LOGI(TAG, "Work cycle complete, entering deep sleep");
        
// //         // Configure wakeup sources
// //         esp_sleep_enable_timer_wakeup(30 * 1000000); // 30 seconds for testing
// //         //TODO:: fix pins
// //         esp_sleep_enable_ext1_wakeup(GPIO_NUM_1, ESP_EXT1_WAKEUP_ANY_HIGH); // Boot button
// //         esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, ESP_EXT1_WAKEUP_ANY_HIGH); // Boot by manager
        
// //         // Enter deep sleep
// //         esp_deep_sleep_start();
// //     }
// // }
// #endif

// void app_main() {
//     // Initialize CAN bus manager
//     ESP_ERROR_CHECK(can_bus_manager_init());
    
//     // Initialize LED for status indication
//     gpio_reset_pin(GPIO_NUM_2);
//     gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    
//     // Create role-specific task
// #ifdef ROLE_MANAGER
//     xTaskCreate(manager_main_task, "manager_main", 8192, NULL, 5, NULL);
//     ESP_LOGI(TAG, "Started as MANAGER");
// #else
//     // this does not work
//     // xTaskCreate(worker_main_task, "worker_main", 8192, NULL,5, NULL);
//     // loop:
//     // can_subscribe_set(CAN_MTOTCTRL_WORKER);
//     // motorcontroller_pkg_t pkg;
//     // receive_work_package(&pkg, portMAX_DELAY);
//     // can_unsubscribe_set(CAN_MTOTCTRL_WORKER);
//     // goto loop;
//     // so this works 
//     xTaskCreate(worker_task, "worker_task", 8192, NULL, 5, NULL);

//     ESP_LOGI(TAG, "Started as WORKER");
// #endif
    
//     // Main LED blink loop with role-specific patterns
//     int blink_count = 0;
//     while (1) {
// #ifdef ROLE_MANAGER
//         // Manager: 2 short blinks
//         for (int i = 0; i < 2; i++) {
//             gpio_set_level(GPIO_NUM_2, 1);
//             vTaskDelay(pdMS_TO_TICKS(100));
//             gpio_set_level(GPIO_NUM_2, 0);
//             vTaskDelay(pdMS_TO_TICKS(100));
//         }
//         vTaskDelay(pdMS_TO_TICKS(800));
// #else
//         // Worker: 1 long blink
//         gpio_set_level(GPIO_NUM_2, 1);
//         vTaskDelay(pdMS_TO_TICKS(200));
//         gpio_set_level(GPIO_NUM_2, 0);
//         vTaskDelay(pdMS_TO_TICKS(800));
// #endif
        
//         // Log every 10 seconds
//         if (++blink_count % 10 == 0) {
// #ifdef ROLE_MANAGER
//             ESP_LOGI(TAG, "Manager operational - cycles running");
// #else
//             ESP_LOGI(TAG, "Worker operational - waiting for work");
// #endif
//         }
//         taskYIELD();
//     }
// }
























































#include "can_bus_manager.h"
#include "can_motctrl_manager.h"
#include "can_motctrl_worker.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/task.h"

// Uncomment for manager, comment for worker
#define ROLE_MANAGER
// #define STRESS_TEST_ENABLED  // Comment to disable stress test

static const char *TAG = "MAIN";

// Stress test task
void can_stress_task(void *arg) {
    ESP_LOGI(TAG, "Starting CAN stress test");
    
    while (1) {
        can_message_t msg;
        
        // Generate random message parameters
        msg.identifier = esp_random() % 0x7FF;  // 11-bit IDs
        msg.extd = false;
        msg.rtr = false;
        msg.data_length_code = esp_random() % 9;  // 0-8 bytes
        
        // Fill data with random bytes
        uint32_t rand_data = esp_random();
        for (int i = 0; i < msg.data_length_code; i++) {
            msg.data[i] = ((uint8_t *)&rand_data)[i % 4];
        }
        
        // Send with random priority and timeout
        uint32_t timeout = 10 + (esp_random() % 50);
        esp_err_t ret = can_bus_send_message(&msg, timeout);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Stress send failed: %s (ID: 0x%03X)", 
                    esp_err_to_name(ret), msg.identifier);
        }
        
        // Random delay between messages (0-20ms)
        vTaskDelay(pdMS_TO_TICKS(10*esp_random() % 2000));
    }
}

void app_main() {
    // Initialize CAN bus manager
    ESP_ERROR_CHECK(can_bus_manager_init());
    
    // Initialize LED
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    
    // Create role-specific task
#ifdef ROLE_MANAGER
    xTaskCreate(manager_task, "manager_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started as MANAGER");
#else
    xTaskCreate(worker_task, "worker_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started as WORKER");
#endif

#ifdef STRESS_TEST_ENABLED
    // Create stress test task (lower priority than application tasks)
    xTaskCreate(can_stress_task, "can_stress", 4096, NULL, 2, NULL);
    ESP_LOGI(TAG, "CAN stress test ENABLED");
#endif

    // Blink LED with pattern based on role
    int blink_count = 0;
    while (1) {
        // Different blink patterns for manager/worker
#ifdef ROLE_MANAGER
        // Manager: 2 short blinks
        for (int i = 0; i < 2; i++) {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(800));
#else
        // Worker: 1 long blink
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(800));
#endif
        
        // Log every 10 seconds
        if (++blink_count % 20 == 0) {
            ESP_LOGI(TAG, "System operational");
        }
    }
}