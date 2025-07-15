#include "can_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

// Configuration - Uncomment ONE of these to set the role
// #define ROLE_MANAGER
#define ROLE_WORKER

// CAN message IDs
#define CAN_ID_WORK_REQUEST  0x100
#define CAN_ID_WORK_RESPONSE 0x101

static const char *TAG = "can_test";

// Worker simulation function
void simulate_work(int work_duration_ms) {
    ESP_LOGI(TAG, "Worker: Starting work (%d ms)", work_duration_ms);
    vTaskDelay(pdMS_TO_TICKS(work_duration_ms));
    ESP_LOGI(TAG, "Worker: Work complete");
}

#ifdef ROLE_MANAGER

void manager_task(void *arg) {
    can_message_t request;
    request.identifier = CAN_ID_WORK_REQUEST;
    request.data_length_code = 1;  // Sending one byte of data
    request.flags = CAN_MSG_FLAG_NONE;
    request.data[0] = 50;  // Work duration in 10ms units (50*10 = 500ms)

    can_message_t response;

    while (1) {
        ESP_LOGI(TAG, "Manager: Sending work request");
        if (can_bus_send_message(&request, CAN_PRIORITY_NORMAL, 1000) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send work request");
        } else {
            ESP_LOGI(TAG, "Manager: Sent work request, waiting for response...");
            
            // Wait for response with timeout
            if (can_bus_receive_message(&response, 2000) == ESP_OK) {
                if (response.identifier == CAN_ID_WORK_RESPONSE && 
                    response.data_length_code >= 1) {
                    ESP_LOGI(TAG, "Manager: Received response. Status: %s", 
                            response.data[0] ? "SUCCESS" : "FAIL");
                } else {
                    ESP_LOGW(TAG, "Manager: Unexpected response ID: 0x%X", 
                            response.identifier);
                }
            } else {
                ESP_LOGE(TAG, "Manager: Timeout waiting for response");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3000));  // Send every 3 seconds
    }
}

#endif  // ROLE_MANAGER

#ifdef ROLE_WORKER

void worker_task(void *arg) {
    can_message_t request;
    can_message_t response;

    while (1) {
        // Wait for a work request
        ESP_LOGI(TAG, "Worker: Waiting for work request...");
        if (can_bus_receive_message(&request, portMAX_DELAY) == ESP_OK) {
            if (request.identifier == CAN_ID_WORK_REQUEST && 
                request.data_length_code >= 1) {
                
                // Extract work duration (in 10ms units)
                uint8_t work_units = request.data[0];
                ESP_LOGI(TAG, "Worker: Received work request. Duration: %d units", work_units);
                
                // Simulate work
                simulate_work(work_units * 10);
                
                // Prepare response
                response.identifier = CAN_ID_WORK_RESPONSE;
                response.data_length_code = 1;
                response.flags = CAN_MSG_FLAG_NONE;
                response.data[0] = 1;  // 1 = success
                
                // Send response
                if (can_bus_send_message(&response, CAN_PRIORITY_NORMAL, 1000) != ESP_OK) {
                    ESP_LOGE(TAG, "Worker: Failed to send response");
                } else {
                    ESP_LOGI(TAG, "Worker: Sent response");
                }
            } else {
                ESP_LOGW(TAG, "Worker: Received unexpected message ID: 0x%X", 
                        request.identifier);
            }
        }
    }
}

#endif  // ROLE_WORKER

void app_main() {
    // Initialize CAN bus manager
    esp_err_t ret = can_bus_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN bus manager init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "CAN bus manager initialized");

    // Create LED GPIO configuration for visual feedback
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int led_state = 0;

    // Create task based on role
    #ifdef ROLE_MANAGER
    ESP_LOGI(TAG, "Starting as MANAGER");
    xTaskCreate(manager_task, "manager_task", 4096, NULL, 5, NULL);
    #endif
    
    #ifdef ROLE_WORKER
    ESP_LOGI(TAG, "Starting as WORKER");
    xTaskCreate(worker_task, "worker_task", 4096, NULL, 5, NULL);
    #endif

    // Blink LED to show the device is alive
    while (1) {
        gpio_set_level(GPIO_NUM_2, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}