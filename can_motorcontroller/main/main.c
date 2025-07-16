#include "can_bus_manager.h"
#include "can_motctrl_manager.h"
#include "can_motctrl_worker.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/task.h"

// Uncomment for manager, comment for worker
// #define ROLE_MANAGER
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