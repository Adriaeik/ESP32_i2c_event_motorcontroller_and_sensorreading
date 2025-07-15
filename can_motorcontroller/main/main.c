#include "can_bus_manager.h"
#include "can_motctrl_manager.h"  // For manager
#include "can_motctrl_worker.h"   // For worker
#include "driver/gpio.h"

// Uncomment for manager, comment for worker
// #define ROLE_MANAGER      // <----------------------------

void app_main() {
    // Initialize CAN bus manager
    ESP_ERROR_CHECK(can_bus_manager_init());
    
    // Initialize LED
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    
    // Create role-specific task
    #ifdef ROLE_MANAGER
    xTaskCreate(manager_task, "manager_task", 8192, NULL, 5, NULL);
    #else
    xTaskCreate(worker_task, "worker_task", 8192, NULL, 5, NULL);
    #endif

    // Blink LED
    int led_state = 0;
    while (1) {
        gpio_set_level(GPIO_NUM_2, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}