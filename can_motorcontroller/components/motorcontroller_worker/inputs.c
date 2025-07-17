#include "inputs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>

#define DEBOUNCE_US (5000)          // 5ms debounce - increased for stability
#define STABLE_READINGS_REQUIRED 3  // Require 3 consecutive stable readings
#define STABILITY_CHECK_MS 50       // Check stability every 50ms (was 5ms - too aggressive)

static const char* TAG = "inputs";

typedef struct {
    gpio_num_t pin;
    const char* name;
    volatile bool current_state;     // Current debounced state
    volatile bool stable_state;      // Stable state used by application
    volatile bool edge_detected;     // Edge detection flag
    int64_t last_change_us;
    uint8_t stable_count;
    bool last_raw_reading;
} InputState;

typedef struct {
    uint8_t input_index;
    bool new_state;
    bool edge_detected;
} InputEvent;

static QueueHandle_t gpio_evt_queue;
static QueueHandle_t controller_notify_queue = NULL;  // Queue to notify controller

// Only 3 inputs now - removed manual controls
static InputState inputs[] = {
    { GPIO_WINCH_HOME,     "WINCH_HOME",     false, false, false, 0, 0, false },
    { GPIO_WINCH_TENSION,  "WINCH_TENSION",  false, false, false, 0, 0, false },
    { GPIO_WINCH_AUTO,     "WINCH_AUTO",     false, false, false, 0, 0, false },
};

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void notify_controller(uint8_t input_index, bool new_state, bool edge) {
    if (controller_notify_queue != NULL) {
        InputEvent event = {
            .input_index = input_index,
            .new_state = new_state,
            .edge_detected = edge
        };
        xQueueSend(controller_notify_queue, &event, 0); // Don't block
    }
}

static void gpio_task(void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t now = esp_timer_get_time();
            
            for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
                if (inputs[i].pin == io_num) {
                    bool raw = gpio_get_level(io_num);
                    bool level = !raw;  // aktiv LÅG tolkes som HØG
                    
                    // Check if this is a real change or just noise
                    if (level != inputs[i].last_raw_reading) {
                        inputs[i].last_raw_reading = level;
                        
                        // Apply debounce
                        if (now - inputs[i].last_change_us > DEBOUNCE_US) {
                            bool previous_state = inputs[i].current_state;
                            inputs[i].current_state = level;
                            inputs[i].last_change_us = now;
                            inputs[i].stable_count = 0; // Reset stability counter
                            
                            // Detect edges for critical sensors
                            if (level && !previous_state) {
                                inputs[i].edge_detected = true;
                                ESP_LOGI(TAG, "EDGE: GPIO %" PRIu32 " (%s): LOW→HIGH", 
                                        io_num, inputs[i].name);
                                
                                // Immediately notify controller of critical edges
                                if (inputs[i].pin == GPIO_WINCH_HOME) {
                                    notify_controller(i, level, true);
                                }
                            }
                            
                            ESP_LOGI(TAG, "GPIO %" PRIu32 " (%s): %s (debounced)", 
                                    io_num, inputs[i].name, level ? "HØG" : "LÅG");
                        }
                    }
                    break;
                }
            }
        }
    }
}

// Stability checking task - ensures inputs are really stable before using them
static void stability_task(void* arg) {
    while (1) {
        for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
            // Re-read the GPIO to confirm current state
            bool current_raw = gpio_get_level(inputs[i].pin);
            bool current_level = !current_raw;
            
            if (current_level == inputs[i].current_state) {
                // State is stable, increment counter
                if (inputs[i].stable_count < STABLE_READINGS_REQUIRED) {
                    inputs[i].stable_count++;
                    
                    // If we've reached required stable count and state changed
                    if (inputs[i].stable_count >= STABLE_READINGS_REQUIRED && 
                        inputs[i].stable_state != inputs[i].current_state) {
                        
                        inputs[i].stable_state = inputs[i].current_state;
                        
                        ESP_LOGI(TAG, "GPIO %d (%s): %s (STABLE)", 
                                inputs[i].pin, inputs[i].name, 
                                inputs[i].stable_state ? "HØG" : "LÅG");
                        
                        // Notify controller of stable state changes
                        notify_controller(i, inputs[i].stable_state, false);
                    }
                }
            } else {
                // State is not stable, reset
                inputs[i].current_state = current_level;
                inputs[i].stable_count = 0;
            }
            
            // Yield between iterations to prevent hogging CPU
            taskYIELD();
        }
        
        vTaskDelay(pdMS_TO_TICKS(STABILITY_CHECK_MS));
    }
}

void inputs_init(void) {
    ESP_LOGI(TAG, "Initializing input system...");
    
    // Initialize GPIO configuration
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << inputs[0].pin) |
                        (1ULL << inputs[1].pin) |
                        (1ULL << inputs[2].pin),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // Create queues
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Read initial states
    for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
        bool raw = gpio_get_level(inputs[i].pin);
        bool level = !raw;
        inputs[i].current_state = level;
        inputs[i].stable_state = level;
        inputs[i].last_raw_reading = level;
        inputs[i].stable_count = STABLE_READINGS_REQUIRED; // Start as stable
        ESP_LOGI(TAG, "Initial: %s = %s", 
                inputs[i].name, level ? "HØG" : "LÅG");
    }

    // Create tasks
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    xTaskCreate(stability_task, "stability_task", 2048, NULL, 5, NULL);  // Lower priority

    // Install ISR service and handlers
    gpio_install_isr_service(0);
    for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
        gpio_isr_handler_add(inputs[i].pin, gpio_isr_handler, (void*)inputs[i].pin);
    }
    
    ESP_LOGI(TAG, "Input system ready - HOME, TENSION, AUTO configured");
}

// Set the controller notification queue (called by controller)
void inputs_set_controller_queue(QueueHandle_t queue) {
    controller_notify_queue = queue;
}

// Standard state getters - use stable state
bool inputs_get_winch_home(void)   { return inputs[0].stable_state; }
bool inputs_get_winch_tension(void){ return inputs[1].stable_state; }
bool inputs_get_winch_auto(void)   { return inputs[2].stable_state; }

// Edge detection functions - check and clear flags
bool inputs_home_edge_detected(void) {
    bool detected = inputs[0].edge_detected;
    inputs[0].edge_detected = false; // Clear flag after reading
    return detected;
}

bool inputs_tension_edge_detected(void) {
    bool detected = inputs[1].edge_detected;
    inputs[1].edge_detected = false;
    return detected;
}

// Get input name for logging
const char* inputs_get_name(uint8_t index) {
    if (index < sizeof(inputs) / sizeof(inputs[0])) {
        return inputs[index].name;
    }
    return "UNKNOWN";
}