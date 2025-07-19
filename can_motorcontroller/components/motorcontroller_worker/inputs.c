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
    InputEventType event_type; //might replace name? 
    volatile bool current_state;     // Current debounced state
    volatile bool stable_state;      // Stable state used by application
    volatile bool edge_detected;     // Edge detection flag
    int64_t last_change_us;
    uint8_t stable_count;
    bool last_raw_reading;
} InputState;

// Global state tracking
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t controller_notify_queue = NULL;  // Queue to notify controller
static TaskHandle_t gpio_task_handle = NULL;
static TaskHandle_t stability_task_handle = NULL;
static volatile bool tasks_should_exit = false;
static bool inputs_initialized = false;

// Only 3 inputs now - removed manual controls
static InputState inputs[] = {
    { GPIO_WINCH_HOME,     "WINCH_HOME",     HOME, false, false, false, 0, 0, false },
    { GPIO_WINCH_TENSION,  "WINCH_TENSION",  TENTION, false, false, false, 0, 0, false },
    { GPIO_WINCH_AUTO,     "WINCH_AUTO",     AUTO, false, false, false, 0, 0, false },
};

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    if (gpio_evt_queue != NULL) {
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
    portYIELD_FROM_ISR();
}

static InputEventType get_input_event_type(int i){
    if (i>2) return TIMEOUT;
    return inputs[i].event_type;
}

static void notify_controller(uint8_t input_index, bool new_state, bool edge) {
    if (controller_notify_queue != NULL) {
        // ESP_LOGI(TAG, "Notifying! Queue: %p, Space: %d", 
        //          controller_notify_queue, uxQueueSpacesAvailable(controller_notify_queue));
        InputEvent event = {
            .event_type = get_input_event_type(input_index),
            .state = new_state,
            .edge_detected = edge
        };
        if ( xQueueSend(controller_notify_queue, &event, pdMS_TO_TICKS(10)) != pdPASS) {
            ESP_LOGW(TAG, "Kø full! Mistet event: %d", event.event_type);
        } 
    } else {
        ESP_LOGE(TAG, "Queue was NULL!!!");
    }
}

static void gpio_task(void* arg) {
    ESP_LOGI(TAG, "GPIO task started");
    uint32_t io_num;
    
    while (!tasks_should_exit) {
        if (xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (tasks_should_exit) break;
            
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
                            
                            // ESP_LOGI(TAG, "GPIO %" PRIu32 " (%s): %s (debounced)", 
                            //         io_num, inputs[i].name, level ? "HØG" : "LÅG");
                        }
                    }
                    break;
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "GPIO task exiting");
    gpio_task_handle = NULL;
    vTaskDelete(NULL);
}

// Stability checking task - ensures inputs are really stable before using them
static void stability_task(void* arg) {
    ESP_LOGI(TAG, "Stability task started");
    
    while (!tasks_should_exit) {
        for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
            if (tasks_should_exit) break;
            
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
    
    ESP_LOGI(TAG, "Stability task exiting");
    stability_task_handle = NULL;
    vTaskDelete(NULL);
}

void inputs_init(QueueHandle_t queue) {
    if (inputs_initialized) {
        ESP_LOGW(TAG, "Input system already initialized");
        return;
    }
    controller_notify_queue = queue;
    
    ESP_LOGI(TAG, "Initializing input system...");
    
    // Reset flags
    tasks_should_exit = false;
    
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
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return;
    }
    
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
        ESP_LOGI(TAG,
                 "GPIO %d konfigurert som INPUT for «%s» (pull‑up aktivert)",
                 inputs[i].pin,
                 inputs[i].name);
    }

    // Create tasks
    BaseType_t gpio_task_result = xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, &gpio_task_handle);
    BaseType_t stability_task_result = xTaskCreate(stability_task, "stability_task", 2048, NULL, 5, &stability_task_handle);
    
    if (gpio_task_result != pdPASS || stability_task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create input tasks");
        inputs_deinit(); // Cleanup on failure
        return;
    }

    // Install ISR service and handlers
    esp_err_t isr_result = gpio_install_isr_service(0);
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(isr_result));
        inputs_deinit();
        return;
    }
    
    for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
        esp_err_t add_result = gpio_isr_handler_add(inputs[i].pin, gpio_isr_handler, (void*)inputs[i].pin);
        if (add_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler for pin %d: %s", inputs[i].pin, esp_err_to_name(add_result));
        }
    }
    
    inputs_initialized = true;
    ESP_LOGI(TAG, "Input system ready - HOME, TENSION, AUTO configured");
}

void inputs_deinit(void) {
    if (!inputs_initialized) {
        ESP_LOGD(TAG, "Input system not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Deinitializing input system...");
    
    // Signal tasks to exit
    tasks_should_exit = true;
    
    // Remove ISR handlers first
    for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
        gpio_isr_handler_remove(inputs[i].pin);
    }
    
    // Wait for tasks to exit with timeout
    if (gpio_task_handle != NULL) {
        // Send dummy message to wake up gpio task
        uint32_t dummy = 0;
        if (gpio_evt_queue != NULL) {
            xQueueSend(gpio_evt_queue, &dummy, pdMS_TO_TICKS(100));
        }
        
        // Give task time to exit
        for (int i = 0; i < 10 && gpio_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        // Force delete if still running
        if (gpio_task_handle != NULL) {
            ESP_LOGW(TAG, "Force deleting gpio_task");
            vTaskDelete(gpio_task_handle);
            gpio_task_handle = NULL;
        }
    }
    
    if (stability_task_handle != NULL) {
        // Give task time to exit
        for (int i = 0; i < 10 && stability_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        // Force delete if still running
        if (stability_task_handle != NULL) {
            ESP_LOGW(TAG, "Force deleting stability_task");
            vTaskDelete(stability_task_handle);
            stability_task_handle = NULL;
        }
    }
    
    // Delete queue
    if (gpio_evt_queue != NULL) {
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
    }
    
    // Clear controller notification queue reference
    controller_notify_queue = NULL;
    
    // Reset all input states
    for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
        inputs[i].current_state = false;
        inputs[i].stable_state = false;
        inputs[i].edge_detected = false;
        inputs[i].last_change_us = 0;
        inputs[i].stable_count = 0;
        inputs[i].last_raw_reading = false;
    }
    
    inputs_initialized = false;
    ESP_LOGI(TAG, "Input system deinitialized");
}



// Standard state getters - use stable state
bool inputs_get_winch_home(void)   { return inputs_initialized ? inputs[0].stable_state : false; }
bool inputs_get_winch_tension(void){ return inputs_initialized ? inputs[1].stable_state : false; }
bool inputs_get_winch_auto(void)   { return inputs_initialized ? inputs[2].stable_state : false; }
bool inputs_get_auto_enable(void)  { return !inputs_get_winch_auto();}
// Edge detection functions - check and clear flags
bool inputs_home_edge_detected(void) {
    if (!inputs_initialized) return false;
    bool detected = inputs[0].edge_detected;
    inputs[0].edge_detected = false; // Clear flag after reading
    return detected;
}

bool inputs_tension_edge_detected(void) {
    if (!inputs_initialized) return false;
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