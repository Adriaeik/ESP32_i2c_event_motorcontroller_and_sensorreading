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




static const char *TAG = "MAIN";

// Initialize default packages
static void init_default_packages(motorcontroller_pkg_t *lowering_pkg, motorcontroller_pkg_t *rising_pkg) {
    // Default LOWERING package (ALPHA_DEPTH)
    memset(lowering_pkg, 0, sizeof(motorcontroller_pkg_t));
    motorcontroller_pkg_init_default(lowering_pkg);
    lowering_pkg->STATE = LOWERING;

    // Default RISING package (ALPHA_DEPTH)
    memset(rising_pkg, 0, sizeof(motorcontroller_pkg_t));
    motorcontroller_pkg_init_default(rising_pkg);
}

// Update packages with operation results and prepare for next cycle
static void update_packages_after_operation(motorcontroller_pkg_t *pkg, const motorcontroller_response_t *resp, 
                                           uint16_t new_reported_depth) {
    // Update previous operation data
    pkg->prev_working_time = resp->working_time;
    pkg->prev_reported_depth = new_reported_depth;
    
    ESP_LOGI(TAG, "Package updated - Working time: %ds, Speed: %d cm/s, Depth: %d cm",
             resp->working_time, resp->estimated_cm_per_s, new_reported_depth);
}

// Create static depth configuration
static void setup_static_depth_packages(motorcontroller_pkg_t *lowering_pkg, motorcontroller_pkg_t *rising_pkg) {
    // LOWERING with static points
    lowering_pkg->poll_type = STATIC_DEPTH;
    lowering_pkg->end_depth = 80;  // 8 meters total
    
    // Static points for lowering: 200cm, 400cm, 600cm, 800cm
    lowering_pkg->static_points[0] = 20;  // 2m
    lowering_pkg->static_points[1] = 40;  // 4m  
    lowering_pkg->static_points[2] = 60;  // 6m
    lowering_pkg->static_points[3] = 80;  // 8m
    lowering_pkg->static_points[4] = 0;    // Terminator
    
    // RISING with static points (same points in reverse)
    rising_pkg->poll_type = STATIC_DEPTH;
    rising_pkg->end_depth = 80;     // Home position
    rising_pkg->prev_reported_depth = 80; // Starting from bottom
    
    // Static points for rising: 600cm, 400cm, 200cm, then home
    rising_pkg->static_points[0] = 60;    // 6m
    rising_pkg->static_points[1] = 40;    // 4m
    rising_pkg->static_points[2] = 20;    // 2m
    rising_pkg->static_points[3] = 0;      // Terminator (then continue to home)
    
    ESP_LOGI(TAG, "Static depth packages configured - 4 points each direction");
}

// Simulate depth changes based on operation
static uint16_t simulate_depth_after_operation(const motorcontroller_pkg_t *pkg, const motorcontroller_response_t *resp) {
    if (resp->result != ESP_OK) {
        // If operation failed, assume we didn't move much
        return pkg->prev_reported_depth;
    }
    
    // Simulate depth based on state
    switch (pkg->STATE) {
        case LOWERING:
            // Assume we reached target depth (or close to it)
            return pkg->end_depth;
            
        case RISING:
        case INIT:
            // Assume we reached home (depth 0)
            return pkg->end_depth;
            
        default:
            return pkg->prev_reported_depth;
    }
}

// Uncomment for manager, comment for worker
// #define ROLE_MANAGER         // <------------

#ifdef ROLE_MANAGER
#include "motorcontroller_manager.h"
#include "esp_random.h"
static float random_float_range(float min, float max)
{
    // esp_random() returns 32‑bit tilfeldig
    uint32_t r = esp_random();
    float t = (float)r / (float)UINT32_MAX;  // normalisert [0,1]
    return min + t * (max - min);
}

void manager_main_task(void *arg) {
    ESP_LOGI(TAG, "Manager main task started");
    can_bus_manager_deinit();
    
    motorcontroller_pkg_t lowering_pkg, rising_pkg;
    esp_err_t ret;
        init_default_packages(&lowering_pkg, &rising_pkg);
        rtc_save_motorcontroller_pkg_lowering(&lowering_pkg);
        rtc_save_motorcontroller_pkg_rising(&rising_pkg);
        

    uint32_t cycle_count = 0;
    state_t state = INIT;
    while (1) {
        switch (state)
        {
        case LOWERING:
            state = RISING;
            break;
        
        default:
            state = LOWERING;
            break;
        }
        cycle_count++;

        ESP_LOGI(TAG, "=== Starting Cycle #%d ===", cycle_count);
        
        // Initialize CAN bus for everyone to use
        ESP_ERROR_CHECK(can_bus_manager_init());

        // Start manager worker task (non-blocking)
        ret = motorcontroller_manager_start_worker(state); // maybe make this retur/modify a timout to use also + starttick -> then call motorcontroller_manager_wait_completion(start_tick,wait_ticks); so it behaves lig wait until..
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start manager worker: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        
        // Wait for completion (high timeout, will return when worker responds)
        ret = motorcontroller_manager_wait_completion(300000); // 5 minutes timeout
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 1 failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        ESP_LOGW(TAG,"================================================================");
        ESP_LOGI(TAG,"|                  === Cycle #%d Complete ===                    |", cycle_count);
        ESP_LOGW(TAG,"================================================================");
        printf("\n\n\n");
        can_bus_manager_deinit();
        float depth = random_float_range(3.0f, 5.0f);
        ESP_LOGI(TAG,"reported depth: %.2f", depth);
        update_reported_depth(depth);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Long pause between full cycles
    }
}

#else
#include "motorcontroller_worker.h"
#define ONLY_WINCH //<-----------------

#ifndef ONLY_WINCH

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
        esp_sleep_enable_timer_wakeup(1 * 1000000); // 1 seconds for testing
        //TODO:: fix pins
        // esp_sleep_enable_ext1_wakeup(GPIO_NUM_1, ESP_EXT1_WAKEUP_ANY_HIGH); // Boot button
        // esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, ESP_EXT1_WAKEUP_ANY_HIGH); // Boot by manager
        
        // Enter deep sleep
        esp_deep_sleep_start();
    }
}
#else
static const char* TEST_TAG = "winch_test";

// ============================================================================
// TEST OPERATION DEFINITIONS
// ============================================================================

typedef enum {
    TEST_GO_HOME = 0,
    TEST_LOWER_LIN_TIME,
    TEST_RISING_LIN_TIME, 
    TEST_LOWER_ALPHA_DEPTH,
    TEST_RISING_ALPHA_DEPTH,
    TEST_LOWERING_STATIC_DEPTH,
    TEST_RISING_STATIC_DEPTH,
    TEST_MAX_OPERATIONS
} test_operation_t;

static const char* test_operation_names[] = {
    "GO_HOME",
    "LOWER_LIN_TIME", 
    "RISING_LIN_TIME",
    "LOWER_ALPHA_DEPTH",
    "RISING_ALPHA_DEPTH", 
    "LOWERING_STATIC_DEPTH",
    "RISING_STATIC_DEPTH"
};

// ============================================================================
// TEST HELPER FUNCTIONS
// ============================================================================

static void print_test_separator(int cycle_count, test_operation_t operation) {
    ESP_LOGW(TEST_TAG,"================================================================");
    ESP_LOGI(TEST_TAG,"|              === Cycle #%d: %s ===              |", 
             cycle_count, test_operation_names[operation]);
    ESP_LOGW(TEST_TAG,"================================================================");
}

static void print_cycle_complete(int cycle_count) {
    ESP_LOGW(TEST_TAG,"================================================================");
    ESP_LOGI(TEST_TAG,"|                  === Cycle #%d Complete ===                    |", cycle_count);
    ESP_LOGW(TEST_TAG,"================================================================");
    printf("\n\n\n");
}

static void print_operation_result(esp_err_t result, const motorcontroller_response_t *resp) {
    if (result == ESP_OK) {
        ESP_LOGI(TEST_TAG, "✅ Operation completed successfully!");
        if (resp) {
            print_motorcontroller_response_info(resp, TEST_TAG);
        }
    } else {
        ESP_LOGE(TEST_TAG, "❌ Operation failed with error: %s", esp_err_to_name(result));
        if (resp) {
            print_motorcontroller_response_info(resp, TEST_TAG);
        }
    }
}

// ============================================================================
// TEST PACKAGE SETUP
// ============================================================================

static void setup_test_package(motorcontroller_pkg_t *pkg, test_operation_t operation) {
    // Start with defaults
    motorcontroller_pkg_init_default(pkg);
    
    switch (operation) {
        case TEST_GO_HOME:
            pkg->STATE = INIT;
            pkg->poll_type = ALPHA_DEPTH;
            pkg->end_depth = 300; 
            pkg->estimated_cm_per_s_x1000 = 15000; // 15.000 cm/s for safety
            break;
            
        case TEST_LOWER_LIN_TIME:
            pkg->STATE = LOWERING;
            pkg->poll_type = LIN_TIME;
            pkg->end_depth = 300; // Not used for LIN_TIME but set for clarity
            pkg->static_poll_interval_s = 10; // 10 seconds lowering
            pkg->estimated_cm_per_s_x1000 = 12000; // 12.000 cm/s
            break;
            
        case TEST_RISING_LIN_TIME:
            pkg->STATE = RISING;
            pkg->poll_type = LIN_TIME;
            pkg->end_depth = 300;  // Going to home
            pkg->static_poll_interval_s = 8; // 8 seconds max before home expected
            pkg->rising_timeout_percent = 150; // 8 * 1.5 = 12 seconds timeout
            pkg->estimated_cm_per_s_x1000 = 10000; // 10.000 cm/s upward
            break;
            
        case TEST_LOWER_ALPHA_DEPTH:
            pkg->STATE = LOWERING;
            pkg->poll_type = ALPHA_DEPTH;
            pkg->end_depth = 400; // 4 meters down
            pkg->estimated_cm_per_s_x1000 = 15000; // 15.000 cm/s
            break;
            
        case TEST_RISING_ALPHA_DEPTH:
            pkg->STATE = RISING;
            pkg->poll_type = ALPHA_DEPTH;
            pkg->end_depth = 400; // 4 meters down
            pkg->rising_timeout_percent = 120; // 20% safety margin
            pkg->estimated_cm_per_s_x1000 = 12000; // 12.000 cm/s upward
            break;
            
        case TEST_LOWERING_STATIC_DEPTH:
            pkg->STATE = LOWERING;
            pkg->poll_type = STATIC_DEPTH;
            pkg->end_depth = 600; // 6 meters total
            pkg->static_points[0] = 100; // 1m
            pkg->static_points[1] = 250; // 2.5m  
            pkg->static_points[2] = 450; // 4.5m
            pkg->static_points[3] = 600; // 6m (final)
            pkg->static_points[4] = 0;   // Null terminator
            pkg->samples = 3; // 3 samples at each point
            pkg->static_poll_interval_s = 2; // 2 seconds between samples
            pkg->estimated_cm_per_s_x1000 = 8000; // 8.000 cm/s slow for precision
            break;
            
        case TEST_RISING_STATIC_DEPTH:
            pkg->STATE = RISING;
            pkg->poll_type = STATIC_DEPTH;
            pkg->end_depth = 600; // 6 meters total
            pkg->static_points[0] = 100; // 1m
            pkg->static_points[1] = 250; // 2.5m  
            pkg->static_points[2] = 450; // 4.5m
            pkg->static_points[3] = 600; // 6m (final)
            pkg->static_points[4] = 0;   // Null terminator
            pkg->samples = 2; // 2 samples at each point
            pkg->static_poll_interval_s = 3; // 3 seconds between samples
            pkg->rising_timeout_percent = 130; // 30% safety margin
            pkg->estimated_cm_per_s_x1000 = 10000; // 10.000 cm/s upward
            break;
            
        default:
            ESP_LOGE(TEST_TAG, "Unknown test operation: %d", operation);
            break;
    }
}

// ============================================================================
// INDIVIDUAL TEST OPERATIONS
// ============================================================================

static esp_err_t run_test_operation(test_operation_t operation, int cycle_count) {
    motorcontroller_pkg_t pkg;
    motorcontroller_response_t resp;
    esp_err_t result;
    
    print_test_separator(cycle_count, operation);
    
    // Setup package for this operation
    if (operation == TEST_GO_HOME) {
        // Use the dedicated go home function
        ESP_LOGI(TEST_TAG, "Executing: winch_go_to_home_position()");
        result = winch_go_to_home_position();
        
        // For go home, we don't have package info to print
        ESP_LOGI(TEST_TAG, "Go home operation completed with result: %s", esp_err_to_name(result));
        
    } else {
        // Setup and print package info
        setup_test_package(&pkg, operation);
        ESP_LOGI(TEST_TAG, "📦 Prepared test package:");
        print_motorcontroller_pkg_info(&pkg, TEST_TAG);
        
        ESP_LOGI(TEST_TAG, "🚀 Executing: winch_execute_operation()");
        
        // Execute the operation
        result = winch_execute_operation(&pkg, &resp);
        
        // Print results
        print_operation_result(result, &resp);
    }
    
    ESP_LOGI(TEST_TAG, "⏰ Waiting 3 seconds before next operation...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    return result;
}

// ============================================================================
// MAIN TEST LOOP
// ============================================================================

static void run_test_cycle(int cycle_count) {
    ESP_LOGI(TEST_TAG, "🔄 Starting test cycle #%d", cycle_count);
    
    for (test_operation_t op = 0; op < TEST_MAX_OPERATIONS; op++) {
        esp_err_t result = run_test_operation(op, cycle_count);
        // esp_err_t result = run_test_operation(TEST_RISING_STATIC_DEPTH, cycle_count);
        
        if (result != ESP_OK) {
            ESP_LOGE(TEST_TAG, "💥 Test operation %s FAILED! Aborting cycle.", 
                     test_operation_names[op]);
            return;
        }
    }
    
    print_cycle_complete(cycle_count);
}

void start_winch_main(void) {
    ESP_LOGI(TEST_TAG, "🧪 Winch State Machine Test Starting...");
    
    // Initialize the winch controller
    ESP_LOGI(TEST_TAG, "🔧 Initializing winch controller...");
    esp_err_t init_result = winch_controller_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TEST_TAG, "❌ Failed to initialize winch controller: %s", 
                 esp_err_to_name(init_result));
        return;
    }
    ESP_LOGI(TEST_TAG, "✅ Winch controller initialized successfully!");
    
    // Wait a moment for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Run test cycles
    int cycle_count = 1;
    while (true) {
        ESP_LOGI(TEST_TAG, "\n🏁 === STARTING TEST CYCLE #%d ===", cycle_count);
        
        run_test_cycle(cycle_count);
        
        ESP_LOGI(TEST_TAG, "😴 Cycle #%d complete. Waiting 10 seconds before next cycle...", 
                 cycle_count);
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        cycle_count++;
        
        // Optional: Limit number of cycles for testing
        if (cycle_count > 5) {
            ESP_LOGI(TEST_TAG, "🛑 Test completed after %d cycles. Stopping.", cycle_count - 1);
            break;
        }
    }
    
    // Cleanup
    ESP_LOGI(TEST_TAG, "🧹 Deinitializing winch controller...");
    winch_controller_deinit();
    ESP_LOGI(TEST_TAG, "✅ Test complete!");
}

// ============================================================================
// ALTERNATIVE SINGLE OPERATION TEST FUNCTION
// ============================================================================

/**
 * @brief Test a single operation (useful for debugging specific operations)
 * @param operation The operation to test
 */
void test_single_operation(test_operation_t operation) {
    ESP_LOGI(TEST_TAG, "🔍 Testing single operation: %s", test_operation_names[operation]);
    
    esp_err_t init_result = winch_controller_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(TEST_TAG, "Failed to initialize winch controller");
        return;
    }
    
    esp_err_t result = run_test_operation(operation, 1);
    
    ESP_LOGI(TEST_TAG, "Single operation test result: %s", esp_err_to_name(result));
    
    winch_controller_deinit();
}

#endif //ONLY_WINCH
#endif //ROLE_MANAGER

void app_main() {
    // Initialize CAN bus manager
    ESP_ERROR_CHECK(can_bus_manager_init());    
    // Create role-specific task
#ifdef ROLE_MANAGER

    xTaskCreate(manager_main_task, "manager_main", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started as MANAGER");
#else
    #ifdef ONLY_WINCH
    start_winch_main();
    #else
    xTaskCreate(worker_main_task, "worker_main", 8192, NULL,5, NULL);
    #endif //ONLY_WINCH
    ESP_LOGI(TAG, "Started as WORKER");
#endif
    // keepalive
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}