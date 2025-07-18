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

// Uncomment for manager, comment for worker
#define MOTORCONTROLL_TEST         // <------------

static const char *TAG = "MAIN";

#ifdef MOTORCONTROLL_TEST


//////////////////////////////////////TEST FUNCTIONS - speed estimate etc..

// Test helper macros
#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            ESP_LOGE(TAG, "FAIL: %s", message); \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_EQUAL_UINT32(expected, actual, message) \
    do { \
        if ((expected) != (actual)) { \
            ESP_LOGE(TAG, "FAIL: %s - Expected: %u, Actual: %u", message, (unsigned)(expected), (unsigned)(actual)); \
            return false; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual, message) \
    do { \
        if (fabs((expected) - (actual)) > (delta)) { \
            ESP_LOGE(TAG, "FAIL: %s - Expected: %.3f, Actual: %.3f", message, (expected), (actual)); \
            return false; \
        } \
    } while(0)

// Function declarations - these should be implemented in your motor controller module

bool test_calculate_expected_time_ms_normal_cases() {
    ESP_LOGI(TAG, "Testing calculate_expected_time_ms normal cases...");
    
    // Test: 100cm at 10 cm/s (scaled 10000) should take 10 seconds = 10000ms
    TEST_ASSERT_EQUAL_UINT32(10000, calculate_expected_time_ms(100, 10000), "100cm at 10cm/s");
    
    // Test: 50cm at 5 cm/s (scaled 5000) should take 10 seconds = 10000ms
    TEST_ASSERT_EQUAL_UINT32(10000, calculate_expected_time_ms(50, 5000), "50cm at 5cm/s");
    
    // Test: 200cm at 20 cm/s (scaled 20000) should take 10 seconds = 10000ms
    TEST_ASSERT_EQUAL_UINT32(10000, calculate_expected_time_ms(200, 20000), "200cm at 20cm/s");
    
    // Test: 30cm at 15 cm/s (scaled 15000) should take 2 seconds = 2000ms
    TEST_ASSERT_EQUAL_UINT32(2000, calculate_expected_time_ms(30, 15000), "30cm at 15cm/s");
    
    ESP_LOGI(TAG, "PASS: calculate_expected_time_ms normal cases");
    return true;
}

bool test_calculate_expected_time_ms_edge_cases() {
    ESP_LOGI(TAG, "Testing calculate_expected_time_ms edge cases...");
    
    // Test: Zero speed should return default 10000ms
    TEST_ASSERT_EQUAL_UINT32(10000, calculate_expected_time_ms(100, 0), "Zero speed");
    
    // Test: Very high speed should return default 10000ms
    TEST_ASSERT_EQUAL_UINT32(10000, calculate_expected_time_ms(100, 50000), "Very high speed");
    
    ESP_LOGI(TAG, "PASS: calculate_expected_time_ms edge cases");
    return true;
}

bool test_update_speed_estimate_convergence() {
    ESP_LOGI(TAG, "Testing update_speed_estimate convergence...");
    
    motorcontroller_pkg_t pkg;
    memset(&pkg, 0, sizeof(pkg));
    
    // Set up test parameters - aggressive learning for slow system
    pkg.alpha = 0.9;  // Aggressive learning since updates are infrequent
    pkg.beta = 0.1;   // Not used anymore
    pkg.prev_estimated_cm_per_s = UINT16_MAX; // Initial estimate: 10 cm/s
    
    ESP_LOGI(TAG, "Initial speed estimate: %.1f cm/s", pkg.prev_estimated_cm_per_s / 1000.0);
    
    // Simulate multiple operations where actual speed is consistently 8.3 cm/s
    // (100cm in 12 seconds = 8.33 cm/s)
    for (int iteration = 1; iteration <= 5; iteration++) {  // Reduced iterations since aggressive learning
        pkg.prev_working_time = 12; 
        pkg.prev_reported_depth = 100;
        
        uint16_t new_estimate = update_speed_estimate_pre_operation(&pkg);
        
        ESP_LOGI(TAG, "Iteration %d: Speed estimate = %.1f cm/s", 
               iteration, new_estimate / 1000.0);
        
        // Update for next iteration
        pkg.prev_estimated_cm_per_s = new_estimate;
    }
    
    // With aggressive learning, should converge quickly to actual 8.3 cm/s
    double final_estimate = pkg.prev_estimated_cm_per_s / 1000.0;
    TEST_ASSERT_FLOAT_WITHIN(0.5, 8.3, final_estimate, "Speed estimate convergence to actual speed");
    
    ESP_LOGI(TAG, "PASS: update_speed_estimate convergence");
    return true;
}

bool test_update_speed_estimate_edge_cases() {
    ESP_LOGI(TAG, "Testing update_speed_estimate edge cases...");
    
    motorcontroller_pkg_t pkg;
    memset(&pkg, 0, sizeof(pkg));
    
    // Test: No previous operation data
    pkg.prev_working_time = 0;
    pkg.prev_reported_depth = 0;
    pkg.prev_estimated_cm_per_s = 15000;
    
    uint16_t result = update_speed_estimate_pre_operation(&pkg);
    TEST_ASSERT_EQUAL_UINT32(15000, result, "No previous data should return previous estimate");
    
    // Test: Zero working time
    pkg.prev_working_time = 0;
    pkg.prev_reported_depth = 100;
    pkg.prev_estimated_cm_per_s = 15000;
    
    result = update_speed_estimate_pre_operation(&pkg);
    TEST_ASSERT_EQUAL_UINT32(15000, result, "Zero working time should return previous estimate");
    
    ESP_LOGI(TAG, "PASS: update_speed_estimate edge cases");
    return true;
}

bool test_realistic_scenario() {
    ESP_LOGI(TAG, "Testing realistic operational scenario...");
    
    motorcontroller_pkg_t pkg;
    memset(&pkg, 0, sizeof(pkg));
    
    // Realistic parameters for hourly updates
    pkg.alpha = 0.9;  // Aggressive learning for infrequent updates
    pkg.beta = 0.2;   // Not used in current implementation
    pkg.prev_estimated_cm_per_s = 12000; // Initial estimate: 12 cm/s
    
    ESP_LOGI(TAG, "Initial speed estimate: %.1f cm/s", pkg.prev_estimated_cm_per_s / 1000.0);
    
    // Simulate 5 operations with varying actual speeds
    int operations[][2] = {
        {150, 15}, // 150cm in 15s = 10 cm/s
        {200, 18}, // 200cm in 18s = 11.1 cm/s  
        {120, 12}, // 120cm in 12s = 10 cm/s
        {180, 17}, // 180cm in 17s = 10.6 cm/s
        {160, 16}  // 160cm in 16s = 10 cm/s
    };
    
    for (int i = 0; i < 5; i++) {
        pkg.prev_reported_depth = operations[i][0];
        pkg.prev_working_time = operations[i][1];
        
        double actual_speed = (double)operations[i][0] / operations[i][1];
        
        uint16_t new_estimate = update_speed_estimate_pre_operation(&pkg);
        
        // Test expected time calculation with new estimate
        uint32_t expected_time = calculate_expected_time_ms(operations[i][0], new_estimate);
        
        ESP_LOGI(TAG, "Op %d: %dcm in %ds (%.1f cm/s actual) -> estimate: %.1f cm/s, expected time: %.1fs",
               i+1, operations[i][0], operations[i][1], actual_speed,
               new_estimate / 1000.0, expected_time / 1000.0);
        
        // Update for next iteration
        pkg.prev_estimated_cm_per_s = new_estimate;
    }
    
    // Final estimate should be reasonable (around 10-11 cm/s)
    double final_estimate = pkg.prev_estimated_cm_per_s / 1000.0;
    TEST_ASSERT(final_estimate > 9.0 && final_estimate < 12.0, "Final estimate should be in reasonable range");
    
    ESP_LOGI(TAG, "PASS: realistic scenario");
    return true;
}

void run_all_tests() {
    ESP_LOGI(TAG, "========== MOTOR CONTROLLER TEST SUITE ==========");
    
    int passed = 0;
    int total = 0;
    
    // Test calculate_expected_time_ms
    total++; if (test_calculate_expected_time_ms_normal_cases()) passed++;
    total++; if (test_calculate_expected_time_ms_edge_cases()) passed++;
    
    // Test update_speed_estimate_pre_operation  
    total++; if (test_update_speed_estimate_convergence()) passed++;
    total++; if (test_update_speed_estimate_edge_cases()) passed++;
    
    // Integration test
    total++; if (test_realistic_scenario()) passed++;
    
    ESP_LOGI(TAG, "========== TEST RESULTS ==========");
    ESP_LOGI(TAG, "Passed: %d/%d tests", passed, total);
    
    if (passed == total) {
        ESP_LOGI(TAG, "🎉 ALL TESTS PASSED! 🎉");
    } else {
        ESP_LOGE(TAG, "❌ Some tests failed. Check output above.");
    }
    ESP_LOGI(TAG, "=====================================");
}

// Initialize default packages
static void init_default_packages(motorcontroller_pkg_t *lowering_pkg, motorcontroller_pkg_t *rising_pkg) {
    // Default LOWERING package (ALPHA_DEPTH)
    memset(lowering_pkg, 0, sizeof(motorcontroller_pkg_t));
    lowering_pkg->STATE = LOWERING;
    lowering_pkg->prev_working_time = 0;
    lowering_pkg->rising_timeout_percent = 30;
    lowering_pkg->prev_reported_depth = 0;
    lowering_pkg->prev_end_depth = 0;
    lowering_pkg->prev_estimated_cm_per_s = 15000;  // 15 cm/s initially
    lowering_pkg->poll_type = ALPHA_DEPTH;
    lowering_pkg->end_depth = 100;  // 5 meters
    lowering_pkg->samples = 3;
    lowering_pkg->static_poll_interval_s = 1;
    lowering_pkg->alpha = 0.2;
    lowering_pkg->beta = 0.1;
    
    // Default RISING package (ALPHA_DEPTH)
    memset(rising_pkg, 0, sizeof(motorcontroller_pkg_t));
    rising_pkg->STATE = RISING;
    rising_pkg->prev_working_time = 0;
    rising_pkg->rising_timeout_percent = 130;
    rising_pkg->prev_reported_depth = 200;
    rising_pkg->prev_end_depth = 100;
    rising_pkg->prev_estimated_cm_per_s = 15000;   // 1.8 cm/s initially (faster up)
    rising_pkg->poll_type = ALPHA_DEPTH;
    rising_pkg->end_depth = 100;  // start position when rising
    rising_pkg->samples = 3;
    rising_pkg->static_poll_interval_s = 1;
    rising_pkg->alpha = 0.2;
    rising_pkg->beta = 0.1;
}

// Update packages with operation results and prepare for next cycle
static void update_packages_after_operation(motorcontroller_pkg_t *pkg, const motorcontroller_response_t *resp, 
                                           uint16_t new_reported_depth) {
    // Update previous operation data
    pkg->prev_working_time = resp->working_time;
    pkg->prev_estimated_cm_per_s = resp->estimated_cm_per_s;
    pkg->prev_end_depth = pkg->end_depth;
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
    rising_pkg->end_depth = 0;     // Home position
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

// Main worker task - cycles through all operation modes
void winch_main_task(void *pvParameters) {
    ESP_LOGI(TAG, "Worker main task started");
    
    motorcontroller_pkg_t lowering_pkg, rising_pkg;
    motorcontroller_response_t resp;
    esp_err_t ret;
    // Try to load from RTC, use defaults if not available
    if (rtc_load_motorcontroller_pkg_lowering(&lowering_pkg) != ESP_OK) {
        ESP_LOGI(TAG, "No saved lowering package, using defaults");
    }
    if (rtc_load_motorcontroller_pkg_rising(&rising_pkg) != ESP_OK) {
        ESP_LOGI(TAG, "No saved rising package, using defaults");
    }
    
    // Initialize default packages
    init_default_packages(&lowering_pkg, &rising_pkg);
    print_motorcontroller_pkg_info(&rising_pkg, TAG);

    uint32_t cycle_count = 0;
    
    while (1) {
        cycle_count++;
        ESP_LOGI(TAG, "=== Starting Cycle #%d ===", cycle_count);
        
        // ====================================================================
        // PHASE 1: LOWERING (ALPHA_DEPTH)
        // ====================================================================
        ESP_LOGI(TAG, "Phase 1: LOWERING (ALPHA_DEPTH)");
        lowering_pkg.poll_type = ALPHA_DEPTH;
        
        loop1:
        ret = do_work(&lowering_pkg, &resp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 1 failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
            goto loop1;
        }
        
        // Update package and simulate new depth
        uint16_t new_depth = simulate_depth_after_operation(&lowering_pkg, &resp);
        update_packages_after_operation(&lowering_pkg, &resp, new_depth);
        rtc_save_motorcontroller_pkg_lowering(&lowering_pkg);
        
        // Update rising package with current position
        rising_pkg.prev_reported_depth = new_depth;
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Brief pause
        
        // ====================================================================
        // PHASE 2: RISING (ALPHA_DEPTH)
        // ====================================================================
        ESP_LOGI(TAG, "Phase 2: RISING (ALPHA_DEPTH)");
        rising_pkg.poll_type = ALPHA_DEPTH;
        ESP_LOGI(TAG, "depth %d", rising_pkg.end_depth);
        loop2:
        ret = do_work(&rising_pkg, &resp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 2 failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
            goto loop2;
        }
        
        // Update package
        new_depth = simulate_depth_after_operation(&rising_pkg, &resp);
        update_packages_after_operation(&rising_pkg, &resp, new_depth);
        rtc_save_motorcontroller_pkg_rising(&rising_pkg);
        
        // Update lowering package with current position
        lowering_pkg.prev_reported_depth = new_depth;
        
        vTaskDelay(pdMS_TO_TICKS(3000)); // Longer pause between cycles
        
        // ====================================================================
        // PHASE 3: LOWERING (STATIC_DEPTH)
        // ====================================================================
        ESP_LOGI(TAG, "Phase 3: LOWERING (STATIC_DEPTH)");
        setup_static_depth_packages(&lowering_pkg, &rising_pkg);
        
        loop3:
        ret = do_work(&lowering_pkg, &resp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 3 failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
            goto loop3;
        }
        
        // Update package
        new_depth = simulate_depth_after_operation(&lowering_pkg, &resp);
        update_packages_after_operation(&lowering_pkg, &resp, new_depth);
        rtc_save_motorcontroller_pkg_lowering(&lowering_pkg);
        
        // Update rising package
        rising_pkg.prev_reported_depth = new_depth;
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // ====================================================================
        // PHASE 4: RISING (STATIC_DEPTH)
        // ====================================================================
        ESP_LOGI(TAG, "Phase 4: RISING (STATIC_DEPTH)");
        // Static setup already done in phase 3
        
        loop4:
        ret = do_work(&rising_pkg, &resp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 4 failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(2000));
            goto loop4;
        }
        
        // Update package
        new_depth = simulate_depth_after_operation(&rising_pkg, &resp);
        update_packages_after_operation(&rising_pkg, &resp, new_depth);
        rtc_save_motorcontroller_pkg_rising(&rising_pkg);
        
        // Update lowering package for next cycle
        lowering_pkg.prev_reported_depth = new_depth;
        
        ESP_LOGI(TAG, "=== Cycle #%d Complete ===", cycle_count);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Long pause between full cycles
    }
}


// Function to start the worker (call this from app_main)
void start_winch_main(void) {

    xTaskCreate(winch_main_task, "worker_main", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Worker main task created");
}

#else
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
#endif //MOTORCONTROLL_TEST


void app_main() {
#ifdef MOTORCONTROLL_TEST

    // start_winch_main();
    run_all_tests();

    while (1) vTaskDelay(portMAX_DELAY);  // keep alive
#else
    // Initialize CAN bus manager
    ESP_ERROR_CHECK(can_bus_manager_init());
    
    // Initialize LED for status indication
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    
    // Create role-specific task
#ifdef ROLE_MANAGER
    xTaskCreate(manager_main_task, "manager_main", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started as MANAGER");
#else
    // this does not work
    xTaskCreate(worker_main_task, "worker_main", 8192, NULL,5, NULL);

    ESP_LOGI(TAG, "Started as WORKER");
#endif

    
    // Main LED blink loop with role-specific patterns
    int blink_count = 0;
    while (1) {
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
        if (++blink_count % 10 == 0) {
#ifdef ROLE_MANAGER
            ESP_LOGI(TAG, "Manager operational - cycles running");
#else
            ESP_LOGI(TAG, "Worker operational - waiting for work");
#endif
        }
        taskYIELD();
    }
#endif //MOTORCONTROLL_TEST
}
























































// #include "can_bus_manager.h"
// #include "can_motctrl_manager.h"
// #include "can_motctrl_worker.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "esp_random.h"
// #include "freertos/task.h"

// // Uncomment for manager, comment for worker
// #define ROLE_MANAGER
// // #define STRESS_TEST_ENABLED  // Comment to disable stress test

// static const char *TAG = "MAIN";

// // Stress test task
// void can_stress_task(void *arg) {
//     ESP_LOGI(TAG, "Starting CAN stress test");
    
//     while (1) {
//         can_message_t msg;
        
//         // Generate random message parameters
//         msg.identifier = esp_random() % 0x7FF;  // 11-bit IDs
//         msg.extd = false;
//         msg.rtr = false;
//         msg.data_length_code = esp_random() % 9;  // 0-8 bytes
        
//         // Fill data with random bytes
//         uint32_t rand_data = esp_random();
//         for (int i = 0; i < msg.data_length_code; i++) {
//             msg.data[i] = ((uint8_t *)&rand_data)[i % 4];
//         }
        
//         // Send with random priority and timeout
//         uint32_t timeout = 10 + (esp_random() % 50);
//         esp_err_t ret = can_bus_send_message(&msg, timeout);
        
//         if (ret != ESP_OK) {
//             ESP_LOGW(TAG, "Stress send failed: %s (ID: 0x%03X)", 
//                     esp_err_to_name(ret), msg.identifier);
//         }
        
//         // Random delay between messages (0-20ms)
//         vTaskDelay(pdMS_TO_TICKS(10*esp_random() % 2000));
//     }
// }

// void app_main() {
//     // Initialize CAN bus manager
//     ESP_ERROR_CHECK(can_bus_manager_init());
    
//     // Initialize LED
//     gpio_reset_pin(GPIO_NUM_2);
//     gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    
//     // Create role-specific task
// #ifdef ROLE_MANAGER
//     xTaskCreate(manager_task, "manager_task", 8192, NULL, 5, NULL);
//     ESP_LOGI(TAG, "Started as MANAGER");
// #else
//     xTaskCreate(worker_task, "worker_task", 8192, NULL, 5, NULL);
//     ESP_LOGI(TAG, "Started as WORKER");
// #endif

// #ifdef STRESS_TEST_ENABLED
//     // Create stress test task (lower priority than application tasks)
//     xTaskCreate(can_stress_task, "can_stress", 4096, NULL, 2, NULL);
//     ESP_LOGI(TAG, "CAN stress test ENABLED");
// #endif

//     // Blink LED with pattern based on role
//     int blink_count = 0;
//     while (1) {
//         // Different blink patterns for manager/worker
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
//         if (++blink_count % 20 == 0) {
//             ESP_LOGI(TAG, "System operational");
//         }
//     }
// }