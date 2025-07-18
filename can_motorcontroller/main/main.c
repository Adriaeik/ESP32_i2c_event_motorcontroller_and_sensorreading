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
    lowering_pkg->STATE = LOWERING;
    lowering_pkg->prev_working_time = 0;
    lowering_pkg->rising_timeout_percent = 30;
    lowering_pkg->prev_reported_depth = 0;
    lowering_pkg->prev_end_depth = 0;
    lowering_pkg->prev_estimated_cm_per_s = 15000;  // 15 cm/s initially
    lowering_pkg->poll_type = ALPHA_DEPTH;
    lowering_pkg->end_depth = 300;  // 3 meters
    lowering_pkg->samples = 3;
    lowering_pkg->static_poll_interval_s = 1;
    lowering_pkg->alpha = 0.9;
    lowering_pkg->beta = 0.1;
    
    // Default RISING package (ALPHA_DEPTH)
    memset(rising_pkg, 0, sizeof(motorcontroller_pkg_t));
    rising_pkg->STATE = RISING;
    rising_pkg->prev_working_time = 20;
    rising_pkg->rising_timeout_percent = 30;
    rising_pkg->prev_reported_depth = 300;
    rising_pkg->prev_end_depth = 300;
    rising_pkg->prev_estimated_cm_per_s = 15000;   // 1.8 cm/s initially (faster up)
    rising_pkg->poll_type = ALPHA_DEPTH;
    rising_pkg->end_depth = 300;  // start position when rising
    rising_pkg->samples = 3;
    rising_pkg->static_poll_interval_s = 1;
    rising_pkg->alpha = 0.9;
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
#define ROLE_MANAGER         // <------------

#ifdef ROLE_MANAGER
#include "motorcontroller_manager.h"

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
        vTaskDelay(pdMS_TO_TICKS(5000)); // Long pause between full cycles
    }
}

#else
#include "motorcontroller_worker.h"
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