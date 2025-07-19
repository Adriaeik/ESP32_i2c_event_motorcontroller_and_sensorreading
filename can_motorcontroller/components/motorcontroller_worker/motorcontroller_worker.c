#include "motorcontroller_worker.h"
#include "can_motctrl_common.h"
#include "winch_controller.h"
#include "inputs.h"
#include "outputs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

static const char *TAG = "motorcontroller_worker";

esp_err_t motorcontroller_worker_init_hardware(void) {
    ESP_LOGI(TAG, "Initializing worker hardware");
    
    outputs_init();
    // inputs_init(NULL);
    
    ESP_LOGI(TAG, "Worker hardware initialized successfully");
    return ESP_OK;
}


esp_err_t do_work(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    ESP_LOGI(TAG, "Starting work - State: %s, End depth: %d cm", 
             state_to_string(pkg->STATE), pkg->end_depth);
    
    esp_err_t result = winch_controller_init();     // Safe multiple calls
    if (result != ESP_OK) return result;

    memset(resp, 0, sizeof(motorcontroller_response_t));
    resp->STATE = pkg->STATE;
    
    result = winch_execute_operation(pkg, resp);
    if (result != ESP_OK) {
        winch_controller_deinit();                  // Cleanup on error
        return result;
    }
    
    result = winch_controller_deinit();             // Cleanup after success
    ESP_LOGI(TAG, "Ferdig");
    return result;
}


bool wakeup_reason_is_ext1(void) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    return (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1);
}

uint32_t calculate_pure_movement_time(uint32_t total_time_ms,
                                     const uint16_t *static_points,
                                     uint16_t samples,
                                     uint16_t static_poll_interval_s) {
    if (static_points == NULL || samples == 0 || static_poll_interval_s == 0) {
        return total_time_ms; // No static polling, all time is movement
    }
    
    // Count static points
    uint16_t static_point_count = 0;
    for (int i = 0; i < MAX_POINTS && static_points[i] > 0; i++) {
        static_point_count++;
    }
    
    // Calculate total static time
    uint32_t total_static_time_ms = static_point_count * samples * static_poll_interval_s * 1000;
    
    // Subtract static time from total time
    if (total_static_time_ms >= total_time_ms) {
        ESP_LOGW(TAG, "Static time (%d ms) >= total time (%d ms)", 
                 total_static_time_ms, total_time_ms);
        return 100; // Minimum movement time
    }
    
    uint32_t pure_movement_time = total_time_ms - total_static_time_ms;
    
    ESP_LOGD(TAG, "Pure movement time: %d ms (total: %d ms, static: %d ms)", 
             pure_movement_time, total_time_ms, total_static_time_ms);
    
    return pure_movement_time;
}

// maybe remove this.. 
uint16_t calculate_new_speed_estimate(uint16_t distance_cm, uint32_t pure_movement_time_ms) {
    if (pure_movement_time_ms == 0 || distance_cm == 0) {
        ESP_LOGW(TAG, "Invalid parameters for speed calculation: distance=%d, time=%d", 
                 distance_cm, pure_movement_time_ms);
        return 50; // Default 50 cm/s
    }
    
    // Speed = distance(cm) / time(s)
    uint32_t pure_movement_time_s = pure_movement_time_ms / 1000;
    if (pure_movement_time_s == 0) {
        pure_movement_time_s = 1; // Minimum 1 second
    }
    
    uint16_t speed_estimate = distance_cm / pure_movement_time_s;
    
    // Sanity check - reasonable winch speeds
    if (speed_estimate < 5) {
        speed_estimate = 5;   // Minimum 5 cm/s
    } else if (speed_estimate > 200) {
        speed_estimate = 200; // Maximum 200 cm/s
    }
    
    ESP_LOGI(TAG, "Speed estimate: %d cm/s (distance: %d cm, time: %d s)", 
             speed_estimate, distance_cm, pure_movement_time_s);
    
    return speed_estimate;
}