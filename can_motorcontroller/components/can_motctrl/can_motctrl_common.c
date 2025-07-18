/**
 * @file can_motctrl_common.c
 * @brief Implementation of common utility functions for CAN motor controller
 */

#include "can_motctrl_common.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "can_motctrl_common";


int calculate_operation_timeout(state_t state, 
                               uint16_t prev_estimated_cm_per_s,
                               int rising_timeout_percent,
                               uint16_t end_depth,
                               const uint16_t *static_points,
                               uint16_t samples,
                               uint16_t static_poll_interval_s)
{
    // Input validation
    if (prev_estimated_cm_per_s == 0) {
        prev_estimated_cm_per_s = 50; // Default 50 cm/s
    }
    if (end_depth == 0) {
        return CAN_MOTCTRL_MIN_TIMEOUT; // No movement needed
    }
    
    // Calculate movement time: distance(cm) / speed(cm/s) = time(s)
    int movement_time = end_depth / prev_estimated_cm_per_s;
    
    // Add time for static measurements
    int static_time = 0;
    if (static_points != NULL) {
        int point_count = 0;
        for (int i = 0; i < MAX_POINTS && static_points[i] > 0; i++) {
            point_count++;
        }
        static_time = point_count * samples * static_poll_interval_s;
    }
    
    // Base timeout = movement + static measurements
    int base_timeout = movement_time + static_time;
    
    // Apply rising timeout percentage for rising operations
    if (state == RISING && rising_timeout_percent > 0) {
        base_timeout = (base_timeout * (100 + rising_timeout_percent)) / 100;
    }
    
    // Add safety margin (50% for robustness)
    base_timeout = (base_timeout * 150) / 100;
    
    // Enforce reasonable limits
    if (base_timeout < CAN_MOTCTRL_MIN_TIMEOUT) {
        base_timeout = CAN_MOTCTRL_MIN_TIMEOUT;
    }
    if (base_timeout > CAN_MOTCTRL_MAX_TIMEOUT) {
        base_timeout = CAN_MOTCTRL_MAX_TIMEOUT;
    }
    
    ESP_LOGD("timeout_calc", "Calculated timeout: %ds (movement:%ds, static:%ds, state:%s)", 
             base_timeout, movement_time, static_time, state_to_string(state));
    
    return base_timeout;
}



const char* get_worker_status_string(worker_status_t status)
{
    switch (status) {
        case WORKER_STATUS_IDLE:        return "INIT";
        case WORKER_STATUS_READY:       return "READY";
        case WORKER_STATUS_WORKING:     return "WORKING";
        case WORKER_STATUS_RESP_READY:  return "RESP_READY";
        case WORKER_STATUS_ERROR:       return "ERROR";
        default:                        return "UNKNOWN";
    }
}

bool is_motorcontroller_pkg_valid(const motorcontroller_pkg_t *pkg)
{
    if (pkg == NULL) {
        return false;
    }
    
    // Check state validity
    if (pkg->STATE != INIT && pkg->STATE != LOWERING && pkg->STATE != RISING) {
        ESP_LOGE(TAG, "Invalid state: %d", pkg->STATE);
        return false;
    }
    
    // Check poll type validity
    if (pkg->poll_type != STATIC_DEPTH && pkg->poll_type != ALPHA_DEPTH) {
        ESP_LOGE(TAG, "Invalid poll type: %d", pkg->poll_type);
        return false;
    }
    
    // Check depth ranges (reasonable limits)
    if (pkg->end_depth > 10000) {  // 100 meters max
        ESP_LOGE(TAG, "End depth too large: %d cm", pkg->end_depth);
        return false;
    }
    
    if (pkg->prev_reported_depth > 10000) {
        ESP_LOGE(TAG, "Previous depth too large: %d cm", pkg->prev_reported_depth);
        return false;
    }
    
    // Check speed estimate reasonableness
    if (pkg->prev_estimated_cm_per_s/1000 > 1000) {  // 10 m/s max
        ESP_LOGE(TAG, "Speed estimate too high: %d cm/s", pkg->prev_estimated_cm_per_s/1000);
        return false;
    }
    
    // Check sample count
    if (pkg->samples > 100) {  // Reasonable limit
        ESP_LOGE(TAG, "Too many samples: %d", pkg->samples);
        return false;
    }
    
    // Check poll interval
    if (pkg->static_poll_interval_s > 3600) {  // 1 hour max
        ESP_LOGE(TAG, "Poll interval too long: %d seconds", pkg->static_poll_interval_s);
        return false;
    }
    
    // Check filter parameters
    if (pkg->alpha < 0.0 || pkg->alpha > 1.0) {
        ESP_LOGE(TAG, "Alpha parameter out of range: %f", pkg->alpha);
        return false;
    }
    
    if (pkg->beta < 0.0 || pkg->beta > 1.0) {
        ESP_LOGE(TAG, "Beta parameter out of range: %f", pkg->beta);
        return false;
    }
    
    // Check static points validity
    uint16_t prev_point = 0;
    for (int i = 0; i < MAX_POINTS; i++) {
        if (pkg->static_points[i] == 0) {
            break;  // End of array
        }
        
        if (pkg->static_points[i] <= prev_point) {
            ESP_LOGE(TAG, "Static points not in ascending order at index %d: %d <= %d", 
                     i, pkg->static_points[i], prev_point);
            return false;
        }
        
        if (pkg->static_points[i] > pkg->end_depth) {
            ESP_LOGE(TAG, "Static point beyond end depth at index %d: %d > %d", 
                     i, pkg->static_points[i], pkg->end_depth);
            return false;
        }
        
        prev_point = pkg->static_points[i];
    }
    
    return true;
}

void print_motorcontroller_pkg_info(const motorcontroller_pkg_t *pkg, const char *tag)
{
    if (pkg == NULL || tag == NULL) {
        return;
    }
    
    ESP_LOGI(tag, "=== Motor Controller Package ===");
    ESP_LOGI(tag, "  State: %s", state_to_string(pkg->STATE));
    ESP_LOGI(tag, "  End Depth: %d cm", pkg->end_depth);
    ESP_LOGI(tag, "  Previous Depth: %d cm", pkg->prev_reported_depth);
    ESP_LOGI(tag, "  Poll Type: %s", pkg->poll_type == STATIC_DEPTH ? "STATIC" : "ALPHA_DEPTH");
    ESP_LOGI(tag, "  Samples: %d", pkg->samples);
    ESP_LOGI(tag, "  Poll Interval: %d seconds", pkg->static_poll_interval_s);
    ESP_LOGI(tag, "  Previous Speed: %d cm/s", pkg->prev_estimated_cm_per_s);
    ESP_LOGI(tag, "  Previous Work Time: %d seconds", pkg->prev_working_time);
    ESP_LOGI(tag, "  Rising Timeout: %d%%", pkg->rising_timeout_percent);
    ESP_LOGI(tag, "  Filter: α=%.3f, β=%.3f", pkg->alpha, pkg->beta);
    
    // Print static points
    ESP_LOGI(tag, "  Static Points:");
    bool has_points = false;
    for (int i = 0; i < MAX_POINTS && pkg->static_points[i] > 0; i++) {
        ESP_LOGI(tag, "    [%d] %d cm", i, pkg->static_points[i]);
        has_points = true;
    }
    if (!has_points) {
        ESP_LOGI(tag, "    (none)");
    }
    
    // Validation
    bool valid = is_motorcontroller_pkg_valid(pkg);
    ESP_LOGI(tag, "  Validation: %s", valid ? "VALID" : "INVALID");
    
    // Calculate estimated timeout
    int timeout = calculate_operation_timeout(
        pkg->STATE, pkg->prev_estimated_cm_per_s, pkg->rising_timeout_percent,
        pkg->end_depth, pkg->static_points, pkg->samples, pkg->static_poll_interval_s
    );
    ESP_LOGI(tag, "  Estimated Timeout: %d seconds", timeout);
}

void print_motorcontroller_response_info(const motorcontroller_response_t *resp, const char *tag)
{
    if (resp == NULL || tag == NULL) {
        return;
    }
    
    ESP_LOGI(tag, "=== Motor Controller Response ===");
    ESP_LOGI(tag, "  State: %s", state_to_string(resp->STATE));
    ESP_LOGI(tag, "  Result: %s (%d)", esp_err_to_name(resp->result), resp->result);
    ESP_LOGI(tag, "  Working Time: %d seconds", resp->working_time);
    ESP_LOGI(tag, "  Estimated Speed: %d cm/s", resp->estimated_cm_per_s);
    
    // Calculate some derived information
    if (resp->working_time > 0) {
        ESP_LOGI(tag, "  Performance: %.1f cm/s average", 
                (float)resp->estimated_cm_per_s);
    }
    
    // Status interpretation
    if (resp->result == ESP_OK) {
        ESP_LOGI(tag, "  Status: Operation completed successfully");
    } else {
        ESP_LOGW(tag, "  Status: Operation failed - %s", esp_err_to_name(resp->result));
    }
}