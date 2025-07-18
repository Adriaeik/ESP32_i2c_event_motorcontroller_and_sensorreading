/**
 * @file can_motctrl_common.c
 * @brief Implementation of common utility functions for CAN motor controller
 */

#include "can_motctrl_common.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

// Validation limits - easy to modify
#define MAX_DEPTH                    8000    // 80m in cm
#define MAX_PREV_END_DEPTH          10000    // 100m in cm  
#define MAX_SPEED_SCALED            50000    // 50 cm/s scaled (50.000 cm/s)
#define MAX_RISING_TIMEOUT_PERCENT  300      // 300%
#define MAX_STATIC_POINTS           MAX_POINTS       // Maximum number of static points (from buoye_structs.h)
#define MIN_SAMPLES_STATIC          1        // Minimum samples for STATIC_DEPTH
#define MIN_POLL_INTERVAL           1        // Minimum poll interval in seconds
static const char *TAG = "can_motctrl_common";


#define STATIC_DEPTH_OFFSET_S  1  // 1 second offset for static depth operations

/**
 * @brief Count number of static points in package
 * 
 * @param pkg Package containing static points
 * @return Number of non-zero static points
 */
static uint16_t count_static_points(const motorcontroller_pkg_t *pkg) {
    uint16_t count = 0;
    
    for (int i = 0; i < MAX_STATIC_POINTS; i++) {
        if (pkg->static_points[i] == 0) {
            break;  // End of array (zero-terminated)
        }
        count++;
    }
    
    return count;
}


uint32_t calculate_operation_timeout(const motorcontroller_pkg_t *pkg) {
    if (pkg == NULL) {
        ESP_LOGE(TAG, "Package is NULL");
        return 0;
    }

    uint32_t timeout_seconds = 0;

    switch (pkg->poll_type) {
        case LIN_TIME:
            // Simple: return poll interval in seconds
            timeout_seconds = pkg->static_poll_interval_s;
            ESP_LOGD(TAG, "LIN_TIME timeout: %d s", timeout_seconds);
            break;

        case ALPHA_DEPTH:
            // Calculate timeout as: end_depth / (prev_estimated_cm_per_s / 1000.0)
            // Use double precision to preserve accuracy
            if (pkg->prev_estimated_cm_per_s == 0) {
                ESP_LOGW(TAG, "Zero speed for ALPHA_DEPTH, using default timeout");
                timeout_seconds = 10; // 10 second default
            } else {
                // time = distance / speed (using double precision)
                double actual_speed_cm_per_s = pkg->prev_estimated_cm_per_s / 1000.0;
                double timeout_double = pkg->end_depth / actual_speed_cm_per_s;
                timeout_seconds = (uint32_t)(timeout_double + 0.5); // Round to nearest integer
            }
            
            ESP_LOGD(TAG, "ALPHA_DEPTH timeout: %d s (distance: %d cm / speed: %.3f cm/s)", 
                     timeout_seconds, pkg->end_depth, pkg->prev_estimated_cm_per_s / 1000.0);
            break;

        case STATIC_DEPTH: {
            // Count static points
            uint16_t num_points = count_static_points(pkg);
            
            if (num_points == 0) {
                ESP_LOGW(TAG, "No static points for STATIC_DEPTH, using default timeout");
                timeout_seconds = 10; // 10 second default
                break;
            }

            // Calculate travel time component: end_depth / (prev_estimated_cm_per_s / 1000.0)
            uint32_t travel_time_s = 0;
            if (pkg->prev_estimated_cm_per_s > 0) {
                // Use double precision to preserve accuracy
                double actual_speed_cm_per_s = pkg->prev_estimated_cm_per_s / 1000.0;
                double travel_time_double = pkg->end_depth / actual_speed_cm_per_s;
                travel_time_s = (uint32_t)(travel_time_double + 0.5); // Round to nearest integer
            }

            // Calculate static wait time per point: samples * (static_poll_interval_s + OFFSET)
            uint32_t static_wait_per_point_s = pkg->samples * 
                                              (pkg->static_poll_interval_s + STATIC_DEPTH_OFFSET_S);

            // Total time per point
            uint32_t time_per_point_s = travel_time_s + static_wait_per_point_s;

            // Total timeout for all points
            timeout_seconds = num_points * time_per_point_s;

            ESP_LOGD(TAG, "STATIC_DEPTH timeout: %d s (%d points × (%d s travel + %d s static))", 
                     timeout_seconds, num_points, travel_time_s, static_wait_per_point_s);
            break;
        }

        default:
            ESP_LOGE(TAG, "Unknown poll type: %d", pkg->poll_type);
            timeout_seconds = 10; // 10 second default
            break;
    }

    // Ensure minimum timeout
    if (timeout_seconds < 1) {
        ESP_LOGW(TAG, "Calculated timeout too short (%d s), using minimum 1 s", timeout_seconds);
        timeout_seconds = 1;
    }

    ESP_LOGI(TAG, "Calculated operation timeout: %d s for poll_type %d", 
             timeout_seconds, pkg->poll_type);
    
    return timeout_seconds;
}

uint32_t calculate_operation_timeout_with_margin(const motorcontroller_pkg_t *pkg, 
                                                uint8_t safety_margin_percent) {
    uint32_t base_timeout = calculate_operation_timeout(pkg);
    uint32_t margin = (base_timeout * safety_margin_percent) / 100;
    uint32_t total_timeout = base_timeout + margin;
    
    ESP_LOGI(TAG, "Timeout with %d%% margin: %d s (base: %d s + margin: %d s)",
             safety_margin_percent, total_timeout, base_timeout, margin);
    
    return total_timeout;
}


bool is_motorcontroller_pkg_valid(const motorcontroller_pkg_t *pkg, state_t expected_state) {
    if (pkg == NULL) {
        ESP_LOGE(TAG, "Package is NULL");
        return false;
    }

    // Check state validity - must match expected state
    if (pkg->STATE != expected_state) {
        ESP_LOGE(TAG, "Invalid state: %d, expected: %d", pkg->STATE, expected_state);
        return false;
    }

    // Check poll type validity
    if (pkg->poll_type != STATIC_DEPTH && pkg->poll_type != ALPHA_DEPTH) {
        ESP_LOGE(TAG, "Invalid poll type: %d", pkg->poll_type);
        return false;
    }

    // Check depth ranges
    if (pkg->end_depth > MAX_DEPTH) {
        ESP_LOGE(TAG, "End depth exceeds maximum: %d cm > %d cm", pkg->end_depth, MAX_DEPTH);
        return false;
    }

    if (pkg->prev_end_depth > MAX_PREV_END_DEPTH) {
        ESP_LOGE(TAG, "Previous end depth too large: %d cm > %d cm", 
                 pkg->prev_end_depth, MAX_PREV_END_DEPTH);
        return false;
    }

    // Check speed estimate (scaled value)
    if (pkg->prev_estimated_cm_per_s > MAX_SPEED_SCALED) {
        ESP_LOGE(TAG, "Speed estimate too high: %d (%.3f cm/s) > %d (%.3f cm/s)", 
                 pkg->prev_estimated_cm_per_s, pkg->prev_estimated_cm_per_s / 1000.0,
                 MAX_SPEED_SCALED, MAX_SPEED_SCALED / 1000.0);
        return false;
    }

    // Check rising timeout percentage
    if (pkg->rising_timeout_percent > MAX_RISING_TIMEOUT_PERCENT) {
        ESP_LOGE(TAG, "Rising timeout percentage too high: %d%% > %d%%", 
                 pkg->rising_timeout_percent, MAX_RISING_TIMEOUT_PERCENT);
        return false;
    }

    // Check poll interval
    if (pkg->static_poll_interval_s < MIN_POLL_INTERVAL) {
        ESP_LOGE(TAG, "Poll interval too short: %d s < %d s", 
                 pkg->static_poll_interval_s, MIN_POLL_INTERVAL);
        return false;
    }

    // Check alpha filter parameter (exclusive range)
    if (pkg->alpha <= 0.0f || pkg->alpha >= 1.0f) {
        ESP_LOGE(TAG, "Alpha parameter out of range: %f (must be 0.0 < α < 1.0)", pkg->alpha);
        return false;
    }

    // Beta is not used - no validation needed

    // Additional checks for STATIC_DEPTH mode
    if (pkg->poll_type == STATIC_DEPTH) {
        // Check minimum samples for static depth
        if (pkg->samples < MIN_SAMPLES_STATIC) {
            ESP_LOGE(TAG, "Insufficient samples for STATIC_DEPTH: %d < %d", 
                     pkg->samples, MIN_SAMPLES_STATIC);
            return false;
        }

        // Validate static points
        uint16_t point_count = 0;
        uint16_t highest_point = 0;
        uint16_t prev_point = 0;
        bool found_zero = false;

        for (int i = 0; i < MAX_STATIC_POINTS; i++) {
            uint16_t current_point = pkg->static_points[i];

            // Check for end of array (zero termination)
            if (current_point == 0) {
                found_zero = true;
                // All remaining points should be zero
                for (int j = i + 1; j < MAX_STATIC_POINTS; j++) {
                    if (pkg->static_points[j] != 0) {
                        ESP_LOGE(TAG, "Non-zero point after termination at index %d: %d", 
                                 j, pkg->static_points[j]);
                        return false;
                    }
                }
                break;
            }

            // Check point is within depth limit
            if (current_point > MAX_DEPTH) {
                ESP_LOGE(TAG, "Static point exceeds maximum depth at index %d: %d cm > %d cm",
                         i, current_point, MAX_DEPTH);
                return false;
            }

            // Check ascending order (except for first point)
            if (i > 0 && current_point <= prev_point) {
                ESP_LOGE(TAG, "Static points not in ascending order at index %d: %d <= %d",
                         i, current_point, prev_point);
                return false;
            }

            prev_point = current_point;
            highest_point = current_point;
            point_count++;
        }

        // Must have at least one static point
        if (point_count == 0) {
            ESP_LOGE(TAG, "No static points defined for STATIC_DEPTH mode");
            return false;
        }

        // For STATIC_DEPTH, prev_end_depth must equal highest static point
        if (pkg->end_depth != highest_point) {
            ESP_LOGE(TAG, "end_depth (%d) must equal highest static point (%d) for STATIC_DEPTH",
                     pkg->prev_end_depth, highest_point);
            return false;
        }

        ESP_LOGD(TAG, "STATIC_DEPTH validation passed: %d points, highest: %d cm", 
                 point_count, highest_point);
    }

    ESP_LOGD(TAG, "Package validation successful for state %d, poll_type %d", 
             pkg->STATE, pkg->poll_type);
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
    ESP_LOGI(tag, "  Previous Speed: %d cm/s", pkg->prev_estimated_cm_per_s/1000);
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
    bool valid = is_motorcontroller_pkg_valid(pkg, pkg->STATE);
    ESP_LOGI(tag, "  Validation: %s", valid ? "VALID" : "INVALID");
    
    // Calculate estimated timeout
    int timeout = calculate_operation_timeout(pkg);
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
    ESP_LOGI(tag, "  Estimated Speed: %d cm/s", resp->estimated_cm_per_s/1000);
    
    // Status interpretation
    if (resp->result == ESP_OK) {
        ESP_LOGI(tag, "  Status: Operation completed successfully");
    } else {
        ESP_LOGW(tag, "  Status: Operation failed - %s", esp_err_to_name(resp->result));
    }
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
