#include "motorcontroller_manager.h"
#include "can_motctrl_manager.h"
#include "can_motctrl_common.h"
#include "RTC_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "motorcontroller_manager";

// Manager event group bits with failure handling
#define MANAGER_TASK_STARTED_BIT        (1 << 0)
#define MANAGER_TASK_PKG_SENT_BIT       (1 << 1)
#define MANAGER_TASK_RESP_RECEIVED_BIT  (1 << 2)
#define MANAGER_TASK_COMPLETED_BIT      (1 << 3)
#define MANAGER_TASK_ERROR_BIT          (1 << 4)
#define MANAGER_TASK_TIMEOUT_BIT        (1 << 5)
#define MANAGER_TASK_CAN_FAIL_BIT       (1 << 6)
#define MANAGER_TASK_WORKER_FAIL_BIT    (1 << 7)

#define MANAGER_ALL_DONE_BITS (MANAGER_TASK_COMPLETED_BIT | MANAGER_TASK_ERROR_BIT | MANAGER_TASK_TIMEOUT_BIT)
#define MANAGER_ALL_FAIL_BITS (MANAGER_TASK_ERROR_BIT | MANAGER_TASK_TIMEOUT_BIT | MANAGER_TASK_CAN_FAIL_BIT | MANAGER_TASK_WORKER_FAIL_BIT)

// Manager context
typedef struct {
    EventGroupHandle_t event_group;
    TaskHandle_t task_handle;
    state_t target_state;
    motorcontroller_pkg_t current_pkg;
    motorcontroller_response_t received_resp;
    bool task_running;
} ManagerContext;

static ManagerContext s_manager_ctx = {0};

// Forward declarations
static void motorcontroller_manager_task(void *arg);

esp_err_t load_system_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t target_state) {
    ESP_LOGI(TAG, "Loading system motorcontroller package for state: %s", state_to_string(target_state));
    
    esp_err_t result = load_or_init_motorcontroller_pkg(pkg, target_state);
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Package loaded successfully from RTC");
    } else {
        ESP_LOGI(TAG, "Package initialized with defaults");
    }
    
    // Update for current operation
    pkg->STATE = target_state;
    
    // Validate package before returning
    if (!validate_motorcontroller_pkg(pkg)) {
        ESP_LOGE(TAG, "Package validation failed");
        return ESP_ERR_INVALID_ARG;
    }
    
    print_motorcontroller_pkg_info(pkg, TAG);
    return ESP_OK;
}

esp_err_t update_and_store_pkg(motorcontroller_pkg_t *pkg, const motorcontroller_response_t *resp) {
    ESP_LOGI(TAG, "Updating package with response data");
    
    if (!pkg || !resp) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Apply alpha-beta filtering to update speed estimate
    uint16_t new_speed = apply_manager_alpha_beta_filter(
        pkg->prev_estimated_cm_per_s,
        resp->working_time,
        pkg->end_depth,
        pkg->alpha,
        pkg->beta
    );
    
    // Update package with response data
    pkg->prev_working_time = resp->working_time;
    pkg->prev_estimated_cm_per_s = new_speed;
    pkg->prev_reported_depth = pkg->end_depth; // We reached the target depth
    pkg->prev_end_depth = pkg->end_depth;
    
    ESP_LOGI(TAG, "Speed updated: %d → %d cm/s", resp->estimated_cm_per_s, new_speed);
    
    // Store to RTC based on state
    esp_err_t result = ESP_OK;
    if (pkg->STATE == LOWERING) {
        result = rtc_save_motorcontroller_pkg_lowering(pkg);
    } else if (pkg->STATE == RISING) {
        result = rtc_save_motorcontroller_pkg_rising(pkg);
    }
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Package stored to RTC successfully");
    } else {
        ESP_LOGE(TAG, "Failed to store package to RTC: %s", esp_err_to_name(result));
    }
    
    return result;
}

esp_err_t motorcontroller_manager_start_worker(void) {
    ESP_LOGI(TAG, "Starting manager worker task");
    
    if (s_manager_ctx.task_running) {
        ESP_LOGW(TAG, "Manager task already running");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create event group if needed
    if (s_manager_ctx.event_group == NULL) {
        s_manager_ctx.event_group = xEventGroupCreate();
        if (s_manager_ctx.event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create event group");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Clear event group
    xEventGroupClearBits(s_manager_ctx.event_group, 0xFF);
    
    // Create task
    BaseType_t result = xTaskCreate(
        motorcontroller_manager_task,
        "motctrl_manager",
        4096,
        NULL,
        5,
        &s_manager_ctx.task_handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create manager task");
        return ESP_ERR_NO_MEM;
    }
    
    s_manager_ctx.task_running = true;
    ESP_LOGI(TAG, "Manager task started successfully");
    
    return ESP_OK;
}

esp_err_t motorcontroller_manager_wait_completion(uint32_t timeout_ms) {
    ESP_LOGI(TAG, "Waiting for manager completion (timeout: %d ms)", timeout_ms);
    
    if (s_manager_ctx.event_group == NULL) {
        ESP_LOGE(TAG, "Event group not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    EventBits_t bits = xEventGroupWaitBits(
        s_manager_ctx.event_group,
        MANAGER_ALL_DONE_BITS,
        pdFALSE, // clearOnExit = false
        pdFALSE, // wait for ANY bit (not all)
        pdMS_TO_TICKS(timeout_ms)
    );
    
    s_manager_ctx.task_running = false;
    
    if (bits & MANAGER_TASK_COMPLETED_BIT) {
        ESP_LOGI(TAG, "Manager task completed successfully");
        return ESP_OK;
    } else if (bits & MANAGER_TASK_TIMEOUT_BIT) {
        ESP_LOGE(TAG, "Manager task timed out");
        return ESP_ERR_TIMEOUT;
    } else if (bits & MANAGER_TASK_CAN_FAIL_BIT) {
        ESP_LOGE(TAG, "Manager task failed - CAN communication error");
        return ESP_FAIL;
    } else if (bits & MANAGER_TASK_WORKER_FAIL_BIT) {
        ESP_LOGE(TAG, "Manager task failed - Worker reported error");
        return ESP_ERR_INVALID_RESPONSE;
    } else if (bits & MANAGER_TASK_ERROR_BIT) {
        ESP_LOGE(TAG, "Manager task failed - General error");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Manager task timed out (no response)");
        return ESP_ERR_TIMEOUT;
    }
}

static void motorcontroller_manager_task(void *arg) {
    ESP_LOGI(TAG, "Manager task started");
    
    xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_STARTED_BIT);
    
    esp_err_t result = ESP_OK;
    
    // 1. Load package from RTC
    result = load_system_motorcontroller_pkg(&s_manager_ctx.current_pkg, s_manager_ctx.target_state);
    
    if (result == ESP_OK) {
        // 2. Send to worker via CAN
        ESP_LOGI(TAG, "Sending package to worker");
        result = start_worker(&s_manager_ctx.current_pkg, 5000);
        
        if (result == ESP_OK) {
            xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_PKG_SENT_BIT);
            ESP_LOGI(TAG, "Package sent successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send package: %s", esp_err_to_name(result));
            xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_CAN_FAIL_BIT);
            goto cleanup;
        }
    } else {
        ESP_LOGE(TAG, "Failed to load package: %s", esp_err_to_name(result));
        xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_ERROR_BIT);
        goto cleanup;
    }
    
    if (result == ESP_OK) {
        // 3. Calculate timeout based on package content
        uint32_t estimated_time = calculate_operation_timeout(
            s_manager_ctx.current_pkg.STATE,
            s_manager_ctx.current_pkg.prev_estimated_cm_per_s,
            s_manager_ctx.current_pkg.rising_timeout_percent,
            s_manager_ctx.current_pkg.end_depth,
            s_manager_ctx.current_pkg.static_points,
            s_manager_ctx.current_pkg.samples,
            s_manager_ctx.current_pkg.static_poll_interval_s
        ) * 1000; // Convert to ms
        
        ESP_LOGI(TAG, "Calculated operation timeout: %d ms", estimated_time);
        
        // 4. Wait for response with calculated timeout
        result = wait_for_worker(&s_manager_ctx.received_resp, estimated_time, estimated_time + 15000);
        
        if (result == ESP_OK) {
            xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_RESP_RECEIVED_BIT);
            ESP_LOGI(TAG, "Response received successfully");
            
            // 5. Update and store to RTC
            result = update_and_store_pkg(&s_manager_ctx.current_pkg, &s_manager_ctx.received_resp);
            
            // Check if worker reported failure
            if (s_manager_ctx.received_resp.result != ESP_OK) {
                ESP_LOGW(TAG, "Worker reported failure: %s", esp_err_to_name(s_manager_ctx.received_resp.result));
                xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_WORKER_FAIL_BIT);
            }
        } else if (result == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Timeout waiting for worker response");
            xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_TIMEOUT_BIT);
        } else {
            ESP_LOGE(TAG, "Failed to receive response: %s", esp_err_to_name(result));
            xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_CAN_FAIL_BIT);
        }
    }
    
cleanup:
    if (result == ESP_OK && !(xEventGroupGetBits(s_manager_ctx.event_group) & MANAGER_ALL_FAIL_BITS)) {
        xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_COMPLETED_BIT);
        ESP_LOGI(TAG, "Manager task completed successfully");
    } else {
        xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_ERROR_BIT);
        ESP_LOGE(TAG, "Manager task failed");
    }
    
    s_manager_ctx.task_handle = NULL;
    vTaskDelete(NULL); // Task deletes itself
}

// Helper function implementations

esp_err_t load_or_init_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t for_state) {
    if (!pkg) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_FAIL;
    
    // Try to load from RTC based on state
    if (for_state == LOWERING) {
        result = rtc_load_motorcontroller_pkg_lowering(pkg);
    } else if (for_state == RISING) {
        result = rtc_load_motorcontroller_pkg_rising(pkg);
    }
    
    if (result != ESP_OK) {
        // Initialize with defaults if no valid data in RTC
        ESP_LOGI(TAG, "No valid RTC data, initializing with defaults");
        motorcontroller_pkg_init_default(pkg);
        
        // Set state-specific defaults
        if (for_state == LOWERING) {
            pkg->STATE = LOWERING;
            pkg->end_depth = 500; // Default 5m depth
        } else if (for_state == RISING) {
            pkg->STATE = RISING;
            pkg->end_depth = 0; // Rise to surface
        } else {
            pkg->STATE = INIT;
            pkg->end_depth = 0;
        }
        pkg->STATE = LOWERING; // remove this!
        
        result = ESP_OK; // Default initialization succeeded
    }
    
    return result;
}

uint16_t apply_manager_alpha_beta_filter(uint16_t prev_estimate, 
                                        int actual_time, 
                                        uint16_t distance,
                                        double alpha, 
                                        double beta) {
    if (actual_time <= 0 || distance == 0) {
        ESP_LOGW(TAG, "Invalid parameters for alpha-beta filter");
        return prev_estimate;
    }
    
    // Calculate actual speed from this operation
    uint16_t actual_speed = distance / actual_time;
    
    // Apply alpha-beta filtering
    // Simple implementation: filtered_speed = alpha * actual_speed + (1 - alpha) * prev_estimate
    double filtered_speed = alpha * actual_speed + (1.0 - alpha) * prev_estimate;
    
    // Clamp to reasonable values
    if (filtered_speed < 5.0) {
        filtered_speed = 5.0;
    } else if (filtered_speed > 200.0) {
        filtered_speed = 200.0;
    }
    
    uint16_t result = (uint16_t)filtered_speed;
    
    ESP_LOGI(TAG, "Alpha-beta filter: prev=%d, actual=%d, filtered=%d (α=%.2f)", 
             prev_estimate, actual_speed, result, alpha);
    
    return result;
}

bool validate_motorcontroller_pkg(const motorcontroller_pkg_t *pkg) {
    if (!pkg) {
        ESP_LOGE(TAG, "NULL package pointer");
        return false;
    }
    
    // Use existing validation function
    bool valid = is_motorcontroller_pkg_valid(pkg);
    
    if (!valid) {
        ESP_LOGE(TAG, "Package validation failed");
    }
    
    return valid;
}