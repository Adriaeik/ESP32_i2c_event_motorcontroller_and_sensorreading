#include "motorcontroller_manager.h"
#include "can_motctrl_manager.h"
#include "can_motctrl_common.h"
#include "RTC_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
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
// === Helper Functions (Internal) ===

/**
 * @brief Load motorcontroller package from RTC or initialize with defaults
 * @param pkg Output package
 * @param for_state Load package for specific state (LOWERING/RISING have separate estimates)
 * @return ESP_OK on success, ESP_FAIL if no valid data (will use defaults)
 */
esp_err_t load_state_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t for_state);

/**
 * @brief Validate motorcontroller package before sending
 * @param pkg Package to validate
 * @return true if valid, false otherwise
 */
bool validate_motorcontroller_pkg(const motorcontroller_pkg_t *pkg, state_t state);

esp_err_t load_system_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t target_state) {
    ESP_LOGI(TAG, "Loading system motorcontroller package for state: %s", state_to_string(target_state));
    
    esp_err_t result = load_state_motorcontroller_pkg(pkg, target_state);
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Package loaded successfully from RTC");
    } else {
        ESP_LOGI(TAG, "Package initialized with defaults");
    }
    
    // Update for current operation
    pkg->STATE = target_state; // should be done beforehand maybe?
    
    // Validate package before returning
    if (!validate_motorcontroller_pkg(pkg, target_state)) {
        ESP_LOGE(TAG, "Package validation failed");
        return ESP_ERR_INVALID_ARG;
    }
    
    print_motorcontroller_pkg_info(pkg, TAG);
    return ESP_OK;
}

/**
 * @brief Set the depth reached to both rising and lowering and store to RTC
 *
 * This updates both the rising and lowering motorcontroller packages'
 * prev_reported_depth fields in centimeters, saving them to RTC
 *
 * @param depth_meters  Depth in meters; must be > 0
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t update_reported_depth(float depth_meters) {
    uint16_t depth_cm = (uint16_t)(depth_meters * 100.0f);
    if (depth_cm == 0) return ESP_ERR_INVALID_STATE;

    motorcontroller_pkg_t lowering_pkg, rising_pkg;
    ESP_RETURN_ON_ERROR(rtc_load_motorcontroller_pkg_lowering(&lowering_pkg), TAG, "failed to load lowering_pkg");
    ESP_RETURN_ON_ERROR(rtc_load_motorcontroller_pkg_rising(&rising_pkg),   TAG, "failed to load rising_pkg");

    lowering_pkg.prev_reported_depth = depth_cm;
    rising_pkg.prev_reported_depth   = depth_cm;

    ESP_RETURN_ON_ERROR(rtc_save_motorcontroller_pkg_lowering(&lowering_pkg), TAG, "failed to store lowering_pkg");
    ESP_RETURN_ON_ERROR(rtc_save_motorcontroller_pkg_rising(&rising_pkg),     TAG, "failed to store rising_pkg");

    return ESP_OK;
}

esp_err_t update_and_store_pkg(motorcontroller_pkg_t *pkg, const motorcontroller_response_t *resp) {
    ESP_LOGI(TAG, "Updating package with response data");
    
    if (!pkg || !resp) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update package with response data
    pkg->prev_working_time = resp->working_time;
    // all these should be the same .. maybe create som checks?
    // pkg->prev_estimated_cm_per_s = resp->estimated_cm_per_s; 
    // pkg->prev_end_depth = pkg->end_depth; // should equal pkg->end_depth
    
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

esp_err_t motorcontroller_manager_start_worker(state_t state) {
    ESP_LOGI(TAG, "Starting manager worker task");
    s_manager_ctx.target_state = state;
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


uint16_t update_speed_estimate(const motorcontroller_pkg_t *pkg) {
    // Start with previous estimate - fallback
    uint16_t prev_speed_estimate = pkg->prev_estimated_cm_per_s;
    
    // If we have previous operation data, update speed estimate
    if (prev_speed_estimate > 0 && pkg->prev_reported_depth > 0) {
        
        double actual_distance_cm = (double)pkg->prev_reported_depth;
        double actual_time_s = (double)pkg->prev_working_time;
        
        if (actual_time_s > 0.1 && actual_distance_cm > 0) {
            // Calculate measured speed from last operation
            double measured_speed_cm_per_s = actual_distance_cm / actual_time_s;
            double previous_speed_cm_per_s = (double)pkg->prev_estimated_cm_per_s / 1000.0;
            
            // Use exponential moving average instead of alpha-beta filter
            // alpha acts as the learning rate (0.0 = no update, 1.0 = full replacement)
            double updated_speed = previous_speed_cm_per_s * (1.0 - pkg->alpha) + 
                                 measured_speed_cm_per_s * pkg->alpha;
            
            // Convert back to scaled format
            uint16_t new_estimate = (uint16_t)(updated_speed * 1000.0);
            
            if (new_estimate > 0 && new_estimate < 50000) {
                ESP_LOGI(TAG, "Speed estimate updated: %.2f -> %.2f cm/s (measured: %.1f cm/s, distance: %.0f cm, time: %.1f s)", 
                         previous_speed_cm_per_s, updated_speed, measured_speed_cm_per_s,
                         actual_distance_cm, actual_time_s);
                return new_estimate;
            } else {
                ESP_LOGW(TAG, "Speed estimate rejected (out of range): %.1f cm/s", 
                         updated_speed);
                return 0; // fail
            }
        }
    }
    return prev_speed_estimate;
}

esp_err_t update_and_save_speed_estimate(motorcontroller_pkg_t *pkg){
    uint16_t speed_estimate = update_speed_estimate(pkg);
    if (speed_estimate>0){
        pkg->prev_estimated_cm_per_s = speed_estimate;
        if (pkg->STATE == LOWERING) {
            return rtc_save_motorcontroller_pkg_lowering(pkg);
        } else if (pkg->STATE == RISING) {
            return rtc_save_motorcontroller_pkg_rising(pkg);
        }
    }
    return ESP_FAIL;
}

static void motorcontroller_manager_task(void *arg) {
    ESP_LOGI(TAG, "Manager task started");

    xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_STARTED_BIT);
    esp_err_t result = ESP_OK;
    
    // 1. Load package from RTC
    result = load_system_motorcontroller_pkg(&s_manager_ctx.current_pkg, s_manager_ctx.target_state);
    // calculate and update speed estimate
    if (result == ESP_OK) {
        // 2. Send to worker via CAN
        result = update_and_save_speed_estimate(&s_manager_ctx.current_pkg); // updates and stores speed change to RTC
        // call for - wait untill worktime function before sending maybe..? it not that critical tho.
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
        uint32_t worstcase_recovery_case_ms = (4*15)*1000; // add time to allow recovery sequenses to happen befor we call it a timeout
        uint32_t estimated_timeout = calculate_operation_timeout_with_margin(&s_manager_ctx.current_pkg, 
                                                            s_manager_ctx.current_pkg.rising_timeout_percent) 
                                                            * 1000 + worstcase_recovery_case_ms; // Convert to ms
        
        // 4. Wait for response with calculated timeout/ can have offset, but it soo good we dont need to!
        result = wait_for_worker(&s_manager_ctx.received_resp, 0, estimated_timeout);
        if (result == ESP_OK) {            
            // 5. Update and store to RTC
            print_motorcontroller_response_info(&s_manager_ctx.received_resp, TAG);
            result = update_and_store_pkg(&s_manager_ctx.current_pkg, &s_manager_ctx.received_resp);
            // update and store it

            // Check if worker reported failure
            if (s_manager_ctx.received_resp.result != ESP_OK) {
                ESP_LOGW(TAG, "Worker reported failure: %s", esp_err_to_name(s_manager_ctx.received_resp.result));
                xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_WORKER_FAIL_BIT);
            } 
            xEventGroupSetBits(s_manager_ctx.event_group, MANAGER_TASK_RESP_RECEIVED_BIT);
            ESP_LOGI(TAG, "Response received successfully");
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

esp_err_t load_state_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t for_state) {
    if (!pkg) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_FAIL;
    
    // Try to load from RTC based on state
    if (for_state == LOWERING) {
        result = rtc_load_motorcontroller_pkg_lowering(pkg);
    } else if (for_state == RISING) {
        result = rtc_load_motorcontroller_pkg_rising(pkg);
    } // ad a init case too!!    
    return result;
}


bool validate_motorcontroller_pkg(const motorcontroller_pkg_t *pkg, state_t state) {
    // Use existing common validation function    
    return is_motorcontroller_pkg_valid(pkg, state);
}