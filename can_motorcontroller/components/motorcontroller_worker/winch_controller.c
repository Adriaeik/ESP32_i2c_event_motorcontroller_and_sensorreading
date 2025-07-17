#include "winch_controller.h"
#include "outputs.h"
#include "inputs.h"
#include "timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char* TAG = "winch_controller";

// Timing and safety constants
#define WINCH_TIMEOUT_SAFETY_MARGIN_PERCENT  30
#define SOFT_START_STOP_COMPENSATION_MS      500
#define STATIC_POLL_STOP_OFFSET_MS          1000
#define STATIC_POLL_TIMING_BUFFER_MS         200
#define MIN_OPERATION_TIME_MS                100
#define RECOVERY_DOWN_TIME_MS               2000
#define RECOVERY_UP_TIMEOUT_MS              8000
#define RECOVERY_PAUSE_TIME_MS              1000
#define MAX_RECOVERY_RETRIES                3
#define TENSION_REVERSE_TIME_MS             1000
#define TENSION_RECOVERY_PAUSE_MS           500

// Event group bits for operation control
#define OPERATION_COMPLETE_BIT      BIT0
#define OPERATION_ERROR_BIT         BIT1
#define HOME_REACHED_BIT            BIT2
#define TENSION_ALARM_BIT           BIT3

typedef struct {
    InputEvent event;
    uint32_t remaining_events;  // How many more events are queued
} InputEventExt;

// Winch direction control
typedef enum {
    WINCH_STOP = 0,
    WINCH_UP,
    WINCH_DOWN,
    WINCH_INVALID = 255  // Used to force first direction change
} winch_direction_t;


// Operation state tracking
typedef struct {
    // Timers
    w_timer_t operation_timer;              // Total operation time
    w_timer_t working_timer;                // Active motor time only
    w_timer_t static_wait_timer;            // Static polling wait time
    w_timer_t recovery_timer;               // Recovery operation timer
    
    // Operation parameters
    const motorcontroller_pkg_t *current_pkg;
    motorcontroller_response_t *current_resp;
    uint16_t updated_speed_estimate;        // Fresh speed estimate for this operation
    
    // Static depth handling
    uint8_t current_static_point;           // Index in static_points array
    uint16_t current_target_depth;          // Current depth target
    bool in_static_wait;                    // Currently waiting at static point
    
    // Error handling and recovery
    uint8_t recovery_attempts;
    uint32_t total_recovery_time_ms;        // Time spent in recovery (exclude from working time)
    uint32_t total_static_wait_time_ms;     // Time spent in static waits (exclude from working time)
    
    // State management
    winch_direction_t current_direction;
    bool operation_active;
    bool home_reached;
    
    // Event handling
    QueueHandle_t event_queue;
    EventGroupHandle_t event_group;
} winch_operation_state_t;

// Private state - only one operation at a time
static winch_operation_state_t g_op_state = {.current_direction = WINCH_INVALID};
static bool g_controller_initialized = false;
static TaskHandle_t g_monitor_task_handle = NULL;
static volatile bool g_monitor_task_should_exit = false;

// Forward declarations
static InputEventExt wait_until_notified_or_time(TickType_t start_tick, uint32_t wait_time_ms);
static esp_err_t check_initial_input_state(void);
static esp_err_t execute_operation_internal(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp);
static void update_speed_estimate_pre_operation(const motorcontroller_pkg_t *pkg);
static esp_err_t handle_static_depth_mode(state_t state);
static esp_err_t handle_alpha_depth_mode(state_t state);
static esp_err_t handle_lin_time_mode(state_t state);
static esp_err_t wait_for_operation_complete(state_t STATE, uint32_t timeout_ms);
static esp_err_t perform_recovery_sequence(void);
static esp_err_t handle_tension_recovery(winch_direction_t original_direction);
static void winch_move(winch_direction_t direction);
static void reset_operation_state(void);
static bool validate_package(const motorcontroller_pkg_t *pkg);
static uint32_t calculate_expected_time_ms(uint16_t distance_cm, uint16_t speed_cm_per_s, POLL_TYPE poll_type, uint16_t samples, uint16_t interval_s);
static void alpha_beta_update(double dt, double z_meas, double *x, double *v, double alpha, double beta);
static const char* get_direction_string(winch_direction_t direction);
static const char* get_poll_type_string(POLL_TYPE poll_type);
static void get_elapsed_time_string(uint32_t ms, char* buffer, size_t buffer_size);




// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

esp_err_t winch_controller_init(void) {
    if (g_controller_initialized) {
        ESP_LOGD(TAG, "Controller already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing winch controller...");
    
    // Initialize inputs and outputs
    memset(&g_op_state, 0, sizeof(winch_operation_state_t));
    g_monitor_task_should_exit = false;
    // Create event queue and group
    g_op_state.event_queue = xQueueCreate(10, sizeof(InputEvent));  // Minst 10 elementer
    // g_op_state.event_group = xEventGroupCreate();
    
    if (!g_op_state.event_queue || !g_op_state.event_group) {
        ESP_LOGE(TAG, "Failed to create event structures");
        return ESP_ERR_NO_MEM;
    }
    inputs_init(g_op_state.event_queue);
    outputs_init();

    // Initialize state
    reset_operation_state();
    
    g_controller_initialized = true;
    ESP_LOGI(TAG, "Winch controller initialized successfully");
    
    return ESP_OK;
}

esp_err_t winch_controller_deinit(void) {
    if (!g_controller_initialized) {
        ESP_LOGD(TAG, "Controller not initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing winch controller...");
    
    // Stop any ongoing operation
    winch_move(WINCH_STOP);
    g_op_state.operation_active = false;
    
    // Signal monitor task to exit
    g_monitor_task_should_exit = true;
    
    // Wait for monitor task to exit (with timeout)
    if (g_monitor_task_handle) {
        // Send dummy message to wake up task
        uint32_t dummy = 0;
        xQueueSend(g_op_state.event_queue, &dummy, pdMS_TO_TICKS(100));
        
        // Wait for task to actually exit (check handle becomes NULL)
        for (int i = 0; i < 20 && g_monitor_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        // If still running, force delete
        if (g_monitor_task_handle != NULL) {
            ESP_LOGW(TAG, "Force deleting monitor task");
            TaskHandle_t temp_handle = g_monitor_task_handle;
            g_monitor_task_handle = NULL;
            vTaskDelete(temp_handle);
        }
    }
    
    // Clean up event structures
    if (g_op_state.event_queue) {
        vQueueDelete(g_op_state.event_queue);
        g_op_state.event_queue = NULL;
    }
    
    if (g_op_state.event_group) {
        vEventGroupDelete(g_op_state.event_group);
        g_op_state.event_group = NULL;
    }
    
    
    // Deinitialize inputs and outputs systems
    inputs_deinit();
    outputs_deinit();
    
    // Reset state
    memset(&g_op_state, 0, sizeof(winch_operation_state_t));
    g_controller_initialized = false;
    
    ESP_LOGI(TAG, "Winch controller deinitialized");
    return ESP_OK;
}

esp_err_t winch_execute_operation(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    if (!g_controller_initialized) {
        ESP_LOGE(TAG, "Controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!pkg || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!validate_package(pkg)) {
        ESP_LOGE(TAG, "Package validation failed");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting operation - State: %s, Poll: %s, End depth: %d cm, Speed: %.0f cm/s", 
             state_to_string(pkg->STATE), get_poll_type_string(pkg->poll_type), 
             pkg->end_depth, (double)pkg->prev_estimated_cm_per_s / 1000.0);
    
    return execute_operation_internal(pkg, resp);
}

esp_err_t winch_go_to_home_position(void) {
    if (!g_controller_initialized) {
        ESP_LOGE(TAG, "Controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Going to home position");
    
    // Safety checks
    if (!inputs_get_winch_auto()) {
        ESP_LOGE(TAG, "AUTO mode not enabled");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (inputs_get_winch_tension()) {
        ESP_LOGE(TAG, "Tension alarm active");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if already at home
    if (inputs_get_winch_home()) {
        ESP_LOGI(TAG, "Already at home position");
        return ESP_OK;
    }
    
    // Reset state and start operation
    reset_operation_state();
    g_op_state.operation_active = true;
    timer_start(&g_op_state.operation_timer);
    timer_start(&g_op_state.working_timer);
    
    // Start moving up
    winch_move(WINCH_UP);
    
    // Wait for home with timeout (use default rising timeout)
    uint32_t timeout_ms = 30000; // 30 second default timeout
    esp_err_t result = wait_for_operation_complete(RISING, timeout_ms);
    
    // Stop motor
    winch_move(WINCH_STOP);
    g_op_state.operation_active = false;
    
    if (result == ESP_OK) {
        char time_str[32];
        get_elapsed_time_string(timer_ms_since_start(&g_op_state.working_timer), time_str, sizeof(time_str));
        ESP_LOGI(TAG, "Home position reached in %s", time_str);
    } else {
        ESP_LOGE(TAG, "Failed to reach home position: %s", esp_err_to_name(result));
    }
    
    return result;
}

// ============================================================================
// PRIVATE IMPLEMENTATION
// ============================================================================

static InputEventExt wait_until_notified_or_time(TickType_t start_tick, uint32_t wait_time_ms) {
    TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
    TickType_t total_wait_ticks = pdMS_TO_TICKS(wait_time_ms);
    
    InputEvent received_event = {
        .event_type = TIMEOUT,
        .state = true,
        .edge_detected = false
    };
    
    if (elapsed_ticks >= total_wait_ticks) {
        goto end;
    }
    
    TickType_t remaining_ticks = total_wait_ticks - elapsed_ticks;
    xQueueReceive(g_op_state.event_queue, &received_event, remaining_ticks);

end:
    return (InputEventExt){
        .event = received_event,
        .remaining_events = uxQueueMessagesWaiting(g_op_state.event_queue)
    };
}
static esp_err_t check_initial_input_state(void){
    // Safety checks
    if (!inputs_get_winch_auto()) {
        ESP_LOGE(TAG, "AUTO mode not enabled");
        return ESP_ERR_INVALID_STATE;
    } 
    if (inputs_get_winch_tension()) {
        ESP_LOGE(TAG, "Tension alarm active");
    }
}

static esp_err_t execute_operation_internal(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    
    esp_err_t result = check_initial_inputstate();
    if (result != ESP_OK){
        resp->result = result;
        resp->working_time = 0;
        resp->estimated_cm_per_s = pkg->prev_estimated_cm_per_s;
        return result;
    }
    // Initialize response
    memset(resp, 0, sizeof(motorcontroller_response_t));
    resp->STATE = pkg->STATE;
    resp->estimated_cm_per_s = pkg->prev_estimated_cm_per_s; // Default fallback
    
    // Reset operation state
    reset_operation_state();
    g_op_state.current_pkg = pkg;
    g_op_state.current_resp = resp;
    
    // Update speed estimate before starting operation
    update_speed_estimate_pre_operation(pkg);
    
    // Start operation timing
    timer_start(&g_op_state.operation_timer);
    timer_start(&g_op_state.working_timer);
    g_op_state.operation_active = true;
    
    
    
    // Execute based on state
    switch (pkg->STATE) {
        case INIT:
            ESP_LOGI(TAG, "Executing INIT - going to home position");
            result = winch_go_to_home_position();
            break;
            
        case LOWERING:
        case RISING:
            ESP_LOGI(TAG, "Executing %s operation (Speed: %.0f cm/s)", 
                     state_to_string(pkg->STATE), (double)g_op_state.updated_speed_estimate / 1000.0);
            
            switch (pkg->poll_type) {
                case ALPHA_DEPTH:
                    result = handle_alpha_depth_mode(pkg->STATE);
                    break;
                case STATIC_DEPTH:
                    result = handle_static_depth_mode(pkg->STATE);
                    break;
                case LIN_TIME:
                    result = handle_lin_time_mode(pkg->STATE);
                    break;
                default:
                    ESP_LOGE(TAG, "Invalid poll type: %d", pkg->poll_type);
                    result = ESP_ERR_INVALID_ARG;
                    break;
            }
            break;
            
        default:
            ESP_LOGE(TAG, "Invalid state: %d", pkg->STATE);
            result = ESP_ERR_INVALID_ARG;
            break;
    }
    
    // Stop motor and finalize
    winch_move(WINCH_STOP);
    g_op_state.operation_active = false;
    
    // Calculate final working time (exclude recovery and static waits)
    uint32_t total_time_ms = timer_ms_since_start(&g_op_state.working_timer);
    uint32_t working_time_ms = total_time_ms - g_op_state.total_recovery_time_ms - g_op_state.total_static_wait_time_ms;
    
    // Fill response
    resp->result = result;
    resp->working_time = working_time_ms / 1000; // Convert to seconds
    resp->estimated_cm_per_s = g_op_state.updated_speed_estimate;
    
    ESP_LOGI(TAG, "Operation complete - Result: %s, Working time: %ds, Speed: %.0f cm/s", 
             esp_err_to_name(result), resp->working_time, (double)resp->estimated_cm_per_s / 1000.0);
    
    return result;
}

static void update_speed_estimate_pre_operation(const motorcontroller_pkg_t *pkg) {
    // Start with previous estimate - fallback
    g_op_state.updated_speed_estimate = pkg->prev_estimated_cm_per_s;
    
    // If we have previous operation data, apply alpha-beta filter
    if (pkg->prev_working_time > 0 && pkg->prev_reported_depth > 0) {
        
        // Calculate avrage speed
        double actual_distance_cm = (double)pkg->prev_reported_depth;
        double actual_time_s = (double)pkg->prev_working_time;
        
        if (actual_time_s > 0.1 && actual_distance_cm > 0) {
            // Current state for filter
            double predicted_distance = actual_distance_cm;
            double predicted_velocity = (double)pkg->prev_estimated_cm_per_s / 1000.0;
            
            // Measured velocity from actual operation
            double measured_velocity = actual_distance_cm / (actual_time_s * 1000.0);
            
            // Apply alpha-beta filter
            alpha_beta_update(actual_time_s, actual_distance_cm, 
                            &predicted_distance, &predicted_velocity, 
                            pkg->alpha, pkg->beta);
            
            // Update estimate (convert back to scaled format)
            uint16_t new_estimate = (uint16_t)(predicted_velocity * 1000.0);
            
            if (new_estimate > 0 && new_estimate < 50000) {
                g_op_state.updated_speed_estimate = new_estimate;
                ESP_LOGI(TAG, "Speed estimate updated: %.0f -> %.0f cm/s (distance: %.0f cm, time: %.1f s)", 
                         (double)pkg->prev_estimated_cm_per_s / 1000.0,
                         (double)new_estimate / 1000.0, actual_distance_cm, actual_time_s);
            } else {
                ESP_LOGW(TAG, "Speed estimate rejected (out of range): %.0f cm/s", 
                         (double)new_estimate / 1000.0);
            }
        }
    }
}

static esp_err_t handle_alpha_depth_mode(state_t state) {
    const motorcontroller_pkg_t *pkg = g_op_state.current_pkg;
    bool is_rising = (state == RISING);
    
    // Calculate expected time based on distance and speed
    uint32_t expected_time_ms = calculate_expected_time_ms(pkg->end_depth, g_op_state.updated_speed_estimate, 
                                                          pkg->poll_type, pkg->samples, pkg->static_poll_interval_s);
    
    // Apply timeout safety margin
    uint32_t timeout_ms;
    if (is_rising) {
        timeout_ms = (expected_time_ms * (100 + pkg->rising_timeout_percent)) / 100;
    } else {
        timeout_ms = (expected_time_ms * (100 + WINCH_TIMEOUT_SAFETY_MARGIN_PERCENT)) / 100;
    }
    
    char time_str[32];
    char timeout_str[32];
    get_elapsed_time_string(expected_time_ms, time_str, sizeof(time_str));
    get_elapsed_time_string(timeout_ms, timeout_str, sizeof(timeout_str));
    ESP_LOGI(TAG, "ALPHA_DEPTH mode: Expected %s, Timeout %s, Distance %d cm", 
             time_str, timeout_str, pkg->end_depth);
    
    // Start movement
    winch_direction_t direction = is_rising ? WINCH_UP : WINCH_DOWN;
    winch_move(direction);
    
    esp_err_t result = ESP_OK;
    
    if (is_rising) {
        // RISING: Wait for home sensor or timeout
        result = wait_for_operation_complete(state, timeout_ms);
        if (result == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ALPHA_DEPTH timeout - attempting recovery");
            result = perform_recovery_sequence();
        }
    } else {
        // LOWERING: Just run for the calculated time
        w_timer_t operation_timer;
        timer_start(&operation_timer);
        
        while (timer_ms_since_start(&operation_timer) < expected_time_ms) {
            // Check for error conditions
            if (!inputs_get_winch_auto()) {
                ESP_LOGW(TAG, "AUTO mode disabled during lowering");
                result = ESP_ERR_INVALID_STATE;
                break;
            }
            
            if (inputs_get_winch_tension()) {
                ESP_LOGW(TAG, "Tension detected during lowering - attempting recovery");
                result = handle_tension_recovery(direction);
                if (result != ESP_OK) {
                    break;
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
        }
        
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "ALPHA_DEPTH lowering completed after %s", time_str);
        }
    }
    
    return result;
}

static esp_err_t handle_lin_time_mode(state_t state) {
    const motorcontroller_pkg_t *pkg = g_op_state.current_pkg;
    bool is_rising = (state == RISING);
    
    // Use static_poll_interval_s as the operation time
    uint32_t operation_time_ms = pkg->static_poll_interval_s * 1000;
    uint32_t timeout_ms = (operation_time_ms * (100 + WINCH_TIMEOUT_SAFETY_MARGIN_PERCENT)) / 100;
    
    char op_time_str[32];
    char timeout_str[32];
    get_elapsed_time_string(operation_time_ms, op_time_str, sizeof(op_time_str));
    get_elapsed_time_string(timeout_ms, timeout_str, sizeof(timeout_str));
    ESP_LOGI(TAG, "LIN_TIME mode: Operating for %s, Timeout %s", op_time_str, timeout_str);
    
    // Start movement
    winch_direction_t direction = is_rising ? WINCH_UP : WINCH_DOWN;
    winch_move(direction);
    
    // Wait for specified time or early completion
    w_timer_t lin_timer;
    timer_start(&lin_timer);
    
    while (timer_ms_since_start(&lin_timer) < operation_time_ms) {
        // Check for early completion conditions (rising only)
        if (is_rising && inputs_get_winch_home()) {
            ESP_LOGI(TAG, "LIN_TIME: Home reached early");
            return ESP_OK;
        }
        
        // Check for errors
        if (!inputs_get_winch_auto()) {
            ESP_LOGW(TAG, "LIN_TIME: AUTO mode disabled");
            return ESP_ERR_INVALID_STATE;
        }
        
        if (inputs_get_winch_tension()) {
            ESP_LOGW(TAG, "LIN_TIME: Tension detected - attempting recovery");
            esp_err_t recovery_result = handle_tension_recovery(direction);
            if (recovery_result != ESP_OK) {
                return recovery_result;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
    }
    
    ESP_LOGI(TAG, "LIN_TIME: Time completed");
    return ESP_OK;
}

static esp_err_t handle_static_depth_mode(state_t state) {
    const motorcontroller_pkg_t *pkg = g_op_state.current_pkg;
    bool is_rising = (state == RISING);
    
    // Validate static points array
    uint8_t num_points = 0;
    for (int i = 0; i < MAX_POINTS && pkg->static_points[i] != 0; i++) {
        num_points++;
    }
    
    if (num_points == 0) {
        ESP_LOGE(TAG, "STATIC_DEPTH: No static points defined");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "STATIC_DEPTH mode: %d points, %d samples per point, %ds interval, %dms offset", 
             num_points, pkg->samples, pkg->static_poll_interval_s, STATIC_POLL_STOP_OFFSET_MS);
    
    // Process each static point
    for (uint8_t i = 0; i < num_points; i++) {
        g_op_state.current_static_point = i;
        g_op_state.current_target_depth = pkg->static_points[i];
        
        ESP_LOGI(TAG, "Moving to static point %d: %d cm", i + 1, g_op_state.current_target_depth);
        
        // Calculate time to reach this point
        uint32_t distance_to_point;
        if (i == 0) {
            distance_to_point = g_op_state.current_target_depth;
        } else {
            distance_to_point = abs(g_op_state.current_target_depth - pkg->static_points[i-1]);
        }
        
        uint32_t travel_time_ms = calculate_expected_time_ms(distance_to_point, g_op_state.updated_speed_estimate,
                                                             ALPHA_DEPTH, 0, 0);  // Simple travel time, no static pauses
        
        char travel_str[32];
        get_elapsed_time_string(travel_time_ms, travel_str, sizeof(travel_str));
        ESP_LOGI(TAG, "Point %d: Distance %d cm, Travel time %s, Speed %.0f cm/s", 
                 i + 1, distance_to_point, travel_str, (double)g_op_state.updated_speed_estimate / 1000.0);
        
        // Start movement to this point
        winch_direction_t direction = is_rising ? WINCH_UP : WINCH_DOWN;
        winch_move(direction);
        
        // Wait for point to be reached (time-based estimate)
        w_timer_t point_timer;
        timer_start(&point_timer);
        
        while (timer_ms_since_start(&point_timer) < travel_time_ms) {
            // Check for early home detection (rising only)
            if (is_rising && inputs_get_winch_home()) {
                ESP_LOGI(TAG, "STATIC_DEPTH: Home reached early during travel to point %d", i + 1);
                return ESP_OK;
            }
            
            // Check for errors
            if (!inputs_get_winch_auto()) {
                ESP_LOGW(TAG, "STATIC_DEPTH: AUTO disabled at point %d", i + 1);
                return ESP_ERR_INVALID_STATE;
            }
            
            if (inputs_get_winch_tension()) {
                ESP_LOGW(TAG, "STATIC_DEPTH: Tension detected at point %d - attempting recovery", i + 1);
                esp_err_t recovery_result = handle_tension_recovery(direction);
                if (recovery_result != ESP_OK) {
                    return recovery_result;
                }
                // Resume movement after recovery
                winch_move(direction);
            }
            
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        // Stop at this point
        winch_move(WINCH_STOP);
        ESP_LOGI(TAG, "Reached point %d, starting static wait", i + 1);
        
        // Static wait period: static_poll_interval_s * samples + OFFSET
        uint32_t static_wait_ms = (pkg->static_poll_interval_s * pkg->samples * 1000) + STATIC_POLL_STOP_OFFSET_MS;
        
        char wait_str[32];
        get_elapsed_time_string(static_wait_ms, wait_str, sizeof(wait_str));
        ESP_LOGI(TAG, "Static wait: %d samples × %ds + %dms offset = %s total", 
                 pkg->samples, pkg->static_poll_interval_s, STATIC_POLL_STOP_OFFSET_MS, wait_str);
        
        // Track static wait time (exclude from working time)
        w_timer_t wait_timer;
        timer_start(&wait_timer);
        g_op_state.in_static_wait = true;
        
        // Wait for static period
        vTaskDelay(pdMS_TO_TICKS(static_wait_ms));
        
        // Update exclusion time
        g_op_state.total_static_wait_time_ms += timer_ms_since_start(&wait_timer);
        g_op_state.in_static_wait = false;
        
        ESP_LOGI(TAG, "Static wait complete at point %d", i + 1);
    }
    
    // All static points completed
    ESP_LOGI(TAG, "STATIC_DEPTH: All points completed");
    
    // For rising operations, continue to home after last static point
    if (is_rising) {
        ESP_LOGI(TAG, "STATIC_DEPTH rising: Moving to home after static points");
        winch_move(WINCH_UP);
        uint32_t timeout_ms = 10000;
        return wait_for_operation_complete(state, timeout_ms ); // 10 second timeout to reach home
    }
    
    return ESP_OK;
}

static esp_err_t wait_for_operation_complete(state_t STATE, uint32_t timeout_ms) {
    // This function is specifically for RISING operations waiting for home sensor
    w_timer_t timeout_timer;
    timer_start(&timeout_timer);
    
    while (timer_ms_since_start(&timeout_timer) < timeout_ms) {
        // Check for completion conditions (home sensor)
        if (g_op_state.current_direction == WINCH_UP && inputs_get_winch_home()) {
            ESP_LOGI(TAG, "Home position reached");
            return ESP_OK;
        }
        
        // Check for error conditions
        if (!inputs_get_winch_auto()) {
            ESP_LOGW(TAG, "AUTO mode disabled during operation");
            return ESP_ERR_INVALID_STATE;
        }
        
        if (inputs_get_winch_tension()) {
            ESP_LOGW(TAG, "Tension alarm during rising - attempting recovery");
            esp_err_t recovery_result = handle_tension_recovery(WINCH_UP);
            if (recovery_result != ESP_OK) {
                return recovery_result;
            }
            // Resume upward movement after recovery
            winch_move(WINCH_UP);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
    }
    
    char timeout_str[32];
    get_elapsed_time_string(timeout_ms, timeout_str, sizeof(timeout_str));
    ESP_LOGW(TAG, "Operation timeout after %s", timeout_str);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t handle_tension_recovery(winch_direction_t original_direction) {
    ESP_LOGI(TAG, "Starting tension recovery - reversing direction for %dms", TENSION_REVERSE_TIME_MS);
    
    w_timer_t recovery_start;
    timer_start(&recovery_start);
    
    // Reverse direction briefly
    winch_direction_t reverse_dir = (original_direction == WINCH_DOWN) ? WINCH_UP : WINCH_DOWN;
    winch_move(reverse_dir);
    vTaskDelay(pdMS_TO_TICKS(TENSION_REVERSE_TIME_MS));
    
    // Check if tension cleared
    if (!inputs_get_winch_tension()) {
        ESP_LOGI(TAG, "Tension recovered - resuming original direction after %dms pause", TENSION_RECOVERY_PAUSE_MS);
        winch_move(original_direction);
        vTaskDelay(pdMS_TO_TICKS(TENSION_RECOVERY_PAUSE_MS));
        
        // Track recovery time (exclude from working time)
        g_op_state.total_recovery_time_ms += timer_ms_since_start(&recovery_start);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Tension recovery failed - tension still present");
        g_op_state.total_recovery_time_ms += timer_ms_since_start(&recovery_start);
        return ESP_ERR_INVALID_STATE;
    }
}

static esp_err_t perform_recovery_sequence(void) {
    ESP_LOGI(TAG, "Starting recovery sequence");
    
    w_timer_t recovery_start;
    timer_start(&recovery_start);
    
    // Recovery: go down a bit, then up to home
    ESP_LOGI(TAG, "Recovery: Moving down");
    winch_move(WINCH_DOWN);
    vTaskDelay(pdMS_TO_TICKS(RECOVERY_DOWN_TIME_MS));
    
    ESP_LOGI(TAG, "Recovery: Moving up to home");
    winch_move(WINCH_UP);
    
    // Wait for home with timeout
    esp_err_t result = wait_for_operation_complete(RISING, RECOVERY_UP_TIMEOUT_MS);
    
    // Track recovery time (exclude from working time)
    g_op_state.total_recovery_time_ms += timer_ms_since_start(&recovery_start);
    g_op_state.recovery_attempts++;
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Recovery successful");
    } else {
        ESP_LOGE(TAG, "Recovery failed: %s", esp_err_to_name(result));
    }
    
    return result;
}

static void winch_move(winch_direction_t direction) {
    if (g_op_state.current_direction != direction) {
        g_op_state.current_direction = direction;
        ESP_LOGI(TAG, "Winch direction: %s", get_direction_string(direction));
    }
    
    outputs_set_winch_down(direction == WINCH_DOWN);
    outputs_set_winch_up(direction == WINCH_UP);
}

static void reset_operation_state(void) {
    // Preserve event structures BEFORE zeroing
    QueueHandle_t queue = g_op_state.event_queue;
    EventGroupHandle_t group = g_op_state.event_group;
    
    // Zero out the entire state
    memset(&g_op_state, 0, sizeof(winch_operation_state_t));
    
    // Restore event structures
    g_op_state.event_queue = queue;
    g_op_state.event_group = group;
    g_op_state.current_direction = WINCH_INVALID; // Force first direction change to be logged
}

static bool validate_package(const motorcontroller_pkg_t *pkg) {
    if (pkg->STATE < INIT || pkg->STATE > RISING) {
        ESP_LOGE(TAG, "Invalid STATE: %d", pkg->STATE);
        return false;
    }
    
    if (pkg->poll_type < ALPHA_DEPTH || pkg->poll_type > LIN_TIME) {
        ESP_LOGE(TAG, "Invalid poll_type: %d", pkg->poll_type);
        return false;
    }
    
    if (pkg->end_depth > 10000) { // Sanity check - 100m max
        ESP_LOGE(TAG, "Invalid end_depth: %d", pkg->end_depth);
        return false;
    }
    
    if (pkg->poll_type == STATIC_DEPTH) {
        // Validate static points array
        bool has_points = false;
        for (int i = 0; i < MAX_POINTS; i++) {
            if (pkg->static_points[i] != 0) {
                has_points = true;
                if (i > 0) {
                    // For LOWERING: points should be ascending (0 < 200 < 400)
                    // For RISING: points can be descending (600 > 400 > 200)
                    bool ascending = pkg->static_points[i] > pkg->static_points[i-1];
                    bool descending = pkg->static_points[i] < pkg->static_points[i-1];
                    if (pkg->STATE == LOWERING && !ascending) {
                        ESP_LOGE(TAG, "LOWERING static points must be ascending at index %d (%d <= %d)", 
                                 i, pkg->static_points[i], pkg->static_points[i-1]);
                        return false;
                    } else if (pkg->STATE == RISING && !descending) {
                        ESP_LOGE(TAG, "RISING static points must be descending at index %d (%d >= %d)", 
                                 i, pkg->static_points[i], pkg->static_points[i-1]);
                        return false;
                    }
                }
            } else {
                break; // End of array
            }
        }
        if (!has_points) {
            ESP_LOGE(TAG, "STATIC_DEPTH mode requires static points");
            return false;
        }
    }
    
    return true;
}

static uint32_t calculate_expected_time_ms(uint16_t distance_cm, uint16_t speed_cm_per_s, POLL_TYPE poll_type, uint16_t samples, uint16_t interval_s) {
    if (speed_cm_per_s == 0) {
        ESP_LOGW(TAG, "Zero speed estimate, using default time");
        return 10000; // 10 second default
    }
    
    // Convert scaled speed back to real cm/s (scaling: 1000 = 1 cm/s)
    double real_speed = (double)speed_cm_per_s / 1000.0;
    
    // Calculate basic travel time in seconds, convert to ms
    uint32_t travel_time_ms = (uint32_t)((distance_cm / real_speed) * 1000.0);
    
    // Add static pause times if applicable
    uint32_t static_pause_time_ms = 0;
    if (poll_type == STATIC_DEPTH && samples > 0 && interval_s > 0) {
        // Estimate number of static points (rough approximation)
        uint8_t estimated_points = (distance_cm / 200) + 1; // Assume points every 2m
        if (estimated_points > MAX_POINTS) estimated_points = MAX_POINTS;
        
        // Each point: (samples * interval_s * 1000) + STOP_OFFSET_MS
        uint32_t pause_per_point = (samples * interval_s * 1000) + STATIC_POLL_STOP_OFFSET_MS;
        static_pause_time_ms = estimated_points * pause_per_point;
        
        ESP_LOGD(TAG, "Static mode: %" PRIu32 " estimated points, %" PRIu32 " ms pause per point", 
                 estimated_points, pause_per_point);
    }
    
    uint32_t total_time_ms = travel_time_ms + static_pause_time_ms;
    
    // Sanity checks
    if (total_time_ms < MIN_OPERATION_TIME_MS) {
        total_time_ms = MIN_OPERATION_TIME_MS;
    } else if (total_time_ms > 300000) { // 5 minute max
        total_time_ms = 300000;
    }
    
    return total_time_ms;
}

static const char* get_direction_string(winch_direction_t direction) {
    switch (direction) {
        case WINCH_STOP: return "STOP";
        case WINCH_UP: return "UP";
        case WINCH_DOWN: return "DOWN";
        case WINCH_INVALID: return "INVALID";
        default: return "UNKNOWN";
    }
}

static void get_elapsed_time_string(uint32_t ms, char* buffer, size_t buffer_size) {
    uint32_t total_seconds = ms / 1000;
    uint32_t remaining_ms  = ms % 1000;
    uint32_t hours         = total_seconds / 3600;
    uint32_t minutes       = (total_seconds % 3600) / 60;
    uint32_t seconds       = total_seconds % 60;
    
    if (hours > 0) {
        snprintf(buffer, buffer_size,
                 "%" PRIu32 ":%02" PRIu32 ":%02" PRIu32 ".%03" PRIu32,
                 hours, minutes, seconds, remaining_ms);
    } else if (minutes > 0) {
        snprintf(buffer, buffer_size,
                 "%" PRIu32 ":%02" PRIu32 ".%03" PRIu32,
                 minutes, seconds, remaining_ms);
    } else {
        snprintf(buffer, buffer_size,
                 "%" PRIu32 ".%03" PRIu32 "s",
                 seconds, remaining_ms);
    }
}

static void alpha_beta_update(double dt, double z_meas, double *x, double *v, double alpha, double beta) {
    // Prediction step
    double x_pred = *x + (*v) * dt;
    double v_pred = *v;
    
    // Residual
    double r = z_meas - x_pred;
    
    // Correction
    *x = x_pred + alpha * r;
    *v = v_pred + (beta * r) / dt;
}

static const char* get_poll_type_string(POLL_TYPE poll_type) {
    switch (poll_type) {
        case ALPHA_DEPTH: return "ALPHA_DEPTH";
        case STATIC_DEPTH: return "STATIC_DEPTH";
        case LIN_TIME: return "LIN_TIME";
        default: return "UNKNOWN";
    }
}
