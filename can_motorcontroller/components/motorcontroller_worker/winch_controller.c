#include "winch_controller.h"
#include "outputs.h"
#include "inputs.h"
#include "timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char* TAG = "winch_controller";

// Timing and safety constants
#define MAXIMUM_DEPTH                        8000 //80m
#define MAXIMUM_SPEED                       50000 //50cm/s
#define MAX_RECOVERY_RETRIES                    3
#define WINCH_TIMEOUT_SAFETY_MARGIN_PERCENT    30
#define SOFT_START_STOP_COMPENSATION_MS       500
#define STATIC_POLL_STOP_OFFSET_MS           1000
#define STATIC_POLL_TIMING_BUFFER_MS          200
#define MIN_OPERATION_TIME_MS                 100
#define RECOVERY_DOWN_TIME_MS                2000
#define RECOVERY_UP_TIMEOUT_MS               8000
#define RECOVERY_PAUSE_TIME_MS               1000
#define TENSION_REVERSE_TIME_MS              1000
#define TENSION_RECOVERY_PAUSE_MS             500

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
    w_timer_t operation_timer;              // Total run time

    uint32_t total_recovery_time_ms;        // Time spent in recovery (exclude from working time)
    uint32_t total_static_wait_time_ms;     // Time spent in static waits (exclude from working time)
    int working_time;                       // = operation_timer - total_recovery_time_ms - total_static_wait_time_ms;

    // Operation parameters
    const motorcontroller_pkg_t *current_pkg;
    motorcontroller_response_t *current_resp;
    uint16_t updated_speed_estimate;        // Fresh speed estimate for this operation
    
    // Static depth handling
    uint8_t current_static_point;           // Index in static_points array
    uint16_t current_target_depth;          // Current depth target
    bool in_static_wait;                    // Currently waiting at static point
    uint8_t num_points;
    uint16_t sorted_static_points[MAX_POINTS]; // Sorted based on direction

    // Error handling and recovery
    uint8_t recovery_attempts;
    
    // State management
    winch_direction_t current_direction;
    bool operation_active;
    bool home_reached;
    
    // Event handling
    QueueHandle_t event_queue;
} winch_operation_state_t;

// Private state - only one operation at a time
static winch_operation_state_t g_op_state = {.current_direction = WINCH_INVALID};
static bool g_controller_initialized = false;
static TaskHandle_t g_monitor_task_handle = NULL;
static volatile bool g_monitor_task_should_exit = false;

// Forward declarations
static esp_err_t execute_operation_internal(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp);
static InputEventExt wait_until_notified_or_time(TickType_t start_tick, uint32_t wait_time_ms);
static esp_err_t check_initial_input_state(void);
static esp_err_t await_completion_and_handle_events(state_t STATE, uint32_t timeout_ms);
static void winch_move(winch_direction_t direction);

static esp_err_t handle_time_based_operation(state_t state, uint32_t operation_time_ms, uint32_t timeout_ms, const char* mode_name);
static esp_err_t handle_static_depth_mode(state_t state);
static esp_err_t handle_alpha_depth_mode(state_t state);
static esp_err_t handle_lin_time_mode(state_t state);
static esp_err_t handle_tension_recovery(state_t state);
static esp_err_t handle_rising_timeout_recovery_sequence(void);

static void reset_operation_state(void);

static bool validate_package(const motorcontroller_pkg_t *pkg);

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
    
    if (!g_op_state.event_queue ) {
        ESP_LOGE(TAG, "Failed to create event queue");
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
    if(check_initial_input_state() != ESP_OK) return ESP_ERR_INVALID_STATE;
    
    // Check if already at home
    if (inputs_get_winch_home()) {
        ESP_LOGI(TAG, "Already at home position");
        return ESP_OK;
    }
    
    // Reset state and start operation
    reset_operation_state();
    g_op_state.operation_active = true;
    timer_start(&g_op_state.operation_timer);   
    
    // Wait for home with timeout (use default rising timeout)
    winch_move(WINCH_DOWN); // incase we are above the sensor
    esp_err_t result = await_completion_and_handle_events(LOWERING, RECOVERY_DOWN_TIME_MS);
    uint32_t timeout_ms = 30000; // 30 second default timeout
    winch_move(WINCH_UP);
    result = await_completion_and_handle_events(RISING, timeout_ms);
    // Stop motor
    winch_move(WINCH_STOP);
    g_op_state.operation_active = false;
    g_op_state.working_time = timer_ms_since_start(&g_op_state.operation_timer) - g_op_state.total_recovery_time_ms - g_op_state.total_static_wait_time_ms;
    if (result == ESP_OK) {
        char time_str[32];
        get_elapsed_time_string(g_op_state.working_time, time_str, sizeof(time_str));
        ESP_LOGI(TAG, "Home position reached in %s", time_str);
    } else {
        ESP_LOGE(TAG, "Failed to reach home position: %s", esp_err_to_name(result));
    }
    
    return result;
}

// ============================================================================
// PRIVATE IMPLEMENTATION
// ============================================================================
static esp_err_t execute_operation_internal(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    
    esp_err_t result = check_initial_input_state();
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
    uint32_t total_time_ms = timer_ms_since_start(&g_op_state.operation_timer);
    uint32_t working_time_ms = total_time_ms - g_op_state.total_recovery_time_ms - g_op_state.total_static_wait_time_ms;
    
    // Fill response
    resp->result = result;
    resp->working_time = working_time_ms / 1000; // Convert to seconds
    resp->estimated_cm_per_s = g_op_state.updated_speed_estimate;
    
    ESP_LOGI(TAG, "Operation complete - Result: %s, Working time: %ds, Speed: %.0f cm/s", 
             esp_err_to_name(result), resp->working_time, (double)resp->estimated_cm_per_s / 1000.0);
    

    return result;
}

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
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t handle_time_based_operation(state_t state, uint32_t operation_time_ms, uint32_t timeout_ms, const char* mode_name) {
    char time_str[32];
    get_elapsed_time_string(operation_time_ms, time_str, sizeof(time_str));
    ESP_LOGI(TAG, "%s mode: Operating for %s", mode_name, time_str);
    
    // Start movement
    winch_direction_t direction = (state == RISING) ? WINCH_UP : WINCH_DOWN;
    winch_move(direction);
    
    esp_err_t result = ESP_OK;
    
    switch (state) {
        case RISING: 
            // RISING: Wait for home sensor or timeout
            result = await_completion_and_handle_events(state, timeout_ms);
            if (result == ESP_ERR_TIMEOUT) {
                while (handle_rising_timeout_recovery_sequence() == ESP_ERR_TIMEOUT); // will return ESP_ERR_INVALID_SIZE when max retriez used!
                break; 
            } 
            ESP_LOGI(TAG, "%s Rising completed after %s", mode_name, time_str);
            break;
            
        case LOWERING: 
            // LOWERING: Just run for the calculated time            
            result = await_completion_and_handle_events(state, operation_time_ms);
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "%s Lowering completed after %s", mode_name, time_str);
            }
            break;

        default:
            break;
    }

    winch_move(WINCH_STOP); 
    return result;
}

static esp_err_t handle_alpha_depth_mode(state_t state) {
    const motorcontroller_pkg_t *pkg = g_op_state.current_pkg;
    
    // Calculate expected time based on distance and speed
    uint32_t operation_time_ms = calculate_expected_time_ms(pkg->end_depth, g_op_state.updated_speed_estimate);
    // Apply timeout safety margin
    uint32_t timeout_ms = (operation_time_ms * (100 + pkg->rising_timeout_percent)) / 100;
    
    ESP_LOGI(TAG, "ALPHA_DEPTH mode: Expected time, Distance %d cm", pkg->end_depth);
    
    return handle_time_based_operation(state, operation_time_ms, timeout_ms, "ALPHA_DEPTH");
}

static esp_err_t handle_lin_time_mode(state_t state) {
    const motorcontroller_pkg_t *pkg = g_op_state.current_pkg;
    
    // Use static_poll_interval_s as the operation time
    uint32_t operation_time_ms = pkg->static_poll_interval_s * 1000;
    uint32_t timeout_ms = (operation_time_ms * (100 + pkg->rising_timeout_percent)) / 100;
    
    return handle_time_based_operation(state, operation_time_ms, timeout_ms, "LIN_TIME");
}

static esp_err_t handle_static_depth_mode(state_t state) {
    const motorcontroller_pkg_t *pkg = g_op_state.current_pkg;
    if (pkg->static_poll_interval_s == 0 || pkg->samples <= 1) {
        uint32_t operation_time_ms = calculate_expected_time_ms(pkg->end_depth, g_op_state.updated_speed_estimate);
        uint32_t timeout_ms = (operation_time_ms * (100 + pkg->rising_timeout_percent)) / 100;
        return handle_time_based_operation(state, operation_time_ms, timeout_ms, "STATIC_DEPTH");
    }
    bool is_rising = (state == RISING);
    
    // Validate static points array
    uint8_t num_points = g_op_state.num_points;
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
        g_op_state.current_target_depth = g_op_state.sorted_static_points[i];
        
        ESP_LOGI(TAG, "Moving to static point %d: %d cm", i + 1, g_op_state.current_target_depth);
        
        // Calculate time to reach this point
        uint32_t distance_to_point;
        if (i == 0) {
            distance_to_point = g_op_state.current_target_depth;
        } else {
            distance_to_point = abs(g_op_state.current_target_depth - g_op_state.sorted_static_points[i-1]);
        }
        
        uint32_t travel_time_ms = calculate_expected_time_ms(distance_to_point, g_op_state.updated_speed_estimate);  // Simple travel time, no static pauses
        
        char travel_str[32];
        get_elapsed_time_string(travel_time_ms, travel_str, sizeof(travel_str));
        ESP_LOGI(TAG, "Point %d: Distance %d cm, Travel time %s, Speed %.0f cm/s", 
                 i + 1, distance_to_point, travel_str, (double)g_op_state.updated_speed_estimate / 1000.0);
        
        // Start movement to this point
        winch_direction_t direction = is_rising ? WINCH_UP : WINCH_DOWN;
        winch_move(direction);
        
        esp_err_t result = await_completion_and_handle_events(state, travel_time_ms);
  
        // Stop at this point
        winch_move(WINCH_STOP);
        if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
            return result;  // Error occurred (AUTO disabled, etc.)
        }
        
        ESP_LOGI(TAG, "Reached point %d/%d, starting static wait", i + 1, num_points);
        
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
        
        // Event-driven static wait - monitor for HOME (if rising) and AUTO disable
        TickType_t wait_start = xTaskGetTickCount();
        while (1) {
            InputEventExt result = wait_until_notified_or_time(wait_start, static_wait_ms);
            
            switch (result.event.event_type) {
                case HOME:
                    if (result.event.state && is_rising) {
                        ESP_LOGI(TAG, "Home reached during static wait at point %d - operation complete", i + 1);
                        g_op_state.total_static_wait_time_ms += timer_ms_since_start(&wait_timer);
                        g_op_state.in_static_wait = false;
                        return ESP_OK;  // Early completion
                    }
                    break;
                case AUTO:
                    if (!result.event.state) {
                        ESP_LOGW(TAG, "AUTO disabled during static wait at point %d", i + 1);
                        g_op_state.total_static_wait_time_ms += timer_ms_since_start(&wait_timer);
                        g_op_state.in_static_wait = false;
                        return ESP_ERR_INVALID_STATE;
                    }
                    
                case TIMEOUT:
                    goto wait_complete;  // Static wait finished normally
                
                default:
                    break;  // Ignore other events during static wait
            }
        }
        
wait_complete:
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
        
        // Calculate timeout based on distance from last point to surface plus buffer
        uint32_t distance_to_home = pkg->static_points[num_points-1]; // Distance from last point to surface
        uint32_t timeout_ms = calculate_expected_time_ms(distance_to_home, g_op_state.updated_speed_estimate) + 5000; // 5 second buffer
        
        return handle_time_based_operation(state, timeout_ms, timeout_ms, "STATIC_DEPTH");
    }
    
    return ESP_OK;
}

static esp_err_t await_completion_and_handle_events(state_t STATE, uint32_t timeout_ms) {
    TickType_t start_tick = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Waiting for operation complete , timeout: %d s", timeout_ms/1000);
    
    while (1) {
        InputEventExt result = wait_until_notified_or_time(start_tick, timeout_ms);
        
        switch (result.event.event_type) {
            case HOME:
                if (result.event.state && ((STATE == RISING) || (STATE == INIT))) {
                    ESP_LOGI(TAG, "Home position reached");
                    return ESP_OK;
                }
                break;
                
            case AUTO:
                if (!result.event.state) {
                    ESP_LOGW(TAG, "AUTO mode disabled during operation");
                    winch_move(WINCH_STOP);
                    return ESP_ERR_INVALID_STATE;
                }
                break;
                
            case TENTION:
                if (result.event.state) {
                    ESP_LOGW(TAG, "Tension alarm - attempting recovery");
                    esp_err_t recovery_result = handle_tension_recovery(STATE);
                    if (recovery_result != ESP_OK) {
                        winch_move(WINCH_STOP); 
                        return recovery_result;
                    }
                }
                break;
                
            case TIMEOUT:
                if (STATE == LOWERING) return ESP_OK;
                // else its a timeout on rising
                ESP_LOGW(TAG, "Operation timeout");
                return ESP_ERR_TIMEOUT;
            default: return ESP_ERR_INVALID_STATE;
        }
    }
}

static esp_err_t handle_tension_recovery(state_t state) {
    ESP_LOGI(TAG, "Starting tension recovery - reversing direction for %dms", TENSION_REVERSE_TIME_MS);
    w_timer_t recovery_timer;
    timer_start(&recovery_timer);
    winch_direction_t original_direction = (state == LOWERING) ? WINCH_DOWN : WINCH_UP;
    winch_direction_t reverse_dir = (original_direction == WINCH_DOWN) ? WINCH_UP : WINCH_DOWN;
    
    // Reverse direction briefly
    winch_move(reverse_dir);
    
    // Wait during reverse movement, checking for events
    TickType_t reverse_start = xTaskGetTickCount();
    while (1) {
        InputEventExt result = wait_until_notified_or_time(reverse_start, TENSION_REVERSE_TIME_MS);
        
        switch (result.event.event_type) {
            case HOME:
                if (result.event.state) {  // Home sensor active
                    ESP_LOGI(TAG, "Home reached during tension recovery");
                    goto recovery_complete;
                }
                break;
                
            case AUTO:
                if (!result.event.state) {  // Auto mode disabled
                    ESP_LOGW(TAG, "AUTO disabled during tension recovery");
                    return ESP_ERR_INVALID_STATE;
                }
                break;
                
            case TENTION:
                if (!result.event.state) {  // Tension cleared
                    ESP_LOGI(TAG, "Tension cleared during reverse - resuming original direction");
                    goto tension_cleared;
                }
                break;
                
            case TIMEOUT:
                goto tension_cleared;  // Time to check if we should continue
        }
    }
    
tension_cleared:
    if (inputs_get_winch_tension()){
        winch_move(WINCH_STOP);
        return ESP_FAIL; //return if its still tention..
    }
    winch_move(original_direction);
    
    // Wait during recovery pause, checking for events
    TickType_t pause_start = xTaskGetTickCount();
    while (1) {
        InputEventExt result = wait_until_notified_or_time(pause_start, TENSION_RECOVERY_PAUSE_MS);
        
        switch (result.event.event_type) {
            case HOME:
                if (result.event.state) {  // Home sensor active
                    ESP_LOGI(TAG, "Home reached during recovery pause");
                    goto recovery_complete;
                }
                break;
                
            case AUTO:
                if (!result.event.state) {  // Auto mode disabled
                    ESP_LOGW(TAG, "AUTO disabled during recovery pause");
                    return ESP_ERR_INVALID_STATE;
                }
                break;
                
            case TENTION:
                if (result.event.state) {  // Tension came back
                    ESP_LOGE(TAG, "Tension returned during recovery - failed");
                    return ESP_ERR_INVALID_STATE;
                }
                break;
                
            case TIMEOUT:
                goto recovery_complete;
        }
    }
    
recovery_complete:
    g_op_state.total_recovery_time_ms += timer_ms_since_start(&recovery_timer);
    return ESP_OK;
}

static esp_err_t handle_rising_timeout_recovery_sequence(void) {
    g_op_state.recovery_attempts++;
    if (g_op_state.recovery_attempts > MAX_RECOVERY_RETRIES) return ESP_ERR_INVALID_SIZE;
    ESP_LOGW(TAG, "Timeout on rising - attempting recovery nr: %d/%d", g_op_state.recovery_attempts, MAX_RECOVERY_RETRIES);
    w_timer_t recovery_timer;
    timer_start(&recovery_timer);
    
    // Recovery: go down a bit
    ESP_LOGI(TAG, "Recovery: Moving down");
    winch_move(WINCH_DOWN);
    
    esp_err_t result = await_completion_and_handle_events(LOWERING, RECOVERY_DOWN_TIME_MS);
    if (result != ESP_OK) return result;
    
    ESP_LOGI(TAG, "Recovery: Moving up to home");
    winch_move(WINCH_UP);
    result = await_completion_and_handle_events(RISING, RECOVERY_UP_TIMEOUT_MS);
    
    // Track recovery time
    g_op_state.total_recovery_time_ms += timer_ms_since_start(&recovery_timer);
    
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
    
    // Zero out the entire state
    memset(&g_op_state, 0, sizeof(winch_operation_state_t));
    
    // Restore event structures
    g_op_state.event_queue = queue;
    g_op_state.current_direction = WINCH_INVALID; // Force first direction change to be logged
}

static uint8_t sort_and_deduplicate_points(uint16_t *points, uint8_t count) {
    if (count <= 1) return count;
    
    // Simple bubble sort (small arrays, so efficiency doesn't matter)
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - 1 - i; j++) {
            if (points[j] > points[j + 1]) {
                uint16_t temp = points[j];
                points[j] = points[j + 1];
                points[j + 1] = temp;
            }
        }
    }
    
    // Remove duplicates (array is now sorted)
    uint8_t unique_count = 1;
    for (int i = 1; i < count; i++) {
        if (points[i] != points[unique_count - 1]) {
            points[unique_count] = points[i];
            unique_count++;
        }
    }
    
    return unique_count;
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
    
    if (pkg->end_depth > MAXIMUM_DEPTH) { // Sanity check - 80m max
        ESP_LOGE(TAG, "Invalid end_depth: %d", pkg->end_depth);
        return false;
    }
    // TODO:: ADD one for speed when we know what we can expect! 

    // make sorting of this ourself
     if (pkg->poll_type == STATIC_DEPTH) {
        // Step 1: Extract valid points (within depth limit)
        uint16_t temp_points[MAX_POINTS];
        uint8_t valid_count = 0;
        
        for (int i = 0; i < MAX_POINTS && pkg->static_points[i] != 0; i++) {
            if (pkg->static_points[i] <= MAXIMUM_DEPTH) {
                temp_points[valid_count] = pkg->static_points[i];
                valid_count++;
                ESP_LOGD(TAG, "Accepted point %d: %d cm", valid_count, pkg->static_points[i]);
            } else {
                ESP_LOGW(TAG, "Rejected point %d: %d cm (exceeds max depth %d)", 
                         i, pkg->static_points[i], MAXIMUM_DEPTH);
            }
        }
        
        if (valid_count == 0) {
            ESP_LOGE(TAG, "STATIC_DEPTH: No valid static points after filtering");
            return false;
        }
        
        // Step 2: Sort ascending and remove duplicates
        uint8_t original_count = valid_count;
        valid_count = sort_and_deduplicate_points(temp_points, valid_count);
        
        if (valid_count != original_count) {
            ESP_LOGI(TAG, "Removed %d duplicate points, %d unique points remaining", 
                     original_count - valid_count, valid_count);
        }
        
        // Step 3: Log sorted points for verification
        ESP_LOGI(TAG, "Sorted static points (ascending):");
        for (int i = 0; i < valid_count; i++) {
            ESP_LOGI(TAG, "  Point %d: %d cm", i + 1, temp_points[i]);
        }
        
        // Step 4: Copy to global state in correct order based on direction
        g_op_state.num_points = valid_count;
        
        if (pkg->STATE == RISING) {
            // Reverse for rising (descending: 600 → 400 → 200)
            for (int i = 0; i < valid_count; i++) {
                g_op_state.sorted_static_points[i] = temp_points[valid_count - 1 - i];
            }
            ESP_LOGI(TAG, "RISING: Using %d static points in descending order", valid_count);
        } else {
            // Keep ascending for lowering (0 → 200 → 400)
            for (int i = 0; i < valid_count; i++) {
                g_op_state.sorted_static_points[i] = temp_points[i];
            }
            ESP_LOGI(TAG, "LOWERING: Using %d static points in ascending order", valid_count);
        }
        
        // Step 5: Log final order for debugging
        ESP_LOGI(TAG, "Final static points order for %s:", 
                 pkg->STATE == RISING ? "RISING" : "LOWERING");
        for (int i = 0; i < valid_count; i++) {
            ESP_LOGI(TAG, "  Point %d: %d cm", i + 1, g_op_state.sorted_static_points[i]);
        }
        
        // Step 6: Validate final sequence makes sense
        if (pkg->STATE == RISING) {
            // Should be descending
            for (int i = 1; i < valid_count; i++) {
                if (g_op_state.sorted_static_points[i] >= g_op_state.sorted_static_points[i-1]) {
                    ESP_LOGE(TAG, "RISING validation failed: point %d (%d) >= point %d (%d)", 
                             i+1, g_op_state.sorted_static_points[i], 
                             i, g_op_state.sorted_static_points[i-1]);
                    return false;
                }
            }
        } else {
            // Should be ascending  
            for (int i = 1; i < valid_count; i++) {
                if (g_op_state.sorted_static_points[i] <= g_op_state.sorted_static_points[i-1]) {
                    ESP_LOGE(TAG, "LOWERING validation failed: point %d (%d) <= point %d (%d)", 
                             i+1, g_op_state.sorted_static_points[i], 
                             i, g_op_state.sorted_static_points[i-1]);
                    return false;
                }
            }
        }
        
        ESP_LOGI(TAG, "Static points validation successful: %d points, direction %s", 
                 valid_count, pkg->STATE == RISING ? "RISING" : "LOWERING");
    }
    
    return true;
}

uint32_t calculate_expected_time_ms(uint16_t distance_cm, uint16_t speed_cm_per_s) {
    if (speed_cm_per_s == 0 || speed_cm_per_s > UINT16_MAX) {
        ESP_LOGW(TAG, "Zero speed estimate, using default time");
        return 10000; // 10 second default
    }
    // Convert scaled speed back to real cm/s (scaling: 1000 = 1 cm/s)
    double real_speed = (double)speed_cm_per_s / 1000.0;
    
    // Calculate basic travel time in seconds, convert to ms
    return (uint32_t)((distance_cm / real_speed) * 1000.0);
}

uint16_t update_speed_estimate_pre_operation(const motorcontroller_pkg_t *pkg) {
    // Start with previous estimate - fallback - calculation is moved to the manager
    g_op_state.updated_speed_estimate = pkg->prev_estimated_cm_per_s;
    return g_op_state.updated_speed_estimate;
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

static const char* get_poll_type_string(POLL_TYPE poll_type) {
    switch (poll_type) {
        case ALPHA_DEPTH: return "ALPHA_DEPTH";
        case STATIC_DEPTH: return "STATIC_DEPTH";
        case LIN_TIME: return "LIN_TIME";
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


