// winch_controller_sm.c - State Machine Implementation
// Keeps EXACT same interface, but internally uses formal state machine

#include "winch_controller.h"
#include "outputs.h"
#include "inputs.h"
#include "timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char* TAG = "winch_controller_sm";

// ============================================================================
// WINCH DIRECTION TYPE (other types included from buoye_structs.h)
// ============================================================================

// Winch direction control
typedef enum {
    WINCH_STOP = 0,
    WINCH_UP,
    WINCH_DOWN,
    WINCH_INVALID = 255  // Used to force first direction change
} winch_direction_t;

// Lamp control states
typedef enum {
    AUTO_DISABLED,           // We don't handle manual direction input so we don't light anything other than error if present
    AUTO_DISABLED_W_ERROR,   // Auto disabled but with error indication
    // Rest is implicit that we are in auto mode
    MOVING,                  // Winch is moving (up or down)
    STATIC,                  // Static wait (stopped but operation active)
    ERROR,                   // Error state - show alarm
    TURN_OFF_LAMPS          // Set all false
} lamp_state_t;

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// Timing and safety constants
#define MAXIMUM_DEPTH 8000 //80m
#define MAXIMUM_SPEED 50000 //50.000 cm/s (scaled by 1000: 50 cm/s actual)
#define MAX_RECOVERY_RETRIES 3
#define WINCH_TIMEOUT_SAFETY_MARGIN_PERCENT 30
#define SOFT_START_STOP_COMPENSATION_MS 500
#define STATIC_POLL_STOP_OFFSET_MS 1000
#define STATIC_POLL_TIMING_BUFFER_MS 200
#define MIN_OPERATION_TIME_MS 100
#define RECOVERY_DOWN_TIME_MS 2000
#define RECOVERY_UP_TIMEOUT_MS 8000
#define RECOVERY_PAUSE_TIME_MS 1000
#define TENSION_REVERSE_TIME_MS 1000
#define TENSION_RECOVERY_PAUSE_MS 500
#define MIN_TRAVEL_PERCENT_BEFOR_HOME_SENSOR_IS_ACTIVE 50 // 0 - 100 

// State machine specific timeouts
#define STATIC_WAIT_BUFFER_MS 1000
#define MAX_TIMEOUT_RECOVERY_ATTEMPTS 3
#define DEFAULT_MOVING_TIMEOUT_MS 10000
#define DEFAULT_INIT_TIMEOUT_MS 30000

// INIT operation specific timeouts
#define GOING_HOME_TIMEOUT_MS 100000            // 100 seconds for going home

// Home sensor protection against false triggers
#define MIN_UPWARD_TIME_FOR_HOME_MS 3000        // Minimum 3 seconds upward before accepting home
#define MIN_REASONABLE_SPEED_CM_S_x1000 500     // 0.500 cm/s minimum reasonable speed (scaled by 1000)

// Static point validation and safety
#define MIN_SAFE_DEPTH_CM 10                    // Minimum depth (safety above water)

// Scaling factors for clarity
#define SPEED_SCALING_FACTOR 1000               // Speed values scaled by 1000 (e.g. 10000 = 10.000 cm/s)

// Safety philosophy: If any critical system component fails (like event queue),
// we immediately stop motors and enter error state rather than crash the system.

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================

typedef enum {
    // Main operation states
    WS_IDLE,
    WS_MOVING_DOWN,
    WS_MOVING_UP,
    WS_STATIC_WAIT,
    WS_COMPLETE,
    WS_ERROR,
    
    // Recovery states
    WS_TENSION_RECOVERY_REVERSE,
    WS_TENSION_RECOVERY_PAUSE,
    WS_TIMEOUT_RECOVERY_DOWN,
    WS_TIMEOUT_RECOVERY_UP,
    WS_TIMEOUT_RECOVERY_PAUSE,
    
    // Special states
    WS_INIT_GOING_HOME,
    WS_STATIC_MOVING_TO_POINT,
} winch_state_t;

typedef enum {
    WE_START_OPERATION,
    WE_HOME_REACHED,
    WE_TENSION_ALARM,
    WE_TENSION_CLEARED,
    WE_AUTO_DISABLED,
    WE_TIMEOUT,
    WE_STATIC_POINT_REACHED,
    WE_RECOVERY_COMPLETE,
    WE_OPERATION_TIMEOUT,
    WE_CONTINUE_OPERATION,
    WE_NO_EVENT  // Used when timeout occurs
} winch_event_t;

typedef struct {
    winch_event_t event;
    uint32_t timestamp_ms;
    bool edge_detected;
} winch_event_data_t;

// State context - contains all the operation data
typedef struct {
    // Current state machine state
    winch_state_t current_state;
    winch_state_t previous_state;
    
    // Operation parameters (updated for new pkg structure)
    const motorcontroller_pkg_t *current_pkg;
    motorcontroller_response_t *current_resp;
    uint16_t validated_speed_cm_per_s_x1000;   // Validated speed from pkg (scaled by 1000)
    
    // Timers - simplified approach
    w_timer_t operation_timer;          // Started once, never restarted
    w_timer_t recovery_timer;           // For tracking individual recovery phases
    w_timer_t static_timer;             // For tracking individual static wait phases
    
    // Time tracking for working time calculation
    uint32_t total_recovery_time_ms;
    uint32_t total_static_wait_time_ms;
    uint32_t state_start_time_ms;       // When current state started (relative to operation_timer)
    
    // Static depth handling
    uint8_t current_static_point;
    uint16_t current_target_depth;
    uint8_t num_points;
    uint16_t sorted_static_points[MAX_POINTS];
    
    // Recovery tracking
    uint8_t recovery_attempts;
    uint32_t current_timeout_ms;
    
    // State management
    bool operation_active;
    QueueHandle_t input_queue;
    
    // Current direction (for logging state changes)
    winch_direction_t current_direction;
    
    // Home sensor protection (simplified for stateless operation)
    uint32_t upward_movement_start_time_ms;     // When did we start moving up
    uint32_t estimated_min_return_time_ms;      // Based on distance and speed
} winch_state_context_t;

// ============================================================================
// STATE HANDLER FUNCTION TYPES
// ============================================================================

typedef winch_state_t (*state_enter_func_t)(winch_state_context_t *ctx);
typedef winch_state_t (*state_event_func_t)(winch_state_context_t *ctx, winch_event_t event);
typedef void (*state_exit_func_t)(winch_state_context_t *ctx);

typedef struct {
    const char *name;
    state_enter_func_t on_enter;
    state_event_func_t on_event;
    state_exit_func_t on_exit;
    uint32_t default_timeout_ms;
} state_handler_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================

static winch_state_context_t g_ctx = {0};
static bool g_controller_initialized = false;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

// State handlers
static winch_state_t idle_enter(winch_state_context_t *ctx);
static winch_state_t idle_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t moving_down_enter(winch_state_context_t *ctx);
static winch_state_t moving_down_event(winch_state_context_t *ctx, winch_event_t event);
static void moving_down_exit(winch_state_context_t *ctx);

static winch_state_t moving_up_enter(winch_state_context_t *ctx);
static winch_state_t moving_up_event(winch_state_context_t *ctx, winch_event_t event);
static void moving_up_exit(winch_state_context_t *ctx);

static winch_state_t static_wait_enter(winch_state_context_t *ctx);
static winch_state_t static_wait_event(winch_state_context_t *ctx, winch_event_t event);
static void static_wait_exit(winch_state_context_t *ctx);

static winch_state_t complete_enter(winch_state_context_t *ctx);
static winch_state_t complete_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t error_enter(winch_state_context_t *ctx);
static winch_state_t error_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t tension_recovery_reverse_enter(winch_state_context_t *ctx);
static winch_state_t tension_recovery_reverse_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t tension_recovery_pause_enter(winch_state_context_t *ctx);
static winch_state_t tension_recovery_pause_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t timeout_recovery_down_enter(winch_state_context_t *ctx);
static winch_state_t timeout_recovery_down_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t timeout_recovery_up_enter(winch_state_context_t *ctx);
static winch_state_t timeout_recovery_up_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t timeout_recovery_pause_enter(winch_state_context_t *ctx);
static winch_state_t timeout_recovery_pause_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t init_going_home_enter(winch_state_context_t *ctx);
static winch_state_t init_going_home_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t static_moving_to_point_enter(winch_state_context_t *ctx);
static winch_state_t static_moving_to_point_event(winch_state_context_t *ctx, winch_event_t event);

// Home sensor protection functions
static void track_upward_movement_start(winch_state_context_t *ctx);
static void calculate_minimum_return_time_from_distance(winch_state_context_t *ctx);

// Lamp control functions
static void update_lamps_for_state(const winch_state_t state);
static void set_operation_lamps(const lamp_state_t mode);

// Static point validation and sanitization
static uint8_t validate_and_sanitize_static_points(const motorcontroller_pkg_t *pkg, 
                                                   uint16_t *sanitized_points, 
                                                   uint8_t max_points);

// Helper functions
static winch_event_data_t wait_for_event_with_timeout(const uint32_t timeout_ms);
static void set_winch_direction(const winch_direction_t direction);
static uint32_t calculate_state_timeout(const winch_state_context_t *ctx);
static uint32_t calculate_movement_timeout(const winch_state_context_t *ctx);
static esp_err_t setup_operation_for_mode(winch_state_context_t *ctx);
static winch_state_t get_next_movement_state(const winch_state_context_t *ctx);
static bool should_go_to_static_wait(const winch_state_context_t *ctx);
static uint32_t calculate_expected_time_ms(uint16_t distance_cm, uint16_t speed_cm_per_s_x1000);
static uint32_t get_working_time_ms(const winch_state_context_t *ctx);
static uint16_t validate_speed_estimate(const uint16_t estimated_cm_per_s_x1000);
static const char* state_name(const winch_state_t state);
static const char* event_name(const winch_event_t event);
static const char* direction_to_string(const winch_direction_t direction);

// Time formatting helper
static void format_time_string(const uint32_t time_ms, char *buffer, const size_t buffer_size);


// ============================================================================
// STATE TRANSITION TABLE
// ============================================================================

static const state_handler_t state_handlers[] = {
    [WS_IDLE] = {
        .name = "IDLE",
        .on_enter = idle_enter,
        .on_event = idle_event,
        .on_exit = NULL,
        .default_timeout_ms = 0
    },
    [WS_MOVING_DOWN] = {
        .name = "MOVING_DOWN", 
        .on_enter = moving_down_enter,
        .on_event = moving_down_event,
        .on_exit = moving_down_exit,
        .default_timeout_ms = DEFAULT_MOVING_TIMEOUT_MS
    },
    [WS_MOVING_UP] = {
        .name = "MOVING_UP",
        .on_enter = moving_up_enter, 
        .on_event = moving_up_event,
        .on_exit = moving_up_exit,
        .default_timeout_ms = DEFAULT_INIT_TIMEOUT_MS
    },
    [WS_STATIC_WAIT] = {
        .name = "STATIC_WAIT",
        .on_enter = static_wait_enter,
        .on_event = static_wait_event, 
        .on_exit = static_wait_exit,
        .default_timeout_ms = DEFAULT_MOVING_TIMEOUT_MS
    },
    [WS_COMPLETE] = {
        .name = "COMPLETE",
        .on_enter = complete_enter,
        .on_event = complete_event,
        .on_exit = NULL,
        .default_timeout_ms = 0
    },
    [WS_ERROR] = {
        .name = "ERROR",
        .on_enter = error_enter,
        .on_event = error_event,
        .on_exit = NULL,
        .default_timeout_ms = 0
    },
    [WS_TENSION_RECOVERY_REVERSE] = {
        .name = "TENSION_RECOVERY_REVERSE",
        .on_enter = tension_recovery_reverse_enter,
        .on_event = tension_recovery_reverse_event,
        .on_exit = NULL,
        .default_timeout_ms = TENSION_REVERSE_TIME_MS
    },
    [WS_TENSION_RECOVERY_PAUSE] = {
        .name = "TENSION_RECOVERY_PAUSE",
        .on_enter = tension_recovery_pause_enter,
        .on_event = tension_recovery_pause_event,
        .on_exit = NULL,
        .default_timeout_ms = TENSION_RECOVERY_PAUSE_MS
    },
    [WS_TIMEOUT_RECOVERY_DOWN] = {
        .name = "TIMEOUT_RECOVERY_DOWN", 
        .on_enter = timeout_recovery_down_enter,
        .on_event = timeout_recovery_down_event,
        .on_exit = NULL,
        .default_timeout_ms = RECOVERY_DOWN_TIME_MS
    },
    [WS_TIMEOUT_RECOVERY_UP] = {
        .name = "TIMEOUT_RECOVERY_UP",
        .on_enter = timeout_recovery_up_enter,
        .on_event = timeout_recovery_up_event,
        .on_exit = NULL,
        .default_timeout_ms = RECOVERY_UP_TIMEOUT_MS
    },
    [WS_TIMEOUT_RECOVERY_PAUSE] = {
        .name = "TIMEOUT_RECOVERY_PAUSE",
        .on_enter = timeout_recovery_pause_enter,
        .on_event = timeout_recovery_pause_event,
        .on_exit = NULL,
        .default_timeout_ms = RECOVERY_PAUSE_TIME_MS
    },
    [WS_INIT_GOING_HOME] = {
        .name = "INIT_GOING_HOME",
        .on_enter = init_going_home_enter,
        .on_event = init_going_home_event,
        .on_exit = NULL,
        .default_timeout_ms = GOING_HOME_TIMEOUT_MS
    },
    [WS_STATIC_MOVING_TO_POINT] = {
        .name = "STATIC_MOVING_TO_POINT",
        .on_enter = static_moving_to_point_enter,
        .on_event = static_moving_to_point_event,
        .on_exit = NULL,
        .default_timeout_ms = DEFAULT_MOVING_TIMEOUT_MS
    }
};

// ============================================================================
// STATE MACHINE ENGINE
// ============================================================================

static winch_state_t transition_to_state(winch_state_t new_state) {
    if (new_state == g_ctx.current_state) {
        return new_state; // No transition needed
    }
    
    ESP_LOGI(TAG, "State transition: %s -> %s", 
             state_name(g_ctx.current_state), state_name(new_state));
    
    // Exit current state (with null check)
    if (state_handlers[g_ctx.current_state].on_exit) {
        state_handlers[g_ctx.current_state].on_exit(&g_ctx);
    }
    
    // Update state
    g_ctx.previous_state = g_ctx.current_state;
    g_ctx.current_state = new_state;
    
    // Record when this state started (relative to operation timer)
    g_ctx.state_start_time_ms = timer_ms_since_start(&g_ctx.operation_timer);
    
    // Update lamps for new state
    update_lamps_for_state(new_state);
    
    // Enter new state (with null check)
    if (state_handlers[new_state].on_enter) {
        winch_state_t next_state = state_handlers[new_state].on_enter(&g_ctx);
        if (next_state != new_state) {
            // Immediate transition requested
            return transition_to_state(next_state);
        }
    }
    
    return new_state;
}

static winch_state_t process_event(winch_event_t event) {
    ESP_LOGD(TAG, "Processing event %s in state %s", 
             event_name(event), state_name(g_ctx.current_state));
    
    if (state_handlers[g_ctx.current_state].on_event) {
        winch_state_t new_state = state_handlers[g_ctx.current_state].on_event(&g_ctx, event);
        if (new_state != g_ctx.current_state) {
            return transition_to_state(new_state);
        } else {
            // Staying in same state - DO NOT restart timer for movement operations
            // Timer should only restart for specific state transitions, not ignored events
            ESP_LOGD(TAG, "Staying in state %s, timer NOT restarted (preserving timeout)", 
                     state_name(g_ctx.current_state));
        }
    }
    
    return g_ctx.current_state; // No handler or no transition
}

// ============================================================================
// MAIN STATE MACHINE LOOP
// ============================================================================

static esp_err_t run_state_machine(void) {
    ESP_LOGI(TAG, "Starting state machine");
    
    // Initialize to idle and start operation
    g_ctx.current_state = WS_IDLE;
    transition_to_state(WS_IDLE);
    process_event(WE_START_OPERATION);
    
    // Main state machine loop
    while (g_ctx.operation_active) {
        uint32_t timeout = calculate_state_timeout(&g_ctx);
        uint32_t elapsed_in_state = timer_ms_since_start(&g_ctx.operation_timer) - g_ctx.state_start_time_ms;
        uint32_t remaining_timeout = (timeout > elapsed_in_state) ? (timeout - elapsed_in_state) : 0;
        
        // If we've already exceeded the timeout, generate timeout event immediately
        if (remaining_timeout == 0) {
            winch_state_t new_state = process_event(WE_TIMEOUT);
            if (new_state == WS_COMPLETE) {
                g_ctx.current_resp->result = ESP_OK;
                break;
            } else if (new_state == WS_ERROR) {
                g_ctx.current_resp->result = ESP_FAIL;
                break;
            }
            continue; // Check again with new state
        }
        
        winch_event_data_t event_data = wait_for_event_with_timeout(remaining_timeout);
        
        winch_state_t new_state = process_event(event_data.event);
        
        // Check for completion or error
        if (new_state == WS_COMPLETE) {
            g_ctx.current_resp->result = ESP_OK;
            break;
        } else if (new_state == WS_ERROR) {
            g_ctx.current_resp->result = ESP_FAIL;
            break;
        }
    }
    
    // Final cleanup
    set_winch_direction(WINCH_STOP);
    
    // Calculate working time: total time minus recovery and static wait time
    uint32_t working_time_ms = get_working_time_ms(&g_ctx);
    
    // Fill response
    g_ctx.current_resp->working_time = working_time_ms / 1000; // Convert to seconds
    g_ctx.current_resp->estimated_cm_per_s = g_ctx.validated_speed_cm_per_s_x1000;
    
    char total_time_str[64], working_time_str[64], recovery_time_str[64], static_time_str[64];
    format_time_string(timer_ms_since_start(&g_ctx.operation_timer), total_time_str, sizeof(total_time_str));
    format_time_string(working_time_ms, working_time_str, sizeof(working_time_str));
    format_time_string(g_ctx.total_recovery_time_ms, recovery_time_str, sizeof(recovery_time_str));
    format_time_string(g_ctx.total_static_wait_time_ms, static_time_str, sizeof(static_time_str));
    
    ESP_LOGI(TAG, "State machine completed - Result: %s", esp_err_to_name(g_ctx.current_resp->result));
    ESP_LOGI(TAG, "  Total: %s, Working: %s, Recovery: %s, Static: %s", 
             total_time_str, working_time_str, recovery_time_str, static_time_str);
    
    return g_ctx.current_resp->result;
}

// ============================================================================
// LAMP CONTROL FUNCTIONS
// ============================================================================

static void set_operation_lamps(lamp_state_t mode) {
    switch (mode) {
        case AUTO_DISABLED:
            outputs_set_lamp(false);           // No movement
            outputs_set_auto_lamp(false);      // Auto mode disabled
            outputs_set_alarm_lamp(false);     // No error
            break;
            
        case AUTO_DISABLED_W_ERROR:
            outputs_set_lamp(false);           // No movement
            outputs_set_auto_lamp(false);      // Auto mode disabled
            outputs_set_alarm_lamp(true);      // Error indicator
            break;
            
        case MOVING:
            outputs_set_lamp(true);            // Movement active
            outputs_set_auto_lamp(true);       // Auto mode enabled
            outputs_set_alarm_lamp(false);     // No error
            break;
            
        case STATIC:
            outputs_set_lamp(true);            // Operation active
            outputs_set_auto_lamp(true);       // Auto mode enabled
            outputs_set_alarm_lamp(false);     // No error
            break;
            
        case ERROR:
            outputs_set_lamp(false);           // Stop movement indication
            outputs_set_auto_lamp(false);      // Error overrides auto
            outputs_set_alarm_lamp(true);      // Error indicator
            break;
            
        case TURN_OFF_LAMPS:
            outputs_set_lamp(false);           // All off
            outputs_set_auto_lamp(false);      // All off
            outputs_set_alarm_lamp(false);     // All off
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown lamp state: %d, turning off all lamps", mode);
            outputs_set_lamp(false);
            outputs_set_auto_lamp(false);
            outputs_set_alarm_lamp(true);      // Unknown state = error
            break;
    }
}

static void update_lamps_for_state(winch_state_t state) {
    bool auto_mode = inputs_get_auto_enable();
    
    // If auto mode is disabled, handle accordingly
    if (!auto_mode) {
        set_operation_lamps(AUTO_DISABLED);
        return;
    }
    
    // Auto mode is enabled, set lamps based on state
    switch (state) {
        case WS_IDLE:
        case WS_COMPLETE:
            set_operation_lamps(STATIC);       // Show auto enabled but not moving
            break;
            
        case WS_MOVING_DOWN:
        case WS_MOVING_UP:
        case WS_STATIC_MOVING_TO_POINT:
        case WS_INIT_GOING_HOME:
            set_operation_lamps(MOVING);       // Active movement
            break;
            
        case WS_STATIC_WAIT:
            set_operation_lamps(STATIC);       // Static sampling
            break;
            
        case WS_ERROR:
            set_operation_lamps(ERROR);        // Error state
            break;
            
        case WS_TENSION_RECOVERY_REVERSE:
        case WS_TENSION_RECOVERY_PAUSE:
        case WS_TIMEOUT_RECOVERY_DOWN:
        case WS_TIMEOUT_RECOVERY_UP:
        case WS_TIMEOUT_RECOVERY_PAUSE:
            set_operation_lamps(ERROR);        // Recovery = error indication
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown state for lamp control: %d", state);
            set_operation_lamps(ERROR);        // Unknown state = error
            break;
    }
}

// ============================================================================
// STATE HANDLERS IMPLEMENTATION
// ============================================================================

static winch_state_t idle_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_STOP);
    return WS_IDLE;
}

static winch_state_t idle_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_START_OPERATION:
            return get_next_movement_state(ctx);
        default:
            return WS_IDLE;
    }
}

static winch_state_t moving_down_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_DOWN);
    return WS_MOVING_DOWN;
}

static winch_state_t moving_down_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            // Shouldn't happen when moving down, but handle gracefully
            ESP_LOGW(TAG, "HOME reached while moving down - unexpected but completing");
            return WS_COMPLETE;
            
        case WE_TENSION_ALARM:
            return WS_TENSION_RECOVERY_REVERSE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            ESP_LOGI(TAG, "MOVING_DOWN timeout - checking completion conditions");
            
            // Check if we should go to static wait or complete
            if (should_go_to_static_wait(ctx)) {
                ESP_LOGI(TAG, "Going to static wait");
                return WS_STATIC_WAIT;
            }
            
            // For ALPHA_DEPTH lowering operations, timeout means we've reached target depth
            if (ctx->current_pkg->poll_type == ALPHA_DEPTH) {
                ESP_LOGI(TAG, "ALPHA_DEPTH lowering complete - reached target depth");
                return WS_COMPLETE;
            }
            
            // For other poll types, timeout during lowering usually means completion
            ESP_LOGI(TAG, "Lowering operation complete");
            return WS_COMPLETE;
            
        default:
            return WS_MOVING_DOWN;
    }
}

static void moving_down_exit(winch_state_context_t *ctx) {
    // No longer tracking downward time for next operation (stateless operation)
    // Any other cleanup when exiting moving down state
}

static winch_state_t moving_up_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_UP);
    track_upward_movement_start(ctx);
    calculate_minimum_return_time_from_distance(ctx);
    return WS_MOVING_UP;
}

static winch_state_t moving_up_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            ESP_LOGI(TAG, "HOME reached while moving up - operation complete");
            return WS_COMPLETE;
            
        case WE_TENSION_ALARM:
            return WS_TENSION_RECOVERY_REVERSE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            if (ctx->current_pkg->poll_type == STATIC_DEPTH && 
                ctx->current_static_point == 0 &&
                ctx->current_pkg->end_depth == ctx->sorted_static_points[0]) 
            {
                ESP_LOGI(TAG, "Already at first static point (%d cm), starting static wait", 
                         ctx->sorted_static_points[0]);
                return WS_STATIC_WAIT;
            }
            ESP_LOGI(TAG, "MOVING_UP timeout - checking completion conditions");
            
            // Check if we should go to static wait first
            if (should_go_to_static_wait(ctx)) {
                ESP_LOGI(TAG, "Going to static wait");
                return WS_STATIC_WAIT;
            }
            
            // For timeout during rising we triggers recovery
            ESP_LOGW(TAG, "Timeout during rising operation - starting recovery");
            return WS_TIMEOUT_RECOVERY_DOWN;
            
        default:
            return WS_MOVING_UP;
    }
}

static void moving_up_exit(winch_state_context_t *ctx) {
    // Any cleanup when exiting moving up state
}

static winch_state_t static_wait_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_STOP);
    
    // Start tracking static wait time
    timer_start(&ctx->static_timer);
    
    ESP_LOGI(TAG, "Static wait at point %d/%d (depth %d cm)", 
             ctx->current_static_point + 1, ctx->num_points,
             ctx->sorted_static_points[ctx->current_static_point]);
    
    return WS_STATIC_WAIT;
}

static winch_state_t static_wait_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            if (ctx->current_pkg->STATE == RISING) {
                return WS_COMPLETE;
            }
            break;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Static wait complete, move to next point or complete
            ctx->current_static_point++;
            if (ctx->current_static_point >= ctx->num_points) {
                // All static points done
                if (ctx->current_pkg->STATE == RISING) {
                    return WS_MOVING_UP; // Continue to home
                } else {
                    return WS_COMPLETE;
                }
            } else {
                // Move to next static point
                return get_next_movement_state(ctx);
            }
            
        default:
            break;
    }
    return WS_STATIC_WAIT;
}

static void static_wait_exit(winch_state_context_t *ctx) {
    // Add static wait time to exclusion total
    uint32_t static_wait_time = timer_ms_since_start(&ctx->static_timer);
    ctx->total_static_wait_time_ms += static_wait_time;
    
    char time_str[64];
    format_time_string(static_wait_time, time_str, sizeof(time_str));
    ESP_LOGD(TAG, "Static wait complete, added %s to static wait time", time_str);
}

static winch_state_t complete_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_STOP);
    ctx->operation_active = false;
    ESP_LOGI(TAG, "Operation completed successfully");
    return WS_COMPLETE;
}

static winch_state_t complete_event(winch_state_context_t *ctx, winch_event_t event) {
    // Complete state doesn't respond to events
    return WS_COMPLETE;
}

static winch_state_t error_enter(winch_state_context_t *ctx) {
    // SAFETY: Always stop motors when entering error state
    set_winch_direction(WINCH_STOP);
    ctx->operation_active = false;
    
    ESP_LOGE(TAG, "💥 Operation failed - entering error state");
    ESP_LOGE(TAG, "🛑 Motors stopped for safety");
    
    return WS_ERROR;
}

static winch_state_t error_event(winch_state_context_t *ctx, winch_event_t event) {
    // Error state doesn't respond to events
    return WS_ERROR;
}

static winch_state_t tension_recovery_reverse_enter(winch_state_context_t *ctx) {
    timer_start(&ctx->recovery_timer);
    
    // Reverse direction
    winch_direction_t reverse_dir = (ctx->current_direction == WINCH_DOWN) ? WINCH_UP : WINCH_DOWN;
    set_winch_direction(reverse_dir);
    
    // NOTE: Do NOT call track_upward_movement_start() for recovery operations
    // We want to accept home sensor immediately during any recovery
    
    ESP_LOGW(TAG, "Tension recovery - reversing direction to %s (home sensor active immediately)", 
             direction_to_string(reverse_dir));
    return WS_TENSION_RECOVERY_REVERSE;
}

static winch_state_t tension_recovery_reverse_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            return WS_COMPLETE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TENSION_CLEARED:
            // Resume original operation - add recovery time to total
            uint32_t recovery_time = timer_ms_since_start(&ctx->recovery_timer);
            ctx->total_recovery_time_ms += recovery_time;
            
            char time_str[64];
            format_time_string(recovery_time, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "Tension recovery complete, added %s to recovery time", time_str);
            return get_next_movement_state(ctx);
            
        case WE_TIMEOUT:
            // Recovery time elapsed, check if tension is still there
            if (inputs_get_winch_tension()) {
                return WS_ERROR; // Still have tension
            } else {
                uint32_t recovery_time = timer_ms_since_start(&ctx->recovery_timer);
                ctx->total_recovery_time_ms += recovery_time;
                
                char time_str[64];
                format_time_string(recovery_time, time_str, sizeof(time_str));
                ESP_LOGI(TAG, "Tension recovery timeout, added %s to recovery time", time_str);
                return get_next_movement_state(ctx);
            }
            
        default:
            return WS_TENSION_RECOVERY_REVERSE;
    }
}

static winch_state_t tension_recovery_pause_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_STOP);
    ESP_LOGI(TAG, "Tension recovery pause");
    return WS_TENSION_RECOVERY_PAUSE;
}

static winch_state_t tension_recovery_pause_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Pause complete, resume operation
            ctx->total_recovery_time_ms += timer_ms_since_start(&ctx->recovery_timer);
            return get_next_movement_state(ctx);
            
        default:
            return WS_TENSION_RECOVERY_PAUSE;
    }
}

static winch_state_t timeout_recovery_down_enter(winch_state_context_t *ctx) {
    ctx->recovery_attempts++;
    if (ctx->recovery_attempts > MAX_TIMEOUT_RECOVERY_ATTEMPTS) {
        ESP_LOGE(TAG, "Maximum recovery attempts (%d) exceeded", MAX_TIMEOUT_RECOVERY_ATTEMPTS);
        return WS_ERROR;
    }
    
    timer_start(&ctx->recovery_timer);
    set_winch_direction(WINCH_DOWN);
    
    ESP_LOGW(TAG, "Timeout recovery - going down (attempt %d/%d)", 
             ctx->recovery_attempts, MAX_TIMEOUT_RECOVERY_ATTEMPTS);
    return WS_TIMEOUT_RECOVERY_DOWN;
}

static winch_state_t timeout_recovery_down_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Switch to recovery up
            return WS_TIMEOUT_RECOVERY_UP;
            
        default:
            return WS_TIMEOUT_RECOVERY_DOWN;
    }
}

static winch_state_t timeout_recovery_up_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_UP);
    // NOTE: Do NOT call track_upward_movement_start() or calculate_minimum_return_time_from_distance()
    // for recovery operations - we want to accept home sensor immediately
    ESP_LOGI(TAG, "Timeout recovery - going up (home sensor active immediately)");
    return WS_TIMEOUT_RECOVERY_UP;
}

static winch_state_t timeout_recovery_up_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            // Recovery successful
            ctx->total_recovery_time_ms += timer_ms_since_start(&ctx->recovery_timer);
            ESP_LOGI(TAG, "Timeout recovery successful");
            return WS_COMPLETE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Recovery failed, try again or give up
            if (ctx->recovery_attempts >= MAX_TIMEOUT_RECOVERY_ATTEMPTS) {
                ESP_LOGE(TAG, "Max recovery attempts reached");
                return WS_ERROR;
            }
            return WS_TIMEOUT_RECOVERY_DOWN; // Try again
            
        default:
            return WS_TIMEOUT_RECOVERY_UP;
    }
}

static winch_state_t timeout_recovery_pause_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_STOP);
    ESP_LOGI(TAG, "Timeout recovery pause");
    return WS_TIMEOUT_RECOVERY_PAUSE;
}

static winch_state_t timeout_recovery_pause_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Resume normal operation
            ctx->total_recovery_time_ms += timer_ms_since_start(&ctx->recovery_timer);
            return get_next_movement_state(ctx);
            
        default:
            return WS_TIMEOUT_RECOVERY_PAUSE;
    }
}

static winch_state_t init_going_home_enter(winch_state_context_t *ctx) {
    // INIT sequence: first go down briefly to ensure we're not stuck, then go up to find home
    set_winch_direction(WINCH_DOWN);
    ESP_LOGI(TAG, "INIT: Going down briefly (%d ms) to clear any obstructions", RECOVERY_DOWN_TIME_MS);
    return WS_INIT_GOING_HOME;
}

static winch_state_t init_going_home_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            // For INIT, home can be reached at any time - no minimum time restrictions
            ESP_LOGI(TAG, "INIT: Home reached - initialization complete!");
            return WS_COMPLETE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Check if we've been going down - if so, switch to going up
            if (ctx->current_direction == WINCH_DOWN) {
                ESP_LOGI(TAG, "INIT: Down phase complete, now going up to find home");
                set_winch_direction(WINCH_UP);
                track_upward_movement_start(ctx);
                // Record when the UP phase started (don't restart timers!)
                ctx->state_start_time_ms = timer_ms_since_start(&ctx->operation_timer);
                return WS_INIT_GOING_HOME; // Stay in same state but now going up
            } else {
                // We've been going up and timed out
                ESP_LOGE(TAG, "INIT: Timeout while going up - failed to find home position");
                return WS_ERROR;
            }
            
        default:
            return WS_INIT_GOING_HOME;
    }
}

static winch_state_t static_moving_to_point_enter(winch_state_context_t *ctx) {
    // Determine direction based on current position and target
    if (ctx->current_static_point < ctx->num_points) {
        winch_direction_t direction = (ctx->current_pkg->STATE == RISING) ? WINCH_UP : WINCH_DOWN;
        set_winch_direction(direction);
        
        ESP_LOGI(TAG, "Moving to static point %d/%d (depth %d cm)", 
                 ctx->current_static_point + 1, ctx->num_points,
                 ctx->sorted_static_points[ctx->current_static_point]);
    }
    
    return WS_STATIC_MOVING_TO_POINT;
}

static winch_state_t static_moving_to_point_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            if (ctx->current_pkg->STATE == RISING) {
                return WS_COMPLETE;
            }
            break;
            
        case WE_TENSION_ALARM:
            return WS_TENSION_RECOVERY_REVERSE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Reached static point
            return WS_STATIC_WAIT;
            
        default:
            break;
    }
    return WS_STATIC_MOVING_TO_POINT;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static winch_event_data_t wait_for_event_with_timeout(const uint32_t timeout_ms) {
    InputEvent input_event;
    winch_event_data_t event_data = {
        .event = WE_NO_EVENT,
        .timestamp_ms = timer_ms_since_start(&g_ctx.operation_timer),
        .edge_detected = false
    };
    
    // SAFETY CHECK: If queue is NULL, stop motors and force error
    if (g_ctx.input_queue == NULL) {
        ESP_LOGE(TAG, "💥 CRITICAL: Event queue is NULL! Stopping motors for safety!");
        set_winch_direction(WINCH_STOP);
        event_data.event = WE_AUTO_DISABLED;  // Force error state
        return event_data;
    }
    
    if (xQueueReceive(g_ctx.input_queue, &input_event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        // Convert input event to winch event
        switch (input_event.event_type) {
            case HOME:
                if (input_event.state) {
                    // Only generate HOME_REACHED when in states that care about home position
                    // and when moving UP (not down) - maybe make a function of this?
                    bool is_upward_state = (g_ctx.current_state == WS_MOVING_UP ||
                                            g_ctx.current_state == WS_TIMEOUT_RECOVERY_UP ||
                                            g_ctx.current_state == WS_INIT_GOING_HOME ||
                                          ((g_ctx.current_state == WS_STATIC_MOVING_TO_POINT ||
                                            g_ctx.current_state == WS_TENSION_RECOVERY_REVERSE) && 
                                            g_ctx.current_direction == WINCH_UP));
                    
                    if (is_upward_state) {
                        // For INIT and RECOVERY operations, allow home sensor at any time (unknown starting position)
                        switch (g_ctx.current_state){
                            case WS_INIT_GOING_HOME:
                            case WS_TIMEOUT_RECOVERY_UP:
                                event_data.event = WE_HOME_REACHED;
                                event_data.edge_detected = input_event.edge_detected;
                                ESP_LOGI(TAG, "HOME sensor accepted (no time restrictions for)");
                                break;
                            case WS_TENSION_RECOVERY_REVERSE:
                                if (g_ctx.current_direction == WINCH_UP) {
                                    event_data.event = WE_HOME_REACHED;
                                    event_data.edge_detected = input_event.edge_detected;
                                    ESP_LOGI(TAG, "TENSION_RECOVERY: HOME sensor accepted immediately (recovery priority)");
                                    break;
                                } // else go to default                        
                            default:
                                // Protect against false home sensor triggers for normal operations only
                                uint32_t upward_time = get_working_time_ms(&g_ctx);
                                uint32_t min_active_time = g_ctx.estimated_min_return_time_ms;
                                bool min_return_time_ok = true;
                                if (min_active_time > 0) {
                                    min_return_time_ok = (upward_time >= min_active_time);
                                }
                                if ( min_return_time_ok) {
                                    event_data.event = WE_HOME_REACHED;
                                    event_data.edge_detected = input_event.edge_detected;
                                    char upward_time_str[64];
                                    format_time_string(upward_time, upward_time_str, sizeof(upward_time_str));
                                    ESP_LOGI(TAG, "NORMAL_OP: HOME sensor accepted after %s upward movement", upward_time_str);
                                } else {
                                    char upward_time_str[64], est_time_str[64];
                                    format_time_string(upward_time, upward_time_str, sizeof(upward_time_str));
                                    format_time_string(min_active_time, est_time_str, sizeof(est_time_str));
                                    ESP_LOGW(TAG, "NORMAL_OP: HOME sensor IGNORED - too early! Upward: %s, min time: %s", 
                                            upward_time_str, est_time_str);
                                }
                                break;
                            }
                    } else{
                        char time_str[64];
                        format_time_string(get_working_time_ms(&g_ctx), time_str, sizeof(time_str));
                         ESP_LOGW(TAG, "NORMAL_OP: HOME sensor IGNORED - we are going down! current working time: %s", 
                                            time_str);
                    }
                    // Ignore HOME events when moving down or in wrong states
                }
                break;
            case TENTION:
                event_data.event = input_event.state ? WE_TENSION_ALARM : WE_TENSION_CLEARED;
                break;
            case AUTO:
                if (!input_event.state) {
                    event_data.event = WE_AUTO_DISABLED;
                }
                break;
            default:
                event_data.event = WE_NO_EVENT;
                break;
        }
    } else {
        // Timeout occurred
        event_data.event = WE_TIMEOUT;
    }
    
    return event_data;
}

static void set_winch_direction(winch_direction_t direction) {
    if (g_ctx.current_direction != direction) {
        g_ctx.current_direction = direction;
        ESP_LOGI(TAG, "Winch direction: %s", direction_to_string(direction));
    }
    
    outputs_set_winch_down(direction == WINCH_DOWN);
    outputs_set_winch_up(direction == WINCH_UP);
}

static uint32_t calculate_state_timeout(const winch_state_context_t *ctx) {
    uint32_t timeout_ms = 0;
    char time_str[64];
    
    // State-specific timeout calculation using defined constants
    switch (ctx->current_state) {
        case WS_MOVING_DOWN:
        case WS_MOVING_UP:
        case WS_STATIC_MOVING_TO_POINT:
            timeout_ms = calculate_movement_timeout(ctx);
            // Don't log here - already logged in calculate_movement_timeout
            return timeout_ms;
            
        case WS_INIT_GOING_HOME:
            // For INIT: different timeouts for down vs up phase
            if (ctx->current_direction == WINCH_DOWN) {
                timeout_ms = RECOVERY_DOWN_TIME_MS; // Short down phase
            } else {
                timeout_ms = GOING_HOME_TIMEOUT_MS; // Long up phase (100 seconds)
            }
            format_time_string(timeout_ms, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "⏱️  INIT %s: %s", direction_to_string(ctx->current_direction), time_str);
            return timeout_ms;
            
        case WS_STATIC_WAIT:
            timeout_ms = (ctx->current_pkg->static_poll_interval_s * ctx->current_pkg->samples * 1000) + STATIC_WAIT_BUFFER_MS;
            format_time_string(timeout_ms, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "⏱️  Static wait: %s", time_str);
            return timeout_ms;
            
        case WS_TENSION_RECOVERY_REVERSE:
            timeout_ms = TENSION_REVERSE_TIME_MS;
            format_time_string(timeout_ms, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "⏱️  Tension recovery: %s", time_str);
            return timeout_ms;
            
        case WS_TENSION_RECOVERY_PAUSE:
            timeout_ms = TENSION_RECOVERY_PAUSE_MS;
            return timeout_ms;
            
        case WS_TIMEOUT_RECOVERY_DOWN:
            timeout_ms = RECOVERY_DOWN_TIME_MS;
            return timeout_ms;
            
        case WS_TIMEOUT_RECOVERY_UP:
            timeout_ms = RECOVERY_UP_TIMEOUT_MS;
            return timeout_ms;
            
        case WS_TIMEOUT_RECOVERY_PAUSE:
            timeout_ms = RECOVERY_PAUSE_TIME_MS;
            return timeout_ms;
            
        default:
            timeout_ms = state_handlers[ctx->current_state].default_timeout_ms;
            format_time_string(timeout_ms, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "⏱️  Default %s: %s", state_name(ctx->current_state), time_str);
            return timeout_ms;
    }
}

static winch_state_t get_next_movement_state(const winch_state_context_t *ctx) {
    ESP_LOGD(TAG, "Determining next movement state for %s", state_to_string(ctx->current_pkg->STATE));
    
    switch (ctx->current_pkg->STATE) {
        case LOWERING:
            return WS_MOVING_DOWN;
        case RISING:
            return WS_MOVING_UP;
        case INIT:
            return WS_INIT_GOING_HOME;
        default:
            ESP_LOGE(TAG, "Unknown STATE: %d", ctx->current_pkg->STATE);
            return WS_ERROR;
    }
}

// ============================================================================
// PUBLIC INTERFACE - EXACTLY THE SAME AS BEFORE
// ============================================================================

esp_err_t winch_controller_init(void) {
    if (g_controller_initialized) {
        return ESP_OK;
    }
    
    // Initialize hardware
    outputs_init();
    
    // Create event queue  
    g_ctx.input_queue = xQueueCreate(10, sizeof(InputEvent));
    if (!g_ctx.input_queue) {
        return ESP_ERR_NO_MEM;
    }
    
    inputs_init(g_ctx.input_queue);
    
    // Initialize state machine - PRESERVE queue handle!
    QueueHandle_t temp_queue = g_ctx.input_queue;  // Save queue handle
    memset(&g_ctx, 0, sizeof(winch_state_context_t));
    g_ctx.input_queue = temp_queue;  // Restore queue handle
    g_ctx.current_state = WS_IDLE;
    g_ctx.current_direction = WINCH_INVALID;
    
    g_controller_initialized = true;
    ESP_LOGI(TAG, "State machine winch controller initialized");
    
    return ESP_OK;
}

esp_err_t winch_controller_deinit(void) {
    if (!g_controller_initialized) {
        return ESP_OK;
    }
    
    g_ctx.operation_active = false;
    set_winch_direction(WINCH_STOP);
    
    // Turn off all lamps safely
    set_operation_lamps(TURN_OFF_LAMPS);
    
    inputs_deinit();
    outputs_deinit();
    
    if (g_ctx.input_queue) {
        vQueueDelete(g_ctx.input_queue);
        g_ctx.input_queue = NULL;
    }
    
    g_controller_initialized = false;
    ESP_LOGI(TAG, "State machine winch controller deinitialized");
    
    return ESP_OK;
}

esp_err_t winch_execute_operation(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    if (!g_controller_initialized || !pkg || !resp) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // SAFETY CHECK: Verify event queue is valid before starting operation
    if (g_ctx.input_queue == NULL) {
        ESP_LOGE(TAG, "💥 CRITICAL: Event queue is NULL! Cannot start operation safely!");
        set_winch_direction(WINCH_STOP);
        if (resp) {
            resp->result = ESP_ERR_INVALID_STATE;
        }
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting operation: %s, %s, end_depth=%d cm, speed=%.3f cm/s", 
             state_to_string(pkg->STATE), poll_to_string(pkg->poll_type),
             pkg->end_depth, pkg->estimated_cm_per_s_x1000 / 1000.0);
    
    // Initialize operation context
    memset(resp, 0, sizeof(motorcontroller_response_t));
    resp->STATE = pkg->STATE;
    resp->estimated_cm_per_s = pkg->estimated_cm_per_s_x1000;  // Use provided estimate (scaled)
    
    g_ctx.current_pkg = pkg;
    g_ctx.current_resp = resp;
    g_ctx.validated_speed_cm_per_s_x1000 = validate_speed_estimate(pkg->estimated_cm_per_s_x1000);
    g_ctx.operation_active = true;
    g_ctx.recovery_attempts = 0;
    g_ctx.current_static_point = 0;
    g_ctx.total_recovery_time_ms = 0;
    g_ctx.total_static_wait_time_ms = 0;
    
    // Reset home sensor protection
    g_ctx.upward_movement_start_time_ms = 0;
    g_ctx.estimated_min_return_time_ms = 0;
    
    timer_start(&g_ctx.operation_timer);
    
    // Setup operation based on mode
    esp_err_t setup_result = setup_operation_for_mode(&g_ctx);
    if (setup_result != ESP_OK) {
        g_ctx.operation_active = false;
        resp->result = setup_result;
        return setup_result;
    }
    
    // Run the state machine
    return run_state_machine();
}

esp_err_t winch_go_to_home_position(void) {
    // SAFETY CHECK: Verify system is properly initialized
    if (!g_controller_initialized || g_ctx.input_queue == NULL) {
        ESP_LOGE(TAG, "💥 CRITICAL: Winch controller not properly initialized! Cannot go home safely!");
        set_winch_direction(WINCH_STOP);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create a simple package for going home
    motorcontroller_pkg_t home_pkg = {
        .STATE = INIT,
        .poll_type = ALPHA_DEPTH,
        .end_depth = 0,
        .estimated_cm_per_s_x1000 = 15000, // 15.000 cm/s (scaled by 1000)
        .rising_timeout_percent = 30
    };
    
    motorcontroller_response_t resp;
    return winch_execute_operation(&home_pkg, &resp);
}

// ============================================================================
// HELPER FUNCTION IMPLEMENTATIONS (STUBS - IMPLEMENT AS NEEDED)
// ============================================================================

static const char* state_name(winch_state_t state) {
    if (state < sizeof(state_handlers)/sizeof(state_handlers[0])) {
        return state_handlers[state].name;
    }
    return "UNKNOWN";
}

static const char* event_name(winch_event_t event) {
    switch (event) {
        case WE_START_OPERATION: return "START_OPERATION";
        case WE_HOME_REACHED: return "HOME_REACHED";
        case WE_TENSION_ALARM: return "TENSION_ALARM";
        case WE_TENSION_CLEARED: return "TENSION_CLEARED";
        case WE_AUTO_DISABLED: return "AUTO_DISABLED";
        case WE_TIMEOUT: return "TIMEOUT";
        case WE_STATIC_POINT_REACHED: return "STATIC_POINT_REACHED";
        case WE_NO_EVENT: return "NO_EVENT";
        default: return "UNKNOWN";
    }
}

static esp_err_t setup_operation_for_mode(winch_state_context_t *ctx) {
    const motorcontroller_pkg_t *pkg = ctx->current_pkg;
    
    ESP_LOGI(TAG, "Setting up operation for poll type: %s", poll_to_string(pkg->poll_type));
    
    if (pkg->STATE == INIT) {
        ESP_LOGI(TAG, "INIT operation: going home from unknown position");
    } else if (pkg->STATE == RISING) {
        ESP_LOGI(TAG, "RISING operation: starting from depth %d cm, going to home (0 cm)", pkg->end_depth);
    } else {
        ESP_LOGI(TAG, "LOWERING operation: starting from home (0 cm), going to depth %d cm", pkg->end_depth);
    }
    
    // Setup static points if needed
    if (pkg->poll_type == STATIC_DEPTH) {
        // Validate and sanitize static points
        ctx->num_points = validate_and_sanitize_static_points(pkg, ctx->sorted_static_points, MAX_POINTS);
        
        if (ctx->num_points == 0) {
            ESP_LOGE(TAG, "STATIC_DEPTH operation but no valid static points after sanitization!");
            return ESP_ERR_INVALID_ARG;
        }
        
        // Sort based on direction
        if (pkg->STATE == RISING) {
            // FIXED: Create temporary reversed array
            uint16_t reversed_points[MAX_POINTS];
            for (int i = 0; i < ctx->num_points; i++) {
                reversed_points[i] = ctx->sorted_static_points[ctx->num_points - 1 - i];
            }
            // Copy reversed points back to original array
            memcpy(ctx->sorted_static_points, reversed_points, ctx->num_points * sizeof(uint16_t));
            
            ESP_LOGI(TAG, "STATIC_DEPTH RISING: %d valid points configured, starting from %d cm", 
                     ctx->num_points, pkg->end_depth);
            
            // Log the path for rising operations
            ESP_LOGI(TAG, "RISING path: %d cm (start)", pkg->end_depth);
            for (int i = 0; i < ctx->num_points; i++) {
                ESP_LOGI(TAG, "  -> %d cm (static point %d)", ctx->sorted_static_points[i], i + 1);
            }
            ESP_LOGI(TAG, "  -> 0 cm (home)");
            
        } else {
            // No reversal needed for lowering (points already in ascending order)
            ESP_LOGI(TAG, "STATIC_DEPTH LOWERING: %d valid points configured, ending at %d cm", 
                     ctx->num_points, pkg->end_depth);
            
            // Log the path for lowering operations
            ESP_LOGI(TAG, "LOWERING path: 0 cm (home)");
            for (int i = 0; i < ctx->num_points; i++) {
                ESP_LOGI(TAG, "  -> %d cm (static point %d)", ctx->sorted_static_points[i], i + 1);
            }
            if (pkg->end_depth > ctx->sorted_static_points[ctx->num_points - 1]) {
                ESP_LOGI(TAG, "  -> %d cm (final depth)", pkg->end_depth);
            }
        }
    }
    
    return ESP_OK;
}

static uint32_t get_working_time_ms(const winch_state_context_t *ctx){
    // Calculate working time: total time minus recovery and static wait time
    uint32_t total_time_ms = timer_ms_since_start(&ctx->operation_timer);
    return total_time_ms - ctx->total_recovery_time_ms - ctx->total_static_wait_time_ms;
}

// Helper to check if we should transition to static wait
static bool should_go_to_static_wait(const winch_state_context_t *ctx) {
    if (ctx->current_pkg->poll_type != STATIC_DEPTH) {
        return false;
    }
    // Check if we've reached the current target static point
    if (ctx->current_static_point < ctx->num_points) {
        uint32_t expected_time = calculate_expected_time_ms(abs(ctx->sorted_static_points[ctx->current_static_point] - 
                                                                ctx->current_target_depth), 
                                                            ctx->validated_speed_cm_per_s_x1000);
        uint32_t elapsed_time = get_working_time_ms(ctx);
        
        if (elapsed_time >= expected_time) {
            ESP_LOGI(TAG, "Reached static point %d/%d (time-based)", 
                     ctx->current_static_point + 1, ctx->num_points);
            return true;
        }
    }
    
    return false;
}

// Calculate timeout for current operation
static uint32_t calculate_movement_timeout(const winch_state_context_t *ctx) {
    const motorcontroller_pkg_t *pkg = ctx->current_pkg;
    uint32_t calculated_timeout = 0; //ms
    char time_str[64];
    
    switch (pkg->poll_type) {
        case ALPHA_DEPTH: {
            // For both RISING and LOWERING, travel distance is always end_depth
            uint16_t travel_distance_cm = pkg->end_depth;
            calculated_timeout = calculate_expected_time_ms(travel_distance_cm, ctx->validated_speed_cm_per_s_x1000);
            
            format_time_string(calculated_timeout, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "🕐 ALPHA_DEPTH: %d cm at %.3f cm/s = %s", 
                     travel_distance_cm, ctx->validated_speed_cm_per_s_x1000 / 1000.0, time_str);
            
            return calculated_timeout;
        }
            
        case LIN_TIME:
            calculated_timeout = pkg->static_poll_interval_s * 1000;
            format_time_string(calculated_timeout, time_str, sizeof(time_str));
            ESP_LOGI(TAG, "🕐 LIN_TIME: %s", time_str);
            return calculated_timeout;
            
        case STATIC_DEPTH:
            if (ctx->current_static_point < ctx->num_points) {
                // Calculate time to next static point
                uint16_t distance_cm;
                
                if (ctx->current_static_point == 0) {
                    if (pkg->STATE == RISING) {
                        distance_cm = abs(pkg->end_depth - ctx->sorted_static_points[0]);
                    } else {
                        distance_cm = ctx->sorted_static_points[0];
                    }
                } else {
                    distance_cm = abs(ctx->sorted_static_points[ctx->current_static_point] - 
                                    ctx->sorted_static_points[ctx->current_static_point - 1]);
                }
                
                calculated_timeout = calculate_expected_time_ms(distance_cm, ctx->validated_speed_cm_per_s_x1000);
                format_time_string(calculated_timeout, time_str, sizeof(time_str));
                ESP_LOGI(TAG, "🕐 STATIC: %d cm segment = %s", distance_cm, time_str);
                return calculated_timeout;
            } else {
                // Moving to final position
                uint16_t distance_cm;
                if (pkg->STATE == RISING) {
                    distance_cm = ctx->sorted_static_points[ctx->num_points - 1];
                } else {
                    uint16_t last_point = ctx->sorted_static_points[ctx->num_points - 1];
                    distance_cm = abs(pkg->end_depth - last_point);
                }
                calculated_timeout = calculate_expected_time_ms(distance_cm, ctx->validated_speed_cm_per_s_x1000);
                format_time_string(calculated_timeout, time_str, sizeof(time_str));
                ESP_LOGI(TAG, "🕐 STATIC final: %d cm = %s", distance_cm, time_str);
                return calculated_timeout;
            }
            
        default:
            format_time_string(DEFAULT_MOVING_TIMEOUT_MS, time_str, sizeof(time_str));
            ESP_LOGW(TAG, "🕐 Default timeout: %s", time_str);
            return DEFAULT_MOVING_TIMEOUT_MS;
    }
}

// Time calculation with clear units and scaling
static uint32_t calculate_expected_time_ms(const uint16_t distance_cm, const uint16_t speed_cm_per_s_x1000) {
    if (speed_cm_per_s_x1000 == 0) {
        ESP_LOGW(TAG, "⚠️  Speed is 0, using default timeout");
        return DEFAULT_MOVING_TIMEOUT_MS;
    }
    
    // Calculate time with proper unit conversion:
    // distance_cm [cm] / (speed_cm_per_s_x1000 [cm/s * 1000] / 1000) = time [s]
    // Rearranged: distance_cm * 1000 / speed_cm_per_s_x1000 = time [s]
    // Convert to ms: time [s] * 1000 = time [ms]
    // Final formula: distance_cm * 1000 * 1000 / speed_cm_per_s_x1000 = time [ms]
    uint32_t time_ms = (distance_cm * 1000UL * 1000UL) / speed_cm_per_s_x1000;
    
    // Add compensation for motor start/stop delays
    uint32_t total_time_ms = time_ms + SOFT_START_STOP_COMPENSATION_MS;    
    return total_time_ms;
}

// ============================================================================
// TIME FORMATTING HELPER FUNCTION
// ============================================================================

static void format_time_string(uint32_t time_ms, char *buffer, size_t buffer_size) {
    uint32_t total_seconds = time_ms / 1000;
    uint32_t hours         = total_seconds / 3600;
    uint32_t minutes       = (total_seconds % 3600) / 60;
    uint32_t seconds       = total_seconds % 60;
    uint32_t milliseconds  = time_ms % 1000;
         
    if (hours > 0) {
        snprintf(buffer, buffer_size,
            "%02lu:%02lu:%02lu.%03lu (%lu.%03lus)",
            (unsigned long)hours,
            (unsigned long)minutes,
            (unsigned long)seconds,
            (unsigned long)milliseconds,
            (unsigned long)total_seconds,
            (unsigned long)milliseconds);
    } else if (minutes > 0) {
        snprintf(buffer, buffer_size,
            "%02lu:%02lu.%03lu (%lu.%03lus)",
            (unsigned long)minutes,
            (unsigned long)seconds,
            (unsigned long)milliseconds,
            (unsigned long)total_seconds,
            (unsigned long)milliseconds);
    } else {
        snprintf(buffer, buffer_size,
            "%02lu.%03lus (%lums)",
            (unsigned long)seconds,
            (unsigned long)milliseconds,
            (unsigned long)time_ms);
    }
}

// ============================================================================
// STATIC POINT VALIDATION AND SANITIZATION
// ============================================================================

static uint8_t validate_and_sanitize_static_points(const motorcontroller_pkg_t *pkg, 
                                                   uint16_t *sanitized_points, 
                                                   uint8_t max_points) {
    uint8_t valid_count = 0;
    uint8_t removed_count = 0;
    
    ESP_LOGI(TAG, "🔍 Validating static points for %s operation (end_depth=%d cm)...", 
             state_to_string(pkg->STATE), pkg->end_depth);
    
    // Process each point in the input array
    for (int i = 0; i < MAX_POINTS && pkg->static_points[i] != 0 && valid_count < max_points; i++) {
        uint16_t point = pkg->static_points[i];
        bool keep_point = true;
        const char* reason = "";
        
        // 1. Safety check: Convert anything less than MIN_SAFE_DEPTH_CM to MIN_SAFE_DEPTH_CM
        if (point < MIN_SAFE_DEPTH_CM) {
            ESP_LOGW(TAG, "⚠️  Point %d cm below minimum safe depth, adjusting to %d cm", 
                     point, MIN_SAFE_DEPTH_CM);
            point = MIN_SAFE_DEPTH_CM;
        }
        
        // 2. Remove points above MAXIMUM_DEPTH
        if (point > MAXIMUM_DEPTH) {
            keep_point = false;
            reason = "exceeds MAXIMUM_DEPTH";
            removed_count++;
        }
        
        // 3. Remove points above end_depth
        else if (point > pkg->end_depth) {
            keep_point = false;
            reason = (pkg->STATE == RISING) ? "deeper than starting depth" : "deeper than target depth";
            removed_count++;
        }
        
        // 4. Check for duplicates
        else {
            for (int j = 0; j < valid_count; j++) {
                if (sanitized_points[j] == point) {
                    keep_point = false;
                    reason = "duplicate point";
                    removed_count++;
                    break;
                }
            }
        }
        
        if (keep_point) {
            sanitized_points[valid_count] = point;
            valid_count++;
            ESP_LOGD(TAG, "✅ Point %d: %d cm (valid)", i + 1, point);
        } else {
            ESP_LOGW(TAG, "❌ Point %d: %d cm (removed - %s)", i + 1, pkg->static_points[i], reason);
        }
    }
    
    // 5. Verify ordering (should be in ascending order for lowering operations)
    bool correctly_ordered = true;
    for (int i = 1; i < valid_count; i++) {
        if (sanitized_points[i] <= sanitized_points[i - 1]) {
            correctly_ordered = false;
            ESP_LOGW(TAG, "⚠️  Point ordering issue: %d cm <= %d cm at positions %d,%d", 
                     sanitized_points[i], sanitized_points[i - 1], i, i - 1);
        }
    }
    
    if (!correctly_ordered) {
        ESP_LOGW(TAG, "⚠️  Points not in ascending order - master should send ordered points!");
        // Could sort here, but better to warn and let master fix it
    }
    
    // Summary
    if (removed_count > 0) {
        ESP_LOGW(TAG, "🧹 Sanitization complete: %d valid points, %d removed", valid_count, removed_count);
    } else {
        ESP_LOGI(TAG, "✅ All %d static points are valid", valid_count);
    }
    
    // Log final sanitized points
    if (valid_count > 0) {
        ESP_LOGI(TAG, "📍 Final static points:");
        for (int i = 0; i < valid_count; i++) {
            ESP_LOGI(TAG, "   [%d] %d cm", i, sanitized_points[i]);
        }
    }
    
    return valid_count;
}

// ============================================================================
// DEBUG HELPER FUNCTION IMPLEMENTATIONS
// ============================================================================

static const char* direction_to_string(winch_direction_t direction) {
    switch (direction) {
        case WINCH_STOP: return "STOP";
        case WINCH_UP: return "UP";
        case WINCH_DOWN: return "DOWN";
        case WINCH_INVALID: return "INVALID";
        default: return "UNKNOWN_DIRECTION";
    }
}

// ============================================================================
// HOME SENSOR PROTECTION FUNCTIONS
// ============================================================================

static void track_upward_movement_start(winch_state_context_t *ctx) {
    ctx->upward_movement_start_time_ms = get_working_time_ms(ctx) - timer_ms_since_start(&ctx->operation_timer); // zero if no recowery happend
    ESP_LOGD(TAG, "Tracking upward movement start at %d ms", ctx->upward_movement_start_time_ms);
}

static void calculate_minimum_return_time_from_distance(winch_state_context_t *ctx) {
    // Calculate minimum time to return home based on distance and speed
    const motorcontroller_pkg_t *pkg = ctx->current_pkg;
    
    uint16_t travel_distance_cm = 0;
    
    if (pkg->STATE == RISING && pkg->end_depth > 0 && ctx->validated_speed_cm_per_s_x1000 > 0) {
        // For RISING: end_depth is starting depth, distance is end_depth -> 0 (home)
        travel_distance_cm = pkg->end_depth;
        uint32_t calculated_time_ms = calculate_expected_time_ms(travel_distance_cm, ctx->validated_speed_cm_per_s_x1000);
        uint16_t depth = pkg->end_depth;
        if (g_ctx.current_pkg->poll_type == LIN_TIME){
            calculated_time_ms = g_ctx.current_pkg->static_poll_interval_s*1000;
            depth = calculated_time_ms*ctx->validated_speed_cm_per_s_x1000/1000;
        } 
        // Conservative estimate: assume upward movement takes 20% longer than calculated
        // due to load or different motor characteristics
        ctx->estimated_min_return_time_ms = (calculated_time_ms * MIN_TRAVEL_PERCENT_BEFOR_HOME_SENSOR_IS_ACTIVE) / 100; 
        
        char time_str[64];
        format_time_string(ctx->estimated_min_return_time_ms, time_str, sizeof(time_str));
        ESP_LOGI(TAG, "🛡️  Home sensor protection: %s (from %d cm at %.3f cm/s)", 
                 time_str, depth, ctx->validated_speed_cm_per_s_x1000 / 1000.0);
    } else {
        // No distance or speed info, use default minimum
        ctx->estimated_min_return_time_ms = 0;
        ESP_LOGD(TAG, "No distance/speed info or not rising operation, using default minimum return time");
    }
}

// Speed validation and sanity checking
static uint16_t validate_speed_estimate(uint16_t estimated_cm_per_s_x1000) {
    if (estimated_cm_per_s_x1000 == 0) {
        ESP_LOGW(TAG, "Speed estimate is 0, using default 15.000 cm/s");
        return 15000; // 15.000 cm/s (scaled by 1000)
    }
    
    if (estimated_cm_per_s_x1000 > MAXIMUM_SPEED) {
        ESP_LOGW(TAG, "Speed estimate %.3f cm/s exceeds maximum %.3f cm/s, capping", 
                 estimated_cm_per_s_x1000 / 1000.0, MAXIMUM_SPEED / 1000.0);
        return MAXIMUM_SPEED;
    }
    
    if (estimated_cm_per_s_x1000 < MIN_REASONABLE_SPEED_CM_S_x1000) {
        ESP_LOGW(TAG, "Speed estimate %.3f cm/s below minimum reasonable %.3f cm/s, using minimum", 
                 estimated_cm_per_s_x1000 / 1000.0, MIN_REASONABLE_SPEED_CM_S_x1000 / 1000.0);
        return MIN_REASONABLE_SPEED_CM_S_x1000;
    }
    
    ESP_LOGI(TAG, "Using validated speed estimate: %d (%.3f cm/s)", 
             estimated_cm_per_s_x1000, estimated_cm_per_s_x1000 / 1000.0);
    return estimated_cm_per_s_x1000;
}