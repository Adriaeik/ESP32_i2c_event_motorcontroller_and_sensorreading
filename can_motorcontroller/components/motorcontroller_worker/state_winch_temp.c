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

// Winch direction control
typedef enum {
    WINCH_STOP = 0,
    WINCH_UP,
    WINCH_DOWN,
    WINCH_INVALID = 255  // Used to force first direction change
} winch_direction_t;

// State context - contains all the operation data
typedef struct {
    // Current state machine state
    winch_state_t current_state;
    winch_state_t previous_state;
    
    // Operation parameters (same as before)
    const motorcontroller_pkg_t *current_pkg;
    motorcontroller_response_t *current_resp;
    uint16_t updated_speed_estimate;
    
    // Timers
    w_timer_t operation_timer;
    w_timer_t state_timer;              // Timer for current state
    w_timer_t recovery_timer;           // Separate timer for recovery tracking
    
    uint32_t total_recovery_time_ms;
    uint32_t total_static_wait_time_ms;
    
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
    QueueHandle_t event_queue;
    
    // Current direction (for logging state changes)
    winch_direction_t current_direction;
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

static winch_state_t tension_recovery_reverse_enter(winch_state_context_t *ctx);
static winch_state_t tension_recovery_reverse_event(winch_state_context_t *ctx, winch_event_t event);

static winch_state_t timeout_recovery_down_enter(winch_state_context_t *ctx);
static winch_state_t timeout_recovery_down_event(winch_state_context_t *ctx, winch_event_t event);

// Helper functions
static winch_event_data_t wait_for_event_with_timeout(uint32_t timeout_ms);
static void set_winch_direction(winch_direction_t direction);
static uint32_t calculate_state_timeout(winch_state_context_t *ctx);
static esp_err_t setup_operation_for_mode(winch_state_context_t *ctx);
static winch_state_t get_next_movement_state(winch_state_context_t *ctx);
static const char* state_name(winch_state_t state);
static const char* event_name(winch_event_t event);

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
        .default_timeout_ms = 10000
    },
    [WS_MOVING_UP] = {
        .name = "MOVING_UP",
        .on_enter = moving_up_enter, 
        .on_event = moving_up_event,
        .on_exit = moving_up_exit,
        .default_timeout_ms = 30000
    },
    [WS_STATIC_WAIT] = {
        .name = "STATIC_WAIT",
        .on_enter = static_wait_enter,
        .on_event = static_wait_event, 
        .on_exit = static_wait_exit,
        .default_timeout_ms = 10000
    },
    [WS_TENSION_RECOVERY_REVERSE] = {
        .name = "TENSION_RECOVERY_REVERSE",
        .on_enter = tension_recovery_reverse_enter,
        .on_event = tension_recovery_reverse_event,
        .on_exit = NULL,
        .default_timeout_ms = 1000
    },
    [WS_TIMEOUT_RECOVERY_DOWN] = {
        .name = "TIMEOUT_RECOVERY_DOWN", 
        .on_enter = timeout_recovery_down_enter,
        .on_event = timeout_recovery_down_event,
        .on_exit = NULL,
        .default_timeout_ms = 2000
    },
    // ... add other states
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
    
    // Exit current state
    if (state_handlers[g_ctx.current_state].on_exit) {
        state_handlers[g_ctx.current_state].on_exit(&g_ctx);
    }
    
    // Update state
    g_ctx.previous_state = g_ctx.current_state;
    g_ctx.current_state = new_state;
    
    // Enter new state
    if (state_handlers[new_state].on_enter) {
        winch_state_t next_state = state_handlers[new_state].on_enter(&g_ctx);
        if (next_state != new_state) {
            // Immediate transition requested
            return transition_to_state(next_state);
        }
    }
    
    // Start state timer
    timer_start(&g_ctx.state_timer);
    return new_state;
}

static winch_state_t process_event(winch_event_t event) {
    ESP_LOGI(TAG, "Processing event %s in state %s", 
             event_name(event), state_name(g_ctx.current_state));
    
    if (state_handlers[g_ctx.current_state].on_event) {
        winch_state_t new_state = state_handlers[g_ctx.current_state].on_event(&g_ctx, event);
        return transition_to_state(new_state);
    }
    
    return g_ctx.current_state; // No handler, stay in current state
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
        winch_event_data_t event_data = wait_for_event_with_timeout(timeout);
        
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
    
    // Fill response
    uint32_t total_time = timer_ms_since_start(&g_ctx.operation_timer);
    uint32_t working_time = total_time - g_ctx.total_recovery_time_ms - g_ctx.total_static_wait_time_ms;
    g_ctx.current_resp->working_time = working_time / 1000;
    g_ctx.current_resp->estimated_cm_per_s = g_ctx.updated_speed_estimate;
    
    ESP_LOGI(TAG, "State machine completed - Result: %s, Working time: %ds",
             esp_err_to_name(g_ctx.current_resp->result), g_ctx.current_resp->working_time);
    
    return g_ctx.current_resp->result;
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
            return WS_COMPLETE;
            
        case WE_TENSION_ALARM:
            return WS_TENSION_RECOVERY_REVERSE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Check if we should go to static wait or complete
            if (should_go_to_static_wait(ctx)) {
                return WS_STATIC_WAIT;
            }
            // For lowering operations, timeout means we're done
            if (ctx->current_pkg->STATE == LOWERING) {
                return WS_COMPLETE;
            }
            return WS_ERROR;
            
        default:
            return WS_MOVING_DOWN;
    }
}

static void moving_down_exit(winch_state_context_t *ctx) {
    // Any cleanup when exiting moving down state
}

static winch_state_t moving_up_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_UP);
    return WS_MOVING_UP;
}

static winch_state_t moving_up_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            return WS_COMPLETE;
            
        case WE_TENSION_ALARM:
            return WS_TENSION_RECOVERY_REVERSE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TIMEOUT:
            // Check if we should go to static wait first
            if (should_go_to_static_wait(ctx)) {
                return WS_STATIC_WAIT;
            }
            // For rising operations, timeout triggers recovery
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
    timer_start(&ctx->state_timer);
    
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
    ctx->total_static_wait_time_ms += timer_ms_since_start(&ctx->state_timer);
}

static winch_state_t tension_recovery_reverse_enter(winch_state_context_t *ctx) {
    timer_start(&ctx->recovery_timer);
    
    // Reverse direction
    winch_direction_t reverse_dir = (ctx->current_direction == WINCH_DOWN) ? WINCH_UP : WINCH_DOWN;
    set_winch_direction(reverse_dir);
    
    ESP_LOGW(TAG, "Tension recovery - reversing direction");
    return WS_TENSION_RECOVERY_REVERSE;
}

static winch_state_t tension_recovery_reverse_event(winch_state_context_t *ctx, winch_event_t event) {
    switch (event) {
        case WE_HOME_REACHED:
            return WS_COMPLETE;
            
        case WE_AUTO_DISABLED:
            return WS_ERROR;
            
        case WE_TENSION_CLEARED:
            // Resume original operation
            ctx->total_recovery_time_ms += timer_ms_since_start(&ctx->recovery_timer);
            return get_next_movement_state(ctx);
            
        case WE_TIMEOUT:
            // Recovery time elapsed, check if tension is still there
            if (inputs_get_winch_tension()) {
                return WS_ERROR; // Still have tension
            } else {
                ctx->total_recovery_time_ms += timer_ms_since_start(&ctx->recovery_timer);
                return get_next_movement_state(ctx);
            }
            
        default:
            return WS_TENSION_RECOVERY_REVERSE;
    }
}

static winch_state_t timeout_recovery_down_enter(winch_state_context_t *ctx) {
    ctx->recovery_attempts++;
    if (ctx->recovery_attempts > 3) {
        return WS_ERROR;
    }
    
    timer_start(&ctx->recovery_timer);
    set_winch_direction(WINCH_DOWN);
    
    ESP_LOGW(TAG, "Timeout recovery - going down (attempt %d/3)", ctx->recovery_attempts);
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

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static winch_event_data_t wait_for_event_with_timeout(uint32_t timeout_ms) {
    InputEvent input_event;
    winch_event_data_t event_data = {
        .event = WE_NO_EVENT,
        .timestamp_ms = timer_ms_since_start(&g_ctx.operation_timer),
        .edge_detected = false
    };
    
    if (xQueueReceive(g_ctx.event_queue, &input_event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        // Convert input event to winch event
        switch (input_event.event_type) {
            case HOME:
                if (input_event.state) {
                    event_data.event = WE_HOME_REACHED;
                    event_data.edge_detected = input_event.edge_detected;
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
        ESP_LOGI(TAG, "Winch direction: %s", 
                 direction == WINCH_STOP ? "STOP" :
                 direction == WINCH_UP ? "UP" : "DOWN");
    }
    
    outputs_set_winch_down(direction == WINCH_DOWN);
    outputs_set_winch_up(direction == WINCH_UP);
}

static uint32_t calculate_state_timeout(winch_state_context_t *ctx) {
    // State-specific timeout calculation
    switch (ctx->current_state) {
        case WS_MOVING_DOWN:
        case WS_MOVING_UP:
            return calculate_movement_timeout(ctx);
            
        case WS_STATIC_WAIT:
            return (ctx->current_pkg->static_poll_interval_s * ctx->current_pkg->samples * 1000) + 1000;
            
        case WS_TENSION_RECOVERY_REVERSE:
            return 1000; // 1 second reverse
            
        case WS_TIMEOUT_RECOVERY_DOWN:
            return 2000; // 2 seconds down
            
        case WS_TIMEOUT_RECOVERY_UP:
            return 8000; // 8 seconds up
            
        default:
            return state_handlers[ctx->current_state].default_timeout_ms;
    }
}

static winch_state_t get_next_movement_state(winch_state_context_t *ctx) {
    switch (ctx->current_pkg->STATE) {
        case LOWERING:
            return WS_MOVING_DOWN;
        case RISING:
        case INIT:
            return WS_MOVING_UP;
        default:
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
    
    memset(&g_ctx, 0, sizeof(winch_state_context_t));
    // Create event queue  
    g_ctx.event_queue = xQueueCreate(10, sizeof(InputEvent));
    if (!g_ctx.event_queue) {
        return ESP_ERR_NO_MEM;
    }
    
    inputs_init(g_ctx.event_queue);
    
    // Initialize state machine
    g_ctx.event_queue = g_ctx.event_queue; // Preserve queue handle
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
    
    inputs_deinit();
    outputs_deinit();
    
    if (g_ctx.event_queue) {
        vQueueDelete(g_ctx.event_queue);
        g_ctx.event_queue = NULL;
    }
    
    g_controller_initialized = false;
    ESP_LOGI(TAG, "State machine winch controller deinitialized");
    
    return ESP_OK;
}

esp_err_t winch_execute_operation(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    if (!g_controller_initialized || !pkg || !resp) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize operation context
    memset(resp, 0, sizeof(motorcontroller_response_t));
    resp->STATE = pkg->STATE;
    resp->estimated_cm_per_s = pkg->prev_estimated_cm_per_s;
    
    g_ctx.current_pkg = pkg;
    g_ctx.current_resp = resp;
    g_ctx.updated_speed_estimate = pkg->prev_estimated_cm_per_s;
    g_ctx.operation_active = true;
    g_ctx.recovery_attempts = 0;
    g_ctx.current_static_point = 0;
    g_ctx.total_recovery_time_ms = 0;
    g_ctx.total_static_wait_time_ms = 0;
    
    timer_start(&g_ctx.operation_timer);
    
    // Setup operation based on mode (same logic as before)
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
    // Create a simple package for going home
    motorcontroller_pkg_t home_pkg = {
        .STATE = INIT,
        .poll_type = ALPHA_DEPTH,
        .end_depth = 0,
        .prev_estimated_cm_per_s = 15000, // 15 cm/s default
        .rising_timeout_percent = 30
    };
    
    motorcontroller_response_t resp;
    return winch_execute_operation(&home_pkg, &resp);
}

// Additional state handlers for timeout recovery
static winch_state_t timeout_recovery_up_enter(winch_state_context_t *ctx) {
    set_winch_direction(WINCH_UP);
    ESP_LOGI(TAG, "Timeout recovery - going up");
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
            if (ctx->recovery_attempts >= 3) { //macic number use define
                ESP_LOGE(TAG, "Max recovery attempts reached");
                return WS_ERROR;
            }
            return WS_TIMEOUT_RECOVERY_DOWN; // Try again
            
        default:
            return WS_TIMEOUT_RECOVERY_UP;
    }
}

// Complete state handler table entry
static void complete_state_handlers(void) {
    // Add the missing handlers to the state_handlers array
    state_handlers[WS_TIMEOUT_RECOVERY_UP] = (state_handler_t){
        .name = "TIMEOUT_RECOVERY_UP",
        .on_enter = timeout_recovery_up_enter,
        .on_event = timeout_recovery_up_event,
        .on_exit = NULL,
        .default_timeout_ms = 8000
    };
}

// ============================================================================
// HELPER FUNCTION IMPLEMENTATIONS
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
    
    // Setup static points if needed (same logic as your original)
    if (pkg->poll_type == STATIC_DEPTH) {
        // Copy your existing static point setup logic here
        ctx->num_points = 0;
        for (int i = 0; i < MAX_POINTS && pkg->static_points[i] != 0; i++) {
            ctx->sorted_static_points[ctx->num_points] = pkg->static_points[i];
            ctx->num_points++;
        }
        
        // Sort based on direction (same as your original)
        if (pkg->STATE == RISING) {
            // Reverse for rising
            for (int i = 0; i < ctx->num_points / 2; i++) {
                uint16_t temp = ctx->sorted_static_points[i];
                ctx->sorted_static_points[i] = ctx->sorted_static_points[ctx->num_points - 1 - i];
                ctx->sorted_static_points[ctx->num_points - 1 - i] = temp;
            }
        }
        
        ESP_LOGI(TAG, "STATIC_DEPTH: %d points configured for %s", 
                 ctx->num_points, pkg->STATE == RISING ? "RISING" : "LOWERING");
    }
    
    return ESP_OK;
}

// Helper to check if we should transition to static wait
static bool should_go_to_static_wait(winch_state_context_t *ctx) {
    if (ctx->current_pkg->poll_type != STATIC_DEPTH) {
        return false;
    }
    
    // Check if we've reached the current target static point
    // TODO: In a real implementation, you could add more sophisticated detection:
    // - Use depth sensors if available
    // - Use encoder feedback from motor
    // - Use more precise time estimation
    // - Combine multiple indicators
    
    if (ctx->current_static_point < ctx->num_points) {
        // For now, we'll use timeout as indication we reached the point
        // This is the same logic as your original implementation
        uint32_t expected_time = calculate_movement_timeout(ctx);
        uint32_t elapsed_time = timer_ms_since_start(&ctx->state_timer);
        
        if (elapsed_time >= expected_time) {
            ESP_LOGI(TAG, "Reached static point %d/%d (time-based)", 
                     ctx->current_static_point + 1, ctx->num_points);
            return true;
        }
    }
    
    return false;
}

// Calculate timeout for current operation
static uint32_t calculate_movement_timeout(winch_state_context_t *ctx) {
    const motorcontroller_pkg_t *pkg = ctx->current_pkg;
    
    switch (pkg->poll_type) {
        case ALPHA_DEPTH:
            return calculate_expected_time_ms(pkg->end_depth, ctx->updated_speed_estimate);
            
        case LIN_TIME:
            return pkg->static_poll_interval_s * 1000;
            
        case STATIC_DEPTH:
            if (ctx->current_static_point < ctx->num_points) {
                // Calculate time to next static point
                uint16_t distance;
                if (ctx->current_static_point == 0) {
                    distance = ctx->sorted_static_points[0];
                } else {
                    distance = abs(ctx->sorted_static_points[ctx->current_static_point] - 
                                 ctx->sorted_static_points[ctx->current_static_point - 1]);
                }
                return calculate_expected_time_ms(distance, ctx->updated_speed_estimate);
            } else {
                // Moving to final position (home for rising)
                return calculate_expected_time_ms(pkg->end_depth, ctx->updated_speed_estimate);
            }
            
        default:
            return 10000; // 10 second default
    }
}