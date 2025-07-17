#pragma once
#include "esp_err.h"
#include "buoye_structs.h"
#include "timer.h"

typedef enum {
    WINCH_STATE_IDLE,
    WINCH_STATE_GOING_HOME,     // INIT mode
    WINCH_STATE_LOWERING,       // LOWERING mode  
    WINCH_STATE_STATIC_SAMPLING,// Static polling pause
    WINCH_STATE_RISING,         // RISING mode
    WINCH_STATE_COMPLETED,
    WINCH_STATE_ERROR
} winch_state_t;

typedef struct {
    winch_state_t state;
    w_timer_t operation_timer;
    w_timer_t movement_timer;       // Tracks pure movement time
    w_timer_t static_timer;         // Tracks static sampling time
    uint16_t current_static_index;
    uint16_t current_sample_count;
    uint32_t pure_movement_time_ms;
    bool operation_active;
} winch_context_t;

typedef struct {
    bool auto_enabled;
    bool tension_ok;
    bool at_home;
    bool home_edge;        // Edge detection for home sensor
    bool tension_edge;     // Edge detection for tension sensor
} winch_event_t;

typedef struct {
    w_timer_t recovery_timer;
    uint8_t retry_count;
    uint8_t max_retries;
    bool in_recovery;
    bool max_retries_logged;  // Prevent spam logging
} RecoveryContext;

typedef struct {
    w_timer_t reverse_timer;
    winch_state_t original_state;    // State we were in when tension occurred
    bool original_direction_up;        // true = up, false = down
    bool tension_retry_attempted;      // Have we tried recovery for this operation?
    bool in_tension_recovery;
} TensionRecoveryContext;

// Winch control functions
esp_err_t winch_controller_init(void);
esp_err_t winch_execute_operation(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp);
esp_err_t winch_go_to_home_position(void);
const char* winch_state_to_string(winch_state_t state);