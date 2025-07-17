#include "winch_controller.h"
#include "motorcontroller_worker.h"
#include "can_motctrl_common.h"
#include "inputs.h"
#include "outputs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"

static const char *TAG = "winch_controller";

// Event types for winch controller
typedef enum {
    WINCH_EVENT_INPUT_CHANGE,
    WINCH_EVENT_w_timer_tICK,
    WINCH_EVENT_STATE_TIMEOUT
} WinchEventType;

typedef struct {
    uint8_t input_index;
    bool new_state;
    bool edge_detected;
} InputEvent;

typedef struct {
    WinchEventType type;
    union {
        InputEvent input;
        uint32_t timer_ms;
    } data;
} WinchMessage;

// Winch controller context
typedef struct {
    winch_context_t context;
    QueueHandle_t event_queue;
    TaskHandle_t w_timer_task_handle;
    EventGroupHandle_t operation_event_group;
    
    // Current operation parameters
    const motorcontroller_pkg_t *current_pkg;
    motorcontroller_response_t *current_resp;
    
    // Error handling
    RecoveryContext recovery;
    TensionRecoveryContext tension_recovery;
    uint32_t error_mask;
    
    // Operation tracking
    uint32_t operation_start_time_ms;
    uint32_t last_movement_start_ms;
    uint32_t accumulated_static_time_ms;
} WinchController;

// Event group bits
#define WINCH_OPERATION_COMPLETED_BIT   (1 << 0)
#define WINCH_OPERATION_ERROR_BIT       (1 << 1)
#define WINCH_OPERATION_TIMEOUT_BIT     (1 << 2)

// Error flags
#define WINCH_ERROR_NONE        0
#define WINCH_ERROR_TENSION     (1 << 0)
#define WINCH_ERROR_TIMEOUT     (1 << 1)
#define WINCH_ERROR_AUTO_LOST   (1 << 2)

static WinchController s_winch_ctrl = {0};

// Forward declarations
static void winch_w_timer_task(void *arg);
static void winch_process_state_machine(winch_event_t *event);
static void winch_set_error(uint32_t error);
static void winch_clear_error(uint32_t error);
static winch_event_t winch_build_event(void);
static esp_err_t winch_wait_for_completion(uint32_t timeout_ms);

// External function from inputs.c
extern void inputs_set_controller_queue(QueueHandle_t queue);

const char* winch_state_to_string(winch_state_t state) {
    switch (state) {
        case WINCH_STATE_IDLE:           return "IDLE";
        case WINCH_STATE_GOING_HOME:     return "GOING_HOME";
        case WINCH_STATE_LOWERING:       return "LOWERING";
        case WINCH_STATE_STATIC_SAMPLING: return "STATIC_SAMPLING";
        case WINCH_STATE_RISING:         return "RISING";
        case WINCH_STATE_COMPLETED:      return "COMPLETED";
        case WINCH_STATE_ERROR:          return "ERROR";
        default:                         return "UNKNOWN";
    }
}

esp_err_t winch_controller_init(void) {
    ESP_LOGI(TAG, "Initializing winch controller");
    
    // Initialize context
    memset(&s_winch_ctrl, 0, sizeof(s_winch_ctrl));
    s_winch_ctrl.context.state = WINCH_STATE_IDLE;
    
    // Create event queue
    s_winch_ctrl.event_queue = xQueueCreate(20, sizeof(WinchMessage));
    if (s_winch_ctrl.event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create operation event group
    s_winch_ctrl.operation_event_group = xEventGroupCreate();
    if (s_winch_ctrl.operation_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create operation event group");
        vQueueDelete(s_winch_ctrl.event_queue);
        return ESP_ERR_NO_MEM;
    }
    
    // Set up input notifications (using your existing inputs.c)
    inputs_set_controller_queue(s_winch_ctrl.event_queue);
    
    ESP_LOGI(TAG, "Winch controller initialized");
    return ESP_OK;
}

esp_err_t winch_execute_operation(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp) {
    ESP_LOGI(TAG, "Starting winch operation - State: %s", get_state_string(pkg->STATE));
    
    // Store operation parameters
    s_winch_ctrl.current_pkg = pkg;
    s_winch_ctrl.current_resp = resp;
    
    // Reset context for new operation
    s_winch_ctrl.context.operation_active = true;
    s_winch_ctrl.context.current_static_index = 0;
    s_winch_ctrl.context.current_sample_count = 0;
    s_winch_ctrl.context.pure_movement_time_ms = 0;
    s_winch_ctrl.accumulated_static_time_ms = 0;
    s_winch_ctrl.error_mask = WINCH_ERROR_NONE;
    
    // Set initial state
    s_winch_ctrl.context.state = (pkg->STATE == LOWERING) ? WINCH_STATE_LOWERING : WINCH_STATE_RISING;
    
    // Start timers
    timer_start(&s_winch_ctrl.context.operation_timer);
    timer_start(&s_winch_ctrl.context.movement_timer);
    s_winch_ctrl.operation_start_time_ms = esp_timer_get_time() / 1000;
    s_winch_ctrl.last_movement_start_ms = s_winch_ctrl.operation_start_time_ms;
    
    // Clear event group
    xEventGroupClearBits(s_winch_ctrl.operation_event_group, 0xFF);
    
    // Create timer task for periodic checks
    xTaskCreate(winch_w_timer_task, "winch_timer", 2048, NULL, 4, &s_winch_ctrl.w_timer_task_handle);
    
    // Calculate operation timeout
    uint32_t timeout_ms = calculate_operation_timeout(
        pkg->STATE, pkg->prev_estimated_cm_per_s, pkg->rising_timeout_percent,
        pkg->end_depth, pkg->static_points, pkg->samples, pkg->static_poll_interval_s
    ) * 1000;
    
    ESP_LOGI(TAG, "Operation timeout: %d ms", timeout_ms);
    
    // Wait for operation completion
    esp_err_t result = winch_wait_for_completion(timeout_ms);
    
    // Clean up timer task
    if (s_winch_ctrl.w_timer_task_handle != NULL) {
        vTaskDelete(s_winch_ctrl.w_timer_task_handle);
        s_winch_ctrl.w_timer_task_handle = NULL;
    }
    
    // Calculate final results
    uint32_t total_time_ms = esp_timer_get_time() / 1000 - s_winch_ctrl.operation_start_time_ms;
    uint32_t pure_movement_time_ms = total_time_ms - s_winch_ctrl.accumulated_static_time_ms;
    
    // Fill response
    resp->working_time = total_time_ms / 1000; // Convert to seconds
    resp->estimated_cm_per_s = calculate_new_speed_estimate(pkg->end_depth, pure_movement_time_ms);
    
    s_winch_ctrl.context.operation_active = false;
    
    ESP_LOGI(TAG, "Operation completed - Result: %s, Time: %d s, Speed: %d cm/s", 
             esp_err_to_name(result), resp->working_time, resp->estimated_cm_per_s);
    
    return result;
}

esp_err_t winch_go_to_home_position(void) {
    ESP_LOGI(TAG, "Going to home position");
    
    // Simple implementation for INIT mode
    s_winch_ctrl.context.state = WINCH_STATE_GOING_HOME;
    s_winch_ctrl.context.operation_active = true;
    
    // Clear event group
    xEventGroupClearBits(s_winch_ctrl.operation_event_group, 0xFF);
    
    // Create timer task
    xTaskCreate(winch_w_timer_task, "winch_timer", 2048, NULL, 4, &s_winch_ctrl.w_timer_task_handle);
    
    // Wait for completion with 60 second timeout
    esp_err_t result = winch_wait_for_completion(60000);
    
    // Clean up
    if (s_winch_ctrl.w_timer_task_handle != NULL) {
        vTaskDelete(s_winch_ctrl.w_timer_task_handle);
        s_winch_ctrl.w_timer_task_handle = NULL;
    }
    
    s_winch_ctrl.context.operation_active = false;
    
    return result;
}

static esp_err_t winch_wait_for_completion(uint32_t timeout_ms) {
    EventBits_t bits = xEventGroupWaitBits(
        s_winch_ctrl.operation_event_group,
        WINCH_OPERATION_COMPLETED_BIT | WINCH_OPERATION_ERROR_BIT | WINCH_OPERATION_TIMEOUT_BIT,
        pdFALSE, // clearOnExit = false
        pdFALSE, // wait for ANY bit
        pdMS_TO_TICKS(timeout_ms)
    );
    
    if (bits & WINCH_OPERATION_COMPLETED_BIT) {
        return ESP_OK;
    } else if (bits & WINCH_OPERATION_ERROR_BIT) {
        return ESP_FAIL;
    } else if (bits & WINCH_OPERATION_TIMEOUT_BIT) {
        return ESP_ERR_TIMEOUT;
    } else {
        return ESP_ERR_TIMEOUT; // Overall timeout
    }
}

// switch to a beter name
static void winch_w_timer_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); // 50ms periodic timer
    
    while (s_winch_ctrl.context.operation_active) {
        WinchMessage msg = {
            .type = WINCH_EVENT_w_timer_tICK,
            .data.timer_ms = 50
        };
        
        // Send timer event (non-blocking)
        if (s_winch_ctrl.event_queue != NULL) {
            xQueueSend(s_winch_ctrl.event_queue, &msg, 0);
        }
        
        // Process events
        while (xQueueReceive(s_winch_ctrl.event_queue, &msg, 0) == pdTRUE) {
            winch_event_t event = winch_build_event();
            winch_process_state_machine(&event);
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    vTaskDelete(NULL);
}

static winch_event_t winch_build_event(void) {
    winch_event_t event;
    event.auto_enabled = inputs_get_winch_auto();
    event.tension_ok = !inputs_get_winch_tension();
    event.at_home = inputs_get_winch_home();
    event.home_edge = inputs_home_edge_detected();
    event.tension_edge = inputs_tension_edge_detected();
    
    return event;
}

static void winch_process_state_machine(winch_event_t *event) {
    winch_state_t old_state = s_winch_ctrl.context.state;
    
    // Check for safety violations
    if (!event->auto_enabled) {
        ESP_LOGW(TAG, "AUTO mode disabled during operation");
        winch_set_error(WINCH_ERROR_AUTO_LOST);
        return;
    }
    
    if (!event->tension_ok) {
        ESP_LOGW(TAG, "Tension alarm during operation");
        winch_set_error(WINCH_ERROR_TENSION);
        return;
    }
    
    switch (s_winch_ctrl.context.state) {
        case WINCH_STATE_GOING_HOME:
            if (event->at_home || event->home_edge) {
                outputs_set_winch_up(false);
                s_winch_ctrl.context.state = WINCH_STATE_COMPLETED;
                ESP_LOGI(TAG, "Home position reached");
            } else if (timer_elapsed(&s_winch_ctrl.context.operation_timer, 60000)) {
                outputs_set_winch_up(false);
                winch_set_error(WINCH_ERROR_TIMEOUT);
            } else {
                outputs_set_winch_up(true);
            }
            break;
            
        case WINCH_STATE_LOWERING:
            {
                // Calculate expected operation time
                uint32_t expected_time_ms = (s_winch_ctrl.current_pkg->end_depth * 1000) / 
                                          s_winch_ctrl.current_pkg->prev_estimated_cm_per_s;
                
                if (timer_elapsed(&s_winch_ctrl.context.movement_timer, expected_time_ms)) {
                    outputs_set_winch_down(false);
                    
                    // Check if we have static points
                    if (s_winch_ctrl.current_pkg->static_points[0] > 0) {
                        s_winch_ctrl.context.state = WINCH_STATE_STATIC_SAMPLING;
                        timer_start(&s_winch_ctrl.context.static_timer);
                        ESP_LOGI(TAG, "Starting static sampling");
                    } else {
                        s_winch_ctrl.context.state = WINCH_STATE_RISING;
                        timer_start(&s_winch_ctrl.context.movement_timer);
                        ESP_LOGI(TAG, "Lowering complete, starting rising");
                    }
                } else {
                    outputs_set_winch_down(true);
                }
            }
            break;
            
        case WINCH_STATE_STATIC_SAMPLING:
            {
                uint32_t static_duration_ms = s_winch_ctrl.current_pkg->samples * 
                                             s_winch_ctrl.current_pkg->static_poll_interval_s * 1000;
                
                if (timer_elapsed(&s_winch_ctrl.context.static_timer, static_duration_ms)) {
                    s_winch_ctrl.accumulated_static_time_ms += static_duration_ms;
                    s_winch_ctrl.context.current_static_index++;
                    
                    // Check if more static points
                    if (s_winch_ctrl.current_pkg->static_points[s_winch_ctrl.context.current_static_index] > 0) {
                        s_winch_ctrl.context.state = WINCH_STATE_LOWERING;
                        timer_start(&s_winch_ctrl.context.movement_timer);
                        ESP_LOGI(TAG, "Moving to next static point");
                    } else {
                        s_winch_ctrl.context.state = WINCH_STATE_RISING;
                        timer_start(&s_winch_ctrl.context.movement_timer);
                        ESP_LOGI(TAG, "All static points done, starting rising");
                    }
                } else {
                    // Blink outputs to indicate static sampling
                    static bool blink_state = false;
                    static w_timer_t blink_timer = {0};
                    if (timer_elapsed(&blink_timer, 500)) { // 500ms blink
                        blink_state = !blink_state;
                        outputs_set_lamp(blink_state);
                        timer_start(&blink_timer);
                    }
                }
            }
            break;
            
        case WINCH_STATE_RISING:
            if (event->at_home || event->home_edge) {
                outputs_set_winch_up(false);
                s_winch_ctrl.context.state = WINCH_STATE_COMPLETED;
                ESP_LOGI(TAG, "Rising complete - home reached");
            } else if (timer_elapsed(&s_winch_ctrl.context.operation_timer, 120000)) { // 2 min timeout
                outputs_set_winch_up(false);
                winch_set_error(WINCH_ERROR_TIMEOUT);
            } else {
                outputs_set_winch_up(true);
            }
            break;
            
        case WINCH_STATE_COMPLETED:
            outputs_set_winch_up(false);
            outputs_set_winch_down(false);
            outputs_set_lamp(false);
            xEventGroupSetBits(s_winch_ctrl.operation_event_group, WINCH_OPERATION_COMPLETED_BIT);
            break;
            
        case WINCH_STATE_ERROR:
            outputs_set_winch_up(false);
            outputs_set_winch_down(false);
            outputs_set_lamp(false);
            outputs_set_alarm_lamp(true);
            xEventGroupSetBits(s_winch_ctrl.operation_event_group, WINCH_OPERATION_ERROR_BIT);
            break;
            
        default:
            break;
    }
    
    if (old_state != s_winch_ctrl.context.state) {
        ESP_LOGI(TAG, "State change: %s → %s", 
                winch_state_to_string(old_state), winch_state_to_string(s_winch_ctrl.context.state));
    }
}

static void winch_set_error(uint32_t error) {
    s_winch_ctrl.error_mask |= error;
    s_winch_ctrl.context.state = WINCH_STATE_ERROR;
    outputs_set_winch_up(false);
    outputs_set_winch_down(false);
    ESP_LOGE(TAG, "Error set: 0x%02X", error);
}

static void winch_clear_error(uint32_t error) {
    s_winch_ctrl.error_mask &= ~error;
    if (s_winch_ctrl.error_mask == WINCH_ERROR_NONE) {
        outputs_set_alarm_lamp(false);
        ESP_LOGI(TAG, "All errors cleared");
    }
}