/**
 * @file can_motctrl_debug.c
 * @brief Debugging and diagnostic utilities for CAN motor controller system
 * 
 * This file provides comprehensive debugging tools including message tracing,
 * performance monitoring, error analysis, and system diagnostics.
 */

#include "can_motctrl_debug.h"
#include "can_bus_manager.h"
#include "can_serde_helper.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "can_motctrl_debug";

// Debug configuration
#define MAX_TRACE_ENTRIES       1000
#define MAX_ERROR_ENTRIES       100
#define PERFORMANCE_WINDOW_MS   10000
#define DEBUG_TASK_STACK_SIZE   4096

// Message trace entry
typedef struct {
    uint64_t timestamp_us;
    uint32_t can_id;
    uint8_t data_length;
    uint8_t data[8];
    can_debug_direction_t direction;
    can_debug_message_type_t type;
} message_trace_entry_t;

// Error trace entry
typedef struct {
    uint64_t timestamp_us;
    esp_err_t error_code;
    uint32_t can_id;
    can_debug_error_type_t error_type;
    char description[64];
} error_trace_entry_t;

// Performance metrics
typedef struct {
    uint64_t window_start_us;
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t fragments_sent;
    uint32_t fragments_received;
    uint32_t packages_completed;
    uint32_t errors_detected;
    uint64_t total_latency_us;
    uint64_t min_latency_us;
    uint64_t max_latency_us;
} performance_metrics_t;

// Debug context
typedef struct {
    bool initialized;
    bool tracing_enabled;
    bool performance_monitoring_enabled;
    can_debug_level_t debug_level;
    
    // Message tracing
    message_trace_entry_t *message_trace;
    uint32_t trace_head;
    uint32_t trace_count;
    SemaphoreHandle_t trace_mutex;
    
    // Error tracking
    error_trace_entry_t *error_trace;
    uint32_t error_head;
    uint32_t error_count;
    SemaphoreHandle_t error_mutex;
    
    // Performance monitoring
    performance_metrics_t current_metrics;
    performance_metrics_t last_metrics;
    SemaphoreHandle_t perf_mutex;
    
    // Debug task
    TaskHandle_t debug_task_handle;
    
    // Statistics
    can_debug_statistics_t stats;
} can_debug_ctx_t;

static can_debug_ctx_t s_debug_ctx = {0};

// Internal function prototypes
static void debug_task(void *pvParameters);
static void add_message_trace(uint32_t can_id, const uint8_t *data, uint8_t length, 
                             can_debug_direction_t direction, can_debug_message_type_t type);
static void add_error_trace(esp_err_t error_code, uint32_t can_id, can_debug_error_type_t error_type, 
                           const char *description);
static void update_performance_metrics(void);
static void print_message_trace_entry(const message_trace_entry_t *entry);
static void print_error_trace_entry(const error_trace_entry_t *entry);
static const char* get_message_type_string(can_debug_message_type_t type);
static const char* get_direction_string(can_debug_direction_t direction);
static const char* get_error_type_string(can_debug_error_type_t type);

esp_err_t can_motctrl_debug_init(can_debug_level_t level)
{
    if (s_debug_ctx.initialized) {
        ESP_LOGW(TAG, "Debug system already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing CAN motor controller debug system...");

    // Initialize context
    memset(&s_debug_ctx, 0, sizeof(s_debug_ctx));
    s_debug_ctx.debug_level = level;
    s_debug_ctx.tracing_enabled = true;
    s_debug_ctx.performance_monitoring_enabled = true;

    // Allocate trace buffers
    s_debug_ctx.message_trace = malloc(MAX_TRACE_ENTRIES * sizeof(message_trace_entry_t));
    if (s_debug_ctx.message_trace == NULL) {
        ESP_LOGE(TAG, "Failed to allocate message trace buffer");
        return ESP_ERR_NO_MEM;
    }

    s_debug_ctx.error_trace = malloc(MAX_ERROR_ENTRIES * sizeof(error_trace_entry_t));
    if (s_debug_ctx.error_trace == NULL) {
        ESP_LOGE(TAG, "Failed to allocate error trace buffer");
        free(s_debug_ctx.message_trace);
        return ESP_ERR_NO_MEM;
    }

    // Create mutexes
    s_debug_ctx.trace_mutex = xSemaphoreCreateMutex();
    s_debug_ctx.error_mutex = xSemaphoreCreateMutex();
    s_debug_ctx.perf_mutex = xSemaphoreCreateMutex();

    if (s_debug_ctx.trace_mutex == NULL || s_debug_ctx.error_mutex == NULL || s_debug_ctx.perf_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create debug mutexes");
        can_motctrl_debug_deinit();
        return ESP_ERR_NO_MEM;
    }

    // Initialize performance metrics
    s_debug_ctx.current_metrics.window_start_us = esp_timer_get_time();
    s_debug_ctx.current_metrics.min_latency_us = UINT64_MAX;

    // Create debug task
    BaseType_t task_ret = xTaskCreate(
        debug_task,
        "can_debug",
        DEBUG_TASK_STACK_SIZE,
        NULL,
        2,  // Low priority
        &s_debug_ctx.debug_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create debug task");
        can_motctrl_debug_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_debug_ctx.initialized = true;
    ESP_LOGI(TAG, "Debug system initialized with level %d", level);
    return ESP_OK;
}

esp_err_t can_motctrl_debug_deinit(void)
{
    if (!s_debug_ctx.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing debug system...");

    // Stop debug task
    if (s_debug_ctx.debug_task_handle != NULL) {
        vTaskDelete(s_debug_ctx.debug_task_handle);
        s_debug_ctx.debug_task_handle = NULL;
    }

    // Clean up mutexes
    if (s_debug_ctx.trace_mutex != NULL) {
        vSemaphoreDelete(s_debug_ctx.trace_mutex);
        s_debug_ctx.trace_mutex = NULL;
    }

    if (s_debug_ctx.error_mutex != NULL) {
        vSemaphoreDelete(s_debug_ctx.error_mutex);
        s_debug_ctx.error_mutex = NULL;
    }

    if (s_debug_ctx.perf_mutex != NULL) {
        vSemaphoreDelete(s_debug_ctx.perf_mutex);
        s_debug_ctx.perf_mutex = NULL;
    }

    // Free trace buffers
    if (s_debug_ctx.message_trace != NULL) {
        free(s_debug_ctx.message_trace);
        s_debug_ctx.message_trace = NULL;
    }

    if (s_debug_ctx.error_trace != NULL) {
        free(s_debug_ctx.error_trace);
        s_debug_ctx.error_trace = NULL;
    }

    s_debug_ctx.initialized = false;
    ESP_LOGI(TAG, "Debug system deinitialized");
    return ESP_OK;
}

void can_motctrl_debug_trace_message(uint32_t can_id, const uint8_t *data, uint8_t length, 
                                    can_debug_direction_t direction)
{
    if (!s_debug_ctx.initialized || !s_debug_ctx.tracing_enabled) {
        return;
    }

    // Determine message type based on CAN ID
    can_debug_message_type_t type = CAN_DEBUG_MSG_UNKNOWN;
    
    switch (can_id) {
        case CAN_ID_MOTCTRL_PKG_START:
        case CAN_ID_MOTCTRL_PKG_DATA:
        case CAN_ID_MOTCTRL_PKG_END:
            type = CAN_DEBUG_MSG_PACKAGE;
            break;
        case CAN_ID_MOTCTRL_RESP_START:
        case CAN_ID_MOTCTRL_RESP_DATA:
        case CAN_ID_MOTCTRL_RESP_END:
            type = CAN_DEBUG_MSG_RESPONSE;
            break;
        case CAN_ID_MOTCTRL_STATUS_REQ:
        case CAN_ID_MOTCTRL_STATUS_RESP:
            type = CAN_DEBUG_MSG_STATUS;
            break;
        default:
            type = CAN_DEBUG_MSG_UNKNOWN;
            break;
    }

    add_message_trace(can_id, data, length, direction, type);
    
    // Update statistics
    if (xSemaphoreTake(s_debug_ctx.perf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (direction == CAN_DEBUG_DIR_TX) {
            s_debug_ctx.stats.total_messages_sent++;
            if (type == CAN_DEBUG_MSG_PACKAGE) {
                s_debug_ctx.current_metrics.fragments_sent++;
            }
        } else {
            s_debug_ctx.stats.total_messages_received++;
            if (type == CAN_DEBUG_MSG_RESPONSE) {
                s_debug_ctx.current_metrics.fragments_received++;
            }
        }
        xSemaphoreGive(s_debug_ctx.perf_mutex);
    }
}

void can_motctrl_debug_trace_error(esp_err_t error_code, uint32_t can_id, 
                                  can_debug_error_type_t error_type, const char *description)
{
    if (!s_debug_ctx.initialized) {
        return;
    }

    add_error_trace(error_code, can_id, error_type, description);
    
    // Update statistics
    if (xSemaphoreTake(s_debug_ctx.perf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_debug_ctx.stats.total_errors++;
        s_debug_ctx.current_metrics.errors_detected++;
        
        switch (error_type) {
            case CAN_DEBUG_ERR_TIMEOUT:
                s_debug_ctx.stats.timeout_errors++;
                break;
            case CAN_DEBUG_ERR_CRC:
                s_debug_ctx.stats.crc_errors++;
                break;
            case CAN_DEBUG_ERR_FRAGMENTATION:
                s_debug_ctx.stats.fragmentation_errors++;
                break;
            case CAN_DEBUG_ERR_BUS:
                s_debug_ctx.stats.bus_errors++;
                break;
            default:
                break;
        }
        
        xSemaphoreGive(s_debug_ctx.perf_mutex);
    }
}

esp_err_t can_motctrl_debug_print_trace(uint32_t count)
{
    if (!s_debug_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Message Trace (last %lu messages) ===", count);

    if (xSemaphoreTake(s_debug_ctx.trace_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        uint32_t entries_to_print = (count < s_debug_ctx.trace_count) ? count : s_debug_ctx.trace_count;
        
        for (uint32_t i = 0; i < entries_to_print; i++) {
            uint32_t index = (s_debug_ctx.trace_head - entries_to_print + i + MAX_TRACE_ENTRIES) % MAX_TRACE_ENTRIES;
            print_message_trace_entry(&s_debug_ctx.message_trace[index]);
        }
        
        xSemaphoreGive(s_debug_ctx.trace_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire trace mutex");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t can_motctrl_debug_print_errors(uint32_t count)
{
    if (!s_debug_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Error Trace (last %lu errors) ===", count);

    if (xSemaphoreTake(s_debug_ctx.error_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        uint32_t entries_to_print = (count < s_debug_ctx.error_count) ? count : s_debug_ctx.error_count;
        
        for (uint32_t i = 0; i < entries_to_print; i++) {
            uint32_t index = (s_debug_ctx.error_head - entries_to_print + i + MAX_ERROR_ENTRIES) % MAX_ERROR_ENTRIES;
            print_error_trace_entry(&s_debug_ctx.error_trace[index]);
        }
        
        xSemaphoreGive(s_debug_ctx.error_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire error mutex");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t can_motctrl_debug_get_statistics(can_debug_statistics_t *stats)
{
    if (!s_debug_ctx.initialized || stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_debug_ctx.perf_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *stats = s_debug_ctx.stats;
        xSemaphoreGive(s_debug_ctx.perf_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t can_motctrl_debug_print_performance(void)
{
    if (!s_debug_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Performance Metrics ===");

    if (xSemaphoreTake(s_debug_ctx.perf_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        uint64_t window_duration = esp_timer_get_time() - s_debug_ctx.current_metrics.window_start_us;
        double window_seconds = (double)window_duration / 1000000.0;

        ESP_LOGI(TAG, "Current Window (%.1f seconds):", window_seconds);
        ESP_LOGI(TAG, "  Messages: %lu sent, %lu received", 
                s_debug_ctx.current_metrics.messages_sent,
                s_debug_ctx.current_metrics.messages_received);
        ESP_LOGI(TAG, "  Fragments: %lu sent, %lu received",
                s_debug_ctx.current_metrics.fragments_sent,
                s_debug_ctx.current_metrics.fragments_received);
        ESP_LOGI(TAG, "  Packages completed: %lu", s_debug_ctx.current_metrics.packages_completed);
        ESP_LOGI(TAG, "  Errors detected: %lu", s_debug_ctx.current_metrics.errors_detected);
        
        if (s_debug_ctx.current_metrics.packages_completed > 0) {
            uint64_t avg_latency = s_debug_ctx.current_metrics.total_latency_us / 
                                  s_debug_ctx.current_metrics.packages_completed;
            ESP_LOGI(TAG, "  Latency: %llu us avg, %llu us min, %llu us max",
                    avg_latency,
                    s_debug_ctx.current_metrics.min_latency_us,
                    s_debug_ctx.current_metrics.max_latency_us);
        }
        
        if (window_seconds > 0) {
            ESP_LOGI(TAG, "  Throughput: %.1f msg/s, %.1f pkg/s",
                    (s_debug_ctx.current_metrics.messages_sent + s_debug_ctx.current_metrics.messages_received) / window_seconds,
                    s_debug_ctx.current_metrics.packages_completed / window_seconds);
        }

        ESP_LOGI(TAG, "Overall Statistics:");
        ESP_LOGI(TAG, "  Total messages: %lu sent, %lu received",
                s_debug_ctx.stats.total_messages_sent,
                s_debug_ctx.stats.total_messages_received);
        ESP_LOGI(TAG, "  Total errors: %lu", s_debug_ctx.stats.total_errors);
        ESP_LOGI(TAG, "  Error breakdown: %lu timeout, %lu CRC, %lu fragmentation, %lu bus",
                s_debug_ctx.stats.timeout_errors,
                s_debug_ctx.stats.crc_errors,
                s_debug_ctx.stats.fragmentation_errors,
                s_debug_ctx.stats.bus_errors);

        xSemaphoreGive(s_debug_ctx.perf_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire performance mutex");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t can_motctrl_debug_enable_tracing(bool enable)
{
    s_debug_ctx.tracing_enabled = enable;
    ESP_LOGI(TAG, "Message tracing %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t can_motctrl_debug_set_level(can_debug_level_t level)
{
    s_debug_ctx.debug_level = level;
    ESP_LOGI(TAG, "Debug level set to %d", level);
    return ESP_OK;
}

// Internal functions

static void debug_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Debug task started");
    
    TickType_t last_performance_update = xTaskGetTickCount();
    
    while (s_debug_ctx.initialized) {
        TickType_t now = xTaskGetTickCount();
        
        // Update performance metrics periodically
        if ((now - last_performance_update) >= pdMS_TO_TICKS(PERFORMANCE_WINDOW_MS)) {
            update_performance_metrics();
            last_performance_update = now;
        }
        
        // Periodic logging based on debug level
        if (s_debug_ctx.debug_level >= CAN_DEBUG_LEVEL_VERBOSE) {
            static uint32_t log_counter = 0;
            if (++log_counter % 60 == 0) {  // Every minute
                can_motctrl_debug_print_performance();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "Debug task exiting");
    vTaskDelete(NULL);
}

static void add_message_trace(uint32_t can_id, const uint8_t *data, uint8_t length, 
                             can_debug_direction_t direction, can_debug_message_type_t type)
{
    if (xSemaphoreTake(s_debug_ctx.trace_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        message_trace_entry_t *entry = &s_debug_ctx.message_trace[s_debug_ctx.trace_head];
        
        entry->timestamp_us = esp_timer_get_time();
        entry->can_id = can_id;
        entry->data_length = (length > 8) ? 8 : length;
        entry->direction = direction;
        entry->type = type;
        
        if (data != NULL && length > 0) {
            memcpy(entry->data, data, entry->data_length);
        } else {
            memset(entry->data, 0, sizeof(entry->data));
        }
        
        s_debug_ctx.trace_head = (s_debug_ctx.trace_head + 1) % MAX_TRACE_ENTRIES;
        if (s_debug_ctx.trace_count < MAX_TRACE_ENTRIES) {
            s_debug_ctx.trace_count++;
        }
        
        xSemaphoreGive(s_debug_ctx.trace_mutex);
    }
}

static void add_error_trace(esp_err_t error_code, uint32_t can_id, can_debug_error_type_t error_type, 
                           const char *description)
{
    if (xSemaphoreTake(s_debug_ctx.error_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        error_trace_entry_t *entry = &s_debug_ctx.error_trace[s_debug_ctx.error_head];
        
        entry->timestamp_us = esp_timer_get_time();
        entry->error_code = error_code;
        entry->can_id = can_id;
        entry->error_type = error_type;
        
        if (description != NULL) {
            strncpy(entry->description, description, sizeof(entry->description) - 1);
            entry->description[sizeof(entry->description) - 1] = '\0';
        } else {
            strcpy(entry->description, "No description");
        }
        
        s_debug_ctx.error_head = (s_debug_ctx.error_head + 1) % MAX_ERROR_ENTRIES;
        if (s_debug_ctx.error_count < MAX_ERROR_ENTRIES) {
            s_debug_ctx.error_count++;
        }
        
        xSemaphoreGive(s_debug_ctx.error_mutex);
    }
}

static void update_performance_metrics(void)
{
    if (xSemaphoreTake(s_debug_ctx.perf_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Save current metrics as last metrics
        s_debug_ctx.last_metrics = s_debug_ctx.current_metrics;
        
        // Reset current metrics for next window
        s_debug_ctx.current_metrics.window_start_us = esp_timer_get_time();
        s_debug_ctx.current_metrics.messages_sent = 0;
        s_debug_ctx.current_metrics.messages_received = 0;
        s_debug_ctx.current_metrics.fragments_sent = 0;
        s_debug_ctx.current_metrics.fragments_received = 0;
        s_debug_ctx.current_metrics.packages_completed = 0;
        s_debug_ctx.current_metrics.errors_detected = 0;
        s_debug_ctx.current_metrics.total_latency_us = 0;
        s_debug_ctx.current_metrics.min_latency_us = UINT64_MAX;
        s_debug_ctx.current_metrics.max_latency_us = 0;
        
        xSemaphoreGive(s_debug_ctx.perf_mutex);
    }
}

static void print_message_trace_entry(const message_trace_entry_t *entry)
{
    printf("[%llu] %s %s ID:0x%03lX DLC:%d Data:",
           entry->timestamp_us,
           get_direction_string(entry->direction),
           get_message_type_string(entry->type),
           entry->can_id,
           entry->data_length);
    
    for (int i = 0; i < entry->data_length; i++) {
        printf(" %02X", entry->data[i]);
    }
    printf("\n");
}

static void print_error_trace_entry(const error_trace_entry_t *entry)
{
    printf("[%llu] ERROR %s ID:0x%03lX Code:%s Desc:%s\n",
           entry->timestamp_us,
           get_error_type_string(entry->error_type),
           entry->can_id,
           esp_err_to_name(entry->error_code),
           entry->description);
}

static const char* get_message_type_string(can_debug_message_type_t type)
{
    switch (type) {
        case CAN_DEBUG_MSG_PACKAGE: return "PKG";
        case CAN_DEBUG_MSG_RESPONSE: return "RESP";
        case CAN_DEBUG_MSG_STATUS: return "STAT";
        case CAN_DEBUG_MSG_UNKNOWN: return "UNK";
        default: return "???";
    }
}

static const char* get_direction_string(can_debug_direction_t direction)
{
    switch (direction) {
        case CAN_DEBUG_DIR_TX: return "TX";
        case CAN_DEBUG_DIR_RX: return "RX";
        default: return "??";
    }
}

static const char* get_error_type_string(can_debug_error_type_t type)
{
    switch (type) {
        case CAN_DEBUG_ERR_TIMEOUT: return "TIMEOUT";
        case CAN_DEBUG_ERR_CRC: return "CRC";
        case CAN_DEBUG_ERR_FRAGMENTATION: return "FRAG";
        case CAN_DEBUG_ERR_BUS: return "BUS";
        case CAN_DEBUG_ERR_OTHER: return "OTHER";
        default: return "UNKNOWN";
    }
}