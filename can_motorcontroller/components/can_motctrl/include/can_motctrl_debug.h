#ifndef CAN_MOTCTRL_DEBUG_H
#define CAN_MOTCTRL_DEBUG_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Debug levels
typedef enum {
    CAN_DEBUG_LEVEL_NONE = 0,     // No debug output
    CAN_DEBUG_LEVEL_ERROR = 1,    // Errors only
    CAN_DEBUG_LEVEL_WARNING = 2,  // Errors and warnings
    CAN_DEBUG_LEVEL_INFO = 3,     // Normal operation info
    CAN_DEBUG_LEVEL_VERBOSE = 4   // Detailed debugging
} can_debug_level_t;

// Message directions
typedef enum {
    CAN_DEBUG_DIR_TX,  // Transmitted message
    CAN_DEBUG_DIR_RX   // Received message
} can_debug_direction_t;

// Message types
typedef enum {
    CAN_DEBUG_MSG_PACKAGE,   // Motor controller package
    CAN_DEBUG_MSG_RESPONSE,  // Motor controller response
    CAN_DEBUG_MSG_STATUS,    // Status message
    CAN_DEBUG_MSG_UNKNOWN    // Unknown message type
} can_debug_message_type_t;

// Error types
typedef enum {
    CAN_DEBUG_ERR_TIMEOUT,       // Timeout error
    CAN_DEBUG_ERR_CRC,           // CRC validation error
    CAN_DEBUG_ERR_FRAGMENTATION, // Fragmentation/reassembly error
    CAN_DEBUG_ERR_BUS,           // CAN bus error
    CAN_DEBUG_ERR_OTHER          // Other error
} can_debug_error_type_t;

// Debug statistics
typedef struct {
    uint64_t uptime_us;              // System uptime
    uint32_t total_messages_sent;    // Total messages transmitted
    uint32_t total_messages_received; // Total messages received
    uint32_t total_errors;           // Total errors detected
    uint32_t timeout_errors;         // Timeout errors
    uint32_t crc_errors;            // CRC errors
    uint32_t fragmentation_errors;   // Fragmentation errors
    uint32_t bus_errors;            // Bus errors
    uint32_t packages_completed;     // Successfully completed packages
    uint32_t responses_completed;    // Successfully completed responses
    uint64_t avg_latency_us;        // Average operation latency
    uint64_t max_latency_us;        // Maximum operation latency
} can_debug_statistics_t;

/**
 * @brief Initialize the debug system
 * 
 * Sets up message tracing, error logging, and performance monitoring.
 * 
 * @param level Debug level to use
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_init(can_debug_level_t level);

/**
 * @brief Deinitialize the debug system
 * 
 * Cleans up debug resources and prints final statistics.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_deinit(void);

/**
 * @brief Trace a CAN message
 * 
 * Records a CAN message in the trace buffer for later analysis.
 * This function should be called for all sent and received messages.
 * 
 * @param can_id CAN message identifier
 * @param data Message data buffer
 * @param length Data length (0-8 bytes)
 * @param direction Message direction (TX or RX)
 */
void can_motctrl_debug_trace_message(uint32_t can_id, const uint8_t *data, uint8_t length, 
                                    can_debug_direction_t direction);

/**
 * @brief Trace an error condition
 * 
 * Records an error in the error trace buffer with timestamp and context.
 * 
 * @param error_code ESP error code
 * @param can_id Associated CAN ID (if applicable)
 * @param error_type Type of error
 * @param description Human-readable error description
 */
void can_motctrl_debug_trace_error(esp_err_t error_code, uint32_t can_id, 
                                  can_debug_error_type_t error_type, const char *description);

/**
 * @brief Print recent message trace
 * 
 * Displays the most recent CAN messages from the trace buffer.
 * 
 * @param count Number of messages to display (0 = all available)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_print_trace(uint32_t count);

/**
 * @brief Print recent error trace
 * 
 * Displays the most recent errors from the error trace buffer.
 * 
 * @param count Number of errors to display (0 = all available)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_print_errors(uint32_t count);

/**
 * @brief Get current debug statistics
 * 
 * Retrieves comprehensive statistics about system operation.
 * 
 * @param stats Pointer to statistics structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_get_statistics(can_debug_statistics_t *stats);

/**
 * @brief Print performance metrics
 * 
 * Displays current and historical performance metrics including
 * throughput, latency, and error rates.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_print_performance(void);

/**
 * @brief Enable or disable message tracing
 * 
 * Controls whether CAN messages are stored in the trace buffer.
 * Disabling tracing can improve performance in production.
 * 
 * @param enable True to enable tracing, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_enable_tracing(bool enable);

/**
 * @brief Set debug level
 * 
 * Changes the current debug verbosity level.
 * 
 * @param level New debug level
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_set_level(can_debug_level_t level);

/**
 * @brief Record operation latency
 * 
 * Records the time taken for a complete operation (e.g., send package + receive response).
 * 
 * @param latency_us Operation latency in microseconds
 */
void can_motctrl_debug_record_latency(uint64_t latency_us);

/**
 * @brief Dump system state for debugging
 * 
 * Prints comprehensive system state including:
 * - Current configuration
 * - Recent message trace
 * - Error history
 * - Performance metrics
 * - Memory usage
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_debug_dump_state(void);

/**
 * @brief Check for potential issues
 * 
 * Analyzes current statistics to identify potential problems:
 * - High error rates
 * - Performance degradation
 * - Memory leaks
 * - Configuration issues
 * 
 * @return ESP_OK if no issues detected, ESP_FAIL if issues found
 */
esp_err_t can_motctrl_debug_health_check(void);

// Convenience macros for debug tracing
#define CAN_DEBUG_TRACE_TX(id, data, len) \
    can_motctrl_debug_trace_message(id, data, len, CAN_DEBUG_DIR_TX)

#define CAN_DEBUG_TRACE_RX(id, data, len) \
    can_motctrl_debug_trace_message(id, data, len, CAN_DEBUG_DIR_RX)

#define CAN_DEBUG_TRACE_ERROR(code, id, type, desc) \
    can_motctrl_debug_trace_error(code, id, type, desc)


#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_DEBUG_H