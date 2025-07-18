#ifndef CAN_MOTCTRL_COMMON_H
#define CAN_MOTCTRL_COMMON_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "buoye_structs.h" // for motorcontroller_pkg_t and motorcontroller_response_t

#ifdef __cplusplus
extern "C" {
#endif

// Version information
#define CAN_MOTCTRL_VERSION_STRING "1.0.0"


// CAN Message IDs for motor controller communication
#define CAN_ID_MOTCTRL_PKG_START    0x100   // Package start frame
#define CAN_ID_MOTCTRL_PKG_DATA     0x101   // Package data frames
#define CAN_ID_MOTCTRL_PKG_END      0x102   // Package end frame
#define CAN_ID_MOTCTRL_STATUS_REQ   0x103   // Status request from manager
#define CAN_ID_MOTCTRL_STATUS_RESP  0x104   // Status response from worker
#define CAN_ID_MOTCTRL_RESP_START   0x105   // Response start frame
#define CAN_ID_MOTCTRL_RESP_DATA    0x106   // Response data frames
#define CAN_ID_MOTCTRL_RESP_END     0x107   // Response end frame
#define CAN_ID_MOTCTRL_RESP_ACK     0x108   // 
#define CAN_ID_MOTCTRL_PKG_ACK      0x109   // 

// Operation timeouts (use config values with fallbacks)
#ifdef CONFIG_CAN_MOTCTRL_DEFAULT_TIMEOUT_SEC
    #define CAN_MOTCTRL_DEFAULT_TIMEOUT     CONFIG_CAN_MOTCTRL_DEFAULT_TIMEOUT_SEC
#else
    #define CAN_MOTCTRL_DEFAULT_TIMEOUT     30
#endif
#define CAN_ACK_TIMEOUT_MS 500  // CAN_MOTCTRL_DEFAULT_TIMEOUT

#ifdef CONFIG_CAN_MOTCTRL_MIN_TIMEOUT_SEC
    #define CAN_MOTCTRL_MIN_TIMEOUT         CONFIG_CAN_MOTCTRL_MIN_TIMEOUT_SEC
#else
    #define CAN_MOTCTRL_MIN_TIMEOUT         10
#endif

#ifdef CONFIG_CAN_MOTCTRL_MAX_TIMEOUT_SEC
    #define CAN_MOTCTRL_MAX_TIMEOUT         CONFIG_CAN_MOTCTRL_MAX_TIMEOUT_SEC
#else
    #define CAN_MOTCTRL_MAX_TIMEOUT         1800
#endif

// Fragment buffer size
#ifdef CONFIG_CAN_MOTCTRL_FRAGMENT_BUFFER_SIZE
    #define MAX_FRAGMENT_SIZE               CONFIG_CAN_MOTCTRL_FRAGMENT_BUFFER_SIZE
#else
    #define MAX_FRAGMENT_SIZE               1024
#endif

#define CAN_MAX_RETRIES 3
#define CAN_RETRY_BASE_DELAY_MS 100  // Base delay for retries

// Worker status codes (sent over CAN)
typedef enum {
    WORKER_STATUS_IDLE         = 0x00,
    WORKER_STATUS_READY        = 0x01,
    WORKER_STATUS_WORKING      = 0x02,
    WORKER_STATUS_RESP_READY   = 0x03,
    WORKER_STATUS_ERROR        = 0xFF
} worker_status_t;



// Common error codes (in addition to standard ESP error codes)
#define ESP_ERR_MOTCTRL_BASE            0x6000
#define ESP_ERR_MOTCTRL_INVALID_ROLE    (ESP_ERR_MOTCTRL_BASE + 1)
#define ESP_ERR_MOTCTRL_NOT_READY       (ESP_ERR_MOTCTRL_BASE + 2)
#define ESP_ERR_MOTCTRL_BUSY            (ESP_ERR_MOTCTRL_BASE + 3)
#define ESP_ERR_MOTCTRL_FRAGMENT_ERROR  (ESP_ERR_MOTCTRL_BASE + 4)
#define ESP_ERR_MOTCTRL_PROTOCOL_ERROR  (ESP_ERR_MOTCTRL_BASE + 5)

// Common utility functions

/**
 * @brief Calculate appropriate timeout based on operation parameters
 * @param state Operation state (LOWERING/RISING)
 * @param prev_estimated_cm_per_s Previous speed estimate
 * @param rising_timeout_percent Additional timeout for rising operations
 * @param end_depth Target depth for operation
 * @param static_points Array of static measurement points
 * @param samples Number of samples per point
 * @param static_poll_interval_s Interval between samples
 * @return Calculated timeout in seconds
 */

 /**
 * @brief Calculate operation timeout based on package parameters
 * 
 * @param pkg Motor controller package containing operation parameters
 * @return Estimated timeout in seconds
 * 
 * Calculation rules:
 * - LIN_TIME: static_poll_interval_s
 * - ALPHA_DEPTH: end_depth / (prev_estimated_cm_per_s / 1000)  [time = distance / speed]
 * - STATIC_DEPTH: number_of_points * (travel_time + static_wait_time)
 *   where travel_time = end_depth / (prev_estimated_cm_per_s / 1000)
 *   and static_wait_time = samples * (static_poll_interval_s + OFFSET)
 */
uint32_t calculate_operation_timeout(const motorcontroller_pkg_t *pkg);

/**
 * @brief Calculate operation timeout with additional safety margin
 * 
 * @param pkg Motor controller package
 * @param safety_margin_percent Additional safety margin (e.g., 30 for 30% extra time)
 * @return Timeout with safety margin in milliseconds
 */
uint32_t calculate_operation_timeout_with_margin(const motorcontroller_pkg_t *pkg, 
                                                uint8_t safety_margin_percent);


/**
 * @brief Validates motorcontroller package parameters
 * 
 * SCALING NOTE: prev_estimated_cm_per_s is scaled (10000 => 10.000 cm/s)
 * 
 * @param pkg Package to validate
 * @param expected_state Expected state for this package
 * @return true if valid, false otherwise
 */
bool is_motorcontroller_pkg_valid(const motorcontroller_pkg_t *pkg, state_t expected_state);

/**
 * @brief Print motor controller package information (for debugging)
 * @param pkg Package to print
 * @param tag Log tag to use
 */
void print_motorcontroller_pkg_info(const motorcontroller_pkg_t *pkg, const char *tag);

/**
 * @brief Print motor controller response information (for debugging)
 * @param resp Response to print
 * @param tag Log tag to use
 */
void print_motorcontroller_response_info(const motorcontroller_response_t *resp, const char *tag);

/**
 * @brief Get string representation of worker status
 * @param status Status to convert  
 * @return String representation
 */
const char* get_worker_status_string(worker_status_t status);

#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_COMMON_H