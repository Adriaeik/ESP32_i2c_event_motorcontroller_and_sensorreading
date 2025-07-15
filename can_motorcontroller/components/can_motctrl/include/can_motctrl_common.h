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

// Common configuration constants
#define MAX_POINTS                  64      // Maximum static measurement points
#define MAX_FRAGMENT_SIZE           1024    // Maximum fragmented message size

// CAN Message IDs for motor controller communication
#define CAN_ID_MOTCTRL_PKG_START    0x100   // Package start frame
#define CAN_ID_MOTCTRL_PKG_DATA     0x101   // Package data frames
#define CAN_ID_MOTCTRL_PKG_END      0x102   // Package end frame
#define CAN_ID_MOTCTRL_STATUS_REQ   0x103   // Status request from manager
#define CAN_ID_MOTCTRL_STATUS_RESP  0x104   // Status response from worker
#define CAN_ID_MOTCTRL_RESP_START   0x105   // Response start frame
#define CAN_ID_MOTCTRL_RESP_DATA    0x106   // Response data frames
#define CAN_ID_MOTCTRL_RESP_END     0x107   // Response end frame

// Operation timeouts (use config values with fallbacks)
#ifdef CONFIG_CAN_MOTCTRL_DEFAULT_TIMEOUT_SEC
    #define CAN_MOTCTRL_DEFAULT_TIMEOUT     CONFIG_CAN_MOTCTRL_DEFAULT_TIMEOUT_SEC
#else
    #define CAN_MOTCTRL_DEFAULT_TIMEOUT     30
#endif

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
 * @brief Initialize motor controller package with default values
 * @param pkg Package to initialize
 */
void motorcontroller_pkg_init_default(motorcontroller_pkg_t *pkg);

/**
 * @brief Initialize motor controller response with default values  
 * @param resp Response to initialize
 */
void motorcontroller_response_init_default(motorcontroller_response_t *resp);

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
int calculate_operation_timeout(state_t state, 
                               uint16_t prev_estimated_cm_per_s,
                               int rising_timeout_percent,
                               uint16_t end_depth,
                               const uint16_t *static_points,
                               uint16_t samples,
                               uint16_t static_poll_interval_s);

/**
 * @brief Get string representation of motor controller state
 * @param state State to convert
 * @return String representation
 */
const char* get_state_string(state_t state);

/**
 * @brief Get string representation of worker status
 * @param status Status to convert  
 * @return String representation
 */
const char* get_worker_status_string(worker_status_t status);

/**
 * @brief Check if motor controller package is valid
 * @param pkg Package to validate
 * @return true if valid, false otherwise
 */
bool is_motorcontroller_pkg_valid(const motorcontroller_pkg_t *pkg);

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

#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_COMMON_H