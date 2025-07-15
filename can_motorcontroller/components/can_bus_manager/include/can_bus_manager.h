#ifndef CAN_BUS_MANAGER_H
#define CAN_BUS_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

// CAN bus manager event types
typedef enum {
    CAN_BUS_EVT_MESSAGE_SENT,      // Message successfully sent
    CAN_BUS_EVT_MESSAGE_RECEIVED,  // Message received
    CAN_BUS_EVT_ERROR,             // CAN bus error occurred
    CAN_BUS_EVT_TIMEOUT,           // Operation timeout
    CAN_BUS_EVT_BUS_OFF,           // Bus off condition
    CAN_BUS_EVT_BUS_RECOVERED      // Bus recovered from error
} can_bus_event_type_t;

// CAN bus event data structure
typedef struct {
    can_bus_event_type_t event_type;
    twai_message_t message;        // Message data (valid for SENT/RECEIVED events)
    esp_err_t error_code;          // Error code (valid for ERROR events)
} can_bus_event_data_t;

// CAN bus manager callback type
typedef void (*can_bus_event_callback_t)(const can_bus_event_data_t *event_data, void *user_data);

/**
 * @brief Initialize CAN bus manager
 * 
 * Sets up the TWAI driver, creates RX/TX tasks, and registers event callback.
 * The CAN bus will be configured for 500 kbps with acceptance of all messages.
 * 
 * @param callback Event callback function (can be NULL)
 * @param user_data User data passed to callback (can be NULL)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_bus_manager_init(can_bus_event_callback_t callback, void *user_data);

/**
 * @brief Deinitialize CAN bus manager
 * 
 * Stops all tasks, uninstalls TWAI driver, and cleans up resources.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_bus_manager_deinit(void);

/**
 * @brief Send CAN message (synchronous)
 * 
 * Sends a CAN message and waits for transmission confirmation.
 * This function will call the event callback on success or failure.
 * Uses priority-based queuing to prevent starvation.
 * 
 * @param message Pointer to CAN message to send
 * @param timeout_ms Timeout in milliseconds for transmission
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, other error codes on failure
 */
esp_err_t can_bus_manager_send_message(const twai_message_t *message, uint32_t timeout_ms);

/**
 * @brief Send CAN message (asynchronous)
 * 
 * Queues a CAN message for transmission without waiting for completion.
 * The event callback will be called when transmission completes.
 * 
 * @param message Pointer to CAN message to send
 * @return ESP_OK if queued successfully, error code otherwise
 */
esp_err_t can_bus_manager_send_message_async(const twai_message_t *message);

/**
 * @brief Check if CAN bus manager is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool can_bus_manager_is_initialized(void);

/**
 * @brief Get CAN bus status information
 * 
 * @param status Pointer to store status information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_bus_manager_get_stats(twai_status_info_t *status);

// Extended statistics structure
typedef struct {
    uint32_t messages_sent;      // Total messages sent
    uint32_t messages_received;  // Total messages received
    uint32_t tx_errors;          // Transmission errors
    uint32_t rx_errors;          // Reception errors
    uint32_t bus_off_count;      // Number of times bus went off
    uint32_t queue_overflows;    // Queue overflow events
    TickType_t last_activity;    // Last activity timestamp
} can_bus_stats_t;

/**
 * @brief Get extended CAN bus manager statistics
 * 
 * @param stats Pointer to store extended statistics
 */
void can_bus_manager_get_extended_stats(can_bus_stats_t *stats);

// GPIO pin definitions for CAN (these should be defined in sdkconfig)
#ifndef CONFIG_CAN_TX_GPIO
#define CONFIG_CAN_TX_GPIO 21
#endif

#ifndef CONFIG_CAN_RX_GPIO
#define CONFIG_CAN_RX_GPIO 22
#endif

// Motor controller specific GPIO pins (these should also be in sdkconfig)
#ifndef CONFIG_MOTCTRL_CAN_TX_GPIO
#define CONFIG_MOTCTRL_CAN_TX_GPIO CONFIG_CAN_TX_GPIO
#endif

#ifndef CONFIG_MOTCTRL_CAN_RX_GPIO
#define CONFIG_MOTCTRL_CAN_RX_GPIO CONFIG_CAN_RX_GPIO
#endif

#ifdef __cplusplus
}
#endif

#endif // CAN_BUS_MANAGER_H