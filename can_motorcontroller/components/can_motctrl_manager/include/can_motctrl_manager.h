#ifndef CAN_MOTCTRL_MANAGER_H
#define CAN_MOTCTRL_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/twai.h"
#include "buoye_structs.h"
#include "can_bus_manager.h"
#include "can_motctrl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Event group bits for manager operations
#define MOTCTRL_MANAGER_PKG_SENT_BIT    BIT0
#define MOTCTRL_MANAGER_RESP_RCVD_BIT   BIT1
#define MOTCTRL_MANAGER_ERROR_BIT       BIT2
#define MOTCTRL_MANAGER_TIMEOUT_BIT     BIT3

// Manager context structure
typedef struct {
    QueueHandle_t can_rx_queue;
    TaskHandle_t rx_task_handle;
} can_motctrl_manager_ctx_t;


// CAN bus manager callback type
typedef void (*can_bus_event_callback_t)(const can_bus_event_data_t *event_data, void *user_data);

/**
 * @brief Initialize CAN motor controller manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_manager_init(void);

/**
 * @brief Deinitialize CAN motor controller manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_manager_deinit(void);

/**
 * @brief Send package to motor controller worker
 * 
 * @param pkg Package to send
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t can_motctrl_manager_send_pkg(const motorcontroller_pkg_t *pkg, int timeout_sec);

/**
 * @brief Wait for response from motor controller worker
 * 
 * @param resp Pointer to store received response
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t can_motctrl_manager_wait_response(motorcontroller_response_t *resp, int timeout_sec);

/**
 * @brief Wake up motor controller worker
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_manager_wake_up_motorcontroller(void);

/**
 * @brief Calculate appropriate wait time based on motor controller state
 * 
 * @param state Current motor state
 * @param prev_estimated_cm_per_s Previous estimated speed
 * @param rising_timeout_percent Additional timeout percentage for rising state
 * @return Calculated wait time in seconds
 */
int motctrl_manager_calculate_wait_time(state_t state, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent);

/**
 * @brief Check if manager is ready for new operations
 * 
 * @return true if ready, false otherwise
 */
bool can_motctrl_manager_is_ready(void);

/**
 * @brief Cancel current operation
 * 
 * @return ESP_OK on success
 */
esp_err_t can_motctrl_manager_cancel_operation(void);

// Functions you'll need to implement in your CAN bus manager:

/**
 * @brief Initialize CAN bus manager
 * 
 * @param callback Event callback function
 * @param user_data User data for callback
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_bus_manager_init(can_bus_event_callback_t callback, void *user_data);

/**
 * @brief Deinitialize CAN bus manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_bus_manager_deinit(void);

/**
 * @brief Send CAN message
 * 
 * @param message Message to send
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_bus_manager_send_message(const twai_message_t *message, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_MANAGER_H