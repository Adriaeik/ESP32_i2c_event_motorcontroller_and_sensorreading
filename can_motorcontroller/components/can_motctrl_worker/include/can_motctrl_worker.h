#ifndef CAN_MOTCTRL_WORKER_H
#define CAN_MOTCTRL_WORKER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/twai.h"
#include "buoye_structs.h"
#include "can_serde_helper.h"
#include "can_motctrl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Event group bits
#define MOTCTRL_WORKER_PKG_RECEIVED_BIT BIT0
#define MOTCTRL_WORKER_RESP_SENT_BIT    BIT1
#define MOTCTRL_WORKER_ERROR_BIT        BIT2

// Worker states
typedef enum {
    CAN_WORKER_STATE_IDLE,
    CAN_WORKER_STATE_PKG_RECEIVED,
    CAN_WORKER_STATE_WORKING,
    CAN_WORKER_STATE_RESP_READY
} can_worker_state_t;

// Internal worker events
typedef enum {
    CAN_WORKER_EVT_PKG_RECEIVED,
    CAN_WORKER_EVT_RESP_REQUESTED,
    CAN_WORKER_EVT_ERROR
} can_worker_event_t;

// Worker context structure
typedef struct {
    bool initialized;
    can_worker_state_t state;
    worker_status_t status_byte;
    
    // FreeRTOS objects
    QueueHandle_t event_queue;
    QueueHandle_t can_rx_queue;
    EventGroupHandle_t operation_event_group;
    TaskHandle_t worker_task_handle;
    TaskHandle_t rx_task_handle;
    
    // Package handling
    motorcontroller_pkg_t received_pkg;
    motorcontroller_response_t response_to_send;
    bool pkg_ready;
    bool resp_ready;
    
    // Fragment assembly
    uint8_t *fragment_buffer;
    size_t fragment_pos;
    uint16_t expected_fragments;
    uint16_t fragments_received;
    can_fragment_list_t rx_fragment_list;  // For proper fragment reassembly
} can_motctrl_worker_ctx_t;

/**
 * @brief Initialize CAN motor controller worker
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_worker_init(void);

/**
 * @brief Deinitialize CAN motor controller worker
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_worker_deinit(void);

/**
 * @brief Wait for a package from the manager
 * 
 * @param pkg Pointer to store received package
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t can_motctrl_worker_wait_pkg(motorcontroller_pkg_t *pkg, int timeout_sec);

/**
 * @brief Send response to the manager
 * 
 * @param resp Response to send
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t can_motctrl_worker_send_response(const motorcontroller_response_t *resp, int timeout_sec);

/**
 * @brief Set worker state to working
 * 
 * @return ESP_OK on success
 */
esp_err_t can_motctrl_worker_set_working(void);

/**
 * @brief Set response ready for manager to read
 * 
 * @param resp Response to make ready
 * @return ESP_OK on success
 */
esp_err_t can_motctrl_worker_set_response_ready(const motorcontroller_response_t *resp);

/**
 * @brief Get current worker state
 * 
 * @return Current worker state
 */
can_worker_state_t can_motctrl_worker_get_state(void);

/**
 * @brief Check if worker is ready for new operations
 * 
 * @return true if ready, false otherwise
 */
bool can_motctrl_worker_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_WORKER_H