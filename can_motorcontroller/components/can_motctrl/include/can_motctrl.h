#ifndef MOTCTRL_H
#define MOTCTRL_H

#include "can_motctrl_common.h"
#include "can_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Message frame structure for fragmented communication
typedef struct {
    uint8_t sequence;
    uint8_t total_fragments;
    uint16_t fragment_size;
    uint8_t data[8];
} motctrl_can_frame_t;

// Communication state
typedef enum {
    MOTCTRL_STATE_IDLE = 0,
    MOTCTRL_STATE_SENDING_PKG,
    MOTCTRL_STATE_WAITING_STATUS,
    MOTCTRL_STATE_RECEIVING_RESP,
    MOTCTRL_STATE_ERROR
} motctrl_comm_state_t;

/**
 * @brief Send a motor controller package over CAN
 * @param pkg Package to send
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_send_package(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms);

/**
 * @brief Receive a motor controller package from CAN
 * @param pkg Buffer for received package
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_receive_package(motorcontroller_pkg_t *pkg, uint32_t timeout_ms);

/**
 * @brief Send a motor controller response over CAN
 * @param response Response to send
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_send_response(const motorcontroller_response_t *response, uint32_t timeout_ms);

/**
 * @brief Receive a motor controller response from CAN
 * @param response Buffer for received response
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_receive_response(motorcontroller_response_t *response, uint32_t timeout_ms);

/**
 * @brief Send a status request
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_send_status_request(uint32_t timeout_ms);

/**
 * @brief Send a status response
 * @param status Worker status to send
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_send_status_response(worker_status_t status, uint32_t timeout_ms);

/**
 * @brief Wait for and receive a status response
 * @param status Buffer for received status
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t motctrl_wait_status_response(worker_status_t *status, uint32_t timeout_ms);

/**
 * @brief Create a simple start command package
 * @param pkg Buffer for package
 * @return ESP_OK on success
 */
esp_err_t motctrl_create_start_command(motorcontroller_pkg_t *pkg);

/**
 * @brief Create a simple work completion response
 * @param response Buffer for response
 * @param work_result Result of the work (0 = success)
 * @return ESP_OK on success
 */
esp_err_t motctrl_create_work_response(motorcontroller_response_t *response, uint32_t work_result);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_H