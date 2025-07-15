
// components/motctrl_manager/include/motctrl_manager.h
#ifndef MOTCTRL_MANAGER_H
#define MOTCTRL_MANAGER_H

#include "can_motctrl.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Manager operation result
typedef struct {
    esp_err_t result;
    worker_status_t final_status;
    uint32_t work_result;
    uint32_t operation_time_ms;
} motctrl_operation_result_t;

/**
 * @brief Initialize the motor controller manager
 * @return ESP_OK on success
 */
esp_err_t motctrl_manager_init(void);

/**
 * @brief Execute a work command on the worker
 * @param timeout_ms Total timeout for the operation
 * @param result Pointer to store operation result
 * @return ESP_OK on success
 */
esp_err_t motctrl_manager_execute_work(uint32_t timeout_ms, motctrl_operation_result_t *result);

/**
 * @brief Check worker status
 * @param status Pointer to store worker status
 * @param timeout_ms Timeout for status request
 * @return ESP_OK on success
 */
esp_err_t motctrl_manager_check_status(worker_status_t *status, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_MANAGER_H