#ifndef MOTCTRL_WORKER_H
#define MOTCTRL_WORKER_H

#include "can_motctrl.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Worker configuration
typedef struct {
    uint32_t work_simulation_time_ms;   // How long to simulate work
    uint32_t status_poll_interval_ms;   // How often to check for status requests
    bool auto_start;                     // Start worker task automatically
} motctrl_worker_config_t;

/**
 * @brief Initialize the motor controller worker
 * @param config Worker configuration
 * @return ESP_OK on success
 */
esp_err_t motctrl_worker_init(const motctrl_worker_config_t *config);

/**
 * @brief Deinitialize the motor controller worker
 * @return ESP_OK on success
 */
esp_err_t motctrl_worker_deinit(void);

/**
 * @brief Start the worker task
 * @return ESP_OK on success
 */
esp_err_t motctrl_worker_start(void);

/**
 * @brief Stop the worker task
 * @return ESP_OK on success
 */
esp_err_t motctrl_worker_stop(void);

/**
 * @brief Get current worker status
 * @return Current worker status
 */
worker_status_t motctrl_worker_get_status(void);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_WORKER_H