#pragma once

#include "esp_err.h"
#include "buoye_structs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set up motorcontroller package for target state
 * Loads previous state from RTC and updates for current operation
 * @param pkg Output package to configure
 * @param target_state Desired state (LOWERING, RISING, INIT)
 * @return ESP_OK on success
 */
esp_err_t load_system_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t target_state);

/**
 * @brief Update package with response data and store to RTC
 * Updates speed estimates and stores state for next cycle
 * @param pkg Package to update
 * @param resp Response from motor controller
 * @return ESP_OK on success
 */
esp_err_t update_and_store_pkg(motorcontroller_pkg_t *pkg, const motorcontroller_response_t *resp);

/**
 * @brief Start manager background task
 * @return ESP_OK on success
 */
esp_err_t motorcontroller_manager_start_worker(state_t state);

/**
 * @brief Wait for manager task completion
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, ESP_ERR_FAIL on error
 */
esp_err_t motorcontroller_manager_wait_completion(uint32_t timeout_ms);

esp_err_t update_reported_depth(float depth_meters);



#ifdef __cplusplus
}
#endif