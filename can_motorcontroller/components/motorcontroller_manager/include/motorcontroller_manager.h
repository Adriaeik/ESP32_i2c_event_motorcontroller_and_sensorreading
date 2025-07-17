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
esp_err_t motorcontroller_manager_start_worker(void);

/**
 * @brief Wait for manager task completion
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, ESP_ERR_FAIL on error
 */
esp_err_t motorcontroller_manager_wait_completion(uint32_t timeout_ms);

// === Helper Functions (Internal) ===

/**
 * @brief Load motorcontroller package from RTC or initialize with defaults
 * @param pkg Output package
 * @param for_state Load package for specific state (LOWERING/RISING have separate estimates)
 * @return ESP_OK on success, ESP_FAIL if no valid data (will use defaults)
 */
esp_err_t load_or_init_motorcontroller_pkg(motorcontroller_pkg_t *pkg, state_t for_state);

/**
 * @brief Apply alpha-beta filtering to update speed estimate
 * @param prev_estimate Previous speed estimate (cm/s)
 * @param actual_time Actual time taken (seconds)
 * @param distance Distance traveled (cm)
 * @param alpha Alpha filter coefficient
 * @param beta Beta filter coefficient
 * @return Updated speed estimate (cm/s)
 */
uint16_t apply_manager_alpha_beta_filter(uint16_t prev_estimate, 
                                        int actual_time, 
                                        uint16_t distance,
                                        double alpha, 
                                        double beta);

/**
 * @brief Validate motorcontroller package before sending
 * @param pkg Package to validate
 * @return true if valid, false otherwise
 */
bool validate_motorcontroller_pkg(const motorcontroller_pkg_t *pkg);

#ifdef __cplusplus
}
#endif