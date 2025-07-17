#pragma once

#include "esp_err.h"
#include "buoye_structs.h"
#include "esp_sleep.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute motor work based on package
 * Handles movement, static polling, and timing
 * @param pkg Work package from manager
 * @param resp Response to fill with results
 * @return ESP_OK on success
 */
esp_err_t do_work(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp);


/**
 * @brief Check if wakeup reason is EXT1 (manual intervention)
 * @return true if woken up by EXT1
 */
bool wakeup_reason_is_ext1(void);

// === Helper Functions (Internal) ===

/**
 * @brief Initialize motor controller hardware
 * @return ESP_OK on success
 */
esp_err_t motorcontroller_worker_init_hardware(void);

/**
 * @brief Calculate pure movement time (excluding static stops)
 * @param total_time_ms Total operation time
 * @param static_points Array of static points
 * @param samples Samples per static point
 * @param static_poll_interval_s Interval per sample
 * @return Pure movement time in milliseconds
 */
uint32_t calculate_pure_movement_time(uint32_t total_time_ms,
                                     const uint16_t *static_points,
                                     uint16_t samples,
                                     uint16_t static_poll_interval_s);

/**
 * @brief Calculate new speed estimate based on actual performance
 * @param distance_cm Distance traveled
 * @param pure_movement_time_ms Time spent moving (excluding static stops)
 * @return Speed estimate in cm/s
 */
uint16_t calculate_new_speed_estimate(uint16_t distance_cm, uint32_t pure_movement_time_ms);

#ifdef __cplusplus
}
#endif