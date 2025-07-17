#pragma once

#include "esp_err.h"
#include "buoye_structs.h"


#ifdef __cplusplus
extern "C" {
#endif


// Main winch control functions
/**
 * @brief Initialize winch controller (creates queues, event groups)
 * @return ESP_OK on success
 */
esp_err_t winch_controller_init(void);

/**
 * @brief Deinitialize winch controller (cleanup resources)
 * @return ESP_OK on success
 */
esp_err_t winch_controller_deinit(void);

/**
 * @brief Execute complete winch operation based on package
 * @param pkg Package containing operation parameters
 * @param resp Response structure to fill with results
 * @return ESP_OK on success
 */
esp_err_t winch_execute_operation(const motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp);

/**
 * @brief Move winch to home position (for INIT operations)
 * @return ESP_OK when home position reached
 */
esp_err_t winch_go_to_home_position(void);



#ifdef __cplusplus
}
#endif