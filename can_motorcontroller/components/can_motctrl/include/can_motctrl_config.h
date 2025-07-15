#ifndef CAN_MOTCTRL_CONFIG_H
#define CAN_MOTCTRL_CONFIG_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief Validate complete CAN motor controller configuration
 * 
 * Performs comprehensive validation of:
 * - GPIO pin configuration
 * - CAN driver functionality
 * - Timing configuration
 * - Memory requirements
 * - Pin conflict detection
 * 
 * @return ESP_OK if configuration is valid, ESP_FAIL if issues detected
 */
esp_err_t can_motctrl_config_validate(void);

/**
 * @brief Print current configuration settings
 * 
 * Displays all current configuration parameters including:
 * - GPIO assignments
 * - CAN bus settings
 * - Message ID assignments
 * - Memory usage
 * 
 * @return ESP_OK on success
 */
esp_err_t can_motctrl_config_print_current(void);

/**
 * @brief Suggest fixes for configuration issues
 * 
 * Analyzes detected configuration problems and provides
 * specific suggestions for resolving them, including:
 * - Recommended pin assignments
 * - Configuration file examples
 * - Hardware setup guidance
 * 
 * @return ESP_OK if no issues, ESP_FAIL if fixes needed
 */
esp_err_t can_motctrl_config_suggest_fixes(void);

/**
 * @brief Run hardware connectivity test
 * 
 * Performs low-level hardware testing including:
 * - GPIO functionality test
 * - CAN driver installation test
 * - Bus traffic monitoring
 * - Error counter analysis
 * 
 * @return ESP_OK if hardware is working, error code otherwise
 */
esp_err_t can_motctrl_config_run_hardware_test(void);

/**
 * @brief Get recommended configuration for specific hardware
 * 
 * Provides board-specific recommendations for common ESP32 variants.
 * 
 * @param board_type Board identifier (e.g., "ESP32-DevKitC", "ESP32-WROVER")
 * @return ESP_OK if recommendations available
 */
esp_err_t can_motctrl_config_get_recommendations(const char* board_type);

// Configuration defaults and limits
#define CAN_MOTCTRL_DEFAULT_TX_PIN      21
#define CAN_MOTCTRL_DEFAULT_RX_PIN      22
#define CAN_MOTCTRL_DEFAULT_BITRATE     500000
#define CAN_MOTCTRL_MIN_HEAP_SIZE       32768
#define CAN_MOTCTRL_RECOMMENDED_HEAP    65536

// Configuration validation flags
typedef enum {
    CAN_CONFIG_CHECK_GPIO       = BIT0,
    CAN_CONFIG_CHECK_TIMING     = BIT1,
    CAN_CONFIG_CHECK_MEMORY     = BIT2,
    CAN_CONFIG_CHECK_CONFLICTS  = BIT3,
    CAN_CONFIG_CHECK_HARDWARE   = BIT4,
    CAN_CONFIG_CHECK_ALL        = 0xFF
} can_config_check_flags_t;

/**
 * @brief Run selective configuration validation
 * 
 * Allows selective validation of specific configuration aspects.
 * 
 * @param check_flags Bitmask of checks to perform
 * @return ESP_OK if selected checks pass, ESP_FAIL otherwise
 */
esp_err_t can_motctrl_config_validate_selective(can_config_check_flags_t check_flags);

#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_CONFIG_H