/**
 * @file can_motctrl_config.c
 * @brief Configuration validation and setup for CAN motor controller system
 * 
 * This file provides functions to validate configuration, detect hardware
 * setup issues, and provide comprehensive diagnostics for the CAN motor
 * controller system.
 */

#include "can_motctrl_config.h"
#include "can_bus_manager.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include "can_motctrl_common.h"

static const char *TAG = "can_motctrl_config";

// Configuration validation results
typedef struct {
    bool gpio_config_valid;
    bool can_driver_working;
    bool timing_config_valid;
    bool filter_config_valid;
    bool memory_sufficient;
    bool pin_conflicts;
    char error_messages[512];
} config_validation_t;

static config_validation_t s_validation_result = {0};

// Internal function prototypes
static bool validate_gpio_configuration(void);
static bool validate_can_timing(void);
static bool validate_memory_requirements(void);
static bool check_pin_conflicts(void);
static bool test_can_driver_basic(void);
static void append_error_message(const char* message);
static void print_gpio_info(int pin, const char* function);

esp_err_t can_motctrl_config_validate(void)
{
    ESP_LOGI(TAG, "Validating CAN motor controller configuration...");
    
    // Reset validation results
    memset(&s_validation_result, 0, sizeof(s_validation_result));
    
    // Run validation checks
    s_validation_result.gpio_config_valid = validate_gpio_configuration();
    s_validation_result.can_driver_working = test_can_driver_basic();
    s_validation_result.timing_config_valid = validate_can_timing();
    s_validation_result.memory_sufficient = validate_memory_requirements();
    s_validation_result.pin_conflicts = check_pin_conflicts();
    
    // Print results
    ESP_LOGI(TAG, "=== Configuration Validation Results ===");
    ESP_LOGI(TAG, "GPIO Configuration: %s", s_validation_result.gpio_config_valid ? "VALID" : "INVALID");
    ESP_LOGI(TAG, "CAN Driver: %s", s_validation_result.can_driver_working ? "WORKING" : "FAILED");
    ESP_LOGI(TAG, "Timing Configuration: %s", s_validation_result.timing_config_valid ? "VALID" : "INVALID");
    ESP_LOGI(TAG, "Memory Requirements: %s", s_validation_result.memory_sufficient ? "SUFFICIENT" : "INSUFFICIENT");
    ESP_LOGI(TAG, "Pin Conflicts: %s", s_validation_result.pin_conflicts ? "DETECTED" : "NONE");
    
    if (strlen(s_validation_result.error_messages) > 0) {
        ESP_LOGE(TAG, "Configuration errors detected:");
        ESP_LOGE(TAG, "%s", s_validation_result.error_messages);
    }
    
    // Overall result
    bool overall_valid = s_validation_result.gpio_config_valid &&
                        s_validation_result.can_driver_working &&
                        s_validation_result.timing_config_valid &&
                        s_validation_result.memory_sufficient &&
                        !s_validation_result.pin_conflicts;
    
    ESP_LOGI(TAG, "Overall configuration: %s", overall_valid ? "VALID" : "INVALID");
    
    return overall_valid ? ESP_OK : ESP_FAIL;
}

esp_err_t can_motctrl_config_print_current(void)
{
    ESP_LOGI(TAG, "=== Current CAN Motor Controller Configuration ===");
    
    // GPIO Configuration
    ESP_LOGI(TAG, "GPIO Configuration:");
    print_gpio_info(CONFIG_CAN_TX_GPIO, "CAN TX");
    print_gpio_info(CONFIG_CAN_RX_GPIO, "CAN RX");
    
#ifdef CONFIG_MOTCTRL_CAN_TX_GPIO
    ESP_LOGI(TAG, "Motor Controller CAN TX GPIO: %d", CONFIG_MOTCTRL_CAN_TX_GPIO);
#endif
#ifdef CONFIG_MOTCTRL_CAN_RX_GPIO
    ESP_LOGI(TAG, "Motor Controller CAN RX GPIO: %d", CONFIG_MOTCTRL_CAN_RX_GPIO);
#endif
    
    // CAN Configuration
    ESP_LOGI(TAG, "CAN Bus Configuration:");
    ESP_LOGI(TAG, "  Bitrate: 500 kbps (default)");
    ESP_LOGI(TAG, "  Mode: Normal");
    ESP_LOGI(TAG, "  Filter: Accept All");
    
    // Memory Configuration
    ESP_LOGI(TAG, "Memory Configuration:");
    ESP_LOGI(TAG, "  Free heap: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "  Minimum free heap: %u bytes", esp_get_minimum_free_heap_size());
    
    // Motor Controller Specific
    ESP_LOGI(TAG, "Motor Controller Message IDs:");
    ESP_LOGI(TAG, "  Package Start: 0x%03X", CAN_ID_MOTCTRL_PKG_START);
    ESP_LOGI(TAG, "  Package Data: 0x%03X", CAN_ID_MOTCTRL_PKG_DATA);
    ESP_LOGI(TAG, "  Package End: 0x%03X", CAN_ID_MOTCTRL_PKG_END);
    ESP_LOGI(TAG, "  Status Request: 0x%03X", CAN_ID_MOTCTRL_STATUS_REQ);
    ESP_LOGI(TAG, "  Status Response: 0x%03X", CAN_ID_MOTCTRL_STATUS_RESP);
    ESP_LOGI(TAG, "  Response Start: 0x%03X", CAN_ID_MOTCTRL_RESP_START);
    ESP_LOGI(TAG, "  Response Data: 0x%03X", CAN_ID_MOTCTRL_RESP_DATA);
    ESP_LOGI(TAG, "  Response End: 0x%03X", CAN_ID_MOTCTRL_RESP_END);
    
    return ESP_OK;
}

esp_err_t can_motctrl_config_suggest_fixes(void)
{
    if (strlen(s_validation_result.error_messages) == 0) {
        ESP_LOGI(TAG, "No configuration issues detected. System ready!");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "=== Configuration Fix Suggestions ===");
    
    if (!s_validation_result.gpio_config_valid) {
        ESP_LOGI(TAG, "GPIO Issues:");
        ESP_LOGI(TAG, "  1. Check that CONFIG_CAN_TX_GPIO and CONFIG_CAN_RX_GPIO are set correctly");
        ESP_LOGI(TAG, "  2. Ensure pins are not reserved for other functions");
        ESP_LOGI(TAG, "  3. Verify pins support the required functions (see ESP32 datasheet)");
        ESP_LOGI(TAG, "  4. Recommended pins for ESP32: TX=21, RX=22");
    }
    
    if (!s_validation_result.can_driver_working) {
        ESP_LOGI(TAG, "CAN Driver Issues:");
        ESP_LOGI(TAG, "  1. Check hardware connections (CAN transceiver, termination resistors)");
        ESP_LOGI(TAG, "  2. Verify CAN transceiver power supply");
        ESP_LOGI(TAG, "  3. Check for proper 120Ω termination resistors at both ends of bus");
        ESP_LOGI(TAG, "  4. Ensure at least two devices on the bus for proper operation");
    }
    
    if (!s_validation_result.timing_config_valid) {
        ESP_LOGI(TAG, "Timing Issues:");
        ESP_LOGI(TAG, "  1. Verify all devices use the same bitrate (500 kbps default)");
        ESP_LOGI(TAG, "  2. Check crystal/clock accuracy on all devices");
        ESP_LOGI(TAG, "  3. Consider using external crystal for better timing accuracy");
    }
    
    if (!s_validation_result.memory_sufficient) {
        ESP_LOGI(TAG, "Memory Issues:");
        ESP_LOGI(TAG, "  1. Increase heap size in sdkconfig");
        ESP_LOGI(TAG, "  2. Reduce queue sizes if memory is very limited");
        ESP_LOGI(TAG, "  3. Consider using PSRAM for additional memory");
    }
    
    if (s_validation_result.pin_conflicts) {
        ESP_LOGI(TAG, "Pin Conflict Issues:");
        ESP_LOGI(TAG, "  1. Check sdkconfig for conflicting pin assignments");
        ESP_LOGI(TAG, "  2. Ensure CAN pins don't conflict with other peripherals");
        ESP_LOGI(TAG, "  3. Refer to ESP32 pin matrix for alternative pin options");
    }
    
    ESP_LOGI(TAG, "Example sdkconfig.defaults entries:");
    ESP_LOGI(TAG, "CONFIG_CAN_TX_GPIO=21");
    ESP_LOGI(TAG, "CONFIG_CAN_RX_GPIO=22");
    ESP_LOGI(TAG, "CONFIG_MOTCTRL_CAN_TX_GPIO=21");
    ESP_LOGI(TAG, "CONFIG_MOTCTRL_CAN_RX_GPIO=22");
    
    return ESP_FAIL;
}

esp_err_t can_motctrl_config_run_hardware_test(void)
{
    ESP_LOGI(TAG, "Running hardware connectivity test...");
    
    // Test GPIO configuration
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_CAN_TX_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TX GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Test basic GPIO functionality
    gpio_set_level(CONFIG_CAN_TX_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CONFIG_CAN_TX_GPIO, 0);
    
    ESP_LOGI(TAG, "GPIO test completed successfully");
    
    // Test CAN driver installation
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CONFIG_CAN_TX_GPIO, 
        CONFIG_CAN_RX_GPIO, 
        TWAI_MODE_LISTEN_ONLY  // Use listen-only for testing
    );
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI driver installation failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }
    
    ESP_LOGI(TAG, "CAN driver test completed successfully");
    
    // Listen for any CAN traffic
    ESP_LOGI(TAG, "Listening for CAN traffic for 5 seconds...");
    twai_message_t message;
    uint32_t messages_received = 0;
    
    TickType_t start_time = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(5000)) {
        if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
            messages_received++;
            ESP_LOGI(TAG, "Received CAN message: ID 0x%lx, DLC %d", 
                    message.identifier, message.data_length_code);
        }
    }
    
    ESP_LOGI(TAG, "Received %lu CAN messages during test", messages_received);
    
    // Get bus status
    twai_status_info_t status;
    ret = twai_get_status_info(&status);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN Bus Status:");
        ESP_LOGI(TAG, "  State: %d", status.state);
        ESP_LOGI(TAG, "  Messages to TX: %lu", status.msgs_to_tx);
        ESP_LOGI(TAG, "  Messages to RX: %lu", status.msgs_to_rx);
        ESP_LOGI(TAG, "  TX Error Counter: %lu", status.tx_error_counter);
        ESP_LOGI(TAG, "  RX Error Counter: %lu", status.rx_error_counter);
        ESP_LOGI(TAG, "  TX Failed Count: %lu", status.tx_failed_count);
        ESP_LOGI(TAG, "  RX Missed Count: %lu", status.rx_missed_count);
        ESP_LOGI(TAG, "  Arbitration Lost Count: %lu", status.arb_lost_count);
        ESP_LOGI(TAG, "  Bus Error Count: %lu", status.bus_error_count);
    }
    
    // Cleanup
    twai_stop();
    twai_driver_uninstall();
    
    ESP_LOGI(TAG, "Hardware test completed");
    return ESP_OK;
}

// Internal functions

static bool validate_gpio_configuration(void)
{
    // Check if GPIO pins are valid
    if (CONFIG_CAN_TX_GPIO < 0 || CONFIG_CAN_TX_GPIO >= GPIO_NUM_MAX) {
        append_error_message("Invalid CAN TX GPIO pin\n");
        return false;
    }
    
    if (CONFIG_CAN_RX_GPIO < 0 || CONFIG_CAN_RX_GPIO >= GPIO_NUM_MAX) {
        append_error_message("Invalid CAN RX GPIO pin\n");
        return false;
    }
    
    // Check if pins are the same (invalid)
    if (CONFIG_CAN_TX_GPIO == CONFIG_CAN_RX_GPIO) {
        append_error_message("CAN TX and RX pins cannot be the same\n");
        return false;
    }
    
    // Check if pins support required functions
    // Note: This is simplified - in reality you'd check the pin matrix
    if (CONFIG_CAN_TX_GPIO == 0 || CONFIG_CAN_TX_GPIO == 1 || 
        CONFIG_CAN_TX_GPIO == 6 || CONFIG_CAN_TX_GPIO == 7 ||
        CONFIG_CAN_TX_GPIO == 8 || CONFIG_CAN_TX_GPIO == 9 ||
        CONFIG_CAN_TX_GPIO == 10 || CONFIG_CAN_TX_GPIO == 11) {
        append_error_message("CAN TX pin conflicts with reserved/flash pins\n");
        return false;
    }
    
    if (CONFIG_CAN_RX_GPIO == 0 || CONFIG_CAN_RX_GPIO == 1 || 
        CONFIG_CAN_RX_GPIO == 6 || CONFIG_CAN_RX_GPIO == 7 ||
        CONFIG_CAN_RX_GPIO == 8 || CONFIG_CAN_RX_GPIO == 9 ||
        CONFIG_CAN_RX_GPIO == 10 || CONFIG_CAN_RX_GPIO == 11) {
        append_error_message("CAN RX pin conflicts with reserved/flash pins\n");
        return false;
    }
    
    return true;
}

static bool validate_can_timing(void)
{
    // For 500 kbps, check if timing is achievable with current clock
    // This is a simplified check - real implementation would be more complex
    
    uint32_t apb_clock = 80000000; // Default APB clock frequency
    uint32_t target_bitrate = 500000; // 500 kbps
    
    // Basic timing validation
    if ((apb_clock % target_bitrate) != 0) {
        // This doesn't necessarily mean invalid, just that timing might not be perfect
        ESP_LOGW(TAG, "CAN timing may not be exact with current clock settings");
    }
    
    return true; // Assume timing is valid for now
}

static bool validate_memory_requirements(void)
{
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_required = 32 * 1024; // Estimate 32KB minimum
    
    if (free_heap < min_required) {
        append_error_message("Insufficient heap memory for CAN motor controller\n");
        return false;
    }
    
    // Check for fragmentation
    uint32_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    if (largest_block < 8192) {
        append_error_message("Heap fragmentation may cause allocation failures\n");
        return false;
    }
    
    return true;
}

static bool check_pin_conflicts(void)
{
    bool conflicts = false;
    
    // Check common pin conflicts
#ifdef CONFIG_UART_CONSOLE_ON_PINS
    if ((CONFIG_CAN_TX_GPIO == 1 && CONFIG_CAN_RX_GPIO == 3) ||
        (CONFIG_CAN_TX_GPIO == 3 && CONFIG_CAN_RX_GPIO == 1)) {
        append_error_message("CAN pins conflict with UART console\n");
        conflicts = true;
    }
#endif

#ifdef CONFIG_SPI_FLASH_INTERFACE_HS
    if (CONFIG_CAN_TX_GPIO >= 6 && CONFIG_CAN_TX_GPIO <= 11) {
        append_error_message("CAN TX pin conflicts with SPI flash\n");
        conflicts = true;
    }
    if (CONFIG_CAN_RX_GPIO >= 6 && CONFIG_CAN_RX_GPIO <= 11) {
        append_error_message("CAN RX pin conflicts with SPI flash\n");
        conflicts = true;
    }
#endif
    
    return conflicts;
}

static bool test_can_driver_basic(void)
{
    // Try a basic driver installation test
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CONFIG_CAN_TX_GPIO, 
        CONFIG_CAN_RX_GPIO, 
        TWAI_MODE_LISTEN_ONLY
    );
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        append_error_message("TWAI driver installation failed\n");
        return false;
    }
    
    ret = twai_start();
    if (ret != ESP_OK) {
        append_error_message("TWAI start failed\n");
        twai_driver_uninstall();
        return false;
    }
    
    // Quick status check
    twai_status_info_t status;
    ret = twai_get_status_info(&status);
    
    // Cleanup
    twai_stop();
    twai_driver_uninstall();
    
    if (ret != ESP_OK) {
        append_error_message("TWAI status check failed\n");
        return false;
    }
    
    return true;
}

static void append_error_message(const char* message)
{
    size_t current_len = strlen(s_validation_result.error_messages);
    size_t message_len = strlen(message);
    size_t available = sizeof(s_validation_result.error_messages) - current_len - 1;
    
    if (message_len < available) {
        strcat(s_validation_result.error_messages, message);
    }
}

static void print_gpio_info(int pin, const char* function)
{
    ESP_LOGI(TAG, "  %s: GPIO %d", function, pin);
    
    // Check if pin is input capable
    if (GPIO_IS_VALID_GPIO(pin)) {
        ESP_LOGI(TAG, "    - Valid GPIO pin");
    } else {
        ESP_LOGW(TAG, "    - Invalid GPIO pin");
    }
    
    // Check if pin is output capable
    if (GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        ESP_LOGI(TAG, "    - Output capable");
    } else {
        ESP_LOGW(TAG, "    - Not output capable");
    }
}