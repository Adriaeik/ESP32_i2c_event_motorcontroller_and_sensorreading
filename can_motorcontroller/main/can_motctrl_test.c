/**
 * @file can_motctrl_test.c
 * @brief Test functions for CAN motor controller (main folder version)
 * 
 * These test functions are in main folder to avoid circular dependencies
 * between manager and worker components.
 */

#include "can_motctrl_common.h"
#include "can_motctrl_config.h"
#include "can_motctrl_debug.h"
#include "can_serde_helper.h"
#include "can_bus_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TEST";

// Internal test functions
static esp_err_t test_configuration(void);
static esp_err_t test_serialization(void);
static esp_err_t test_bus_manager(void);
static esp_err_t test_common_functions(void);

esp_err_t run_can_motctrl_tests(void)
{
    ESP_LOGI(TAG, "=== Starting CAN Motor Controller Tests ===");
    
    esp_err_t overall_result = ESP_OK;
    int tests_passed = 0;
    int tests_failed = 0;
    
    // Test configuration
    ESP_LOGI(TAG, "Running configuration tests...");
    esp_err_t result = test_configuration();
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "✅ Configuration test PASSED");
        tests_passed++;
    } else {
        ESP_LOGE(TAG, "❌ Configuration test FAILED");
        tests_failed++;
        overall_result = ESP_FAIL;
    }
    
    // Test common functions
    ESP_LOGI(TAG, "Running common function tests...");
    result = test_common_functions();
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "✅ Common functions test PASSED");
        tests_passed++;
    } else {
        ESP_LOGE(TAG, "❌ Common functions test FAILED");
        tests_failed++;
        overall_result = ESP_FAIL;
    }
    
    // Test serialization
    ESP_LOGI(TAG, "Running serialization tests...");
    result = test_serialization();
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "✅ Serialization test PASSED");
        tests_passed++;
    } else {
        ESP_LOGE(TAG, "❌ Serialization test FAILED");
        tests_failed++;
        overall_result = ESP_FAIL;
    }
    
    // Test bus manager
    ESP_LOGI(TAG, "Running bus manager tests...");
    result = test_bus_manager();
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "✅ Bus manager test PASSED");
        tests_passed++;
    } else {
        ESP_LOGE(TAG, "❌ Bus manager test FAILED");
        tests_failed++;
        overall_result = ESP_FAIL;
    }
    
    // Summary
    ESP_LOGI(TAG, "=== Test Results ===");
    ESP_LOGI(TAG, "Tests passed: %d", tests_passed);
    ESP_LOGI(TAG, "Tests failed: %d", tests_failed);
    ESP_LOGI(TAG, "Overall result: %s", overall_result == ESP_OK ? "PASS" : "FAIL");
    
    return overall_result;
}

static esp_err_t test_configuration(void)
{
    // Test configuration validation
    esp_err_t ret = can_motctrl_config_validate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration validation failed");
        can_motctrl_config_suggest_fixes();
        return ret;
    }
    
    // Print current configuration
    can_motctrl_config_print_current();
    
    return ESP_OK;
}

static esp_err_t test_common_functions(void)
{
    // Test package initialization
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    
    if (pkg.STATE != INIT) {
        ESP_LOGE(TAG, "Package init failed: STATE = %d", pkg.STATE);
        return ESP_FAIL;
    }
    
    if (pkg.prev_estimated_cm_per_s != 50) {
        ESP_LOGE(TAG, "Package init failed: speed = %d", pkg.prev_estimated_cm_per_s);
        return ESP_FAIL;
    }
    
    // Test response initialization  
    motorcontroller_response_t resp;
    motorcontroller_response_init_default(&resp);
    
    if (resp.STATE != INIT) {
        ESP_LOGE(TAG, "Response init failed: STATE = %d", resp.STATE);
        return ESP_FAIL;
    }
    
    if (resp.result != ESP_OK) {
        ESP_LOGE(TAG, "Response init failed: result = %d", resp.result);
        return ESP_FAIL;
    }
    
    // Test validation
    pkg.STATE = LOWERING;
    pkg.end_depth = 100;
    pkg.samples = 5;
    
    if (!is_motorcontroller_pkg_valid(&pkg)) {
        ESP_LOGE(TAG, "Package validation failed for valid package");
        return ESP_FAIL;
    }
    
    // Test timeout calculation
    int timeout = calculate_operation_timeout(LOWERING, 50, 10, 100, NULL, 1, 5);
    if (timeout < CAN_MOTCTRL_MIN_TIMEOUT || timeout > CAN_MOTCTRL_MAX_TIMEOUT) {
        ESP_LOGE(TAG, "Timeout calculation failed: %d", timeout);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Common functions test completed successfully");
    return ESP_OK;
}

static esp_err_t test_serialization(void)
{
    // Create test package
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    
    pkg.STATE = LOWERING;
    pkg.end_depth = 200;
    pkg.samples = 3;
    pkg.static_points[0] = 50;
    pkg.static_points[1] = 100;
    pkg.static_points[2] = 0;  // End marker
    
    // Test serialization
    can_fragment_list_t frag_list;
    esp_err_t ret = can_serialize_pkg(&pkg, &frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Package serialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Package serialized into %d fragments", frag_list.count);
    
    // Test deserialization
    motorcontroller_pkg_t reconstructed_pkg;
    ret = can_deserialize_pkg(&frag_list, &reconstructed_pkg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Package deserialization failed: %s", esp_err_to_name(ret));
        can_fragment_list_free(&frag_list);
        return ret;
    }
    
    // Verify reconstruction
    if (reconstructed_pkg.STATE != pkg.STATE ||
        reconstructed_pkg.end_depth != pkg.end_depth ||
        reconstructed_pkg.samples != pkg.samples) {
        ESP_LOGE(TAG, "Package reconstruction failed");
        can_fragment_list_free(&frag_list);
        return ESP_FAIL;
    }
    
    can_fragment_list_free(&frag_list);
    
    // Test response serialization
    motorcontroller_response_t resp;
    motorcontroller_response_init_default(&resp);
    resp.STATE = LOWERING;
    resp.working_time = 30;
    resp.estimated_cm_per_s = 45;
    
    can_fragment_list_t resp_frag_list;
    ret = can_serialize_resp(&resp, &resp_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Response serialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    motorcontroller_response_t reconstructed_resp;
    ret = can_deserialize_resp(&resp_frag_list, &reconstructed_resp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Response deserialization failed: %s", esp_err_to_name(ret));
        can_fragment_list_free(&resp_frag_list);
        return ret;
    }
    
    can_fragment_list_free(&resp_frag_list);
    
    ESP_LOGI(TAG, "Serialization test completed successfully");
    return ESP_OK;
}

static esp_err_t test_bus_manager(void)
{
    // Initialize bus manager
    esp_err_t ret = can_bus_manager_init(NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bus manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Test message sending
    twai_message_t test_msg = {
        .identifier = 0x123,
        .flags = TWAI_MSG_FLAG_NONE,
        .data_length_code = 8,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x01}
    };
    
    ret = can_bus_manager_send_message(&test_msg, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Test message send failed: %s (this may be normal if no other device)", 
                esp_err_to_name(ret));
        // Don't fail the test for this - it's normal with no other devices
    }
    
    // Get statistics
    can_bus_stats_t stats;
    can_bus_manager_get_extended_stats(&stats);
    ESP_LOGI(TAG, "Bus stats - Sent: %lu, Errors: %lu", stats.messages_sent, stats.tx_errors);
    
    // Cleanup
    can_bus_manager_deinit();
    
    ESP_LOGI(TAG, "Bus manager test completed");
    return ESP_OK;
}