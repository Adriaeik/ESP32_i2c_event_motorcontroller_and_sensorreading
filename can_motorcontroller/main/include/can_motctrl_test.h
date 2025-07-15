#ifndef CAN_MOTCTRL_TEST_H
#define CAN_MOTCTRL_TEST_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations for your motor controller types
typedef struct motorcontroller_pkg motorcontroller_pkg_t;
typedef struct motorcontroller_response motorcontroller_response_t;

// Test configuration constants
#define MAX_POINTS 64  // Should match your actual MAX_POINTS definition

// State and poll type enums (should match your actual definitions)
typedef enum {
    IDLE,
    LOWERING,
    RISING
} state_t;

typedef enum {
    STATIC_DEPTH,
    CONTINUOUS
} POLL_TYPE;

/**
 * @brief Initialize the CAN motor controller test framework
 * 
 * Sets up test infrastructure including mutexes and statistics tracking.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_test_init(void);

/**
 * @brief Deinitialize the test framework
 * 
 * Cleans up test resources and prints final statistics.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_motctrl_test_deinit(void);

/**
 * @brief Run all CAN motor controller tests
 * 
 * Executes a comprehensive test suite including:
 * - Basic communication tests
 * - Fragmentation and reassembly tests
 * - Error recovery tests
 * - Concurrent operation tests
 * - Performance benchmarks
 * - Stress tests
 * 
 * @return ESP_OK if all tests pass, ESP_FAIL if any test fails
 */
esp_err_t can_motctrl_run_all_tests(void);

/**
 * @brief Test CAN serialization and deserialization
 * 
 * Tests the fragmentation and reassembly of motor controller packages
 * and responses using various data patterns and sizes.
 * 
 * @return ESP_OK if all serialization tests pass, ESP_FAIL otherwise
 */
esp_err_t can_motctrl_test_serialization(void);

/**
 * @brief Test CAN bus manager functionality
 * 
 * Tests the low-level CAN bus manager including:
 * - Message sending (synchronous and asynchronous)
 * - Priority handling
 * - Error recovery
 * - Statistics tracking
 * 
 * @return ESP_OK if bus manager tests pass, ESP_FAIL otherwise
 */
esp_err_t can_motctrl_test_bus_manager(void);

/**
 * @brief Run performance benchmark tests
 * 
 * Measures and reports performance metrics for:
 * - Serialization speed
 * - Deserialization speed
 * - End-to-end communication latency
 * - Throughput under various loads
 * 
 * @return ESP_OK on successful benchmark completion
 */
esp_err_t can_motctrl_test_performance(void);

/**
 * @brief Run stress tests
 * 
 * Executes high-load tests to validate system stability:
 * - Continuous operation for extended periods
 * - Memory leak detection
 * - Error rate monitoring
 * - Resource exhaustion scenarios
 * 
 * @return ESP_OK if system remains stable under stress
 */
esp_err_t can_motctrl_test_stress(void);

/**
 * @brief Test error injection and recovery
 * 
 * Validates error handling by injecting various failure modes:
 * - Corrupted CRC values
 * - Missing fragments
 * - Out-of-order messages
 * - Timeout scenarios
 * - Bus-off conditions
 * 
 * @return ESP_OK if error recovery works correctly
 */
esp_err_t can_motctrl_test_error_injection(void);

/**
 * @brief Test concurrent operations
 * 
 * Validates thread safety and concurrent access:
 * - Multiple simultaneous send operations
 * - Concurrent serialization/deserialization
 * - Resource contention scenarios
 * - Deadlock detection
 * 
 * @return ESP_OK if concurrent operations work correctly
 */
esp_err_t can_motctrl_test_concurrency(void);

// Functions that need to be implemented by your motor controller code:

/**
 * @brief Initialize motor controller package with default values
 * @param pkg Package to initialize
 */
void motorcontroller_pkg_init_default(motorcontroller_pkg_t *pkg);

/**
 * @brief Initialize motor controller response with default values
 * @param resp Response to initialize
 */
void motorcontroller_response_init_default(motorcontroller_response_t *resp);

#ifdef __cplusplus
}
#endif

#endif // CAN_MOTCTRL_TEST_H