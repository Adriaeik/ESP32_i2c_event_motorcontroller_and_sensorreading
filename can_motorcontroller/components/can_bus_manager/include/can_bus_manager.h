#ifndef CAN_BUS_MANAGER_H
#define CAN_BUS_MANAGER_H

#include "esp_err.h"
#include "driver/twai.h"  // Updated to TWAI driver
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "freertos/task.h"

// Bus statistics structure
typedef struct {
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t errors_count;
    uint32_t bus_off_count;
} can_bus_stats_t;

// Priority levels for message transmission
typedef enum {
    CAN_PRIORITY_LOW = 0,
    CAN_PRIORITY_NORMAL,
    CAN_PRIORITY_HIGH,
    CAN_PRIORITY_CRITICAL,
    CAN_PRIORITY_MAX
} can_priority_t;

#ifdef __cplusplus
extern "C" {
#endif
typedef union {
    twai_message_t twai;
    struct {
        uint32_t identifier;
        uint8_t data_length_code;
        uint8_t flags;
        uint8_t reserved;
        uint8_t data[8];
    };
} can_message_t;

enum {
    CAN_MSG_FLAG_NONE = TWAI_MSG_FLAG_NONE,
    CAN_MSG_FLAG_EXTD = TWAI_MSG_FLAG_EXTD,
    CAN_MSG_FLAG_RTR = TWAI_MSG_FLAG_RTR,
    CAN_MSG_FLAG_SS = TWAI_MSG_FLAG_SS,
    CAN_MSG_FLAG_SELF = TWAI_MSG_FLAG_SELF,
    CAN_MSG_FLAG_DLC_NON_COMP = TWAI_MSG_FLAG_DLC_NON_COMP
};
/**
 * @brief Initialize the CAN bus manager
 * @return ESP_OK on success
 */
esp_err_t can_bus_manager_init(void);

/**
 * @brief Deinitialize the CAN bus manager
 * @return ESP_OK on success
 */
esp_err_t can_bus_manager_deinit(void);

/**
 * @brief Send a CAN message with priority (thread-safe)
 * @param message CAN message to send
 * @param priority Message priority
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t can_bus_send_message(const can_message_t *message, can_priority_t priority, uint32_t timeout_ms);

/**
 * @brief Receive a CAN message (thread-safe)
 * @param message Buffer for received message
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t can_bus_receive_message(can_message_t *message, uint32_t timeout_ms);

/**
 * @brief Get bus statistics
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success
 */
esp_err_t can_bus_get_stats(can_bus_stats_t *stats);

/**
 * @brief Reset bus statistics
 * @return ESP_OK on success
 */
esp_err_t can_bus_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_BUS_MANAGER_H