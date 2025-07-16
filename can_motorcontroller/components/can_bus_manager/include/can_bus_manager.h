#pragma once
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef twai_message_t can_message_t;

enum {
    CAN_MSG_FLAG_NONE = TWAI_MSG_FLAG_NONE,
    CAN_MSG_FLAG_EXTD = TWAI_MSG_FLAG_EXTD,
    CAN_MSG_FLAG_RTR = TWAI_MSG_FLAG_RTR,
    CAN_MSG_FLAG_SS = TWAI_MSG_FLAG_SS,
    CAN_MSG_FLAG_SELF = TWAI_MSG_FLAG_SELF,
    CAN_MSG_FLAG_DLC_NON_COMP = TWAI_MSG_FLAG_DLC_NON_COMP
};

typedef enum {
    CAN_PRIORITY_LOW = 0,
    CAN_PRIORITY_NORMAL,
    CAN_PRIORITY_HIGH,
    CAN_PRIORITY_CRITICAL,
    CAN_PRIORITY_MAX
} can_priority_t;

typedef struct {
    uint32_t can_id;
    uint8_t queue_size;
} can_subscription_spec_t;

/**
 * @brief Initialize CAN bus manager
 * @return ESP_OK on success
 */
esp_err_t can_bus_manager_init(void);

/**
 * @brief Deinitialize CAN bus manager
 * @return ESP_OK on success
 */
esp_err_t can_bus_manager_deinit(void);

/**
 * @brief Send a CAN message
 * @param message Message to send
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t can_bus_send_message(const can_message_t *message, uint32_t timeout_ms);

/**
 * @brief Subscribe to messages with specific CAN ID
 * @param can_id CAN ID to subscribe to
 * @param queue_size Size of the message queue
 * @return ESP_OK on success
 */
esp_err_t can_bus_subscribe_id(uint32_t can_id, uint8_t queue_size);

/**
 * @brief Subscribe to multiple CAN IDs in one call
 * @param specs Array of CAN ID and queue size specifications
 * @param count Number of specifications
 * @return ESP_OK on success
 */
esp_err_t can_subscribe_multiple(const can_subscription_spec_t *specs, size_t count);

/**
 * @brief Unsubscribe from a CAN ID
 * @param can_id CAN ID to unsubscribe from
 * @return ESP_OK on success
 */
esp_err_t can_bus_unsubscribe_id(uint32_t can_id);

/**
 * @brief Unsubscribe from multiple CAN IDs
 * @param can_ids Array of CAN IDs to unsubscribe from
 * @param count Number of IDs
 * @return ESP_OK on success
 */
esp_err_t can_unsubscribe_multiple(const uint32_t *can_ids, size_t count);

/**
 * @brief Wait for a message on a subscribed CAN ID
 * @param can_id CAN ID to wait for
 * @param msg Output message (stack-allocated)
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t can_bus_wait_for_message(uint32_t can_id, can_message_t *msg, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif