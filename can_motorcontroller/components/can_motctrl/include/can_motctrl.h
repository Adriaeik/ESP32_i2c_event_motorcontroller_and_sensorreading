#ifndef MOTCTRL_H
#define MOTCTRL_H

#include "can_motctrl_common.h"
#include "can_serde_helper.h"
#include "can_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send a fragmented message list
 * @param start_id CAN ID for start frame
 * @param data_id CAN ID for data fragments
 * @param end_id CAN ID for end frame
 * @param frag_list Fragment list to send
 * @return ESP_OK on success
 */
esp_err_t send_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                             const can_fragment_list_t *frag_list);

/**
 * @brief Receive a fragmented message list using subscription system
 * @param start_id CAN ID for start frame
 * @param data_id CAN ID for data fragments  
 * @param end_id CAN ID for end frame
 * @param frag_list Output fragment list
 * @param timeout_ms Timeout for each message
 * @return ESP_OK on success
 * @note You must subscribe to the IDs before calling this function
 */
esp_err_t receive_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                can_fragment_list_t *frag_list, uint32_t timeout_ms);

/**
 * @brief Subscribe to all IDs needed for fragmented communication
 * @param start_id CAN ID for start frame
 * @param data_id CAN ID for data fragments
 * @param end_id CAN ID for end frame
 * @param start_queue_size Queue size for start frame (typically 1-2)
 * @param data_queue_size Queue size for data fragments (should match expected fragment count)
 * @param end_queue_size Queue size for end frame (typically 1-2)
 * @return ESP_OK on success
 */
esp_err_t subscribe_fragment_ids(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                 uint8_t start_queue_size, uint8_t data_queue_size, uint8_t end_queue_size);

/**
 * @brief Unsubscribe from fragment IDs
 * @param start_id CAN ID for start frame
 * @param data_id CAN ID for data fragments
 * @param end_id CAN ID for end frame
 * @return ESP_OK on success
 */
esp_err_t unsubscribe_fragment_ids(uint32_t start_id, uint32_t data_id, uint32_t end_id);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_H