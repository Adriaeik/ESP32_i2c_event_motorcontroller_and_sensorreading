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

typedef enum {
    CAN_MTOTCTRL_MANAGER,    // Subscribe to response channels
    CAN_MTOTCTRL_WORKER      // Subscribe to package channels
} can_motctrl_role_t;


/**
 * @brief Subscribe to a predefined set of CAN IDs
 * @param set Which subscription set to use
 * @return ESP_OK on success
 */
esp_err_t can_subscribe_set(const can_motctrl_role_t role);

/**
 * @brief Unsubscribe from a predefined set of CAN IDs
 * @param set Which subscription set to unsubscribe from
 * @return ESP_OK on success
 */
esp_err_t can_unsubscribe_set(const can_motctrl_role_t role);

/**
 * @brief Send a fragmented message list
 * @param role
 * @param frag_list Fragment list to send
 * @return ESP_OK on success
 */
esp_err_t send_fragment_list_simple_ack(const can_motctrl_role_t role, const can_fragment_list_t *frag_list, uint32_t timeout_ms);


/**
 * @brief Receive a fragmented message list using subscription system
 * @param role
 * @param frag_list Output fragment list
 * @param timeout_ms Timeout for each message
 * @return ESP_OK on success
 * @note You must subscribe to the IDs before calling this function
 */
esp_err_t receive_fragment_list_simple_ack(const can_motctrl_role_t role, const can_fragment_list_t *frag_list, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_H