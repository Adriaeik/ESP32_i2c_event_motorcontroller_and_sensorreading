#ifndef CAN_SERDE_HELPER_H
#define CAN_SERDE_HELPER_H

#include "esp_err.h"
#include "driver/twai.h"
#include <stdint.h>
#include <stddef.h>
#include "buoye_structs.h"
#include "can_motctrl_common.h"
#include "can_bus_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

// Size limits
#define MAX_PKG_WIRE_SIZE     1024  // Adjust based on your actual package size
#define MAX_RESP_WIRE_SIZE    256   // Adjust based on your actual response size

// Fragment list structure for CAN messages
typedef struct {
    can_message_t  *fragments;  // Array of CAN message fragments
    uint16_t count;            // Number of fragments
} can_fragment_list_t;

/**
 * @brief Serialize motor controller package into CAN fragments
 * 
 * This function takes a motor controller package, serializes it using your
 * existing serialize_pkg() function, then fragments it into CAN-sized messages.
 * 
 * @param pkg Package to serialize
 * @param frag_list Output fragment list (caller must free with can_fragment_list_free)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_serialize_pkg(const motorcontroller_pkg_t *pkg, can_fragment_list_t *frag_list);

/**
 * @brief Deserialize motor controller package from CAN fragments
 * 
 * This function reassembles CAN fragments back into a complete serialized package,
 * then deserializes it using your existing deserialize_pkg() function.
 * 
 * @param frag_list Input fragment list
 * @param pkg Output package
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_deserialize_pkg(const can_fragment_list_t *frag_list, motorcontroller_pkg_t *pkg);

/**
 * @brief Serialize motor controller response into CAN fragments
 * 
 * @param resp Response to serialize
 * @param frag_list Output fragment list (caller must free with can_fragment_list_free)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_serialize_resp(const motorcontroller_response_t *resp, can_fragment_list_t *frag_list);

/**
 * @brief Deserialize motor controller response from CAN fragments
 * 
 * @param frag_list Input fragment list
 * @param resp Output response
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_deserialize_resp(const can_fragment_list_t *frag_list, motorcontroller_response_t *resp);

/**
 * @brief Free fragment list memory
 * 
 * @param frag_list Fragment list to free
 */
void can_fragment_list_free(can_fragment_list_t *frag_list);

/**
 * @brief Create start frame for fragmented message
 * 
 * @param total_fragments Total number of fragments to expect
 * @param message_id CAN message ID for start frame
 * @param start_msg Output start message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_create_start_frame(uint16_t total_fragments, uint32_t message_id, twai_message_t *start_msg);

/**
 * @brief Create end frame for fragmented message
 * 
 * @param message_id CAN message ID for end frame
 * @param end_msg Output end message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_create_end_frame(uint32_t message_id, twai_message_t *end_msg);


#ifdef __cplusplus
}
#endif

#endif // CAN_SERDE_HELPER_H