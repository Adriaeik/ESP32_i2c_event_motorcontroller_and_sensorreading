#include "can_motctrl.h"
#include "esp_log.h"

static const char *TAG = "CAN_MOTCTRL";

typedef struct{
   uint32_t start_id;
   uint32_t data_id;
   uint32_t end_id;
   uint32_t ack_id;
} fragment_ids_t; 
// Predefined fragment ID sets
const fragment_ids_t MANAGER_TO_WORKER_IDS = {
    .start_id = CAN_ID_MOTCTRL_PKG_START,
    .data_id = CAN_ID_MOTCTRL_PKG_DATA,
    .end_id = CAN_ID_MOTCTRL_PKG_END,
    .ack_id = CAN_ID_MOTCTRL_RESP_ACK
};

const fragment_ids_t WORKER_TO_MANAGER_IDS = {
    .start_id = CAN_ID_MOTCTRL_RESP_START,
    .data_id = CAN_ID_MOTCTRL_RESP_DATA,
    .end_id = CAN_ID_MOTCTRL_RESP_END,
    .ack_id = CAN_ID_MOTCTRL_PKG_ACK
};

typedef enum {
    CAN_SUBSCRIPTION_SET_MANAGER,    // Subscribe to response channels
    CAN_SUBSCRIPTION_SET_WORKER      // Subscribe to package channels
} can_subscription_set_t;

// Predefined subscription sets
static const can_subscription_spec_t manager_specs[] = {
    {CAN_ID_MOTCTRL_RESP_START, 2},
    {CAN_ID_MOTCTRL_RESP_DATA, 60},
    {CAN_ID_MOTCTRL_RESP_END, 2},
    {CAN_ID_MOTCTRL_RESP_ACK, 5}
};
static const can_subscription_spec_t worker_specs[] = {
    {CAN_ID_MOTCTRL_PKG_START, 2},
    {CAN_ID_MOTCTRL_PKG_DATA, 60},
    {CAN_ID_MOTCTRL_PKG_END, 2},
    {CAN_ID_MOTCTRL_PKG_ACK, 5}
};
static const uint32_t manager_ids[] = {
    CAN_ID_MOTCTRL_RESP_START, CAN_ID_MOTCTRL_RESP_DATA,
    CAN_ID_MOTCTRL_RESP_END, CAN_ID_MOTCTRL_RESP_ACK
};

static const uint32_t worker_ids[] = {
    CAN_ID_MOTCTRL_PKG_START, CAN_ID_MOTCTRL_PKG_DATA,
    CAN_ID_MOTCTRL_PKG_END, CAN_ID_MOTCTRL_PKG_ACK
};
    

/**
 * @brief Subscribe to a predefined set of CAN IDs
 * @param set Which subscription set to use
 * @return ESP_OK on success
 */
esp_err_t can_subscribe_set(const can_motctrl_role_t role) {
    const can_subscription_spec_t *specs;
    size_t count;
    switch (role) {
        case CAN_MTOTCTRL_MANAGER:
            specs = manager_specs;
            count = sizeof(manager_specs) / sizeof(manager_specs[0]);
            ESP_LOGI("CAN_SUB_SET", "Subscribing to MANAGER response channels");
            break;
            
        case CAN_MTOTCTRL_WORKER:
            specs = worker_specs;
            count = sizeof(worker_specs) / sizeof(worker_specs[0]);
            ESP_LOGI("CAN_SUB_SET", "Subscribing to WORKER package channels");
            break;
            
        default:
            ESP_LOGE("CAN_SUB_SET", "Invalid subscription set: %d", role);
            return ESP_ERR_INVALID_ARG;
    }
    
    return can_subscribe_multiple(specs, count);
}

esp_err_t can_unsubscribe_set(const can_motctrl_role_t role) {
    const uint32_t *ids;
    size_t count;
    switch (role) {
        case CAN_MTOTCTRL_MANAGER:
            ids = manager_ids;
            count = sizeof(manager_ids) / sizeof(manager_ids[0]);
            ESP_LOGI("CAN_UNSUB_SET", "Unsubscribing from MANAGER channels");
            break;
            
        case CAN_MTOTCTRL_WORKER:
            ids = worker_ids;
            count = sizeof(worker_ids) / sizeof(worker_ids[0]);
            ESP_LOGI("CAN_UNSUB_SET", "Unsubscribing from WORKER channels");
            break;
            
        default:
            ESP_LOGE("CAN_UNSUB_SET", "Invalid subscription set: %d", role);
            return ESP_ERR_INVALID_ARG;
    }
    
    return can_unsubscribe_multiple(ids, count);
}


// Simple ACK/NACK message structure (8 bytes)
typedef struct {
    uint8_t message_type;     // 0x01 = ACK, 0x02 = NACK
    uint8_t error_code;       // Error reason (for NACK)
    uint16_t fragment_count;  // Number of fragments received
    uint32_t timestamp;       // Timestamp when ACK was sent
} simple_ack_message_t;

// Send simple ACK/NACK
static esp_err_t send_simple_ack(uint32_t ack_id, bool is_ack, uint8_t error_code, uint16_t fragment_count) {
    can_message_t ack_msg = {0};
    ack_msg.identifier = ack_id;
    ack_msg.data_length_code = 8;
    
    simple_ack_message_t *ack_data = (simple_ack_message_t *)ack_msg.data;
    ack_data->message_type = is_ack ? 0x01 : 0x02;
    ack_data->error_code = error_code;
    ack_data->fragment_count = fragment_count;
    ack_data->timestamp = xTaskGetTickCount();
    
    esp_err_t ret = can_bus_send_message(&ack_msg, 500);  // Shorter timeout
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent %s on ID 0x%X (fragments: %d)", 
                 is_ack ? "ACK" : "NACK", ack_id, fragment_count);
    } else {
        ESP_LOGW(TAG, "Failed to send %s on ID 0x%X: %s", 
                 is_ack ? "ACK" : "NACK", ack_id, esp_err_to_name(ret));
    }
    return ret;
}
// Wait for simple ACK/NACK
static esp_err_t wait_for_simple_ack(uint32_t ack_id, uint32_t timeout_ms, bool *received_ack, uint16_t expected_fragments) {
    can_message_t ack_msg;
    esp_err_t ret = can_bus_wait_for_message(ack_id, &ack_msg, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ACK timeout on ID 0x%X after %d ms", ack_id, timeout_ms);
        return ret;
    }
    
    simple_ack_message_t *ack_data = (simple_ack_message_t *)ack_msg.data;
    *received_ack = (ack_data->message_type == 0x01);
    
    if (*received_ack) {
        if (ack_data->fragment_count == expected_fragments) {
            ESP_LOGI(TAG, "Received ACK on ID 0x%X (confirmed %d fragments)", ack_id, ack_data->fragment_count);
        } else {
            ESP_LOGW(TAG, "ACK fragment count mismatch: sent %d, confirmed %d", 
                     expected_fragments, ack_data->fragment_count);
        }
    } else {
        ESP_LOGW(TAG, "Received NACK on ID 0x%X, error: %d, fragments: %d", 
                 ack_id, ack_data->error_code, ack_data->fragment_count);
    }
    
    return ESP_OK;
}

esp_err_t send_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                             const can_fragment_list_t *frag_list) {
    // Send start frame
    can_message_t start_msg;
    can_create_start_frame(frag_list->count, start_id, &start_msg);
    esp_err_t ret = can_bus_send_message(&start_msg, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start frame");
        return ret;
    }
    
    // Send all data fragments
    for (int i = 0; i < frag_list->count; i++) {
        can_message_t frag_msg = frag_list->fragments[i];
        frag_msg.identifier = data_id;  // Set correct ID
        ret = can_bus_send_message(&frag_msg, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send fragment %d/%d", i+1, frag_list->count);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between fragments
    }
    
    // Send end frame
    can_message_t end_msg;
    can_create_end_frame(end_id, &end_msg);
    ret = can_bus_send_message(&end_msg, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send end frame");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t receive_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                can_fragment_list_t *frag_list, uint32_t timeout_ms) {
    
    // Make sure we're subscribed to the required IDs
    // Note: This should be done beforehand, but we can check/ensure here
    esp_err_t ret;
    
    // Wait for start frame using subscription system
    can_message_t start_msg;
    ret = can_bus_wait_for_message(start_id, &start_msg, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive start frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Extract total fragments (2 bytes in data[0] and data[1])
    uint16_t total_fragments = (start_msg.data[0] << 8) | start_msg.data[1];
    ESP_LOGI(TAG, "Expecting %d fragments", total_fragments);
    
    // Allocate memory for fragments
    frag_list->fragments = malloc(total_fragments * sizeof(can_message_t));
    if (!frag_list->fragments) {
        ESP_LOGE(TAG, "Memory allocation failed for %d fragments", total_fragments);
        return ESP_ERR_NO_MEM;
    }
    frag_list->count = total_fragments;
    
    // Receive all data fragments using subscription system
    for (int i = 0; i < total_fragments; i++) {
        ret = can_bus_wait_for_message(data_id, &frag_list->fragments[i], timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive fragment %d/%d: %s", i+1, total_fragments, esp_err_to_name(ret));
            can_fragment_list_free(frag_list);
            return ret;
        }
        
        // No need to check identifier since subscription system guarantees correct ID
        ESP_LOGD(TAG, "Received fragment %d/%d", i+1, total_fragments);
    }
    
    // Receive end frame (optional) - use shorter timeout for end frame
    can_message_t end_msg;
    ret = can_bus_wait_for_message(end_id, &end_msg, 100);
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Failed to receive end frame: %s", esp_err_to_name(ret));
        // Don't fail the whole operation for missing end frame
    }
    
    ESP_LOGI(TAG, "Successfully received %d fragments", total_fragments);
    return ESP_OK;
}


esp_err_t send_fragment_list_simple_ack(const can_motctrl_role_t role, const can_fragment_list_t *frag_list, uint32_t timeout_ms)
{
    esp_err_t ret;
    const fragment_ids_t *ids;
    switch (role) {
        case CAN_MTOTCTRL_MANAGER: ids = &MANAGER_TO_WORKER_IDS; break;
        case CAN_MTOTCTRL_WORKER:  ids = &WORKER_TO_MANAGER_IDS; break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    for (int retry = 0; retry < CAN_MAX_RETRIES; retry++) {
        ESP_LOGI(TAG, "Transmission attempt %d/%d (expecting ACK on 0x%X)", 
                 retry + 1, CAN_MAX_RETRIES, ids->ack_id);
        
        // Send fragments
        ret = send_fragment_list(ids->start_id, ids->data_id, ids->end_id, frag_list);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send fragments, attempt %d: %s", retry + 1, esp_err_to_name(ret));
            continue;
        }
        
        // Wait for ACK with validation
        bool received_ack = false;
        ret = wait_for_simple_ack(ids->ack_id, CAN_ACK_TIMEOUT_MS, &received_ack, frag_list->count);
        
        if (ret == ESP_OK && received_ack) {
            ESP_LOGI(TAG, "Transmission successful with ACK confirmation on 0x%X", ids->ack_id);
            return ESP_OK;
        }
        
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ACK timeout on 0x%X, retry %d", ids->ack_id, retry);
        } else if (ret == ESP_OK && !received_ack) {
            ESP_LOGW(TAG, "Received NACK on 0x%X, retry %d", ids->ack_id, retry);
        }
        
        // FIXED: Linear backoff instead of exponential
        if (retry < CAN_MAX_RETRIES - 1) {
            uint32_t delay = CAN_RETRY_BASE_DELAY_MS * (retry + 1);  // 100ms, 200ms, 300ms
            ESP_LOGI(TAG, "Waiting %d ms before retry", delay);
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
    }
    
    ESP_LOGE(TAG, "Failed to send with ACK after %d retries (ACK ID: 0x%X)", CAN_MAX_RETRIES, ids->ack_id);
    return ESP_ERR_TIMEOUT;
}

esp_err_t receive_fragment_list_simple_ack(const can_motctrl_role_t role, const can_fragment_list_t *frag_list, uint32_t timeout_ms) {
    // Receive fragments
    const fragment_ids_t *ids;
    switch (role) {
        case CAN_MTOTCTRL_MANAGER: ids = &WORKER_TO_MANAGER_IDS; break;
        case CAN_MTOTCTRL_WORKER:  ids = &MANAGER_TO_WORKER_IDS; break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = receive_fragment_list(ids->start_id, ids->data_id, ids->end_id, frag_list, timeout_ms);
    
    if (ret == ESP_OK) {
        // Send ACK with fragment count confirmation
        send_simple_ack(ids->ack_id, true, 0, frag_list->count);
        ESP_LOGI(TAG, "Successfully received %d fragments and sent ACK on 0x%X", 
                 frag_list->count, ids->ack_id);
    } else {
        // Send NACK with error info
        uint8_t error_code = (ret == ESP_ERR_TIMEOUT) ? 1 : 2;
        send_simple_ack(ids->ack_id, false, error_code, 0);
        ESP_LOGE(TAG, "Failed to receive fragments (%s), sent NACK on 0x%X", 
                 esp_err_to_name(ret), ids->ack_id);
    }
    
    return ret;
}

