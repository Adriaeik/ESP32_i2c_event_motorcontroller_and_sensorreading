#include "can_motctrl.h"
#include "esp_log.h"

static const char *TAG = "CAN_MOTCTRL";

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

// Helper function to subscribe to all IDs needed for fragmented communication
esp_err_t subscribe_fragment_ids(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                 uint8_t start_queue_size, uint8_t data_queue_size, uint8_t end_queue_size) {
    esp_err_t ret;
    
    // Subscribe to start ID
    ret = can_bus_subscribe_id(start_id, start_queue_size);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // Allow already subscribed
        ESP_LOGE(TAG, "Failed to subscribe to start ID 0x%X: %s", start_id, esp_err_to_name(ret));
        return ret;
    }
    
    // Subscribe to data ID - this needs a larger queue for many fragments
    ret = can_bus_subscribe_id(data_id, data_queue_size);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to subscribe to data ID 0x%X: %s", data_id, esp_err_to_name(ret));
        return ret;
    }
    
    // Subscribe to end ID
    ret = can_bus_subscribe_id(end_id, end_queue_size);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to subscribe to end ID 0x%X: %s", end_id, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Subscribed to fragment IDs: start=0x%X, data=0x%X, end=0x%X", 
             start_id, data_id, end_id);
    return ESP_OK;
}

// Helper function to unsubscribe from fragment IDs
esp_err_t unsubscribe_fragment_ids(uint32_t start_id, uint32_t data_id, uint32_t end_id) {
    esp_err_t ret;
    
    ret = can_bus_unsubscribe_id(start_id);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to unsubscribe from start ID 0x%X: %s", start_id, esp_err_to_name(ret));
    }
    
    ret = can_bus_unsubscribe_id(data_id);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to unsubscribe from data ID 0x%X: %s", data_id, esp_err_to_name(ret));
    }
    
    ret = can_bus_unsubscribe_id(end_id);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to unsubscribe from end ID 0x%X: %s", end_id, esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Unsubscribed from fragment IDs: start=0x%X, data=0x%X, end=0x%X", 
             start_id, data_id, end_id);
    return ESP_OK;
}