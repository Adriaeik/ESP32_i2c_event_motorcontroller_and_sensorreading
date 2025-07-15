#include "can_motctrl.h"
#include "esp_log.h"

static const char *TAG = "CAN_MOTCTRL";

esp_err_t send_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                             const can_fragment_list_t *frag_list) {
    // Send start frame
    can_message_t start_msg;
    can_create_start_frame(frag_list->count, start_id, &start_msg);
    esp_err_t ret = can_bus_send_message(&start_msg, CAN_PRIORITY_HIGH, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start frame");
        return ret;
    }
    
    // Send all data fragments
    for (int i = 0; i < frag_list->count; i++) {
        can_message_t frag_msg = frag_list->fragments[i];
        frag_msg.identifier = data_id;  // Set correct ID
        ret = can_bus_send_message(&frag_msg, CAN_PRIORITY_HIGH, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send fragment %d/%d", i+1, frag_list->count);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between fragments
    }
    
    // Send end frame
    can_message_t end_msg;
    can_create_end_frame(end_id, &end_msg);
    ret = can_bus_send_message(&end_msg, CAN_PRIORITY_HIGH, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send end frame");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t receive_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                can_fragment_list_t *frag_list, uint32_t timeout_ms) {
    // Receive start frame
    can_message_t start_msg;
    esp_err_t ret = can_bus_receive_message(&start_msg, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive start frame");
        return ret;
    }
    
    if (start_msg.identifier != start_id) {
        ESP_LOGE(TAG, "Unexpected start frame ID: 0x%X (expected 0x%X)", 
                start_msg.identifier, start_id);
        return ESP_ERR_INVALID_RESPONSE;
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
    
    // Receive all data fragments
    for (int i = 0; i < total_fragments; i++) {
        ret = can_bus_receive_message(&frag_list->fragments[i], portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive fragment %d/%d", i+1, total_fragments);
            can_fragment_list_free(frag_list);
            return ret;
        }
        
        if (frag_list->fragments[i].identifier != data_id) {
            ESP_LOGE(TAG, "Unexpected fragment ID: 0x%X (expected 0x%X) at pos %d", 
                    frag_list->fragments[i].identifier, data_id, i);
            can_fragment_list_free(frag_list);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    
    // Receive end frame (optional)
    can_message_t end_msg;
    ret = can_bus_receive_message(&end_msg, 100);
    if (ret == ESP_OK && end_msg.identifier != end_id) {
        ESP_LOGW(TAG, "Unexpected end frame ID: 0x%X (expected 0x%X)", 
                end_msg.identifier, end_id);
    }
    
    return ESP_OK;
}
