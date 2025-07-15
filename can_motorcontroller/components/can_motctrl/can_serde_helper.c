#include "can_serde_helper.h"
#include "serde_helper.h"  // Your existing serialization functions
#include "esp_log.h"
#include <string.h>

static const char *TAG = "can_serde";

// CAN-specific fragmentation protocol
#define CAN_FRAG_HEADER_SIZE    4    // Fragment header: seq(2) + total_frags(2)
#define CAN_DATA_PER_FRAME      4    // 8 bytes total - 4 for header = 4 for data
#define CAN_MAX_FRAGMENTS       255  // Maximum fragments per message

typedef struct {
    uint16_t fragment_seq;      // Current fragment sequence number
    uint16_t total_fragments;   // Total number of fragments
    uint8_t data[CAN_DATA_PER_FRAME]; // Fragment data
} __attribute__((packed)) can_fragment_t;

esp_err_t can_serialize_pkg(const motorcontroller_pkg_t *pkg, can_fragment_list_t *frag_list)
{
    if (pkg == NULL || frag_list == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // First, serialize using your existing function
    uint8_t temp_buffer[MAX_PKG_WIRE_SIZE];
    size_t serialized_len;
    uint16_t crc = calculate_pkg_crc(pkg);
    
    esp_err_t ret = serialize_pkg(pkg, temp_buffer, &serialized_len, crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize package");
        return ret;
    }

    ESP_LOGI(TAG, "Serialized package size: %d bytes", serialized_len);

    // Calculate number of fragments needed
    uint16_t total_fragments = (serialized_len + CAN_DATA_PER_FRAME - 1) / CAN_DATA_PER_FRAME;
    
    if (total_fragments > CAN_MAX_FRAGMENTS) {
        ESP_LOGE(TAG, "Package too large: %d fragments (max %d)", total_fragments, CAN_MAX_FRAGMENTS);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate fragment array
    frag_list->fragments = malloc(total_fragments * sizeof(twai_message_t));
    if (frag_list->fragments == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fragment array");
        return ESP_ERR_NO_MEM;
    }

    frag_list->count = total_fragments;
    size_t data_pos = 0;

    // Create fragments
    for (uint16_t i = 0; i < total_fragments; i++) {
        can_fragment_t fragment = {
            .fragment_seq = i,
            .total_fragments = total_fragments
        };

        // Copy data for this fragment
        size_t copy_len = (serialized_len - data_pos > CAN_DATA_PER_FRAME) ? 
                         CAN_DATA_PER_FRAME : (serialized_len - data_pos);
        
        memset(fragment.data, 0, CAN_DATA_PER_FRAME);
        memcpy(fragment.data, temp_buffer + data_pos, copy_len);
        data_pos += copy_len;

        // Create CAN message
        frag_list->fragments[i] = (can_message_t){
            .identifier = CAN_ID_MOTCTRL_PKG_DATA,
            .flags = TWAI_MSG_FLAG_NONE,
            .data_length_code = sizeof(can_fragment_t),
        };

        // Copy fragment to CAN message data
        memcpy(frag_list->fragments[i].data, &fragment, sizeof(can_fragment_t));
    }

    ESP_LOGI(TAG, "Created %d fragments for package", total_fragments);
    return ESP_OK;
}

esp_err_t can_deserialize_pkg(const can_fragment_list_t *frag_list, motorcontroller_pkg_t *pkg)
{
    if (frag_list == NULL || pkg == NULL || frag_list->fragments == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frag_list->count == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Get total fragments from first fragment
    can_fragment_t first_frag;
    memcpy(&first_frag, frag_list->fragments[0].data, sizeof(can_fragment_t));
    uint16_t expected_fragments = first_frag.total_fragments;

    if (frag_list->count != expected_fragments) {
        ESP_LOGE(TAG, "Fragment count mismatch: have %d, expected %d", 
                frag_list->count, expected_fragments);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate reassembly buffer
    uint8_t *reassembly_buffer = malloc(expected_fragments * CAN_DATA_PER_FRAME);
    if (reassembly_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate reassembly buffer");
        return ESP_ERR_NO_MEM;
    }

    // Create sequence tracking array
    bool *received = calloc(expected_fragments, sizeof(bool));
    if (received == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sequence tracking");
        free(reassembly_buffer);
        return ESP_ERR_NO_MEM;
    }

    size_t total_data_len = 0;

    // Reassemble fragments
    for (uint16_t i = 0; i < frag_list->count; i++) {
        can_fragment_t fragment;
        memcpy(&fragment, frag_list->fragments[i].data, sizeof(can_fragment_t));

        // Validate fragment
        if (fragment.fragment_seq >= expected_fragments) {
            ESP_LOGE(TAG, "Invalid fragment sequence: %d (max %d)", 
                    fragment.fragment_seq, expected_fragments - 1);
            free(reassembly_buffer);
            free(received);
            return ESP_ERR_INVALID_ARG;
        }

        if (fragment.total_fragments != expected_fragments) {
            ESP_LOGE(TAG, "Fragment total mismatch: %d vs %d", 
                    fragment.total_fragments, expected_fragments);
            free(reassembly_buffer);
            free(received);
            return ESP_ERR_INVALID_ARG;
        }

        if (received[fragment.fragment_seq]) {
            ESP_LOGW(TAG, "Duplicate fragment sequence: %d", fragment.fragment_seq);
            continue;
        }

        // Copy fragment data
        size_t offset = fragment.fragment_seq * CAN_DATA_PER_FRAME;
        memcpy(reassembly_buffer + offset, fragment.data, CAN_DATA_PER_FRAME);
        received[fragment.fragment_seq] = true;
        
        if (fragment.fragment_seq == expected_fragments - 1) {
            // Last fragment might have less data
            total_data_len = offset + CAN_DATA_PER_FRAME;
        }
    }

    // Check all fragments received
    for (uint16_t i = 0; i < expected_fragments; i++) {
        if (!received[i]) {
            ESP_LOGE(TAG, "Missing fragment: %d", i);
            free(reassembly_buffer);
            free(received);
            return ESP_ERR_INVALID_STATE;
        }
    }

    free(received);

    // Deserialize using your existing function
    uint16_t received_crc;
    esp_err_t ret = deserialize_pkg(reassembly_buffer, total_data_len, pkg, &received_crc);
    
    free(reassembly_buffer);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deserialize package: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRC
    uint16_t calculated_crc = calculate_pkg_crc(pkg);
    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG, "Package CRC mismatch: 0x%04X vs 0x%04X", 
                received_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }

    ESP_LOGI(TAG, "Package deserialized successfully from %d fragments", expected_fragments);
    return ESP_OK;
}

esp_err_t can_serialize_resp(const motorcontroller_response_t *resp, can_fragment_list_t *frag_list)
{
    if (resp == NULL || frag_list == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // First, serialize using your existing function
    uint8_t temp_buffer[MAX_RESP_WIRE_SIZE];
    size_t serialized_len;
    uint16_t crc = calculate_resp_crc(resp);
    
    esp_err_t ret = serialize_resp(resp, temp_buffer, &serialized_len, crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize response");
        return ret;
    }

    ESP_LOGI(TAG, "Serialized response size: %d bytes", serialized_len);

    // Calculate number of fragments needed
    uint16_t total_fragments = (serialized_len + CAN_DATA_PER_FRAME - 1) / CAN_DATA_PER_FRAME;
    
    if (total_fragments > CAN_MAX_FRAGMENTS) {
        ESP_LOGE(TAG, "Response too large: %d fragments (max %d)", total_fragments, CAN_MAX_FRAGMENTS);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate fragment array
    frag_list->fragments = malloc(total_fragments * sizeof(twai_message_t));
    if (frag_list->fragments == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fragment array");
        return ESP_ERR_NO_MEM;
    }

    frag_list->count = total_fragments;
    size_t data_pos = 0;

    // Create fragments
    for (uint16_t i = 0; i < total_fragments; i++) {
        can_fragment_t fragment = {
            .fragment_seq = i,
            .total_fragments = total_fragments
        };

        // Copy data for this fragment
        size_t copy_len = (serialized_len - data_pos > CAN_DATA_PER_FRAME) ? 
                         CAN_DATA_PER_FRAME : (serialized_len - data_pos);
        
        memset(fragment.data, 0, CAN_DATA_PER_FRAME);
        memcpy(fragment.data, temp_buffer + data_pos, copy_len);
        data_pos += copy_len;

        // Create CAN message
        frag_list->fragments[i] = (twai_message_t){
            .identifier = CAN_ID_MOTCTRL_RESP_DATA,
            .flags = TWAI_MSG_FLAG_NONE,
            .data_length_code = sizeof(can_fragment_t),
        };

        // Copy fragment to CAN message data
        memcpy(frag_list->fragments[i].data, &fragment, sizeof(can_fragment_t));
    }

    ESP_LOGI(TAG, "Created %d fragments for response", total_fragments);
    return ESP_OK;
}

esp_err_t can_deserialize_resp(const can_fragment_list_t *frag_list, motorcontroller_response_t *resp)
{
    if (frag_list == NULL || resp == NULL || frag_list->fragments == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frag_list->count == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Get total fragments from first fragment
    can_fragment_t first_frag;
    memcpy(&first_frag, frag_list->fragments[0].data, sizeof(can_fragment_t));
    uint16_t expected_fragments = first_frag.total_fragments;

    if (frag_list->count != expected_fragments) {
        ESP_LOGE(TAG, "Fragment count mismatch: have %d, expected %d", 
                frag_list->count, expected_fragments);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate reassembly buffer
    uint8_t *reassembly_buffer = malloc(expected_fragments * CAN_DATA_PER_FRAME);
    if (reassembly_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate reassembly buffer");
        return ESP_ERR_NO_MEM;
    }

    // Create sequence tracking array
    bool *received = calloc(expected_fragments, sizeof(bool));
    if (received == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sequence tracking");
        free(reassembly_buffer);
        return ESP_ERR_NO_MEM;
    }

    size_t total_data_len = 0;

    // Reassemble fragments
    for (uint16_t i = 0; i < frag_list->count; i++) {
        can_fragment_t fragment;
        memcpy(&fragment, frag_list->fragments[i].data, sizeof(can_fragment_t));

        // Validate fragment
        if (fragment.fragment_seq >= expected_fragments) {
            ESP_LOGE(TAG, "Invalid fragment sequence: %d (max %d)", 
                    fragment.fragment_seq, expected_fragments - 1);
            free(reassembly_buffer);
            free(received);
            return ESP_ERR_INVALID_ARG;
        }

        if (fragment.total_fragments != expected_fragments) {
            ESP_LOGE(TAG, "Fragment total mismatch: %d vs %d", 
                    fragment.total_fragments, expected_fragments);
            free(reassembly_buffer);
            free(received);
            return ESP_ERR_INVALID_ARG;
        }

        if (received[fragment.fragment_seq]) {
            ESP_LOGW(TAG, "Duplicate fragment sequence: %d", fragment.fragment_seq);
            continue;
        }

        // Copy fragment data
        size_t offset = fragment.fragment_seq * CAN_DATA_PER_FRAME;
        memcpy(reassembly_buffer + offset, fragment.data, CAN_DATA_PER_FRAME);
        received[fragment.fragment_seq] = true;
        
        if (fragment.fragment_seq == expected_fragments - 1) {
            // Last fragment might have less data
            total_data_len = offset + CAN_DATA_PER_FRAME;
        }
    }

    // Check all fragments received
    for (uint16_t i = 0; i < expected_fragments; i++) {
        if (!received[i]) {
            ESP_LOGE(TAG, "Missing fragment: %d", i);
            free(reassembly_buffer);
            free(received);
            return ESP_ERR_INVALID_STATE;
        }
    }

    free(received);

    // Deserialize using your existing function
    uint16_t received_crc;
    esp_err_t ret = deserialize_resp(reassembly_buffer, total_data_len, resp, &received_crc);
    
    free(reassembly_buffer);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deserialize response: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRC
    uint16_t calculated_crc = calculate_resp_crc(resp);
    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG, "Response CRC mismatch: 0x%04X vs 0x%04X", 
                received_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }

    ESP_LOGI(TAG, "Response deserialized successfully from %d fragments", expected_fragments);
    return ESP_OK;
}

void can_fragment_list_free(can_fragment_list_t *frag_list)
{
    if (frag_list != NULL && frag_list->fragments != NULL) {
        free(frag_list->fragments);
        frag_list->fragments = NULL;
        frag_list->count = 0;
    }
}

esp_err_t can_create_start_frame(uint16_t total_fragments, uint32_t message_id, twai_message_t *start_msg)
{
    if (start_msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *start_msg = (twai_message_t){
        .identifier = message_id,
        .flags = TWAI_MSG_FLAG_NONE,
        .data_length_code = 2,
        .data = {(total_fragments >> 8) & 0xFF, total_fragments & 0xFF}
    };

    return ESP_OK;
}

esp_err_t can_create_end_frame(uint32_t message_id, twai_message_t *end_msg)
{
    if (end_msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *end_msg = (twai_message_t){
        .identifier = message_id,
        .flags = TWAI_MSG_FLAG_NONE,
        .data_length_code = 0
    };

    return ESP_OK;
}