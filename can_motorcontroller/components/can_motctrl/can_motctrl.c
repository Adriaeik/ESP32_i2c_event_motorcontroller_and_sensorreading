
// components/motctrl/motctrl.c
#include "can_motctrl.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "motctrl";

// Helper function to send fragmented data
static esp_err_t send_fragmented_data(uint32_t start_id, uint32_t data_id, uint32_t end_id,
                                     const void *data, size_t data_size, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    
    // Send start frame
    can_message_t msg = {0};
    msg.identifier = start_id;
    msg.flags = CAN_MSG_FLAG_NONE;
    msg.data_length_code = 4;
    msg.data[0] = (data_size >> 24) & 0xFF;
    msg.data[1] = (data_size >> 16) & 0xFF;
    msg.data[2] = (data_size >> 8) & 0xFF;
    msg.data[3] = data_size & 0xFF;
    
    ret = can_bus_send_message(&msg, CAN_PRIORITY_NORMAL, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start frame");
        return ret;
    }

    // Send data frames
    const uint8_t *data_bytes = (const uint8_t *)data;
    size_t bytes_sent = 0;
    uint8_t sequence = 0;
    
    while (bytes_sent < data_size) {
        size_t chunk_size = (data_size - bytes_sent > 8) ? 8 : (data_size - bytes_sent);
        
        msg.identifier = data_id;
        msg.data_length_code = chunk_size;
        memcpy(msg.data, &data_bytes[bytes_sent], chunk_size);
        
        ret = can_bus_send_message(&msg, CAN_PRIORITY_NORMAL, timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send data frame %d", sequence);
            return ret;
        }
        
        bytes_sent += chunk_size;
        sequence++;
    }

    // Send end frame
    msg.identifier = end_id;
    msg.data_length_code = 1;
    msg.data[0] = sequence; // Total number of data frames sent
    
    ret = can_bus_send_message(&msg, CAN_PRIORITY_NORMAL, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send end frame");
        return ret;
    }

    ESP_LOGD(TAG, "Sent fragmented data: %d bytes in %d frames", data_size, sequence);
    return ESP_OK;
}

// Helper function to receive fragmented data
static esp_err_t receive_fragmented_data(uint32_t start_id, uint32_t data_id, uint32_t end_id,
                                        void *data, size_t max_size, size_t *received_size, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    can_message_t msg;
    size_t expected_size = 0;
    size_t bytes_received = 0;
    uint8_t expected_frames = 0;
    uint8_t frames_received = 0;
    
    // Wait for start frame
    ret = can_bus_receive_message(&msg, timeout_ms);
    if (ret != ESP_OK || msg.identifier != start_id) {
        ESP_LOGE(TAG, "Failed to receive start frame");
        return ESP_ERR_TIMEOUT;
    }
    
    if (msg.data_length_code >= 4) {
        expected_size = (msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3];
        if (expected_size > max_size) {
            ESP_LOGE(TAG, "Expected size %d exceeds buffer size %d", expected_size, max_size);
            return ESP_ERR_NO_MEM;
        }
    }

    // Receive data frames
    uint8_t *data_bytes = (uint8_t *)data;
    while (bytes_received < expected_size) {
        ret = can_bus_receive_message(&msg, timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive data frame %d", frames_received);
            return ret;
        }
        
        if (msg.identifier == data_id) {
            size_t copy_size = (bytes_received + msg.data_length_code > expected_size) ? 
                              (expected_size - bytes_received) : msg.data_length_code;
            memcpy(&data_bytes[bytes_received], msg.data, copy_size);
            bytes_received += copy_size;
            frames_received++;
        } else if (msg.identifier == end_id) {
            // End frame received
            expected_frames = msg.data[0];
            break;
        } else {
            ESP_LOGW(TAG, "Unexpected message ID 0x%x during data reception", msg.identifier);
        }
    }

    // Wait for end frame if not already received
    if (msg.identifier != end_id) {
        ret = can_bus_receive_message(&msg, timeout_ms);
        if (ret != ESP_OK || msg.identifier != end_id) {
            ESP_LOGE(TAG, "Failed to receive end frame");
            return ESP_ERR_TIMEOUT;
        }
        expected_frames = msg.data[0];
    }

    if (frames_received != expected_frames) {
        ESP_LOGW(TAG, "Frame count mismatch: expected %d, received %d", expected_frames, frames_received);
    }

    *received_size = bytes_received;
    ESP_LOGD(TAG, "Received fragmented data: %d bytes in %d frames", bytes_received, frames_received);
    return ESP_OK;
}

esp_err_t motctrl_send_package(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms)
{
    if (!pkg) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_fragmented_data(CAN_ID_MOTCTRL_PKG_START, CAN_ID_MOTCTRL_PKG_DATA, CAN_ID_MOTCTRL_PKG_END,
                               pkg, sizeof(motorcontroller_pkg_t), timeout_ms);
}

esp_err_t motctrl_receive_package(motorcontroller_pkg_t *pkg, uint32_t timeout_ms)
{
    if (!pkg) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t received_size;
    return receive_fragmented_data(CAN_ID_MOTCTRL_PKG_START, CAN_ID_MOTCTRL_PKG_DATA, CAN_ID_MOTCTRL_PKG_END,
                                  pkg, sizeof(motorcontroller_pkg_t), &received_size, timeout_ms);
}

esp_err_t motctrl_send_response(const motorcontroller_response_t *response, uint32_t timeout_ms)
{
    if (!response) {
        return ESP_ERR_INVALID_ARG;
    }

    return send_fragmented_data(CAN_ID_MOTCTRL_RESP_START, CAN_ID_MOTCTRL_RESP_DATA, CAN_ID_MOTCTRL_RESP_END,
                               response, sizeof(motorcontroller_response_t), timeout_ms);
}

esp_err_t motctrl_receive_response(motorcontroller_response_t *response, uint32_t timeout_ms)
{
    if (!response) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t received_size;
    return receive_fragmented_data(CAN_ID_MOTCTRL_RESP_START, CAN_ID_MOTCTRL_RESP_DATA, CAN_ID_MOTCTRL_RESP_END,
                                  response, sizeof(motorcontroller_response_t), &received_size, timeout_ms);
}

esp_err_t motctrl_send_status_request(uint32_t timeout_ms)
{
    can_message_t msg = {0};
    msg.identifier = CAN_ID_MOTCTRL_STATUS_REQ;
    msg.flags = CAN_MSG_FLAG_NONE;
    msg.data_length_code = 0; // No data needed for status request
    
    return can_bus_send_message(&msg, CAN_PRIORITY_HIGH, timeout_ms);
}

esp_err_t motctrl_send_status_response(worker_status_t status, uint32_t timeout_ms)
{
    can_message_t msg = {0};
    msg.identifier = CAN_ID_MOTCTRL_STATUS_RESP;
    msg.flags = CAN_MSG_FLAG_NONE;
    msg.data_length_code = 1;
    msg.data[0] = (uint8_t)status;
    
    return can_bus_send_message(&msg, CAN_PRIORITY_HIGH, timeout_ms);
}

esp_err_t motctrl_wait_status_response(worker_status_t *status, uint32_t timeout_ms)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    can_message_t msg;
    esp_err_t ret = can_bus_receive_message(&msg, timeout_ms);
    
    if (ret == ESP_OK && msg.identifier == CAN_ID_MOTCTRL_STATUS_RESP && msg.data_length_code > 0) {
        *status = (worker_status_t)msg.data[0];
        return ESP_OK;
    }
    
    return (ret != ESP_OK) ? ret : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t motctrl_create_start_command(motorcontroller_pkg_t *pkg)
{
    if (!pkg) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(pkg, 0, sizeof(motorcontroller_pkg_t));
    
    // Create a simple start command - you can customize this based on buoye_structs.h
    // For now, we'll create a minimal command structure
    // Assuming motorcontroller_pkg_t has some command field
    
    ESP_LOGI(TAG, "Created start command package");
    return ESP_OK;
}

esp_err_t motctrl_create_work_response(motorcontroller_response_t *response, uint32_t work_result)
{
    if (!response) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(response, 0, sizeof(motorcontroller_response_t));
    
    // Create a simple work completion response - customize based on buoye_structs.h
    // For now, we'll create a minimal response structure
    
    ESP_LOGI(TAG, "Created work response (result: %d)", work_result);
    return ESP_OK;
}