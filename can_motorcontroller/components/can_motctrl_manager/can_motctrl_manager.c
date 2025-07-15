#include "can_motctrl_manager.h"
#include "can_bus_manager.h"
#include "can_serde_helper.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "can_motctrl_manager";

static EventGroupHandle_t s_motctrl_event_group = NULL;
static motorcontroller_response_t s_received_response;
static bool s_manager_initialized = false;
static bool s_operation_in_progress = false;
static uint8_t *s_fragment_buffer = NULL;
static can_motctrl_manager_ctx_t s_mgr_ctx = {0};

// Internal function prototypes
static esp_err_t execute_motor_ctrl_send_pkg(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms);
static esp_err_t execute_motor_ctrl_get_resp(motorcontroller_response_t *resp, uint32_t timeout_ms);
static esp_err_t send_fragmented_package(const uint8_t *data, size_t len);
static esp_err_t get_worker_status(worker_status_t *status, uint32_t timeout_ms);
static esp_err_t receive_fragmented_response(uint8_t *buffer, size_t *len, uint32_t timeout_ms);
static void can_manager_rx_task(void *pvParameters);

// Callback for CAN bus manager events
static void motctrl_manager_event_callback(const can_bus_event_data_t *event_data, void *user_data)
{
    // Filter for motor controller messages
    if (event_data->message.identifier < CAN_ID_MOTCTRL_PKG_START || 
        event_data->message.identifier > CAN_ID_MOTCTRL_RESP_END) {
        return; // Not our message
    }
    
    switch (event_data->event_type) {
        case CAN_BUS_EVT_MESSAGE_SENT:
            ESP_LOGD(TAG, "CAN message sent successfully: ID 0x%lx", event_data->message.identifier);
            if (event_data->message.identifier == CAN_ID_MOTCTRL_PKG_END) {
                xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MANAGER_PKG_SENT_BIT);
            }
            break;
            
        case CAN_BUS_EVT_MESSAGE_RECEIVED:
            // Forward relevant messages to our RX queue
            if (s_mgr_ctx.can_rx_queue != NULL) {
                xQueueSend(s_mgr_ctx.can_rx_queue, &event_data->message, 0);
            }
            break;
            
        case CAN_BUS_EVT_ERROR:
            ESP_LOGE(TAG, "CAN bus error: %s", esp_err_to_name(event_data->error_code));
            xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MANAGER_ERROR_BIT);
            break;
            
        case CAN_BUS_EVT_TIMEOUT:
            ESP_LOGW(TAG, "CAN operation timeout");
            xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MANAGER_TIMEOUT_BIT);
            break;
            
        default:
            break;
    }
}

esp_err_t can_motctrl_manager_init(void)
{
    if (s_manager_initialized) {
        ESP_LOGW(TAG, "Motor controller manager already initialized");
        return ESP_OK;
    }

    // Initialize context
    memset(&s_mgr_ctx, 0, sizeof(s_mgr_ctx));

    // Create event group
    s_motctrl_event_group = xEventGroupCreate();
    if (s_motctrl_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // Create CAN RX queue
    s_mgr_ctx.can_rx_queue = xQueueCreate(32, sizeof(twai_message_t));
    if (s_mgr_ctx.can_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN RX queue");
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Allocate fragment buffer
    s_fragment_buffer = malloc(MAX_FRAGMENT_SIZE);
    if (s_fragment_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fragment buffer");
        vQueueDelete(s_mgr_ctx.can_rx_queue);
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Initialize CAN bus manager
    esp_err_t ret = can_bus_manager_init(motctrl_manager_event_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN bus manager: %s", esp_err_to_name(ret));
        free(s_fragment_buffer);
        s_fragment_buffer = NULL;
        vQueueDelete(s_mgr_ctx.can_rx_queue);
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
        return ret;
    }

    // Create CAN RX task for motor controller responses
    BaseType_t task_ret = xTaskCreate(
        can_manager_rx_task,
        "can_mgr_rx",
        3072,
        NULL,
        5,
        &s_mgr_ctx.rx_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        can_bus_manager_deinit();
        free(s_fragment_buffer);
        s_fragment_buffer = NULL;
        vQueueDelete(s_mgr_ctx.can_rx_queue);
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_manager_initialized = true;
    ESP_LOGI(TAG, "Motor controller manager initialized successfully");
    return ESP_OK;
}

esp_err_t can_motctrl_manager_deinit(void)
{
    if (!s_manager_initialized) {
        return ESP_OK;
    }

    // Stop RX task
    if (s_mgr_ctx.rx_task_handle != NULL) {
        vTaskDelete(s_mgr_ctx.rx_task_handle);
        s_mgr_ctx.rx_task_handle = NULL;
    }

    // Deinitialize CAN bus manager
    can_bus_manager_deinit();

    // Clean up resources
    if (s_fragment_buffer != NULL) {
        free(s_fragment_buffer);
        s_fragment_buffer = NULL;
    }

    if (s_mgr_ctx.can_rx_queue != NULL) {
        vQueueDelete(s_mgr_ctx.can_rx_queue);
        s_mgr_ctx.can_rx_queue = NULL;
    }

    if (s_motctrl_event_group != NULL) {
        vEventGroupDelete(s_motctrl_event_group);
        s_motctrl_event_group = NULL;
    }

    s_manager_initialized = false;
    s_operation_in_progress = false;
    ESP_LOGI(TAG, "Motor controller manager deinitialized");
    return ESP_OK;
}

esp_err_t can_motctrl_manager_send_pkg(const motorcontroller_pkg_t *pkg, int timeout_sec)
{
    if (!s_manager_initialized) {
        ESP_LOGE(TAG, "Motor controller manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_operation_in_progress) {
        ESP_LOGE(TAG, "Operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear event group
    xEventGroupClearBits(s_motctrl_event_group, 0xFF);
    s_operation_in_progress = true;

    // Execute motor controller package send
    esp_err_t ret = execute_motor_ctrl_send_pkg(pkg, timeout_sec * 1000);
    if (ret != ESP_OK) {
        s_operation_in_progress = false;
        ESP_LOGE(TAG, "Failed to send motor controller package: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for completion
    EventBits_t bits = xEventGroupWaitBits(
        s_motctrl_event_group,
        MOTCTRL_MANAGER_PKG_SENT_BIT | MOTCTRL_MANAGER_ERROR_BIT | MOTCTRL_MANAGER_TIMEOUT_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    s_operation_in_progress = false;

    if (bits & MOTCTRL_MANAGER_ERROR_BIT) {
        return ESP_FAIL;
    }

    if (bits & MOTCTRL_MANAGER_TIMEOUT_BIT) {
        ESP_LOGE(TAG, "Package send timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (!(bits & MOTCTRL_MANAGER_PKG_SENT_BIT)) {
        ESP_LOGE(TAG, "Unexpected timeout waiting for package send");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t can_motctrl_manager_wait_response(motorcontroller_response_t *resp, int timeout_sec)
{
    if (!s_manager_initialized) {
        ESP_LOGE(TAG, "Motor controller manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_operation_in_progress) {
        ESP_LOGE(TAG, "Operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear event group
    xEventGroupClearBits(s_motctrl_event_group, 0xFF);
    s_operation_in_progress = true;

    // Execute motor controller response wait
    esp_err_t ret = execute_motor_ctrl_get_resp(&s_received_response, timeout_sec * 1000);
    if (ret != ESP_OK) {
        s_operation_in_progress = false;
        ESP_LOGE(TAG, "Failed to get motor controller response: %s", esp_err_to_name(ret));
        return ret;
    }

    s_operation_in_progress = false;
    *resp = s_received_response;
    return ESP_OK;
}

esp_err_t can_motctrl_manager_wake_up_motorcontroller(void)
{
    // For CAN, we can send a special wake-up message
    // or just rely on the first status request to wake up the worker
    ESP_LOGI(TAG, "Wake-up signal sent via CAN status request");
    
    worker_status_t status;
    return get_worker_status(&status, 1000); // 1 second timeout
}

int motctrl_manager_calculate_wait_time(state_t state, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent)
{
    int base_time = 60; // Default 1 minute
    
    if (prev_estimated_cm_per_s > 0) {
        // Estimate based on previous speed
        int estimated_depth = 100; // cm - you might want to store this in a static variable
        base_time = (estimated_depth * 100) / prev_estimated_cm_per_s;
    }
    
    if (state == RISING) {
        // Apply rising timeout percentage
        base_time = (base_time * (100 + rising_timeout_percent)) / 100;
    }
    
    // Add 10% buffer
    base_time = (base_time * 110) / 100;
    
    // Minimum 30 seconds, maximum 30 minutes
    if (base_time < 30) base_time = 30;
    if (base_time > 1800) base_time = 1800;
    
    return base_time;
}

bool can_motctrl_manager_is_ready(void)
{
    return (s_manager_initialized && !s_operation_in_progress);
}

esp_err_t can_motctrl_manager_cancel_operation(void)
{
    if (!s_operation_in_progress) {
        return ESP_OK;
    }

    s_operation_in_progress = false;
    xEventGroupSetBits(s_motctrl_event_group, MOTCTRL_MANAGER_ERROR_BIT);

    ESP_LOGI(TAG, "Operation cancelled");
    return ESP_OK;
}

// Internal implementation functions
static esp_err_t execute_motor_ctrl_send_pkg(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms)
{
    // Serialize motor controller package using CAN fragmentation
    can_fragment_list_t frag_list;
    esp_err_t ret = can_serialize_pkg(pkg, &frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize motor controller package: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Package serialized into %d fragments", frag_list.count);
    
    // Send start frame
    twai_message_t start_msg;
    ret = can_create_start_frame(frag_list.count, CAN_ID_MOTCTRL_PKG_START, &start_msg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create start frame");
        can_fragment_list_free(&frag_list);
        return ret;
    }
    
    ret = can_bus_manager_send_message(&start_msg, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start frame: %s", esp_err_to_name(ret));
        can_fragment_list_free(&frag_list);
        return ret;
    }
    
    // Send fragment messages
    for (uint16_t i = 0; i < frag_list.count; i++) {
        ret = can_bus_manager_send_message(&frag_list.fragments[i], 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send fragment %d: %s", i, esp_err_to_name(ret));
            can_fragment_list_free(&frag_list);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between fragments
    }
    
    // Send end frame - this will trigger the PKG_SENT event
    twai_message_t end_msg;
    ret = can_create_end_frame(CAN_ID_MOTCTRL_PKG_END, &end_msg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create end frame");
        can_fragment_list_free(&frag_list);
        return ret;
    }
    
    ret = can_bus_manager_send_message(&end_msg, 1000);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Motor controller package sent successfully (%d fragments)", frag_list.count);
    } else {
        ESP_LOGE(TAG, "Failed to send end frame: %s", esp_err_to_name(ret));
    }
    
    can_fragment_list_free(&frag_list);
    return ret;
}

static esp_err_t execute_motor_ctrl_get_resp(motorcontroller_response_t *resp, uint32_t timeout_ms) 
{
    const int poll_interval_ms = 100;
    const int max_retries = timeout_ms / poll_interval_ms;
    int retry_count = 0;
    worker_status_t status;

    ESP_LOGI(TAG, "Polling for motor controller response...");

    while (retry_count++ < max_retries) {
        // Check worker status
        esp_err_t err = get_worker_status(&status, poll_interval_ms / 2);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "Status check failed: %s (retry %d/%d)", 
                    esp_err_to_name(err), retry_count, max_retries);
            vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
            continue;
        }

        // Handle status codes
        switch (status) {
            case WORKER_STATUS_RESP_READY:
            {
                // Worker has response ready, receive it
                size_t resp_len;
                err = receive_fragmented_response(s_fragment_buffer, &resp_len, poll_interval_ms);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to receive response: %s", esp_err_to_name(err));
                    break;
                }
                
                // Deserialize response
                uint16_t received_crc;
                err = deserialize_resp(s_fragment_buffer, resp_len, resp, &received_crc);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to deserialize response");
                    break;
                }
                
                // Verify CRC
                uint16_t calculated_crc = calculate_resp_crc(resp);
                if (received_crc != calculated_crc) {
                    ESP_LOGW(TAG, "Response CRC mismatch: 0x%04X vs 0x%04X", 
                            received_crc, calculated_crc);
                    break;
                }
                
                ESP_LOGI(TAG, "Response received successfully");
                return ESP_OK;
            }
            
            case WORKER_STATUS_WORKING:
                ESP_LOGD(TAG, "Worker busy, waiting...");
                break;
                
            case WORKER_STATUS_READY: 
            case WORKER_STATUS_IDLE:
                ESP_LOGD(TAG, "Worker idle, waiting for response...");
                break;
                
            case WORKER_STATUS_ERROR:
                ESP_LOGE(TAG, "Worker reported internal error");
                return ESP_ERR_INVALID_STATE;
                
            default:
                ESP_LOGW(TAG, "Unexpected status: 0x%02X", status);
        }
        
        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
    }

    ESP_LOGE(TAG, "Timeout after %d retries", max_retries);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t get_worker_status(worker_status_t *status, uint32_t timeout_ms)
{
    // Send status request
    twai_message_t status_req = {
        .identifier = CAN_ID_MOTCTRL_STATUS_REQ,
        .flags = TWAI_MSG_FLAG_NONE,
        .data_length_code = 0
    };
    
    esp_err_t ret = can_bus_manager_send_message(&status_req, 100);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for status response
    twai_message_t response;
    if (xQueueReceive(s_mgr_ctx.can_rx_queue, &response, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (response.identifier == CAN_ID_MOTCTRL_STATUS_RESP && response.data_length_code >= 1) {
            *status = (worker_status_t)response.data[0];
            return ESP_OK;
        }
    }
    
    return ESP_ERR_TIMEOUT;
}

static esp_err_t receive_fragmented_response(uint8_t *buffer, size_t *len, uint32_t timeout_ms)
{
    twai_message_t message;
    can_fragment_list_t frag_list = {0};
    uint16_t expected_fragments = 0;
    bool started = false;
    
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        if (xQueueReceive(s_mgr_ctx.can_rx_queue, &message, pdMS_TO_TICKS(50)) == pdTRUE) {
            switch (message.identifier) {
                case CAN_ID_MOTCTRL_RESP_START:
                    if (message.data_length_code >= 2) {
                        expected_fragments = (message.data[0] << 8) | message.data[1];
                        
                        // Allocate fragment list
                        if (frag_list.fragments != NULL) {
                            can_fragment_list_free(&frag_list);
                        }
                        
                        frag_list.fragments = malloc(expected_fragments * sizeof(twai_message_t));
                        if (frag_list.fragments == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate fragment list");
                            return ESP_ERR_NO_MEM;
                        }
                        
                        frag_list.count = 0;
                        started = true;
                        ESP_LOGD(TAG, "Response start: expecting %d fragments", expected_fragments);
                    }
                    break;
                    
                case CAN_ID_MOTCTRL_RESP_DATA:
                    if (started && frag_list.count < expected_fragments) {
                        frag_list.fragments[frag_list.count] = message;
                        frag_list.count++;
                    }
                    break;
                    
                case CAN_ID_MOTCTRL_RESP_END:
                    if (started && frag_list.count == expected_fragments) {
                        // Deserialize response
                        motorcontroller_response_t temp_resp;
                        esp_err_t ret = can_deserialize_resp(&frag_list, &temp_resp);
                        
                        can_fragment_list_free(&frag_list);
                        
                        if (ret == ESP_OK) {
                            // Copy response to output buffer (for compatibility)
                            // In practice, you might want to modify the function signature
                            // to directly return the deserialized response
                            size_t resp_len;
                            uint16_t crc = calculate_resp_crc(&temp_resp);
                            ret = serialize_resp(&temp_resp, buffer, &resp_len, crc);
                            if (ret == ESP_OK) {
                                *len = resp_len;
                            }
                            
                            ESP_LOGI(TAG, "Response received: %d fragments, %d bytes", expected_fragments, resp_len);
                            return ret;
                        } else {
                            ESP_LOGE(TAG, "Failed to deserialize response: %s", esp_err_to_name(ret));
                            return ret;
                        }
                    } else {
                        ESP_LOGE(TAG, "Fragment mismatch: expected %d, got %d", expected_fragments, frag_list.count);
                        can_fragment_list_free(&frag_list);
                        return ESP_FAIL;
                    }
                    break;
            }
        }
    }
    
    // Cleanup on timeout
    can_fragment_list_free(&frag_list);
    return ESP_ERR_TIMEOUT;
}

static void can_manager_rx_task(void *pvParameters)
{
    twai_message_t message;
    
    ESP_LOGI(TAG, "CAN manager RX task started");
    
    while (true) {
        // This task just processes messages already filtered by the bus manager callback
        // The actual message handling is done in the callback and main functions
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vTaskDelete(NULL);
}