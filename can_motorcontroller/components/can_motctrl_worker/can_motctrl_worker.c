#include "can_motctrl_worker.h"
#include "can_bus_manager.h"
#include "can_serde_helper.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "can_motctrl_worker";

static can_motctrl_worker_ctx_t s_worker_ctx = {0};


// Forward declarations
static void can_worker_task(void *pvParameters);
static esp_err_t send_status_response(void);
static esp_err_t send_motorcontroller_response(void);
static esp_err_t process_package_frame(const twai_message_t *message);

// Callback for CAN bus manager events
static void worker_can_event_callback(const can_bus_event_data_t *event_data, void *user_data)
{
    // Filter for motor controller messages
    if (event_data->message.identifier < CAN_ID_MOTCTRL_PKG_START || 
        event_data->message.identifier > CAN_ID_MOTCTRL_RESP_END) {
        return; // Not our message
    }
    
    switch (event_data->event_type) {
        case CAN_BUS_EVT_MESSAGE_RECEIVED:
            // Forward relevant messages to our processing queue
            if (event_data->message.identifier >= CAN_ID_MOTCTRL_PKG_START && 
                event_data->message.identifier <= CAN_ID_MOTCTRL_STATUS_REQ) {
                
                if (s_worker_ctx.can_rx_queue != NULL) {
                    xQueueSend(s_worker_ctx.can_rx_queue, &event_data->message, 0);
                }
            }
            break;
            
        case CAN_BUS_EVT_MESSAGE_SENT:
            ESP_LOGD(TAG, "CAN message sent successfully: ID 0x%lx", event_data->message.identifier);
            // Handle specific sent message confirmations if needed
            break;
            
        case CAN_BUS_EVT_ERROR:
            ESP_LOGE(TAG, "CAN bus error: %s", esp_err_to_name(event_data->error_code));
            if (s_worker_ctx.operation_event_group != NULL) {
                xEventGroupSetBits(s_worker_ctx.operation_event_group, MOTCTRL_WORKER_ERROR_BIT);
            }
            break;
            
        case CAN_BUS_EVT_TIMEOUT:
            ESP_LOGW(TAG, "CAN operation timeout");
            break;
            
        case CAN_BUS_EVT_BUS_RECOVERED:
            ESP_LOGI(TAG, "CAN bus recovered");
            break;
            
        default:
            break;
    }
}

esp_err_t can_motctrl_worker_init(void)
{
    if (s_worker_ctx.initialized) {
        ESP_LOGW(TAG, "Worker already initialized");
        return ESP_OK;
    }

    // Initialize worker context
    memset(&s_worker_ctx, 0, sizeof(s_worker_ctx));
    s_worker_ctx.state = CAN_WORKER_STATE_IDLE;
    s_worker_ctx.status_byte = WORKER_STATUS_IDLE;

    // Create event queue
    s_worker_ctx.event_queue = xQueueCreate(16, sizeof(can_worker_event_t));
    if (s_worker_ctx.event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    // Create CAN message queue for RX
    s_worker_ctx.can_rx_queue = xQueueCreate(32, sizeof(twai_message_t));
    if (s_worker_ctx.can_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN RX queue");
        vQueueDelete(s_worker_ctx.event_queue);
        return ESP_ERR_NO_MEM;
    }

    // Create operation event group
    s_worker_ctx.operation_event_group = xEventGroupCreate();
    if (s_worker_ctx.operation_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create operation event group");
        vQueueDelete(s_worker_ctx.event_queue);
        vQueueDelete(s_worker_ctx.can_rx_queue);
        return ESP_ERR_NO_MEM;
    }

    // Initialize fragment assembly buffer
    s_worker_ctx.fragment_buffer = malloc(MAX_FRAGMENT_SIZE);
    if (s_worker_ctx.fragment_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fragment buffer");
        vEventGroupDelete(s_worker_ctx.operation_event_group);
        vQueueDelete(s_worker_ctx.event_queue);
        vQueueDelete(s_worker_ctx.can_rx_queue);
        return ESP_ERR_NO_MEM;
    }

    // Initialize CAN bus manager with our callback
    esp_err_t ret = can_bus_manager_init(worker_can_event_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN bus manager: %s", esp_err_to_name(ret));
        free(s_worker_ctx.fragment_buffer);
        vEventGroupDelete(s_worker_ctx.operation_event_group);
        vQueueDelete(s_worker_ctx.event_queue);
        vQueueDelete(s_worker_ctx.can_rx_queue);
        return ret;
    }

    // Create worker task
    BaseType_t task_ret = xTaskCreate(
        can_worker_task,
        "can_worker",
        4096,
        NULL,
        5,
        &s_worker_ctx.worker_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create worker task");
        can_bus_manager_deinit();
        free(s_worker_ctx.fragment_buffer);
        vEventGroupDelete(s_worker_ctx.operation_event_group);
        vQueueDelete(s_worker_ctx.event_queue);
        vQueueDelete(s_worker_ctx.can_rx_queue);
        return ESP_ERR_NO_MEM;
    }

    s_worker_ctx.initialized = true;
    ESP_LOGI(TAG, "CAN motor controller worker initialized successfully");
    return ESP_OK;
}

esp_err_t can_motctrl_worker_deinit(void)
{
    if (!s_worker_ctx.initialized) {
        return ESP_OK;
    }

    // Stop worker task
    if (s_worker_ctx.worker_task_handle != NULL) {
        vTaskDelete(s_worker_ctx.worker_task_handle);
        s_worker_ctx.worker_task_handle = NULL;
    }

    // Deinitialize CAN bus manager
    can_bus_manager_deinit();

    // Clean up resources
    if (s_worker_ctx.fragment_buffer != NULL) {
        free(s_worker_ctx.fragment_buffer);
        s_worker_ctx.fragment_buffer = NULL;
    }

    if (s_worker_ctx.operation_event_group != NULL) {
        vEventGroupDelete(s_worker_ctx.operation_event_group);
        s_worker_ctx.operation_event_group = NULL;
    }

    if (s_worker_ctx.event_queue != NULL) {
        vQueueDelete(s_worker_ctx.event_queue);
        s_worker_ctx.event_queue = NULL;
    }

    if (s_worker_ctx.can_rx_queue != NULL) {
        vQueueDelete(s_worker_ctx.can_rx_queue);
        s_worker_ctx.can_rx_queue = NULL;
    }

    s_worker_ctx.initialized = false;
    ESP_LOGI(TAG, "CAN motor controller worker deinitialized");
    return ESP_OK;
}

esp_err_t can_motctrl_worker_wait_pkg(motorcontroller_pkg_t *pkg, int timeout_sec)
{
    if (!s_worker_ctx.initialized) {
        ESP_LOGE(TAG, "Worker not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear previous package flag and event bits
    s_worker_ctx.pkg_ready = false;
    xEventGroupClearBits(s_worker_ctx.operation_event_group, 
                        MOTCTRL_WORKER_PKG_RECEIVED_BIT | MOTCTRL_WORKER_ERROR_BIT);

    // Set state to ready
    s_worker_ctx.state = CAN_WORKER_STATE_IDLE;
    s_worker_ctx.status_byte = WORKER_STATUS_READY;

    ESP_LOGI(TAG, "Waiting for package from manager (timeout: %d seconds)...", timeout_sec);

    // Wait for package
    EventBits_t bits = xEventGroupWaitBits(
        s_worker_ctx.operation_event_group,
        MOTCTRL_WORKER_PKG_RECEIVED_BIT | MOTCTRL_WORKER_ERROR_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    if (bits & MOTCTRL_WORKER_ERROR_BIT) {
        ESP_LOGE(TAG, "Error while waiting for package");
        return ESP_FAIL;
    }

    if (!(bits & MOTCTRL_WORKER_PKG_RECEIVED_BIT)) {
        ESP_LOGW(TAG, "Package wait timeout");
        s_worker_ctx.state = CAN_WORKER_STATE_IDLE;
        s_worker_ctx.status_byte = WORKER_STATUS_IDLE;
        return ESP_ERR_TIMEOUT;
    }

    // Copy received package
    *pkg = s_worker_ctx.received_pkg;
    s_worker_ctx.state = CAN_WORKER_STATE_PKG_RECEIVED;
    
    ESP_LOGI(TAG, "Package received successfully");
    return ESP_OK;
}

esp_err_t can_motctrl_worker_send_response(const motorcontroller_response_t *resp, int timeout_sec)
{
    if (!s_worker_ctx.initialized) {
        ESP_LOGE(TAG, "Worker not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare response
    s_worker_ctx.response_to_send = *resp;
    s_worker_ctx.resp_ready = true;
    s_worker_ctx.state = CAN_WORKER_STATE_RESP_READY;
    s_worker_ctx.status_byte = WORKER_STATUS_RESP_READY;

    ESP_LOGI(TAG, "Sending response...");

    // Send response immediately using bus manager
    esp_err_t ret = send_motorcontroller_response();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(ret));
        s_worker_ctx.status_byte = WORKER_STATUS_ERROR;
        return ret;
    }

    // Reset state
    s_worker_ctx.state = CAN_WORKER_STATE_IDLE;
    s_worker_ctx.status_byte = WORKER_STATUS_IDLE;
    s_worker_ctx.resp_ready = false;

    ESP_LOGI(TAG, "Response sent successfully");
    return ESP_OK;
}

esp_err_t can_motctrl_worker_set_working(void)
{
    s_worker_ctx.state = CAN_WORKER_STATE_WORKING;
    s_worker_ctx.status_byte = WORKER_STATUS_WORKING;
    ESP_LOGI(TAG, "Worker state set to WORKING");
    return ESP_OK;
}

esp_err_t can_motctrl_worker_set_response_ready(const motorcontroller_response_t *resp)
{
    s_worker_ctx.response_to_send = *resp;
    s_worker_ctx.resp_ready = true;
    s_worker_ctx.state = CAN_WORKER_STATE_RESP_READY;
    s_worker_ctx.status_byte = WORKER_STATUS_RESP_READY;
    
    ESP_LOGI(TAG, "Response set and ready for manager");
    return ESP_OK;
}

can_worker_state_t can_motctrl_worker_get_state(void)
{
    return s_worker_ctx.state;
}

bool can_motctrl_worker_is_ready(void)
{
    return (s_worker_ctx.initialized && s_worker_ctx.state == CAN_WORKER_STATE_IDLE);
}

static void can_worker_task(void *pvParameters)
{
    can_worker_event_t evt;
    twai_message_t can_msg;
    
    ESP_LOGI(TAG, "CAN worker task started");
    
    while (s_worker_ctx.initialized) {
        // Check for CAN messages first
        if (xQueueReceive(s_worker_ctx.can_rx_queue, &can_msg, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch (can_msg.identifier) {
                case CAN_ID_MOTCTRL_STATUS_REQ:
                    ESP_LOGD(TAG, "Status request received");
                    send_status_response();
                    break;
                    
                case CAN_ID_MOTCTRL_PKG_START:
                case CAN_ID_MOTCTRL_PKG_DATA:
                case CAN_ID_MOTCTRL_PKG_END:
                    process_package_frame(&can_msg);
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown CAN ID: 0x%lx", can_msg.identifier);
                    break;
            }
        }
        
        // Check for internal events
        if (xQueueReceive(s_worker_ctx.event_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch (evt) {
                case CAN_WORKER_EVT_RESP_REQUESTED:
                    send_motorcontroller_response();
                    break;
                    
                case CAN_WORKER_EVT_ERROR:
                    ESP_LOGE(TAG, "Worker error event");
                    s_worker_ctx.status_byte = WORKER_STATUS_ERROR;
                    xEventGroupSetBits(s_worker_ctx.operation_event_group, MOTCTRL_WORKER_ERROR_BIT);
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown event: %d", evt);
                    break;
            }
        }
        
        // Small delay to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGI(TAG, "CAN worker task exiting");
    vTaskDelete(NULL);
}

static esp_err_t send_status_response(void)
{
    twai_message_t message = {
        .identifier = CAN_ID_MOTCTRL_STATUS_RESP,
        .flags = TWAI_MSG_FLAG_NONE,
        .data_length_code = 1,
        .data = {s_worker_ctx.status_byte}
    };
    
    // Use bus manager to send message
    esp_err_t ret = can_bus_manager_send_message(&message, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send status response: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Status response sent: 0x%02X", s_worker_ctx.status_byte);
    }
    
    return ret;
}

static esp_err_t send_motorcontroller_response(void)
{
    if (!s_worker_ctx.resp_ready) {
        ESP_LOGW(TAG, "No response ready to send");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Serialize response using CAN fragmentation
    can_fragment_list_t frag_list;
    esp_err_t ret = can_serialize_resp(&s_worker_ctx.response_to_send, &frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize response: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Response serialized into %d fragments", frag_list.count);
    
    // Send start frame
    twai_message_t start_msg;
    ret = can_create_start_frame(frag_list.count, CAN_ID_MOTCTRL_RESP_START, &start_msg);
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
    
    // Send fragment messages using bus manager
    for (uint16_t i = 0; i < frag_list.count; i++) {
        ret = can_bus_manager_send_message(&frag_list.fragments[i], 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send fragment %d: %s", i, esp_err_to_name(ret));
            can_fragment_list_free(&frag_list);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between fragments
    }
    
    // Send end frame
    twai_message_t end_msg;
    ret = can_create_end_frame(CAN_ID_MOTCTRL_RESP_END, &end_msg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create end frame");
        can_fragment_list_free(&frag_list);
        return ret;
    }
    
    ret = can_bus_manager_send_message(&end_msg, 1000);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Response sent successfully (%d fragments)", frag_list.count);
        xEventGroupSetBits(s_worker_ctx.operation_event_group, MOTCTRL_WORKER_RESP_SENT_BIT);
    } else {
        ESP_LOGE(TAG, "Failed to send end frame: %s", esp_err_to_name(ret));
    }
    
    can_fragment_list_free(&frag_list);
    return ret;
}

static esp_err_t process_package_frame(const twai_message_t *message)
{
    esp_err_t ret = ESP_OK;
    
    switch (message->identifier) {
        case CAN_ID_MOTCTRL_PKG_START:
            // Reset fragment assembly
            s_worker_ctx.fragment_pos = 0;
            s_worker_ctx.expected_fragments = 0;
            s_worker_ctx.fragments_received = 0;
            
            if (message->data_length_code >= 2) {
                s_worker_ctx.expected_fragments = (message->data[0] << 8) | message->data[1];
                ESP_LOGI(TAG, "Package start: expecting %d fragments", s_worker_ctx.expected_fragments);
                
                // Allocate space for fragment list
                if (s_worker_ctx.rx_fragment_list.fragments != NULL) {
                    can_fragment_list_free(&s_worker_ctx.rx_fragment_list);
                }
                
                s_worker_ctx.rx_fragment_list.fragments = malloc(
                    s_worker_ctx.expected_fragments * sizeof(twai_message_t));
                if (s_worker_ctx.rx_fragment_list.fragments == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate fragment list");
                    return ESP_ERR_NO_MEM;
                }
                s_worker_ctx.rx_fragment_list.count = 0;
            }
            break;
            
        case CAN_ID_MOTCTRL_PKG_DATA:
            // Store fragment
            if (s_worker_ctx.rx_fragment_list.fragments != NULL && 
                s_worker_ctx.fragments_received < s_worker_ctx.expected_fragments) {
                
                s_worker_ctx.rx_fragment_list.fragments[s_worker_ctx.fragments_received] = *message;
                s_worker_ctx.fragments_received++;
                s_worker_ctx.rx_fragment_list.count = s_worker_ctx.fragments_received;
                
                ESP_LOGD(TAG, "Fragment %d/%d received", 
                        s_worker_ctx.fragments_received, s_worker_ctx.expected_fragments);
            } else {
                ESP_LOGE(TAG, "Unexpected fragment or buffer overflow");
                ret = ESP_FAIL;
            }
            break;
            
        case CAN_ID_MOTCTRL_PKG_END:
            // Process complete package
            if (s_worker_ctx.fragments_received == s_worker_ctx.expected_fragments && 
                s_worker_ctx.rx_fragment_list.fragments != NULL) {
                
                ret = can_deserialize_pkg(&s_worker_ctx.rx_fragment_list, &s_worker_ctx.received_pkg);
                
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Package received and validated successfully");
                    s_worker_ctx.pkg_ready = true;
                    s_worker_ctx.state = CAN_WORKER_STATE_PKG_RECEIVED;
                    s_worker_ctx.status_byte = WORKER_STATUS_WORKING;
                    xEventGroupSetBits(s_worker_ctx.operation_event_group, MOTCTRL_WORKER_PKG_RECEIVED_BIT);
                } else {
                    ESP_LOGE(TAG, "Package deserialization failed: %s", esp_err_to_name(ret));
                }
                
                // Clean up fragment list
                can_fragment_list_free(&s_worker_ctx.rx_fragment_list);
            } else {
                ESP_LOGE(TAG, "Fragment count mismatch: expected %d, got %d", 
                        s_worker_ctx.expected_fragments, s_worker_ctx.fragments_received);
                ret = ESP_FAIL;
            }
            
            if (ret != ESP_OK) {
                s_worker_ctx.status_byte = WORKER_STATUS_ERROR;
                xEventGroupSetBits(s_worker_ctx.operation_event_group, MOTCTRL_WORKER_ERROR_BIT);
            }
            break;
    }
    
    return ret;
}