#include "can_motctrl_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_motctrl.h"

static const char *TAG = "MOTCTRL_MANAGER";

esp_err_t start_worker(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms, uint8_t retries)
{
    esp_err_t ret;
    
    // Subscribe to response IDs BEFORE sending package
    // Start and end frames: small queue (1-2), data frames: larger queue based on expected fragments
    ret = subscribe_fragment_ids(CAN_ID_MOTCTRL_RESP_START, 
                                CAN_ID_MOTCTRL_RESP_DATA, 
                                CAN_ID_MOTCTRL_RESP_END,
                                2,    // start queue size
                                60,   // data queue size (based on your 53 fragments)
                                2);   // end queue size
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to response IDs");
        return ret;
    }
    
    // Serialize package
    can_fragment_list_t pkg_frag_list = {0};
    ret = can_serialize_pkg(pkg, &pkg_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Package serialization failed");
        unsubscribe_fragment_ids(CAN_ID_MOTCTRL_RESP_START, 
                                CAN_ID_MOTCTRL_RESP_DATA, 
                                CAN_ID_MOTCTRL_RESP_END);
        return ret;
    }
    
    // Send package fragments with retries
    for (int attempt = 0; attempt <= retries; attempt++) {
        ret = send_fragment_list(CAN_ID_MOTCTRL_PKG_START, 
                                CAN_ID_MOTCTRL_PKG_DATA, 
                                CAN_ID_MOTCTRL_PKG_END, 
                                &pkg_frag_list);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "Send attempt %d failed, retrying...", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    can_fragment_list_free(&pkg_frag_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send package after %d attempts", retries + 1);
        unsubscribe_fragment_ids(CAN_ID_MOTCTRL_RESP_START, 
                                CAN_ID_MOTCTRL_RESP_DATA, 
                                CAN_ID_MOTCTRL_RESP_END);
    }
    
    return ret;
}

esp_err_t wait_for_worker(motorcontroller_response_t *resp, uint32_t wait_offset_ms, uint32_t timeout_ms)
{
    // Wait for worker to start processing
    if (wait_offset_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_offset_ms));
    }
    
    // Receive response using subscription system
    can_fragment_list_t resp_frag_list = {0};
    esp_err_t ret = receive_fragment_list(CAN_ID_MOTCTRL_RESP_START, 
                                         CAN_ID_MOTCTRL_RESP_DATA, 
                                         CAN_ID_MOTCTRL_RESP_END, 
                                         &resp_frag_list,
                                         timeout_ms);
    
    // Clean up subscriptions after receiving (or failure)
    unsubscribe_fragment_ids(CAN_ID_MOTCTRL_RESP_START, 
                            CAN_ID_MOTCTRL_RESP_DATA, 
                            CAN_ID_MOTCTRL_RESP_END);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response fragments");
        return ret;
    }
    
    // Deserialize response
    ret = can_deserialize_resp(&resp_frag_list, resp);
    can_fragment_list_free(&resp_frag_list);
    
    return ret;
}

void manager_task(void *arg)
{
    ESP_LOGI(TAG, "Manager task started");
    
    while (1) {
        // Create work package
        motorcontroller_pkg_t pkg;
        motorcontroller_pkg_init_default(&pkg);
        
        // Send to worker with retries
        esp_err_t ret = start_worker(&pkg, 1000, 3);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start worker");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Wait for response (with estimated timeout offset)
        motorcontroller_response_t resp;
        uint32_t estimated_time = calculate_operation_timeout(
            pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent,
            pkg.end_depth, pkg.static_points, pkg.samples, pkg.static_poll_interval_s
        ) * 1000;  // Convert to ms
        
        ret = wait_for_worker(&resp, estimated_time, estimated_time + 10000);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received response: result=%s, time=%ds", 
                    esp_err_to_name(resp.result), resp.working_time);
        } else {
            ESP_LOGE(TAG, "Failed to get response: %s", esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));  // Wait before next cycle
    }
}