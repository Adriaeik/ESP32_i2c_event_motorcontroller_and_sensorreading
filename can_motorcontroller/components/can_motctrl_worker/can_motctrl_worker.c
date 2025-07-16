#include "can_motctrl_worker.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_motctrl.h"

static const char *TAG = "MOTCTRL_WORKER";

esp_err_t receive_work_package(motorcontroller_pkg_t *pkg, uint32_t timeout_ms)
{
    can_fragment_list_t pkg_frag_list = {0};
    esp_err_t ret = receive_fragment_list(CAN_ID_MOTCTRL_PKG_START, 
                                         CAN_ID_MOTCTRL_PKG_DATA, 
                                         CAN_ID_MOTCTRL_PKG_END, 
                                         &pkg_frag_list,
                                         timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = can_deserialize_pkg(&pkg_frag_list, pkg);
    can_fragment_list_free(&pkg_frag_list);
    
    return ret;
}

esp_err_t send_work_response(const motorcontroller_response_t *resp, uint32_t timeout_ms, uint8_t retries)
{
    // Serialize response
    can_fragment_list_t resp_frag_list = {0};
    esp_err_t ret = can_serialize_resp(resp, &resp_frag_list);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Send with retries
    for (int attempt = 0; attempt <= retries; attempt++) {
        ret = send_fragment_list(CAN_ID_MOTCTRL_RESP_START, 
                                CAN_ID_MOTCTRL_RESP_DATA, 
                                CAN_ID_MOTCTRL_RESP_END, 
                                &resp_frag_list);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "Response send attempt %d failed, retrying...", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    can_fragment_list_free(&resp_frag_list);
    return ret;
}

void worker_task(void *arg)
{
    ESP_LOGI(TAG, "Worker task started");
    
    // Subscribe to package IDs at startup - these stay subscribed for the lifetime of the task
    esp_err_t ret = subscribe_fragment_ids(CAN_ID_MOTCTRL_PKG_START, 
                                          CAN_ID_MOTCTRL_PKG_DATA, 
                                          CAN_ID_MOTCTRL_PKG_END,
                                          2,    // start queue size
                                          60,   // data queue size (based on your 53 fragments)
                                          2);   // end queue size
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to package IDs, worker cannot function");
        return;
    }
    
    ESP_LOGI(TAG, "Worker subscribed to package IDs, ready to receive work");
    
    while (1) {
        // Wait for work package
        motorcontroller_pkg_t pkg;
        ret = receive_work_package(&pkg, portMAX_DELAY);
        
        if (ret == ESP_OK) {
            // Simulate work based on package
            int work_time = calculate_operation_timeout(
                pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent,
                pkg.end_depth, pkg.static_points, pkg.samples, pkg.static_poll_interval_s
            );
            
            ESP_LOGI(TAG, "Starting work for %d seconds", work_time);
            vTaskDelay(pdMS_TO_TICKS(work_time * 1000));
            
            // Create response
            motorcontroller_response_t resp;
            motorcontroller_response_init_default(&resp);
            resp.result = ESP_OK;
            resp.working_time = work_time;
            
            // Send response
            ret = send_work_response(&resp, 1000, 3);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to receive package: %s", esp_err_to_name(ret));
            // Small delay before retrying to avoid spam
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    // Cleanup subscriptions if task ever exits (unlikely)
    unsubscribe_fragment_ids(CAN_ID_MOTCTRL_PKG_START, 
                            CAN_ID_MOTCTRL_PKG_DATA, 
                            CAN_ID_MOTCTRL_PKG_END);
}