#include "can_motctrl_worker.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_motctrl.h"

static const char *TAG = "MOTCTRL_WORKER";

esp_err_t receive_work_package(motorcontroller_pkg_t *pkg, uint32_t timeout_ms) {
    // FIXED: Receive with correct ACK ID (0x103 - worker sends ACK here)
    can_fragment_list_t pkg_frag_list = {0};
    esp_err_t ret = receive_fragment_list_simple_ack(CAN_ID_MOTCTRL_PKG_START, 
                                                     CAN_ID_MOTCTRL_PKG_DATA, 
                                                     CAN_ID_MOTCTRL_PKG_END,
                                                     CAN_ID_MOTCTRL_PKG_ACK,  // 0x103 - worker sends ACK here
                                                     &pkg_frag_list,
                                                     timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = can_deserialize_pkg(&pkg_frag_list, pkg);
    can_fragment_list_free(&pkg_frag_list);
    
    return ret;
}

esp_err_t send_work_response(const motorcontroller_response_t *resp, uint32_t timeout_ms) {
    // Serialize response
    can_fragment_list_t resp_frag_list = {0};
    esp_err_t ret = can_serialize_resp(resp, &resp_frag_list);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // FIXED: Send with correct ACK ID (0x108 - manager sends ACK here)
    ret = send_fragment_list_simple_ack(CAN_ID_MOTCTRL_RESP_START, 
                                        CAN_ID_MOTCTRL_RESP_DATA, 
                                        CAN_ID_MOTCTRL_RESP_END,
                                        CAN_ID_MOTCTRL_RESP_ACK,  // 0x108 - manager sends ACK here
                                        &resp_frag_list,
                                        timeout_ms);
    
    can_fragment_list_free(&resp_frag_list);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Response sent successfully with ACK confirmation");
    } else {
        ESP_LOGE(TAG, "Failed to send response with ACK: %s", esp_err_to_name(ret));
    }
    
    return ret;
}






void worker_task(void *arg) {
    ESP_LOGI(TAG, "Worker task started");
    
    // Subscribe once at startup
    esp_err_t ret ;
    if (can_subscribe_set(CAN_SUBSCRIPTION_SET_WORKER) != ESP_OK) {
        ESP_LOGE(TAG, "Worker failed to subscribe to package channels, cannot function");
        return;
    }
    
    ESP_LOGI(TAG, "Worker subscribed to package channels, ready to receive work");
    
    while (1) {
        // Wait for work package
        motorcontroller_pkg_t pkg;
        ret = receive_work_package(&pkg, portMAX_DELAY);
        
        if (ret == ESP_OK) {
            print_motorcontroller_pkg_info(&pkg, "Worker");
            
            // Simulate work
            int work_time = calculate_operation_timeout(
                pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent,
                pkg.end_depth, pkg.static_points, pkg.samples, pkg.static_poll_interval_s
            );
            
            ESP_LOGI(TAG, "Starting work for %d seconds", work_time);
            vTaskDelay(pdMS_TO_TICKS(work_time * 1000));
            
            // Create and send response
            motorcontroller_response_t resp;
            motorcontroller_response_init_default(&resp);
            resp.result = ESP_OK;
            resp.working_time = work_time;
            
            print_motorcontroller_response_info(&resp, "Worker");
            ret = send_work_response(&resp, 5000);  // 5 second timeout
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to receive package: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));  // Brief pause before retry
        }
    }
}