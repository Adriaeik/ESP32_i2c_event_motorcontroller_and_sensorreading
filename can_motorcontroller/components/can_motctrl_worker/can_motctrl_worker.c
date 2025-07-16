#include "can_motctrl_worker.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_motctrl.h"

static const char *TAG = "MOTCTRL_WORKER";

esp_err_t receive_work_package(motorcontroller_pkg_t *pkg, uint32_t timeout_ms) {
    // FIXED: Receive with correct ACK ID (0x103 - worker sends ACK here)
    can_fragment_list_t pkg_frag_list = {0};
    esp_err_t ret = receive_fragment_list_simple_ack(CAN_MTOTCTRL_WORKER,
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
    
    ret = send_fragment_list_simple_ack(CAN_MTOTCTRL_WORKER, 
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
    
    while (1) {
        // Subscribe only when ready to receive work
        esp_err_t ret = can_subscribe_set(CAN_MTOTCTRL_WORKER);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to subscribe to package channels");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        ESP_LOGI(TAG, "Worker ready to receive work");
        
        // Wait for work package
        motorcontroller_pkg_t pkg;
        ret = receive_work_package(&pkg, portMAX_DELAY);
        
        if (ret == ESP_OK) {
            
            print_motorcontroller_pkg_info(&pkg, "Worker");
            
            // Calculate and perform work
            int work_time = calculate_operation_timeout(
                pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent,
                pkg.end_depth, pkg.static_points, pkg.samples, pkg.static_poll_interval_s
            );
            
            ESP_LOGI(TAG, "Starting work for %d seconds", work_time);
            vTaskDelay(pdMS_TO_TICKS(work_time * 1000));
            
            // Create and send response
            motorcontroller_response_t resp;
            motorcontroller_response_init_default(&resp);
            resp.STATE = pkg.STATE;
            resp.result = ESP_OK;
            resp.working_time = work_time;
            resp.estimated_cm_per_s = pkg.prev_estimated_cm_per_s; // or calculate new estimate
            
            print_motorcontroller_response_info(&resp, "Worker");
            ret = send_work_response(&resp, 5000);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send response: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to receive package: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        can_unsubscribe_set(CAN_MTOTCTRL_WORKER); 
    }
}

// // motorcontroller work
// // wake up
// ESP_ERROR_CHECK(can_bus_manager_init());
// can_subscribe_set(CAN_MTOTCTRL_WORKER);
// motorcontroller_pkg_t pkg;
// receive_work_package(&pkg, portMAX_DELAY);
// motorcontroller_response_t resp;//start_work will fill this in ready to send
// start_work(&pkg, &resp);
// send_work_response(&resp, timeout);
// can_unsubscribe_set(CAN_MTOTCTRL_WORKER); 
// can_bus_manager_deinit();
// deepsleep();

// // motorcontroller manager
// ESP_ERROR_CHECK(can_bus_manager_init());
// motorcontroller_pkg_t pkg;
// set_system_motorcontroller_pkg(&pkg, STATE); // reads from file or something, updates based on time
// start_worker(&pkg, timeout); // handels subscription and fault
// motorcontroller_response_t resp;
// uint32_t estimated_time = calculate_operation_timet(&pkg)
// wait_for_worker(&resp, estimated_time, estimated_time + 10000); // handles usubscribe too
// can_bus_manager_deinit();
// update_and_store_pkg(&pkg, &resp);
// // exit and wait for new sycle 