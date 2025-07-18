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
