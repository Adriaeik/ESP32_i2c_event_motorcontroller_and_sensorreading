#include "can_motctrl_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_motctrl.h"

static const char *TAG = "MOTCTRL_MANAGER";

esp_err_t start_worker(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms)
{
    esp_err_t ret = can_subscribe_set(CAN_MTOTCTRL_MANAGER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manager failed to subscribe to response channels");
        return ret;
    }
    
    // Serialize and send package
    can_fragment_list_t pkg_frag_list = {0};
    ret = can_serialize_pkg(pkg, &pkg_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Package serialization failed");
        goto cleanup;
    }
    
    
    ret = send_fragment_list_simple_ack(CAN_MTOTCTRL_MANAGER, 
                                        &pkg_frag_list,
                                        timeout_ms);
    
    can_fragment_list_free(&pkg_frag_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send package with ACK confirmation");
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "Package sent successfully with ACK confirmation");
    return ESP_OK;
    
cleanup:
    // Clean unsubscribe
    can_unsubscribe_set(CAN_MTOTCTRL_MANAGER);
    return ret;
}

esp_err_t wait_for_worker(motorcontroller_response_t *resp, uint32_t wait_offset_ms, uint32_t timeout_ms) {
    if (wait_offset_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_offset_ms));
    }
    
    // FIXED: Receive with correct ACK ID (0x108 - manager sends ACK here)
    can_fragment_list_t resp_frag_list = {0};
    esp_err_t ret = receive_fragment_list_simple_ack(CAN_MTOTCTRL_MANAGER,
                                                     &resp_frag_list,
                                                     timeout_ms);
    
    // Clean up subscriptions
    can_unsubscribe_set(CAN_MTOTCTRL_MANAGER); 
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response fragments");
        return ret;
    }
    
    // Deserialize response
    ret = can_deserialize_resp(&resp_frag_list, resp);
    can_fragment_list_free(&resp_frag_list);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Response received and acknowledged successfully");
    }
    
    return ret;
}
