#include "can_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "can_serde_helper.h"
#include "can_motctrl_common.h"  // Include your common header

// Configuration - Uncomment ONE of these to set the role
#define ROLE_MANAGER
// #define ROLE_WORKER

static const char *TAG = "MOTCTRL_TEST";

// Test local serialization/deserialization
void test_local_serde() {
    ESP_LOGI(TAG, "===== LOCAL SERDE TEST START =====");
    
    // Create test package using your provided function
    motorcontroller_pkg_t original_pkg;
    motorcontroller_pkg_init_default(&original_pkg);
    print_motorcontroller_pkg_info(&original_pkg, TAG);
    
    // Serialize package
    can_fragment_list_t pkg_frag_list = {0};
    esp_err_t ret = can_serialize_pkg(&original_pkg, &pkg_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Package serialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Package serialized into %d fragments", pkg_frag_list.count);
    
    // Deserialize package
    motorcontroller_pkg_t deserialized_pkg;
    ret = can_deserialize_pkg(&pkg_frag_list, &deserialized_pkg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Package deserialization failed: %s", esp_err_to_name(ret));
        can_fragment_list_free(&pkg_frag_list);
        return;
    }
    print_motorcontroller_pkg_info(&deserialized_pkg, TAG);
    
    // Free memory
    can_fragment_list_free(&pkg_frag_list);
    
    // Repeat for response
    motorcontroller_response_t original_resp;
    motorcontroller_response_init_default(&original_resp);
    print_motorcontroller_response_info(&original_resp, TAG);
    
    can_fragment_list_t resp_frag_list = {0};
    ret = can_serialize_resp(&original_resp, &resp_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Response serialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Response serialized into %d fragments", resp_frag_list.count);
    
    motorcontroller_response_t deserialized_resp;
    ret = can_deserialize_resp(&resp_frag_list, &deserialized_resp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Response deserialization failed: %s", esp_err_to_name(ret));
        can_fragment_list_free(&resp_frag_list);
        return;
    }
    print_motorcontroller_response_info(&deserialized_resp, TAG);
    
    can_fragment_list_free(&resp_frag_list);
    
    ESP_LOGI(TAG, "===== LOCAL SERDE TEST COMPLETE =====");
}

// Helper function to send a fragment list
esp_err_t send_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                             const can_fragment_list_t *frag_list) {
    // Send start frame
    can_message_t start_msg;
    can_create_start_frame(frag_list->count, start_id, &start_msg);
    esp_err_t ret = can_bus_send_message(&start_msg, CAN_PRIORITY_HIGH, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start frame");
        return ret;
    }
    
    // Send all data fragments
    for (int i = 0; i < frag_list->count; i++) {
        can_message_t frag_msg = frag_list->fragments[i];
        frag_msg.identifier = data_id;  // Set correct ID
        ret = can_bus_send_message(&frag_msg, CAN_PRIORITY_HIGH, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send fragment %d/%d", i+1, frag_list->count);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between fragments
    }
    
    // Send end frame
    can_message_t end_msg;
    can_create_end_frame(end_id, &end_msg);
    ret = can_bus_send_message(&end_msg, CAN_PRIORITY_HIGH, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send end frame");
        return ret;
    }
    
    return ESP_OK;
}

// Helper function to receive a fragment list
esp_err_t receive_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                can_fragment_list_t *frag_list) {
    // Receive start frame
    can_message_t start_msg;
    esp_err_t ret = can_bus_receive_message(&start_msg, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive start frame");
        return ret;
    }
    
    if (start_msg.identifier != start_id) {
        ESP_LOGE(TAG, "Unexpected start frame ID: 0x%X (expected 0x%X)", 
                start_msg.identifier, start_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Extract total fragments (2 bytes in data[0] and data[1])
    uint16_t total_fragments = (start_msg.data[0] << 8) | start_msg.data[1];
    ESP_LOGI(TAG, "Expecting %d fragments", total_fragments);
    
    // Allocate memory for fragments
    frag_list->fragments = malloc(total_fragments * sizeof(can_message_t));
    if (!frag_list->fragments) {
        ESP_LOGE(TAG, "Memory allocation failed for %d fragments", total_fragments);
        return ESP_ERR_NO_MEM;
    }
    frag_list->count = total_fragments;
    
    // Receive all data fragments
    for (int i = 0; i < total_fragments; i++) {
        ret = can_bus_receive_message(&frag_list->fragments[i], portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive fragment %d/%d", i+1, total_fragments);
            can_fragment_list_free(frag_list);
            return ret;
        }
        
        if (frag_list->fragments[i].identifier != data_id) {
            ESP_LOGE(TAG, "Unexpected fragment ID: 0x%X (expected 0x%X) at pos %d", 
                    frag_list->fragments[i].identifier, data_id, i);
            can_fragment_list_free(frag_list);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    
    // Receive end frame (optional)
    can_message_t end_msg;
    ret = can_bus_receive_message(&end_msg, 100);
    if (ret == ESP_OK && end_msg.identifier != end_id) {
        ESP_LOGW(TAG, "Unexpected end frame ID: 0x%X (expected 0x%X)", 
                end_msg.identifier, end_id);
    }
    
    return ESP_OK;
}

#ifdef ROLE_MANAGER

void manager_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));  // Let worker initialize
    
    // Create test package using your provided function
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    print_motorcontroller_pkg_info(&pkg, TAG);
    
    // Serialize package
    can_fragment_list_t pkg_frag_list = {0};
    esp_err_t ret = can_serialize_pkg(&pkg, &pkg_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manager: Package serialization failed");
        return;
    }
    
    // Send package fragments
    ret = send_fragment_list(CAN_ID_MOTCTRL_PKG_START, 
                            CAN_ID_MOTCTRL_PKG_DATA, 
                            CAN_ID_MOTCTRL_PKG_END, 
                            &pkg_frag_list);
    can_fragment_list_free(&pkg_frag_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manager: Failed to send package");
        return;
    }
    
    ESP_LOGI(TAG, "Manager: Package sent, waiting for response...");
    
    // Receive response
    can_fragment_list_t resp_frag_list = {0};
    ret = receive_fragment_list(CAN_ID_MOTCTRL_RESP_START, 
                               CAN_ID_MOTCTRL_RESP_DATA, 
                               CAN_ID_MOTCTRL_RESP_END, 
                               &resp_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manager: Failed to receive response fragments");
        return;
    }
    
    // Deserialize response
    motorcontroller_response_t resp;
    ret = can_deserialize_resp(&resp_frag_list, &resp);
    can_fragment_list_free(&resp_frag_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manager: Response deserialization failed");
        return;
    }
    
    print_motorcontroller_response_info(&resp, TAG);
    ESP_LOGI(TAG, "Manager: TEST COMPLETE!");
}

#endif  // ROLE_MANAGER

#ifdef ROLE_WORKER

void worker_task(void *arg) {
    ESP_LOGI(TAG, "Worker: Waiting for package...");
    
    // Receive package
    can_fragment_list_t pkg_frag_list = {0};
    esp_err_t ret = receive_fragment_list(CAN_ID_MOTCTRL_PKG_START, 
                                         CAN_ID_MOTCTRL_PKG_DATA, 
                                         CAN_ID_MOTCTRL_PKG_END, 
                                         &pkg_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Worker: Failed to receive package fragments");
        return;
    }
    
    // Deserialize package
    motorcontroller_pkg_t pkg;
    ret = can_deserialize_pkg(&pkg_frag_list, &pkg);
    can_fragment_list_free(&pkg_frag_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Worker: Package deserialization failed");
        return;
    }
    
    print_motorcontroller_pkg_info(&pkg, TAG);
    
    // Simulate work
    ESP_LOGI(TAG, "Worker: Simulating work...");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Simulate 2 seconds of work
    
    // Create response using your provided function
    motorcontroller_response_t resp;
    motorcontroller_response_init_default(&resp);
    print_motorcontroller_response_info(&resp, TAG);
    
    // Serialize response
    can_fragment_list_t resp_frag_list = {0};
    ret = can_serialize_resp(&resp, &resp_frag_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Worker: Response serialization failed");
        return;
    }
    
    // Send response
    ret = send_fragment_list(CAN_ID_MOTCTRL_RESP_START, 
                            CAN_ID_MOTCTRL_RESP_DATA, 
                            CAN_ID_MOTCTRL_RESP_END, 
                            &resp_frag_list);
    can_fragment_list_free(&resp_frag_list);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Worker: Failed to send response");
        return;
    }
    
    ESP_LOGI(TAG, "Worker: Response sent");
    ESP_LOGI(TAG, "Worker: TEST COMPLETE!");
}

#endif  // ROLE_WORKER

void app_main() {
    // Initialize CAN bus manager
    esp_err_t ret = can_bus_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN bus manager init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "CAN bus manager initialized");
    
    // Initialize LED for visual feedback
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int led_state = 0;
    
    // Run local serialization test first
    test_local_serde();
    
    // Create role-specific task
    #ifdef ROLE_MANAGER
    ESP_LOGI(TAG, "Starting as MANAGER");
    xTaskCreate(manager_task, "manager_task", 8192, NULL, 5, NULL);
    #endif
    
    #ifdef ROLE_WORKER
    ESP_LOGI(TAG, "Starting as WORKER");
    xTaskCreate(worker_task, "worker_task", 8192, NULL, 5, NULL);
    #endif
    
    // Blink LED to show device is alive
    while (1) {
        gpio_set_level(GPIO_NUM_2, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}