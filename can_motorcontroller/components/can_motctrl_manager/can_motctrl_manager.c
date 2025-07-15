
// components/motctrl_manager/motctrl_manager.c
#include "can_motctrl_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "motctrl_manager";

static bool initialized = false;

esp_err_t motctrl_manager_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    initialized = true;
    ESP_LOGI(TAG, "Motor controller manager initialized");
    return ESP_OK;
}

esp_err_t motctrl_manager_check_status(worker_status_t *status, uint32_t timeout_ms)
{
    if (!initialized || !status) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Requesting worker status...");
    
    esp_err_t ret = motctrl_send_status_request(timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send status request: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = motctrl_wait_status_response(status, timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive status response: %s", esp_err_to_name(ret));
        return ret;
    }

    const char* status_str;
    switch (*status) {
        case WORKER_STATUS_IDLE: status_str = "IDLE"; break;
        case WORKER_STATUS_READY: status_str = "READY"; break;
        case WORKER_STATUS_WORKING: status_str = "WORKING"; break;
        case WORKER_STATUS_RESP_READY: status_str = "RESP_READY"; break;
        case WORKER_STATUS_ERROR: status_str = "ERROR"; break;
        default: status_str = "UNKNOWN"; break;
    }
    
    ESP_LOGI(TAG, "Worker status: %s", status_str);
    return ESP_OK;
}

esp_err_t motctrl_manager_execute_work(uint32_t timeout_ms, motctrl_operation_result_t *result)
{
    if (!initialized || !result) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(motctrl_operation_result_t));
    
    uint64_t start_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Starting work execution...");

    // Step 1: Check if worker is ready
    worker_status_t status;
    esp_err_t ret = motctrl_manager_check_status(&status, timeout_ms / 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to check initial worker status");
        result->result = ret;
        return ret;
    }

    if (status != WORKER_STATUS_READY && status != WORKER_STATUS_IDLE) {
        ESP_LOGE(TAG, "Worker not ready for work (status: %d)", status);
        result->result = ESP_ERR_MOTCTRL_NOT_READY;
        result->final_status = status;
        return ESP_ERR_MOTCTRL_NOT_READY;
    }

    // Step 2: Send start command
    motorcontroller_pkg_t start_pkg;
    ret = motctrl_create_start_command(&start_pkg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create start command");
        result->result = ret;
        return ret;
    }

    ESP_LOGI(TAG, "Sending start command to worker...");
    ret = motctrl_send_package(&start_pkg, timeout_ms / 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start command: %s", esp_err_to_name(ret));
        result->result = ret;
        return ret;
    }

    // Step 3: Wait for confirmation that command was received
    ESP_LOGI(TAG, "Waiting for worker to confirm command receipt...");
    vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay for worker to process
    
    ret = motctrl_manager_check_status(&status, timeout_ms / 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get confirmation status");
        result->result = ret;
        return ret;
    }

    if (status != WORKER_STATUS_WORKING) {
        ESP_LOGE(TAG, "Worker did not start working (status: %d)", status);
        result->result = ESP_ERR_MOTCTRL_PROTOCOL_ERROR;
        result->final_status = status;
        return ESP_ERR_MOTCTRL_PROTOCOL_ERROR;
    }

    ESP_LOGI(TAG, "Worker confirmed command receipt and started working");

    // Step 4: Wait for work completion
    ESP_LOGI(TAG, "Waiting for worker to complete work...");
    uint32_t remaining_timeout = timeout_ms - ((esp_timer_get_time() - start_time) / 1000);
    
    // Poll for status until work is complete
    do {
        vTaskDelay(pdMS_TO_TICKS(500)); // Poll every 500ms
        ret = motctrl_manager_check_status(&status, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check work status");
            result->result = ret;
            return ret;
        }
        
        remaining_timeout = timeout_ms - ((esp_timer_get_time() - start_time) / 1000);
        if (remaining_timeout <= 0) {
            ESP_LOGE(TAG, "Timeout waiting for work completion");
            result->result = ESP_ERR_TIMEOUT;
            result->final_status = status;
            return ESP_ERR_TIMEOUT;
        }
        
    } while (status == WORKER_STATUS_WORKING);

    if (status != WORKER_STATUS_RESP_READY) {
        ESP_LOGE(TAG, "Worker finished with unexpected status: %d", status);
        result->result = ESP_ERR_MOTCTRL_PROTOCOL_ERROR;
        result->final_status = status;
        return ESP_ERR_MOTCTRL_PROTOCOL_ERROR;
    }

    // Step 5: Receive work response
    ESP_LOGI(TAG, "Receiving work response from worker...");
    motorcontroller_response_t response;
    ret = motctrl_receive_response(&response, remaining_timeout);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive work response: %s", esp_err_to_name(ret));
        result->result = ret;
        result->final_status = status;
        return ret;
    }

    // Step 6: Final status check
    ret = motctrl_manager_check_status(&status, 1000);
    if (ret == ESP_OK) {
        result->final_status = status;
    }

    uint64_t end_time = esp_timer_get_time();
    result->operation_time_ms = (end_time - start_time) / 1000;
    result->result = ESP_OK;
    result->work_result = 0; // Extract from response based on buoye_structs.h

    ESP_LOGI(TAG, "Work execution completed successfully in %d ms", result->operation_time_ms);
    return ESP_OK;
}