// components/motctrl_worker/motctrl_worker.c
#include "can_motctrl_worker.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "motctrl_worker";

// Worker state
static bool initialized = false;
static TaskHandle_t worker_task_handle = NULL;
static worker_status_t current_status = WORKER_STATUS_IDLE;
static SemaphoreHandle_t status_mutex = NULL;
static motctrl_worker_config_t worker_config = {0};

// Internal functions
static void worker_task(void *pvParameters);
static esp_err_t handle_status_request(void);
static esp_err_t process_work_package(const motorcontroller_pkg_t *pkg);
static void set_worker_status(worker_status_t new_status);

static void set_worker_status(worker_status_t new_status)
{
    if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (current_status != new_status) {
            current_status = new_status;
            
            const char* status_str;
            switch (new_status) {
                case WORKER_STATUS_IDLE: status_str = "IDLE"; break;
                case WORKER_STATUS_READY: status_str = "READY"; break;
                case WORKER_STATUS_WORKING: status_str = "WORKING"; break;
                case WORKER_STATUS_RESP_READY: status_str = "RESP_READY"; break;
                case WORKER_STATUS_ERROR: status_str = "ERROR"; break;
                default: status_str = "UNKNOWN"; break;
            }
            ESP_LOGI(TAG, "Status changed to: %s", status_str);
        }
        xSemaphoreGive(status_mutex);
    }
}

static esp_err_t handle_status_request(void)
{
    worker_status_t status = motctrl_worker_get_status();
    esp_err_t ret = motctrl_send_status_response(status, 1000);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Sent status response: %d", status);
    } else {
        ESP_LOGE(TAG, "Failed to send status response: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t process_work_package(const motorcontroller_pkg_t *pkg)
{
    ESP_LOGI(TAG, "Processing work package...");
    
    // Change status to working
    set_worker_status(WORKER_STATUS_WORKING);
    
    // Simulate work
    ESP_LOGI(TAG, "Simulating work for %d ms...", worker_config.work_simulation_time_ms);
    
    uint32_t work_chunks = worker_config.work_simulation_time_ms / 100; // 100ms chunks
    for (uint32_t i = 0; i < work_chunks; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check for status requests during work
        can_message_t msg;
        if (can_bus_receive_message(&msg, 0) == ESP_OK) { // Non-blocking receive
            if (msg.identifier == CAN_ID_MOTCTRL_STATUS_REQ) {
                handle_status_request();
            }
        }
        
        // Log progress
        if (i % 10 == 0) {
            ESP_LOGI(TAG, "Work progress: %d%%", (i * 100) / work_chunks);
        }
    }
    
    // Handle any remaining time
    uint32_t remaining_ms = worker_config.work_simulation_time_ms % 100;
    if (remaining_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(remaining_ms));
    }
    
    ESP_LOGI(TAG, "Work simulation completed");
    
    // Create and send response
    motorcontroller_response_t response;
    esp_err_t ret = motctrl_create_work_response(&response, 0); // 0 = success
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create work response");
        set_worker_status(WORKER_STATUS_ERROR);
        return ret;
    }
    
    // Change status to response ready
    set_worker_status(WORKER_STATUS_RESP_READY);
    
    ESP_LOGI(TAG, "Sending work response...");
    ret = motctrl_send_response(&response, 5000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send work response: %s", esp_err_to_name(ret));
        set_worker_status(WORKER_STATUS_ERROR);
        return ret;
    }
    
    ESP_LOGI(TAG, "Work response sent successfully");
    
    // Return to ready state
    set_worker_status(WORKER_STATUS_READY);
    return ESP_OK;
}

static void worker_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Worker task started");
    set_worker_status(WORKER_STATUS_READY);
    
    while (1) {
        can_message_t msg;
        
        // Wait for incoming messages
        esp_err_t ret = can_bus_receive_message(&msg, pdMS_TO_TICKS(worker_config.status_poll_interval_ms));
        
        if (ret == ESP_OK) {
            if (msg.identifier == CAN_ID_MOTCTRL_STATUS_REQ) {
                // Handle status request
                handle_status_request();
            }
            else if (msg.identifier == CAN_ID_MOTCTRL_PKG_START) {
                // Start of work package
                ESP_LOGI(TAG, "Received work package start frame");
                
                motorcontroller_pkg_t pkg;
                ret = motctrl_receive_package(&pkg, 10000); // 10 second timeout for full package
                
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Work package received successfully");
                    process_work_package(&pkg);
                } else {
                    ESP_LOGE(TAG, "Failed to receive complete work package: %s", esp_err_to_name(ret));
                    set_worker_status(WORKER_STATUS_ERROR);
                    
                    // Brief delay before returning to ready
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    set_worker_status(WORKER_STATUS_READY);
                }
            }
            else {
                ESP_LOGD(TAG, "Received unexpected message ID: 0x%x", msg.identifier);
            }
        }
        else if (ret == ESP_ERR_TIMEOUT) {
            // Normal timeout - continue loop
            ESP_LOGD(TAG, "Worker task timeout - continuing");
        }
        else {
            ESP_LOGE(TAG, "Error receiving message: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000)); // Brief delay before retry
        }
    }
}

esp_err_t motctrl_worker_init(const motctrl_worker_config_t *config)
{
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    worker_config = *config;
    
    // Set defaults if not specified
    if (worker_config.work_simulation_time_ms == 0) {
        worker_config.work_simulation_time_ms = 3000; // 3 seconds default
    }
    if (worker_config.status_poll_interval_ms == 0) {
        worker_config.status_poll_interval_ms = 500; // 500ms default
    }

    // Create mutex for status
    status_mutex = xSemaphoreCreateMutex();
    if (!status_mutex) {
        ESP_LOGE(TAG, "Failed to create status mutex");
        return ESP_ERR_NO_MEM;
    }

    set_worker_status(WORKER_STATUS_IDLE);
    initialized = true;

    ESP_LOGI(TAG, "Worker initialized (work_time: %d ms, poll_interval: %d ms)", 
             worker_config.work_simulation_time_ms, worker_config.status_poll_interval_ms);

    if (worker_config.auto_start) {
        return motctrl_worker_start();
    }

    return ESP_OK;
}

esp_err_t motctrl_worker_deinit(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop worker task if running
    motctrl_worker_stop();

    if (status_mutex) {
        vSemaphoreDelete(status_mutex);
        status_mutex = NULL;
    }

    initialized = false;
    ESP_LOGI(TAG, "Worker deinitialized");
    return ESP_OK;
}

esp_err_t motctrl_worker_start(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (worker_task_handle != NULL) {
        ESP_LOGW(TAG, "Worker task already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t result = xTaskCreate(
        worker_task,
        "motctrl_worker",
        4096,
        NULL,
        5,
        &worker_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create worker task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Worker task started");
    return ESP_OK;
}

esp_err_t motctrl_worker_stop(void)
{
    if (worker_task_handle != NULL) {
        vTaskDelete(worker_task_handle);
        worker_task_handle = NULL;
        set_worker_status(WORKER_STATUS_IDLE);
        ESP_LOGI(TAG, "Worker task stopped");
    }

    return ESP_OK;
}

worker_status_t motctrl_worker_get_status(void)
{
    worker_status_t status = WORKER_STATUS_ERROR;
    
    if (status_mutex && xSemaphoreTake(status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status = current_status;
        xSemaphoreGive(status_mutex);
    }
    
    return status;
}