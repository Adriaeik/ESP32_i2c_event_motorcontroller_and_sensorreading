#include "can_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>

typedef struct {
    uint32_t can_id;
    QueueHandle_t queue;
} can_subscription_t;

static const char *TAG = "can_bus_manager";

// Internal state
static bool initialized = false;
static TaskHandle_t receive_task_handle = NULL;
static can_subscription_t *subscriptions = NULL;
static uint16_t subscription_count = 0;
static SemaphoreHandle_t sub_mutex = NULL;
static SemaphoreHandle_t bus_mutex = NULL;


static StaticSemaphore_t mutex_buffer;
static SemaphoreHandle_t subscription_mutex;


// Configuration from Kconfig
#define CAN_TX_GPIO CONFIG_CAN_TX_GPIO
#define CAN_RX_GPIO CONFIG_CAN_RX_GPIO
#define CAN_BITRATE CONFIG_CAN_BUS_BITRATE
#define TX_QUEUE_SIZE CONFIG_CAN_BUS_TX_QUEUE_SIZE
#define RX_QUEUE_SIZE CONFIG_CAN_BUS_RX_QUEUE_SIZE


static void can_receive_task(void *arg) {
    can_message_t msg;
    
    while (1) {
        // Receive message with infinite timeout
        esp_err_t ret = twai_receive(&msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            if (ret != ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "Receive error: %s", esp_err_to_name(ret));
            }
            continue;
        }
        
        // Lock subscription list
        xSemaphoreTake(sub_mutex, portMAX_DELAY);
        
        // Find matching subscriptions
        bool delivered = false;
        bool queue_full = false;
        for (int i = 0; i < subscription_count; i++) {
            if (subscriptions[i].can_id == msg.identifier) {
                can_message_t *msg_copy = malloc(sizeof(can_message_t));
                if (msg_copy) {
                    *msg_copy = msg;
                    if (xQueueSend(subscriptions[i].queue, &msg_copy, 0) != pdPASS) {
                        ESP_LOGW(TAG, "Queue full for ID 0x%X", msg.identifier);
                        free(msg_copy);
                        queue_full = true;
                    } else {
                        delivered = true;
                    }
                }
            }
        }

        // Only warn about no subscribers if there really are none
        if (!delivered && !queue_full) {
            ESP_LOGW(TAG, "No subscribers for ID 0x%X", msg.identifier);
        }
        
        xSemaphoreGive(sub_mutex);
    }
}


esp_err_t can_bus_manager_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutexes
    bus_mutex = xSemaphoreCreateMutex();
    sub_mutex = xSemaphoreCreateMutex();
    if (!bus_mutex || !sub_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    // Initialize subscriptions array
    subscriptions = malloc(0);  // Start with empty array
    subscription_count = 0;

    // Configure TWAI driver (updated to TWAI)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO, 
        (gpio_num_t)CAN_RX_GPIO,
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = TX_QUEUE_SIZE;
    g_config.rx_queue_len = RX_QUEUE_SIZE;
    g_config.alerts_enabled = TWAI_ALERT_ALL;

    // Configure timing based on bitrate (updated to TWAI)
    twai_timing_config_t t_config = {
        .brp = 0,
        .tseg_1 = 0,
        .tseg_2 = 0,
        .sjw = 0,
        .triple_sampling = false
    };

    switch (CAN_BITRATE) {
        case 125000:
            t_config = (twai_timing_config_t){.brp = 256, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3};
            break;
        case 250000:
            t_config = (twai_timing_config_t){.brp = 128, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3};
            break;
        case 500000:
            t_config = (twai_timing_config_t){.brp = 80, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3};
            break;
        case 1000000:
            t_config = (twai_timing_config_t){.brp = 40, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3};
            break;
        default:
            t_config = (twai_timing_config_t){.brp = 80, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3};
            ESP_LOGW(TAG, "Unsupported bitrate %d, using 500kbps", CAN_BITRATE);
    }

    // Accept all messages (updated to TWAI)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install and start TWAI driver (updated API)
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        goto error;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        goto error;
    }

    // Create receive task
    BaseType_t ret_task = xTaskCreate(can_receive_task, "can_rx", 4096, NULL, 8, &receive_task_handle); // reduce priority later
    if (ret_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receive task");
        vSemaphoreDelete(bus_mutex);
        vSemaphoreDelete(sub_mutex);
        free(subscriptions);
        goto error;
    }

    initialized = true;
    ESP_LOGI(TAG, "TWAI bus manager initialized (TX: %d, RX: %d, Bitrate: %d)", 
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);
    return ESP_OK;

error:
    if (bus_mutex) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }
    if (sub_mutex) {
        vSemaphoreDelete(sub_mutex);
        sub_mutex = NULL;
    }
    return ret;
}

esp_err_t can_bus_manager_deinit(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop receive task
    if (receive_task_handle) {
        vTaskDelete(receive_task_handle);
        receive_task_handle = NULL;
    }

    // Clean up subscriptions
    xSemaphoreTake(sub_mutex, portMAX_DELAY);
    for (int i = 0; i < subscription_count; i++) {
        // Empty and delete queue
        can_message_t *msg;
        while (xQueueReceive(subscriptions[i].queue, &msg, 0) == pdTRUE) {
            free(msg);
        }
        vQueueDelete(subscriptions[i].queue);
    }
    free(subscriptions);
    subscriptions = NULL;
    subscription_count = 0;
    xSemaphoreGive(sub_mutex);
    
    // Delete mutexes
    vSemaphoreDelete(bus_mutex);
    vSemaphoreDelete(sub_mutex);
    
    esp_err_t ret = twai_stop();
    if (ret == ESP_OK) {
        ret = twai_driver_uninstall();
    }

    initialized = false;
    ESP_LOGI(TAG, "CAN bus manager deinitialized");
    return ret;
}


esp_err_t can_bus_send_message(const can_message_t *message, uint32_t timeout_ms) {
    if (!initialized || !message) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take bus mutex
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Directly send the message
    esp_err_t ret = twai_transmit(message, pdMS_TO_TICKS(timeout_ms));
    
    xSemaphoreGive(bus_mutex);
    return ret;
}

esp_err_t can_bus_subscribe_id(uint32_t can_id, uint8_t queue_size) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(sub_mutex, portMAX_DELAY);
    
    // Check if already subscribed
    for (int i = 0; i < subscription_count; i++) {
        if (subscriptions[i].can_id == can_id) {
            xSemaphoreGive(sub_mutex);
            ESP_LOGW(TAG, "Already subscribed to ID 0x%X", can_id);
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // Create new subscription
    can_subscription_t *new_subs = realloc(subscriptions, 
                                         (subscription_count + 1) * sizeof(can_subscription_t));
    if (!new_subs) {
        xSemaphoreGive(sub_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    subscriptions = new_subs;
    can_subscription_t *sub = &subscriptions[subscription_count];
    
    // Create message queue
    sub->queue = xQueueCreate(queue_size, sizeof(can_message_t *));
    if (!sub->queue) {
        xSemaphoreGive(sub_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    sub->can_id = can_id;
    subscription_count++;
    
    ESP_LOGI(TAG, "Subscribed to ID 0x%X with queue size %d", can_id, queue_size);
    xSemaphoreGive(sub_mutex);
    return ESP_OK;
}

esp_err_t can_subscribe_multiple(const can_subscription_spec_t *specs, size_t count) {
    if (!specs || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    size_t subscribed = 0;
    
    for (size_t i = 0; i < count; i++) {
        ret = can_bus_subscribe_id(specs[i].can_id, specs[i].queue_size);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE("CAN_MULTI_SUB", "Failed to subscribe to ID 0x%X: %s", 
                     specs[i].can_id, esp_err_to_name(ret));
            // Cleanup already subscribed IDs
            for (size_t j = 0; j < subscribed; j++) {
                can_bus_unsubscribe_id(specs[j].can_id);
            }
            return ret;
        }
        subscribed++;
    }
    
    ESP_LOGI("CAN_MULTI_SUB", "Successfully subscribed to %d CAN IDs", count);
    return ESP_OK;
}

esp_err_t can_bus_unsubscribe_id(uint32_t can_id)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(sub_mutex, portMAX_DELAY);
    
    // Find subscription
    for (int i = 0; i < subscription_count; i++) {
        if (subscriptions[i].can_id == can_id) {
            // Clean up queue
            can_message_t *msg;
            while (xQueueReceive(subscriptions[i].queue, &msg, 0) == pdTRUE) {
                free(msg);
            }
            vQueueDelete(subscriptions[i].queue);
            
            // Remove from array
            memmove(&subscriptions[i], &subscriptions[i+1], 
                   (subscription_count - i - 1) * sizeof(can_subscription_t));
            subscription_count--;
            
            // Shrink array
            can_subscription_t *new_subs = realloc(subscriptions, 
                                                 subscription_count * sizeof(can_subscription_t));
            if (subscription_count > 0 && !new_subs) {
                // Handle error - we'll keep the smaller array
            } else {
                subscriptions = new_subs;
            }
            
            ESP_LOGI(TAG, "Unsubscribed from ID 0x%X", can_id);
            xSemaphoreGive(sub_mutex);
            return ESP_OK;
        }
    }
    
    xSemaphoreGive(sub_mutex);
    ESP_LOGW(TAG, "Not subscribed to ID 0x%X", can_id);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t can_unsubscribe_multiple(const uint32_t *can_ids, size_t count) {
    if (!can_ids || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (size_t i = 0; i < count; i++) {
        esp_err_t ret = can_bus_unsubscribe_id(can_ids[i]);
        if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW("CAN_MULTI_UNSUB", "Failed to unsubscribe from ID 0x%X: %s", 
                     can_ids[i], esp_err_to_name(ret));
        }
    }
    
    ESP_LOGI("CAN_MULTI_UNSUB", "Unsubscribed from %d CAN IDs", count);
    return ESP_OK;
}

esp_err_t can_bus_wait_for_message(uint32_t can_id, can_message_t *msg, uint32_t timeout_ms) {
    if (!initialized || !msg) {
        return ESP_ERR_INVALID_ARG;
    }

    QueueHandle_t target_queue = NULL;
    
    // Find the queue for this CAN ID
    xSemaphoreTake(sub_mutex, portMAX_DELAY);
    for (int i = 0; i < subscription_count; i++) {
        if (subscriptions[i].can_id == can_id) {
            target_queue = subscriptions[i].queue;
            break;
        }
    }
    xSemaphoreGive(sub_mutex);
    
    if (!target_queue) {
        ESP_LOGE(TAG, "No subscription for ID 0x%X", can_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Wait for message
    can_message_t *msg_ptr;
    if (xQueueReceive(target_queue, &msg_ptr, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        *msg = *msg_ptr;  // Copy message to output
        free(msg_ptr);    // Free the allocated copy
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}