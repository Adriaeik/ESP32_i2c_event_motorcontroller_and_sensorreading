#include "can_bus_manager.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "can_bus_manager";

// Message priorities for starvation prevention
typedef enum {
    CAN_PRIORITY_CRITICAL = 0,  // System critical messages
    CAN_PRIORITY_HIGH = 1,      // Motor control commands
    CAN_PRIORITY_NORMAL = 2,    // Status updates
    CAN_PRIORITY_LOW = 3,       // Diagnostics
    CAN_PRIORITY_MAX = 4
} can_message_priority_t;

// Internal structures for queue messages
typedef struct {
    twai_message_t message;
    uint32_t timeout_ms;
    can_message_priority_t priority;
    TickType_t timestamp;
    esp_err_t *result;
    SemaphoreHandle_t completion_sem;
} can_tx_request_t;

typedef struct {
    bool initialized;
    can_bus_event_callback_t event_callback;
    void *user_data;
    
    // Task handles
    TaskHandle_t rx_task_handle;
    TaskHandle_t tx_task_handle;
    TaskHandle_t watchdog_task_handle;
    
    // Thread safety
    SemaphoreHandle_t bus_mutex;
    SemaphoreHandle_t callback_mutex;
    
    // Queues with priority handling
    QueueHandle_t tx_queues[CAN_PRIORITY_MAX];  // Priority-based TX queues
    QueueHandle_t rx_queue;
    
    // Statistics and monitoring
    struct {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t tx_errors;
        uint32_t rx_errors;
        uint32_t bus_off_count;
        uint32_t queue_overflows;
        TickType_t last_activity;
    } stats;
    
    // Starvation prevention
    uint8_t last_served_priority;
    uint32_t starvation_counter[CAN_PRIORITY_MAX];
    
    // Error recovery
    bool bus_recovery_active;
    TickType_t last_recovery_attempt;
} can_bus_manager_ctx_t;

static can_bus_manager_ctx_t s_bus_ctx = {0};

// Configuration
#define TX_QUEUE_SIZE           16
#define RX_QUEUE_SIZE           32
#define WATCHDOG_INTERVAL_MS    1000
#define BUS_RECOVERY_DELAY_MS   500
#define STARVATION_THRESHOLD    10
#define MAX_TX_TIMEOUT_MS       5000

// Forward declarations
static void can_bus_rx_task(void *pvParameters);
static void can_bus_tx_task(void *pvParameters);
static void can_bus_watchdog_task(void *pvParameters);
static void notify_event(can_bus_event_type_t event_type, const twai_message_t *message, esp_err_t error_code);
static esp_err_t setup_twai_driver(void);
static void cleanup_resources(void);
static can_message_priority_t get_message_priority(uint32_t can_id);
static uint8_t select_next_priority_queue(void);
static esp_err_t recover_bus_if_needed(void);

esp_err_t can_bus_manager_init(can_bus_event_callback_t callback, void *user_data)
{
    if (s_bus_ctx.initialized) {
        ESP_LOGW(TAG, "CAN bus manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing CAN bus manager...");

    // Initialize context
    memset(&s_bus_ctx, 0, sizeof(s_bus_ctx));
    s_bus_ctx.event_callback = callback;
    s_bus_ctx.user_data = user_data;
    s_bus_ctx.last_served_priority = CAN_PRIORITY_MAX - 1;

    // Create mutexes
    s_bus_ctx.bus_mutex = xSemaphoreCreateMutex();
    if (s_bus_ctx.bus_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create bus mutex");
        return ESP_ERR_NO_MEM;
    }

    s_bus_ctx.callback_mutex = xSemaphoreCreateMutex();
    if (s_bus_ctx.callback_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create callback mutex");
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }

    // Create priority-based TX queues
    for (int i = 0; i < CAN_PRIORITY_MAX; i++) {
        s_bus_ctx.tx_queues[i] = xQueueCreate(TX_QUEUE_SIZE, sizeof(can_tx_request_t));
        if (s_bus_ctx.tx_queues[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create TX queue %d", i);
            cleanup_resources();
            return ESP_ERR_NO_MEM;
        }
    }

    // Create RX queue
    s_bus_ctx.rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(twai_message_t));
    if (s_bus_ctx.rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }

    // Setup TWAI driver
    esp_err_t ret = setup_twai_driver();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup TWAI driver: %s", esp_err_to_name(ret));
        cleanup_resources();
        return ret;
    }

    // Create tasks
    BaseType_t task_ret = xTaskCreate(
        can_bus_rx_task,
        "can_bus_rx",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // Highest priority for RX
        &s_bus_ctx.rx_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }

    task_ret = xTaskCreate(
        can_bus_tx_task,
        "can_bus_tx",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,  // High priority for TX
        &s_bus_ctx.tx_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }

    task_ret = xTaskCreate(
        can_bus_watchdog_task,
        "can_watchdog",
        2048,
        NULL,
        2,  // Lower priority for watchdog
        &s_bus_ctx.watchdog_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create watchdog task");
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }

    s_bus_ctx.initialized = true;
    s_bus_ctx.stats.last_activity = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "CAN bus manager initialized successfully");
    return ESP_OK;
}

esp_err_t can_bus_manager_deinit(void)
{
    if (!s_bus_ctx.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing CAN bus manager...");

    // Mark as not initialized to stop tasks
    s_bus_ctx.initialized = false;

    // Give tasks time to exit gracefully
    vTaskDelay(pdMS_TO_TICKS(100));

    cleanup_resources();
    
    ESP_LOGI(TAG, "CAN bus manager deinitialized");
    return ESP_OK;
}

esp_err_t can_bus_manager_send_message(const twai_message_t *message, uint32_t timeout_ms)
{
    if (!s_bus_ctx.initialized) {
        ESP_LOGE(TAG, "CAN bus manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Limit timeout to prevent blocking indefinitely
    if (timeout_ms > MAX_TX_TIMEOUT_MS) {
        timeout_ms = MAX_TX_TIMEOUT_MS;
    }

    // Determine message priority
    can_message_priority_t priority = get_message_priority(message->identifier);

    // Create TX request
    can_tx_request_t tx_req = {
        .message = *message,
        .timeout_ms = timeout_ms,
        .priority = priority,
        .timestamp = xTaskGetTickCount(),
        .result = NULL,
        .completion_sem = xSemaphoreCreateBinary()
    };

    if (tx_req.completion_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create completion semaphore");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = ESP_FAIL;
    tx_req.result = &result;

    // Queue message based on priority
    BaseType_t queue_ret = xQueueSend(s_bus_ctx.tx_queues[priority], &tx_req, pdMS_TO_TICKS(100));
    if (queue_ret != pdTRUE) {
        ESP_LOGW(TAG, "TX queue full for priority %d", priority);
        vSemaphoreDelete(tx_req.completion_sem);
        s_bus_ctx.stats.queue_overflows++;
        return ESP_ERR_NO_MEM;
    }

    // Wait for completion
    if (xSemaphoreTake(tx_req.completion_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        vSemaphoreDelete(tx_req.completion_sem);
        return result;
    } else {
        ESP_LOGW(TAG, "Send timeout for message ID 0x%lx", message->identifier);
        vSemaphoreDelete(tx_req.completion_sem);
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t can_bus_manager_send_message_async(const twai_message_t *message)
{
    if (!s_bus_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_message_priority_t priority = get_message_priority(message->identifier);

    can_tx_request_t tx_req = {
        .message = *message,
        .timeout_ms = 1000,  // Default timeout for async
        .priority = priority,
        .timestamp = xTaskGetTickCount(),
        .result = NULL,
        .completion_sem = NULL  // No completion semaphore for async
    };

    BaseType_t queue_ret = xQueueSend(s_bus_ctx.tx_queues[priority], &tx_req, 0);
    if (queue_ret != pdTRUE) {
        s_bus_ctx.stats.queue_overflows++;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool can_bus_manager_is_initialized(void)
{
    return s_bus_ctx.initialized;
}

esp_err_t can_bus_manager_get_stats(twai_status_info_t *status)
{
    if (!s_bus_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return twai_get_status_info(status);
}

void can_bus_manager_get_extended_stats(can_bus_stats_t *stats)
{
    if (stats == NULL || !s_bus_ctx.initialized) {
        return;
    }

    if (xSemaphoreTake(s_bus_ctx.bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *stats = (can_bus_stats_t){
            .messages_sent = s_bus_ctx.stats.messages_sent,
            .messages_received = s_bus_ctx.stats.messages_received,
            .tx_errors = s_bus_ctx.stats.tx_errors,
            .rx_errors = s_bus_ctx.stats.rx_errors,
            .bus_off_count = s_bus_ctx.stats.bus_off_count,
            .queue_overflows = s_bus_ctx.stats.queue_overflows,
            .last_activity = s_bus_ctx.stats.last_activity
        };
        xSemaphoreGive(s_bus_ctx.bus_mutex);
    }
}

// Internal functions

static esp_err_t setup_twai_driver(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CONFIG_CAN_TX_GPIO, 
        CONFIG_CAN_RX_GPIO, 
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = RX_QUEUE_SIZE;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        twai_driver_uninstall();
        return ret;
    }

    return ESP_OK;
}

static void cleanup_resources(void)
{
    // Stop and uninstall TWAI driver
    if (s_bus_ctx.initialized) {
        twai_stop();
        twai_driver_uninstall();
    }

    // Delete tasks
    if (s_bus_ctx.rx_task_handle != NULL) {
        vTaskDelete(s_bus_ctx.rx_task_handle);
        s_bus_ctx.rx_task_handle = NULL;
    }

    if (s_bus_ctx.tx_task_handle != NULL) {
        vTaskDelete(s_bus_ctx.tx_task_handle);
        s_bus_ctx.tx_task_handle = NULL;
    }

    if (s_bus_ctx.watchdog_task_handle != NULL) {
        vTaskDelete(s_bus_ctx.watchdog_task_handle);
        s_bus_ctx.watchdog_task_handle = NULL;
    }

    // Clean up queues
    for (int i = 0; i < CAN_PRIORITY_MAX; i++) {
        if (s_bus_ctx.tx_queues[i] != NULL) {
            // Drain queue and clean up semaphores
            can_tx_request_t req;
            while (xQueueReceive(s_bus_ctx.tx_queues[i], &req, 0) == pdTRUE) {
                if (req.completion_sem != NULL) {
                    vSemaphoreDelete(req.completion_sem);
                }
            }
            vQueueDelete(s_bus_ctx.tx_queues[i]);
            s_bus_ctx.tx_queues[i] = NULL;
        }
    }

    if (s_bus_ctx.rx_queue != NULL) {
        vQueueDelete(s_bus_ctx.rx_queue);
        s_bus_ctx.rx_queue = NULL;
    }

    // Clean up mutexes
    if (s_bus_ctx.bus_mutex != NULL) {
        vSemaphoreDelete(s_bus_ctx.bus_mutex);
        s_bus_ctx.bus_mutex = NULL;
    }

    if (s_bus_ctx.callback_mutex != NULL) {
        vSemaphoreDelete(s_bus_ctx.callback_mutex);
        s_bus_ctx.callback_mutex = NULL;
    }

    s_bus_ctx.initialized = false;
}

static can_message_priority_t get_message_priority(uint32_t can_id)
{
    // Assign priorities based on CAN ID ranges
    if (can_id >= 0x000 && can_id <= 0x0FF) {
        return CAN_PRIORITY_CRITICAL;  // System messages
    } else if (can_id >= 0x100 && can_id <= 0x1FF) {
        return CAN_PRIORITY_HIGH;      // Motor control
    } else if (can_id >= 0x200 && can_id <= 0x2FF) {
        return CAN_PRIORITY_NORMAL;    // Status updates
    } else {
        return CAN_PRIORITY_LOW;       // Everything else
    }
}

static uint8_t select_next_priority_queue(void)
{
    // Implement round-robin with starvation prevention
    for (int attempts = 0; attempts < CAN_PRIORITY_MAX; attempts++) {
        uint8_t priority = (s_bus_ctx.last_served_priority + 1 + attempts) % CAN_PRIORITY_MAX;
        
        // Check if this priority queue has messages
        if (uxQueueMessagesWaiting(s_bus_ctx.tx_queues[priority]) > 0) {
            // Check for starvation of lower priority queues
            bool lower_priority_starved = false;
            for (int i = priority + 1; i < CAN_PRIORITY_MAX; i++) {
                if (s_bus_ctx.starvation_counter[i] > STARVATION_THRESHOLD) {
                    lower_priority_starved = true;
                    priority = i;  // Serve starved queue
                    break;
                }
            }
            
            s_bus_ctx.last_served_priority = priority;
            
            // Update starvation counters
            for (int i = 0; i < CAN_PRIORITY_MAX; i++) {
                if (i == priority) {
                    s_bus_ctx.starvation_counter[i] = 0;
                } else if (uxQueueMessagesWaiting(s_bus_ctx.tx_queues[i]) > 0) {
                    s_bus_ctx.starvation_counter[i]++;
                }
            }
            
            return priority;
        }
    }
    
    return CAN_PRIORITY_MAX;  // No messages in any queue
}

static esp_err_t recover_bus_if_needed(void)
{
    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret != ESP_OK) {
        return ret;
    }

    if (status.state == TWAI_STATE_BUS_OFF) {
        TickType_t now = xTaskGetTickCount();
        if (!s_bus_ctx.bus_recovery_active || 
            (now - s_bus_ctx.last_recovery_attempt) > pdMS_TO_TICKS(BUS_RECOVERY_DELAY_MS)) {
            
            ESP_LOGW(TAG, "CAN bus off, attempting recovery");
            s_bus_ctx.bus_recovery_active = true;
            s_bus_ctx.last_recovery_attempt = now;
            s_bus_ctx.stats.bus_off_count++;
            
            ret = twai_initiate_recovery();
            if (ret == ESP_OK) {
                // Wait a bit for recovery
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // Check if recovered
                ret = twai_get_status_info(&status);
                if (ret == ESP_OK && status.state != TWAI_STATE_BUS_OFF) {
                    ESP_LOGI(TAG, "CAN bus recovered");
                    s_bus_ctx.bus_recovery_active = false;
                    notify_event(CAN_BUS_EVT_BUS_RECOVERED, NULL, ESP_OK);
                }
            }
        }
    } else {
        s_bus_ctx.bus_recovery_active = false;
    }
    
    return ESP_OK;
}

static void can_bus_rx_task(void *pvParameters)
{
    twai_message_t message;
    
    ESP_LOGI(TAG, "CAN bus RX task started");
    
    while (s_bus_ctx.initialized) {
        esp_err_t ret = twai_receive(&message, pdMS_TO_TICKS(100));
        
        if (ret == ESP_OK) {
            // Update statistics
            if (xSemaphoreTake(s_bus_ctx.bus_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_bus_ctx.stats.messages_received++;
                s_bus_ctx.stats.last_activity = xTaskGetTickCount();
                xSemaphoreGive(s_bus_ctx.bus_mutex);
            }
            
            // Forward to application callback
            notify_event(CAN_BUS_EVT_MESSAGE_RECEIVED, &message, ESP_OK);
            
            ESP_LOGD(TAG, "CAN message received: ID 0x%lx, DLC %d", 
                    message.identifier, message.data_length_code);
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "CAN receive error: %s", esp_err_to_name(ret));
            s_bus_ctx.stats.rx_errors++;
            notify_event(CAN_BUS_EVT_ERROR, NULL, ret);
        }
        
        // Check for bus errors and handle recovery
        recover_bus_if_needed();
    }
    
    ESP_LOGI(TAG, "CAN bus RX task exiting");
    vTaskDelete(NULL);
}

static void can_bus_tx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "CAN bus TX task started");
    
    while (s_bus_ctx.initialized) {
        // Select next priority queue using starvation prevention
        uint8_t priority = select_next_priority_queue();
        
        if (priority < CAN_PRIORITY_MAX) {
            can_tx_request_t tx_req;
            if (xQueueReceive(s_bus_ctx.tx_queues[priority], &tx_req, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Check message timeout
                TickType_t now = xTaskGetTickCount();
                if ((now - tx_req.timestamp) > pdMS_TO_TICKS(tx_req.timeout_ms)) {
                    ESP_LOGW(TAG, "Message timeout before TX: ID 0x%lx", tx_req.message.identifier);
                    if (tx_req.result != NULL) {
                        *tx_req.result = ESP_ERR_TIMEOUT;
                    }
                    if (tx_req.completion_sem != NULL) {
                        xSemaphoreGive(tx_req.completion_sem);
                    }
                    continue;
                }
                
                // Send message
                esp_err_t ret = twai_transmit(&tx_req.message, pdMS_TO_TICKS(100));
                
                // Update statistics
                if (xSemaphoreTake(s_bus_ctx.bus_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (ret == ESP_OK) {
                        s_bus_ctx.stats.messages_sent++;
                    } else {
                        s_bus_ctx.stats.tx_errors++;
                    }
                    s_bus_ctx.stats.last_activity = xTaskGetTickCount();
                    xSemaphoreGive(s_bus_ctx.bus_mutex);
                }
                
                // Notify completion
                if (tx_req.result != NULL) {
                    *tx_req.result = ret;
                }
                if (tx_req.completion_sem != NULL) {
                    xSemaphoreGive(tx_req.completion_sem);
                }
                
                // Notify event
                if (ret == ESP_OK) {
                    notify_event(CAN_BUS_EVT_MESSAGE_SENT, &tx_req.message, ESP_OK);
                    ESP_LOGD(TAG, "CAN message sent: ID 0x%lx, DLC %d", 
                            tx_req.message.identifier, tx_req.message.data_length_code);
                } else {
                    ESP_LOGW(TAG, "Failed to send CAN message: %s", esp_err_to_name(ret));
                    notify_event(CAN_BUS_EVT_ERROR, &tx_req.message, ret);
                }
            }
        } else {
            // No messages in any queue, wait a bit
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    ESP_LOGI(TAG, "CAN bus TX task exiting");
    vTaskDelete(NULL);
}

static void can_bus_watchdog_task(void *pvParameters)
{
    ESP_LOGI(TAG, "CAN bus watchdog task started");
    
    while (s_bus_ctx.initialized) {
        // Check bus health and perform maintenance
        recover_bus_if_needed();
        
        // Log statistics periodically
        static uint32_t log_counter = 0;
        if (++log_counter % 60 == 0) {  // Every minute
            ESP_LOGI(TAG, "Stats - TX: %lu, RX: %lu, TX_ERR: %lu, RX_ERR: %lu, BUS_OFF: %lu",
                    s_bus_ctx.stats.messages_sent,
                    s_bus_ctx.stats.messages_received,
                    s_bus_ctx.stats.tx_errors,
                    s_bus_ctx.stats.rx_errors,
                    s_bus_ctx.stats.bus_off_count);
        }
        
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_INTERVAL_MS));
    }
    
    ESP_LOGI(TAG, "CAN bus watchdog task exiting");
    vTaskDelete(NULL);
}

static void notify_event(can_bus_event_type_t event_type, const twai_message_t *message, esp_err_t error_code)
{
    if (s_bus_ctx.event_callback == NULL) {
        return;
    }

    // Thread-safe callback execution
    if (xSemaphoreTake(s_bus_ctx.callback_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        can_bus_event_data_t event_data = {
            .event_type = event_type,
            .error_code = error_code
        };

        if (message != NULL) {
            event_data.message = *message;
        } else {
            memset(&event_data.message, 0, sizeof(twai_message_t));
        }

        // Call the registered callback
        s_bus_ctx.event_callback(&event_data, s_bus_ctx.user_data);
        
        xSemaphoreGive(s_bus_ctx.callback_mutex);
    }
}