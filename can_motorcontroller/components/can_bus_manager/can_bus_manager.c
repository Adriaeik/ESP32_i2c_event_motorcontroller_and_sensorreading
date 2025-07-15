#include "can_bus_manager.h"
#include <string.h>

static const char *TAG = "can_bus_manager";

// Internal state
static bool initialized = false;
static SemaphoreHandle_t bus_mutex = NULL;
static can_bus_stats_t stats = {0};
static SemaphoreHandle_t stats_mutex = NULL;

// Configuration from Kconfig
#define CAN_TX_GPIO CONFIG_CAN_TX_GPIO
#define CAN_RX_GPIO CONFIG_CAN_RX_GPIO
#define CAN_BITRATE CONFIG_CAN_BUS_BITRATE
#define TX_QUEUE_SIZE CONFIG_CAN_BUS_TX_QUEUE_SIZE
#define RX_QUEUE_SIZE CONFIG_CAN_BUS_RX_QUEUE_SIZE

esp_err_t can_bus_manager_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create mutexes
    bus_mutex = xSemaphoreCreateMutex();
    stats_mutex = xSemaphoreCreateMutex();
    if (!bus_mutex || !stats_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }

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

    initialized = true;
    ESP_LOGI(TAG, "TWAI bus manager initialized (TX: %d, RX: %d, Bitrate: %d)", 
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);
    return ESP_OK;

error:
    if (bus_mutex) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }
    if (stats_mutex) {
        vSemaphoreDelete(stats_mutex);
        stats_mutex = NULL;
    }
    return ret;
}

esp_err_t can_bus_manager_deinit(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = twai_stop();
    if (ret == ESP_OK) {
        ret = twai_driver_uninstall();
    }

    if (bus_mutex) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }
    if (stats_mutex) {
        vSemaphoreDelete(stats_mutex);
        stats_mutex = NULL;
    }

    initialized = false;
    ESP_LOGI(TAG, "CAN bus manager deinitialized");
    return ret;
}

esp_err_t can_bus_send_message(const can_message_t *message, can_priority_t priority, uint32_t timeout_ms)
{
    if (!initialized || !message) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex for thread safety
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }


    // Priority currently not used in TWAI, but kept for interface compatibility
    esp_err_t ret = twai_transmit(message, pdMS_TO_TICKS(timeout_ms));
    
    // Update statistics
    if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (ret == ESP_OK) {
            stats.messages_sent++;
        } else {
            stats.errors_count++;
        }
        xSemaphoreGive(stats_mutex);
    }

    xSemaphoreGive(bus_mutex);
    return ret;
}

esp_err_t can_bus_receive_message(can_message_t *message, uint32_t timeout_ms)
{
    if (!initialized || !message) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = twai_receive(message, pdMS_TO_TICKS(timeout_ms));
    
    if (ret == ESP_OK) {

        // Update statistics
        if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            stats.messages_received++;
            xSemaphoreGive(stats_mutex);
        }
    }

    return ret;
}
esp_err_t can_bus_get_stats(can_bus_stats_t *stats_out)
{
    if (!stats_out) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *stats_out = stats;
        xSemaphoreGive(stats_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t can_bus_reset_stats(void)
{
    if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&stats, 0, sizeof(stats));
        xSemaphoreGive(stats_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}