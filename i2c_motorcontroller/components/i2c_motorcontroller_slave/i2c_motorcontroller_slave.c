#include "i2c_motorcontroller_slave.h"
#include "serde_helper.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "i2c_motctrl_slave";

static i2c_motctrl_slave_ctx_t s_slave_ctx = {0};

// Forward declarations
static void i2c_slave_task(void *pvParameters);
static bool IRAM_ATTR i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg);
static bool IRAM_ATTR i2c_slave_request_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg);

esp_err_t i2c_install_slave_driver_config(void)
{
    if (s_slave_ctx.dev_handle != NULL) {
        ESP_LOGW(TAG, "Slave already initialized");
        return ESP_OK;
    }

    // Initialize slave context
    memset(&s_slave_ctx, 0, sizeof(s_slave_ctx));
    s_slave_ctx.state = I2C_SLAVE_STATE_IDLE;
    s_slave_ctx.status_byte = SLAVE_STATUS_IDLE;

    // Create event queue
    s_slave_ctx.event_queue = xQueueCreate(16, sizeof(i2c_slave_event_t));
    if (s_slave_ctx.event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    // Create operation event group
    s_slave_ctx.operation_event_group = xEventGroupCreate();
    if (s_slave_ctx.operation_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create operation event group");
        vQueueDelete(s_slave_ctx.event_queue);
        return ESP_ERR_NO_MEM;
    }

    // Configure I2C slave
    i2c_slave_config_t i2c_slv_config = {
        .i2c_port = CONFIG_MOTCTRL_SLAVE_I2C_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = CONFIG_MOTCTRL_SLAVE_SCL_GPIO,
        .sda_io_num = CONFIG_MOTCTRL_SLAVE_SDA_GPIO,
        .slave_addr = CONFIG_MOTCTRL_I2C_ADDR,
        .send_buf_depth = 256,
        .receive_buf_depth = 256,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .flags = {
            .enable_internal_pullup = true,
        }
    };
    
    esp_err_t ret = i2c_new_slave_device(&i2c_slv_config, &s_slave_ctx.dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C slave device: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_slave_ctx.operation_event_group);
        vQueueDelete(s_slave_ctx.event_queue);
        return ret;
    }

    // Register callbacks
    i2c_slave_event_callbacks_t cbs = {
        .on_request = i2c_slave_request_cb,    // When master wants to read from slave
        .on_receive = i2c_slave_receive_cb,    // When master writes to slave
    };

    ret = i2c_slave_register_event_callbacks(s_slave_ctx.dev_handle, &cbs, &s_slave_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
        i2c_del_slave_device(s_slave_ctx.dev_handle);
        vEventGroupDelete(s_slave_ctx.operation_event_group);
        vQueueDelete(s_slave_ctx.event_queue);
        s_slave_ctx.dev_handle = NULL;
        return ret;
    }

    // Create slave task
    BaseType_t task_ret = xTaskCreate(
        i2c_slave_task,
        "i2c_slave",
        4096,
        NULL,
        5,
        &s_slave_ctx.slave_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create slave task");
        i2c_del_slave_device(s_slave_ctx.dev_handle);
        vEventGroupDelete(s_slave_ctx.operation_event_group);
        vQueueDelete(s_slave_ctx.event_queue);
        s_slave_ctx.dev_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "I2C slave initialized successfully at address 0x%02X", CONFIG_MOTCTRL_I2C_ADDR);
    return ESP_OK;
}

esp_err_t i2c_motctrl_slave_deinit(void)
{
    if (s_slave_ctx.dev_handle == NULL) {
        return ESP_OK;
    }

    // Stop slave task
    if (s_slave_ctx.slave_task_handle != NULL) {
        vTaskDelete(s_slave_ctx.slave_task_handle);
        s_slave_ctx.slave_task_handle = NULL;
    }

    // Clean up resources
    if (s_slave_ctx.operation_event_group != NULL) {
        vEventGroupDelete(s_slave_ctx.operation_event_group);
        s_slave_ctx.operation_event_group = NULL;
    }

    if (s_slave_ctx.event_queue != NULL) {
        vQueueDelete(s_slave_ctx.event_queue);
        s_slave_ctx.event_queue = NULL;
    }

    i2c_del_slave_device(s_slave_ctx.dev_handle);
    s_slave_ctx.dev_handle = NULL;

    ESP_LOGI(TAG, "I2C slave deinitialized");
    return ESP_OK;
}

esp_err_t i2c_motctrl_slave_wait_pkg(motorcontroller_pkg_t *pkg, int timeout_sec)
{
    if (s_slave_ctx.dev_handle == NULL) {
        ESP_LOGE(TAG, "Slave not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear previous package flag and event bits
    s_slave_ctx.pkg_ready = false;
    xEventGroupClearBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_PKG_RECEIVED_BIT | MOTCTRL_SLAVE_ERROR_BIT);

    // Set state to ready
    s_slave_ctx.state = I2C_SLAVE_STATE_IDLE;
    s_slave_ctx.status_byte = SLAVE_STATUS_READY;

    ESP_LOGI(TAG, "Waiting for package from master (timeout: %d seconds)...", timeout_sec);

    // Wait for package
    EventBits_t bits = xEventGroupWaitBits(
        s_slave_ctx.operation_event_group,
        MOTCTRL_SLAVE_PKG_RECEIVED_BIT | MOTCTRL_SLAVE_ERROR_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    if (bits & MOTCTRL_SLAVE_ERROR_BIT) {
        ESP_LOGE(TAG, "Error while waiting for package");
        return ESP_FAIL;
    }

    if (!(bits & MOTCTRL_SLAVE_PKG_RECEIVED_BIT)) {
        ESP_LOGW(TAG, "Package wait timeout");
        s_slave_ctx.state = I2C_SLAVE_STATE_IDLE;
        s_slave_ctx.status_byte = SLAVE_STATUS_IDLE;
        return ESP_ERR_TIMEOUT;
    }

    // Copy received package
    *pkg = s_slave_ctx.received_pkg;
    s_slave_ctx.state = I2C_SLAVE_STATE_PKG_RECEIVED;
    
    ESP_LOGI(TAG, "Package received successfully");
    return ESP_OK;
}

esp_err_t i2c_motctrl_slave_send_response(const motorcontroller_response_t *resp, int timeout_sec)
{
    if (s_slave_ctx.dev_handle == NULL) {
        ESP_LOGE(TAG, "Slave not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare response
    s_slave_ctx.response_to_send = *resp;
    s_slave_ctx.resp_ready = true;
    s_slave_ctx.state = I2C_SLAVE_STATE_RESP_READY;
    s_slave_ctx.status_byte = SLAVE_STATUS_RESP_READY;

    // Serialize response
    uint16_t crc = calculate_resp_crc(resp);
    esp_err_t ret = serialize_resp(resp, s_slave_ctx.tx_buffer, &s_slave_ctx.tx_len, crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize response");
        return ret;
    }

    ESP_LOGI(TAG, "Response ready, waiting for master to read...");

    // Clear event bits
    xEventGroupClearBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_RESP_SENT_BIT | MOTCTRL_SLAVE_ERROR_BIT);

    // Wait for response to be sent
    EventBits_t bits = xEventGroupWaitBits(
        s_slave_ctx.operation_event_group,
        MOTCTRL_SLAVE_RESP_SENT_BIT | MOTCTRL_SLAVE_ERROR_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    if (bits & MOTCTRL_SLAVE_ERROR_BIT) {
        ESP_LOGE(TAG, "Error while sending response");
        return ESP_FAIL;
    }

    if (!(bits & MOTCTRL_SLAVE_RESP_SENT_BIT)) {
        ESP_LOGW(TAG, "Response send timeout");
        return ESP_ERR_TIMEOUT;
    }

    // Reset state
    s_slave_ctx.state = I2C_SLAVE_STATE_IDLE;
    s_slave_ctx.status_byte = SLAVE_STATUS_IDLE;
    s_slave_ctx.resp_ready = false;

    ESP_LOGI(TAG, "Response sent successfully");
    return ESP_OK;
}

esp_err_t i2c_motctrl_slave_set_working(void)
{
    s_slave_ctx.state = I2C_SLAVE_STATE_WORKING;
    s_slave_ctx.status_byte = SLAVE_STATUS_WORKING;
    ESP_LOGI(TAG, "Slave state set to WORKING");
    return ESP_OK;
}

esp_err_t i2c_motctrl_slave_set_response_ready(const motorcontroller_response_t *resp)
{
    s_slave_ctx.response_to_send = *resp;
    s_slave_ctx.resp_ready = true;
    s_slave_ctx.state = I2C_SLAVE_STATE_RESP_READY;
    s_slave_ctx.status_byte = SLAVE_STATUS_RESP_READY;
    
    // Prepare serialized response
    uint16_t crc = calculate_resp_crc(resp);
    serialize_resp(resp, s_slave_ctx.tx_buffer, &s_slave_ctx.tx_len, crc);
    
    ESP_LOGI(TAG, "Response set and ready for master");
    return ESP_OK;
}

i2c_slave_state_t i2c_motctrl_slave_get_state(void)
{
    return s_slave_ctx.state;
}

bool i2c_motctrl_slave_is_ready(void)
{
    return (s_slave_ctx.dev_handle != NULL && s_slave_ctx.state == I2C_SLAVE_STATE_IDLE);
}

// Callback functions for v2 API
static bool IRAM_ATTR i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    i2c_motctrl_slave_ctx_t *ctx = (i2c_motctrl_slave_ctx_t *)arg;
    BaseType_t xTaskWoken = pdFALSE;
    
    if (evt_data->buffer && evt_data->length > 0) {
        // Clear previous receive buffer first
        memset(ctx->rx_buffer, 0, sizeof(ctx->rx_buffer));
        
        // Copy received data to our buffer
        size_t copy_len = evt_data->length;
        if (copy_len > sizeof(ctx->rx_buffer)) {
            copy_len = sizeof(ctx->rx_buffer);
        }
        
        memcpy(ctx->rx_buffer, evt_data->buffer, copy_len);
        ctx->rx_len = copy_len;
        
        // Send event to task
        i2c_slave_event_t evt = I2C_SLAVE_EVT_PKG_RECEIVED;
        xQueueSendFromISR(ctx->event_queue, &evt, &xTaskWoken);
    }
    
    return xTaskWoken == pdTRUE;
}

static bool IRAM_ATTR i2c_slave_request_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    i2c_motctrl_slave_ctx_t *ctx = (i2c_motctrl_slave_ctx_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_RESP_REQUESTED;
    BaseType_t xTaskWoken = pdFALSE;
    
    xQueueSendFromISR(ctx->event_queue, &evt, &xTaskWoken);
    return xTaskWoken == pdTRUE;
}

static void i2c_slave_task(void *pvParameters)
{
    i2c_slave_event_t evt;
    
    ESP_LOGI(TAG, "I2C slave task started");
    
    while (true) {
        if (xQueueReceive(s_slave_ctx.event_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (evt) {
                case I2C_SLAVE_EVT_PKG_RECEIVED:
                {
                    ESP_LOGI(TAG, "Package data received, length: %d", s_slave_ctx.rx_len);
                    ESP_LOG_BUFFER_HEX_LEVEL(TAG, s_slave_ctx.rx_buffer, s_slave_ctx.rx_len, ESP_LOG_INFO);
                    
                    // Single byte probe - master checking status
                    if (s_slave_ctx.rx_len == 1) {
                        ESP_LOGD(TAG, "Status probe received, command: 0x%02X", s_slave_ctx.rx_buffer[0]);
                        // Update status based on current state
                        s_slave_ctx.status_byte = (s_slave_ctx.state == I2C_SLAVE_STATE_RESP_READY) ? 
                                                  SLAVE_STATUS_RESP_READY : SLAVE_STATUS_READY;
                        continue;
                    }
                    
                    // Full package received - check exact size match
                    ESP_LOGI(TAG, "Expected package size: %d, received: %d", WIRE_PKG_SIZE, s_slave_ctx.rx_len);
                    if (s_slave_ctx.rx_len != WIRE_PKG_SIZE) {
                        ESP_LOGE(TAG, "Package size mismatch: expected %d, got %d", WIRE_PKG_SIZE, s_slave_ctx.rx_len);
                        s_slave_ctx.status_byte = SLAVE_STATUS_ERROR;
                        xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_ERROR_BIT);
                        continue;
                    }
                    
                    // Process package
                    uint16_t received_crc;
                    esp_err_t ret = deserialize_pkg(s_slave_ctx.rx_buffer, s_slave_ctx.rx_len, 
                                                   &s_slave_ctx.received_pkg, &received_crc);
                    
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to deserialize package: %s", esp_err_to_name(ret));
                        s_slave_ctx.status_byte = SLAVE_STATUS_ERROR;
                        xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_ERROR_BIT);
                        continue;
                    }
                    
                    // Verify CRC
                    uint16_t calculated_crc = calculate_pkg_crc(&s_slave_ctx.received_pkg);
                    if (received_crc != calculated_crc) {
                        ESP_LOGE(TAG, "Package CRC mismatch: received 0x%04X, calculated 0x%04X", 
                                  received_crc, calculated_crc);
                        s_slave_ctx.status_byte = SLAVE_STATUS_ERROR;
                        xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_ERROR_BIT);
                        continue;
                    }
                    
                    ESP_LOGI(TAG, "Package received and validated successfully");
                    s_slave_ctx.pkg_ready = true;
                    s_slave_ctx.state = I2C_SLAVE_STATE_PKG_RECEIVED;
                    s_slave_ctx.status_byte = SLAVE_STATUS_WORKING;
                    xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_PKG_RECEIVED_BIT);
                    break;
                }
                
                case I2C_SLAVE_EVT_RESP_REQUESTED:
                {
                    ESP_LOGD(TAG, "Master requesting data");

                    unified_response_wire_t response_wire;

                    // Sett status
                    response_wire.status = s_slave_ctx.status_byte;

                    // Set response data dersom klart
                    if (s_slave_ctx.status_byte == SLAVE_STATUS_RESP_READY &&
                        s_slave_ctx.resp_ready &&
                        s_slave_ctx.state == I2C_SLAVE_STATE_RESP_READY) {
                        response_wire.resp = s_slave_ctx.response_to_send;
                    } else {
                        memset(&response_wire.resp, 0, sizeof(motorcontroller_response_t));
                    }

                    // CRC over status + response
                    response_wire.crc = calculate_unified_crc(&response_wire);

                    // Kopier til tx_buffer for debug/logging
                    memcpy(s_slave_ctx.tx_buffer, &response_wire, sizeof(response_wire));
                    s_slave_ctx.tx_len = sizeof(response_wire);

                    // Send til master
                    uint32_t write_len = 0;
                    esp_err_t ret = i2c_slave_write(
                        s_slave_ctx.dev_handle,
                        (uint8_t*)&response_wire,
                        sizeof(response_wire),
                        &write_len,
                        1000
                    );

                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Transmit failed: %s", esp_err_to_name(ret));
                        xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_ERROR_BIT);
                    } else if (s_slave_ctx.status_byte == SLAVE_STATUS_RESP_READY) {
                        ESP_LOGI(TAG, "Response sent successfully");
                        s_slave_ctx.state = I2C_SLAVE_STATE_IDLE;
                        s_slave_ctx.status_byte = SLAVE_STATUS_IDLE;
                        s_slave_ctx.resp_ready = false;
                        xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_RESP_SENT_BIT);
                    }

                    break;
                }
                
                case I2C_SLAVE_EVT_ERROR:
                    ESP_LOGE(TAG, "Slave error event");
                    s_slave_ctx.status_byte = SLAVE_STATUS_ERROR;
                    xEventGroupSetBits(s_slave_ctx.operation_event_group, MOTCTRL_SLAVE_ERROR_BIT);
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown event: %d", evt);
                    break;
            }
        }
    }
    
    vTaskDelete(NULL);
}