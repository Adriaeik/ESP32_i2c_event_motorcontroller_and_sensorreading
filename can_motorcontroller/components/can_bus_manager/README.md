# I2C MotorController Component

## Purpose

This component wraps ESP-IDF's I2C functionality to create a clean and minimal API for communication between a master (main ESP) and a slave (motorcontroller ESP). It enables the master to send a control package to the motorcontroller and receive a response when the action is complete. The master can safely go into deep sleep and persist state using SD card, while the motorcontroller can wake up fresh and receive everything needed via I2C.


## Structs Used

### motorcontroller\_pkg\_t (from master to controller)

Used to initiate a lowering or raising cycle with complete context:
this is included by `buoye_structs.h` and does not need to be created here
```c
typedef struct {
    state_t STATE;                   // current operation mode: LOWERING or RISING
    int prev_working_time;          // duration (seconds) of last operation, used for estimating next time
    int rising_timeout_percent;     // factor to define timeout threshold for rising if home sensor not seen
    uint16_t prev_reported_depth;   // last measured depth (cm)
    uint16_t prev_end_depth;        // previous target depth (cm)
    uint16_t prev_estimated_cm_per_s; // last estimated lowering speed in cm/s (scaled e.g., 10000 => 10.000 cm/s)
    POLL_TYPE poll_type;            // type of polling (e.g., STATIC_DEPTH or CONTINUOUS)
    uint16_t end_depth;             // target depth for this operation (cm)
    uint16_t static_points[MAX_POINTS]; // depths at which to pause and sample (cm), terminated by 0
    uint16_t samples;               // sample count at each static point
    uint16_t static_poll_interval_s; // interval between static samples (seconds)

    // added for filter operation and timing
    double alpha;                   // alpha gain for alpha-beta filter
    double beta;                    // beta gain for alpha-beta filter
} motorcontroller_pkg_t;
```

Use `motorcontroller_pkg_init_default()` to initialize with defaults.

### motorcontroller\_response\_t (from controller back to master)

Returned when the motorcontroller completes its task:

```c
typedef struct {
    state_t STATE;                  // operation mode performed
    esp_err_t result;              // result code: ESP_OK, ESP_ERR_TIMEOUT, TENSION_ALARM, etc.
    int working_time;              // total time (seconds) taken for the elevation
    uint16_t estimated_cm_per_s;  // new speed estimate (scaled)
} motorcontroller_response_t;
```

Use `motorcontroller_response_init_default()` to zero out/initialize.

## Serialization Helpers (serde\_helper)

These are used internally by the I2C master/slave component to serialize/deserialize packages.

### Functions:
this is included by `serde_helper.h` and does not need to be created here
```c
esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len);
esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg);
esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len);
esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp);
```

All return `ESP_OK` or relevant error codes like `ESP_ERR_INVALID_ARG` or `ESP_ERR_INVALID_SIZE`.

Her er ein finskriven versjon av teksten din, i engelsk og i rein Markdown-stil:

---

### Component Structure

The system is divided into two components:

* `i2c_motorcontroller_master`
* `i2c_motorcontroller_slave`

Both components should share an identical configuration, which includes:

```c
menu "i2c_motorcontroller"

config I2C_MOTCTRL_SDA_GPIO
    int "SDA GPIO pin"
    default 21
    help
        GPIO number for I2C SDA line used by the motor controller.

config I2C_MOTCTRL_SCL_GPIO
    int "SCL GPIO pin"
    default 22
    help
        GPIO number for I2C SCL line used by the motor controller.

config I2C_MOTCTRL_CLK_SPEED
    int "I2C clock speed (Hz)"
    default 400000
    help
        Clock speed for I2C communication in Hz.

config MOTCTRL_I2C_NUM
    int "I2C port number"
    range 0 1
    default 0
    help
        Select the I2C controller port (typically 0 or 1 on ESP32).
        Make sure this matches your hardware setup.

config I2C_MOTCTRL_ADDR
    hex "Motor controller I2C address"
    default 0x01
    help
        I2C address for the motor controller slave.

endmenu
```

> ⚠️ Note: Handling `MOTCTRL_I2C_NUM` may require conditional logic depending on the platform or hardware configuration. This can be resolved either by a `switch` statement or by matching the value to the platform datasheet.

**Simplified Internal API**

All configuration should be handled internally so that external code does **not** need to call verbose I2C functions like:

```c
i2c_function(i2c_port_t i2c_num, uint8_t device_address, uint8_t *read_buffer, size_t read_size, TickType_t ticks_to_wait);
```

Instead, we expose clean and structured interfaces such as:

```c
void i2c_motctrl_function(const uint8_t *write_buffer, int timeout_sec);
```

Where `write_buffer` is a predefined struct representing a motor command or request.

This abstraction improves readability, reduces boilerplate, and makes it easier for others to understand and use the API correctly.


## I2C Motorcontroller Idea

To clarify: the slave is the ESP32 motorcontroller, the master is the coordinator and just sends a `motorcontroller_pkg_t` to start the controller. When finished, the controller should call `i2c_motctrl_slave_send_response()`. In both scenarios, the other must wait for the response.

### Master Example

```c
void motorcontroller_task(void* pvArgs){
    ESP_ERROR_CHECK(i2c_motctrl_master_driver_is_installed()); // must be installed beforehand!
    motorcontroller_pkg_t pkg;
    setup_motorcontroller_pkg(&pkg);
    int retry = 5;
    esp_err_t err;
    for(int i = 0; i < retry; ++i){
        i2c_motctrl_master_wake_up_motorcontroller();
        vTaskDelay(pdMS_TO_TICKS(1000));
        err = i2c_motctrl_master_send_pkg(&pkg, 5);
        if (err == ESP_OK) break;
    }
    if (err != ESP_OK) goto end;

    motorcontroller_response_t resp;
    int wait_seconds = motctrl_master_calculate_wait_time(pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    err = i2c_motctrl_master_wait_response(&resp, wait_seconds);
    if (err != ESP_OK) goto end;

    update_motctrl_pkg(&pkg, &resp);
    save_motctrl_pkg(&pkg);
end:
    if (err != ESP_OK){
        xEventGroupSetBits(work_failed_event_group, MOTCTRL_FAILED_BIT);
    }
    xEventGroupSetBits(MOTCTRL_event_group, MOTCTRL_TASK_FINNISHED_BIT);
    ESP_LOGI_THREAD(TAG, "ferdig");
    motctrl_task_handle = NULL;
    vTaskDelete(NULL);
}
```

### Slave Example

```c
void app_main(void){
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI_THREAD(TAG, "Woke up by GPIO (EXT0) - manual mode");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ESP_LOGI_THREAD(TAG, "Woke up by GPIO (EXT1) - slave mode");
            ESP_ERROR_CHECK(i2c_innstall_slave_driver_cnfig());
            motorcontroller_pkg_t pkg;
            int timeout_sec = 20;
            ESP_ERROR_CHECK(i2c_motctrl_slave_wait_pkg(&pkg, timeout_sec));
            motorcontroller_response_t resp;
            start_and_wait_motctrl(&pkg, &resp);
            timeout_sec = 10;
            esp_err_t err;
            for (int i = 0; i < 5; ++i){
                err = i2c_motctrl_slave_send_response(&resp, timeout_sec);
                if (err == ESP_OK) break;
            }
            break;
        default:
            ESP_LOGI_THREAD(TAG, "Undefined or first boot");
            break;
    }
    esp_sleep_enable_timer_wakeup(UINT64_MAX);
}
```

## I2C Driver Initialization

This component does not install the I2C driver for the master. Make sure it is installed beforehand:



For the slave we provide an installer, as it only needs to respond to the master.
Default I2C address is `0x01` and must match on both ends. The adress is set in config (CONFIG_I2C_MOTCTRL_ADDR).
# API - i2c\_motorcontrolle
this is only what we need to create for now, the rest of the function ether exist, or we can asume they do while coding.
## API - i2c\_motorcontroller\_master

```c
esp_err_t i2c_motctrl_master_send_pkg(i2c_port_t i2c_num, const motorcontroller_pkg_t *pkg, int timeout_sec);
esp_err_t i2c_motctrl_master_wait_response(motorcontroller_response_t *resp, int timeout_sec);
esp_err_t i2c_motctrl_master_wake_up_motorcontroller(void);
int motctrl_master_calculate_wait_time(state_t STATE, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent);
```

## API - i2c\_motorcontroller\_slave

```c
esp_err_t i2c_innstall_slave_driver_cnfig(void);
esp_err_t i2c_motctrl_slave_wait_pkg(motorcontroller_pkg_t *pkg);
esp_err_t i2c_motctrl_slave_send_response(const motorcontroller_response_t *resp);
```

### more etails about function that need to be made

### Function needed for i2c\_motorcontroller\_master
for this component we need to develop 
```c
esp_err_t i2c_motctrl_master_send_pkg(i2c_port_t i2c_num, const motorcontroller_pkg_t *pkg, int timeout_sec);
```

* Sends a serialized `motorcontroller_pkg_t` to slave.
this functtion is hewaly just a wrtaper for the funtion:
```c
esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address, 
const uint8_t *write_buffer, size_t write_size, TickType_t ticks_to_wait);
```
but takes care of `calling serialize_pkg(..)` etc.
it should also werify that the slave got the msg, only then should it return `ESP_OK`.


```c
esp_err_t i2c_motctrl_master_wait_response(motorcontroller_response_t *resp, int timeout_sec);
```
* Waits blocking for `motorcontroller_response_t` with timeout.
its expected a long time for the respone of the controller, it should be about the same as `pkg.prev_estimated_cm_per_s`. make sure it dosent burn CPU.
 the master waits blocking for the response. But we cannot block in I2C read for 10 minutes. So we need to break the wait into smaller intervals and check for a cancellation? The requirement does not specify i think we can just use vTask_wait_unitl(prev_estimated_cm_per_s*end_depth); and then start actuly waiting on the i2c line, since that is the time we should get a response! maybe a litle buffer. its run as a task, so its fine to block
```c
 esp_err_t i2c_motctrl_master_wake_up_motorcontroller(void);
```
* sends a signal on CONFIG_MOTCRTL_WAKEUP_PIN to wakeup the sleeping slave. The motorcontroller slave will go to sleep by itself when finnising its purpous.

```c
int motctrl_master_calculate_wait_time(state_t STATE, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent);
```
* easy formula that looks at the state (rising or lowering) and returns an timeout with a 10% buffer.

and to adress these function
```c
esp_eer_t update_motctrl_pkg(motorcontroller_pkg_t *pkg, motorcontroller_response_t *resp);
esp_eer_t save_motctrl_pkg(motorcontroller_pkg_t *pkg);
```
they are helperfunction from another component, and does what it sais..

## Deep Sleep Consideration

* The master saves previous states to SD card before sleeping.
* The motorcontroller does not retain state and expects a fresh `motorcontroller_pkg_t` on boot.

## Example of Serializing without I2C

```c
#include "serde_helper.h"
#include "buoye_structs.h"
#include "esp_log.h"

void app_main(void) {
    motorcontroller_pkg_t pkg;
    motorcontroller_response_t resp;
    motorcontroller_pkg_init_default(&pkg);
    motorcontroller_response_init_default(&resp);

    uint8_t buf[256];
    size_t len;

    esp_err_t err = serialize_pkg(&pkg, buf, &len);
    if (err == ESP_OK) {
        ESP_LOGI("TEST", "Serialized pkg to %d bytes", len);
        err = deserialize_pkg(buf, len, &pkg);
        if (err == ESP_OK) {
            ESP_LOGI("TEST", "Deserialized pkg OK");
        }
    }

    err = serialize_resp(&resp, buf, &len);
    if (err == ESP_OK) {
        ESP_LOGI("TEST", "Serialized response to %d bytes", len);
        err = deserialize_resp(buf, len, &resp);
        if (err == ESP_OK) {
            ESP_LOGI("TEST", "Deserialized response OK");
        }
    }
}
```
The serilization function exist in serde_helper:
```c
#pragma once

#include "buoye_structs.h"
#include "esp_err.h"
#include <string.h>
#include "esp_crc.h"

#pragma pack(push,1)
typedef struct { 
    motorcontroller_pkg_t pkg;
    uint16_t crc;
} pkg_wire_t;

typedef struct { 
    motorcontroller_response_t resp;
    uint16_t crc;
} resp_wire_t;
#pragma pack(pop)

#define WIRE_PKG_SIZE   sizeof(pkg_wire_t)
#define WIRE_RESP_SIZE  sizeof(resp_wire_t)

// CRC calculation helpers
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg);
uint16_t calculate_resp_crc(const motorcontroller_response_t *resp);

esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len, uint16_t crc);
esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc);
esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len, uint16_t crc);
esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp, uint16_t *crc);
```

and implemented like this:
```c
#include "serde_helper.h"
#include "esp_crc.h"

// Use ESP-IDF's CRC16 implementation
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg) {
    return esp_crc16_le(UINT16_MAX, (const uint8_t*)pkg, sizeof(motorcontroller_pkg_t));
}

uint16_t calculate_resp_crc(const motorcontroller_response_t *resp) {
    return esp_crc16_le(UINT16_MAX, (const uint8_t*)resp, sizeof(motorcontroller_response_t));
}


esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len, uint16_t crc) {
    if (!pkg || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    
    pkg_wire_t wire_pkg = {
        .pkg = *pkg,
        .crc = crc
    };
    
    memcpy(buf, &wire_pkg, WIRE_PKG_SIZE);
    *out_len = WIRE_PKG_SIZE;
    return ESP_OK;
}

esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc) {
    if (!buf || !pkg || !crc) return ESP_ERR_INVALID_ARG;
    if (len < WIRE_PKG_SIZE) return ESP_ERR_INVALID_SIZE;
    
    pkg_wire_t wire_pkg;
    memcpy(&wire_pkg, buf, WIRE_PKG_SIZE);
    *pkg = wire_pkg.pkg;
    *crc = wire_pkg.crc;
    
    return ESP_OK;
}

esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len, uint16_t crc) {
    if (!resp || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    
    resp_wire_t wire_resp = {
        .resp = *resp,
        .crc = crc
    };
    
    memcpy(buf, &wire_resp, WIRE_RESP_SIZE);
    *out_len = WIRE_RESP_SIZE;
    return ESP_OK;
}

esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp, uint16_t *crc) {
    if (!buf || !resp || !crc) return ESP_ERR_INVALID_ARG;
    if (len < WIRE_RESP_SIZE) return ESP_ERR_INVALID_SIZE;
    
    resp_wire_t wire_resp;
    memcpy(&wire_resp, buf, WIRE_RESP_SIZE);
    *resp = wire_resp.resp;
    *crc = wire_resp.crc;
    
    return ESP_OK;
}
```


create cmake and the c/h file of the two components only. use the sugested Kconfig.projbuild file in the tekst above. cand use the helperfuncttion. the structs motorcontroller_response_t and motorcontroller_pkg_t are included from buoye_structs.h and the serilize and crc checking is included from serde_helper.h. this is the testing main we want working for now!

```c
#include <stdio.h>
#include "buoye_structs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "freertos/event_groups.h"

// #define TEST_AS_MASTER

#ifdef TEST_AS_MASTER
#include "i2c_motorcontroller_master.h"
#include "driver/i2c.h"
#else
#include "i2c_motorcontroller_slave.h"
#endif

static const char *TAG = "I2C_TEST";

// Debug print functions for structs
void print_pkg(const motorcontroller_pkg_t *pkg) {
    ESP_LOGI(TAG, "motorcontroller_pkg_t:");
    ESP_LOGI(TAG, "  STATE: %d", pkg->STATE);
    ESP_LOGI(TAG, "  prev_working_time: %d", pkg->prev_working_time);
    ESP_LOGI(TAG, "  rising_timeout_percent: %d", pkg->rising_timeout_percent);
    ESP_LOGI(TAG, "  prev_reported_depth: %u", pkg->prev_reported_depth);
    ESP_LOGI(TAG, "  prev_end_depth: %u", pkg->prev_end_depth);
    ESP_LOGI(TAG, "  prev_estimated_cm_per_s: %u", pkg->prev_estimated_cm_per_s);
    ESP_LOGI(TAG, "  poll_type: %d", pkg->poll_type);
    ESP_LOGI(TAG, "  end_depth: %u", pkg->end_depth);
    ESP_LOGI(TAG, "  samples: %u", pkg->samples);
    ESP_LOGI(TAG, "  static_poll_interval_s: %u", pkg->static_poll_interval_s);
    ESP_LOGI(TAG, "  alpha: %.2f", pkg->alpha);
    ESP_LOGI(TAG, "  beta: %.2f", pkg->beta);
}

void print_resp(const motorcontroller_response_t *resp) {
    ESP_LOGI(TAG, "motorcontroller_response_t:");
    ESP_LOGI(TAG, "  STATE: %d", resp->STATE);
    ESP_LOGI(TAG, "  result: %d", resp->result);
    ESP_LOGI(TAG, "  working_time: %d", resp->working_time);
    ESP_LOGI(TAG, "  estimated_cm_per_s: %u", resp->estimated_cm_per_s);
}


void app_main(void) {
    #ifdef TEST_AS_MASTER
    /********** MASTER TEST **********/
    ESP_LOGI(TAG, "Starting I2C MASTER test");
    
    // Initialize I2C master driver
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MOTCTRL_SDA_GPIO,
        .scl_io_num = CONFIG_I2C_MOTCTRL_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_I2C_MOTCTRL_CLK_SPEED,
    };
    i2c_param_config(CONFIG_MOTCTRL_I2C_NUM, &conf);
    i2c_driver_install(CONFIG_MOTCTRL_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    
    // Create test package
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    pkg.STATE = LOWERING;
    pkg.end_depth = 100;
    pkg.samples = 5;
    pkg.alpha = 0.7;
    pkg.beta = 0.3;
    
    ESP_LOGI(TAG, "Sending package:");
    print_pkg(&pkg);
    
    // Wake up slave
    ESP_LOGI(TAG, "Waking up slave...");
    i2c_motctrl_master_wake_up_motorcontroller();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Send package
    ESP_LOGI(TAG, "Sending package to slave...");
    esp_err_t err = i2c_motctrl_master_send_pkg(&pkg, 5);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Package send failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Calculate and wait for response
    int wait_time = motctrl_master_calculate_wait_time(
        pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    ESP_LOGI(TAG, "Waiting for response (timeout: %d seconds)...", wait_time);
    
    motorcontroller_response_t resp;
    err = i2c_motctrl_master_wait_response(&resp, wait_time);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Response wait failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Received response:");
    print_resp(&resp);
    
    ESP_LOGI(TAG, "MASTER test complete!");
    
    #else
    /********** SLAVE TEST **********/
    ESP_LOGI(TAG, "Starting I2C SLAVE test");
    
    // Install slave driver
    esp_err_t err = i2c_innstall_slave_driver_cnfig();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Slave driver install failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Wait for package
    motorcontroller_pkg_t pkg;
    int timeout_sec = 20;
    ESP_LOGI(TAG, "Waiting for package (timeout: %d seconds)...", timeout_sec);
    err = i2c_motctrl_slave_wait_pkg(&pkg, timeout_sec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Package wait failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Received package:");
    print_pkg(&pkg);
    
    // Simulate work (2 seconds)
    ESP_LOGI(TAG, "Simulating work (2 seconds)...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Prepare response
    motorcontroller_response_t resp;
    motorcontroller_response_init_default(&resp);
    resp.STATE = pkg.STATE;
    resp.result = ESP_OK;
    resp.working_time = 120;
    resp.estimated_cm_per_s = 5000;
    
    ESP_LOGI(TAG, "Sending response:");
    print_resp(&resp);
    
    // Send response
    timeout_sec = 10;
    for (int i = 0; i < 5; ++i) {
        err = i2c_motctrl_slave_send_response(&resp, timeout_sec);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Response sent successfully");
            break;
        }
        ESP_LOGW(TAG, "Attempt %d: %s", i+1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response after 5 attempts");
    }
    
    ESP_LOGI(TAG, "SLAVE test complete!");
    #endif
}
```















































































okay, i struggle with my i2c, im not sure if its code or harwere, but i can see on this example for 12c_slave its used a more sofisticated methid using isr and callbacs and event hjandler. maybe its time to rewrite completly to this style? making it more robust: /*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "driver/i2c_slave.h"

static const char *TAG = "example";

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM    0
#define ESP_SLAVE_ADDR   CONFIG_I2C_SLAVE_ADDRESS           /*!< ESP slave address, you can set any 7bit value */

// Command Lists
#define STARS_COMMAND          (0x10)
#define FORKS_COMMAND          (0x20)
#define OPENISSUES_COMMAND     (0x30)
#define DESCRIPTIONS_COMMAND   (0x40)

#define GITHUB_API_URL "https://api.github.com/repos/espressif/esp-idf"

typedef struct {
    char *json_buffer;
    int json_size;
    uint8_t tmp_buffer_stars[sizeof(int)];
    uint8_t tmp_buffer_forks[sizeof(int)];
    uint8_t tmp_buffer_open_issues[sizeof(int)];
    uint8_t tmp_buffer_descriptions[100];
    QueueHandle_t event_queue;
    uint8_t command_data;
    i2c_slave_dev_handle_t handle;
} i2c_slave_github_context_t;

typedef enum {
    I2C_SLAVE_EVT_RX,
    I2C_SLAVE_EVT_TX
} i2c_slave_event_t;

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)evt->user_data;
    int star_count = 0, forks_count = 0, open_issues_count = 0;

    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (evt->data_len > 0) {
            if (context->json_buffer == NULL) {
                context->json_buffer = malloc(evt->data_len + 1);
            } else {
                context->json_buffer = realloc(context->json_buffer, context->json_size + evt->data_len + 1);
            }
            if (context->json_buffer == NULL) {
                ESP_LOGE("HTTP_CLIENT", "Failed to allocate memory for data json_buffer");
                return ESP_FAIL;
            }
            memcpy(context->json_buffer + context->json_size, evt->data, evt->data_len);
            context->json_size += evt->data_len;
            context->json_buffer[context->json_size] = '\0';  // Null-terminate the string
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        if (context->json_buffer != NULL) {
            // Process received data
            cJSON *root = cJSON_Parse(context->json_buffer);
            cJSON *stars = cJSON_GetObjectItem(root, "stargazers_count");

            if (stars != NULL) {
                star_count = stars->valueint;
                printf("Star count: %d\n", star_count);
                memcpy(context->tmp_buffer_stars, &star_count, sizeof(int));
            }
            cJSON *forks = cJSON_GetObjectItem(root, "forks_count");
            if (forks != NULL) {
                forks_count = forks->valueint;
                printf("Forks count: %d\n", forks_count);
                memcpy(context->tmp_buffer_forks, &forks_count, sizeof(int));
            }
            cJSON *open_issues = cJSON_GetObjectItem(root, "open_issues_count");
            if (open_issues != NULL) {
                open_issues_count = open_issues->valueint;
                printf("issue count: %d\n", open_issues_count);
                memcpy(context->tmp_buffer_open_issues, &open_issues_count, sizeof(int));
            }
            cJSON *descriptions = cJSON_GetObjectItem(root, "description");
            if (descriptions != NULL) {
                printf("the description is: %s\n", descriptions->valuestring);
                memcpy(context->tmp_buffer_descriptions, descriptions->valuestring, strlen(descriptions->valuestring));
            }
            cJSON_Delete(root);
            free(context->json_buffer);
            context->json_buffer = NULL;
            context->json_size = 0;
        }
        break;
    default:
        break;
    }

    return ESP_OK;
}

void http_get_task(void *pvParameters)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)pvParameters;

    esp_http_client_config_t config = {
        .url = GITHUB_API_URL,
        .event_handler = _http_event_handler,
        .method = HTTP_METHOD_GET,
        .buffer_size = 2048,
        .user_data = context,
    };

    while (1) {
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI("HTTP_CLIENT", "HTTP GET Status = %d, content_length = %lld",
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE("HTTP_CLIENT", "HTTP GET request failed: %s", esp_err_to_name(err));
        }
        esp_http_client_cleanup(client);
        vTaskDelay(30 * 60 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static bool i2c_slave_request_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_TX;
    BaseType_t xTaskWoken = 0;
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

static bool i2c_slave_receive_cb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)arg;
    i2c_slave_event_t evt = I2C_SLAVE_EVT_RX;
    BaseType_t xTaskWoken = 0;
    // Command only contains one byte, so just save one bytes here.
    context->command_data = *evt_data->buffer;
    xQueueSendFromISR(context->event_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

static void i2c_slave_task(void *arg)
{
    i2c_slave_github_context_t *context = (i2c_slave_github_context_t *)arg;
    i2c_slave_dev_handle_t handle = (i2c_slave_dev_handle_t)context->handle;

    uint8_t zero_buffer[32] = {}; // Use this buffer to clear the fifo.
    uint32_t write_len, total_written;
    uint32_t buffer_size = 0;

    while (true) {
        i2c_slave_event_t evt;
        if (xQueueReceive(context->event_queue, &evt, 10) == pdTRUE) {
            if (evt == I2C_SLAVE_EVT_TX) {
                uint8_t *data_buffer;
                switch (context->command_data) {
                case STARS_COMMAND:
                    data_buffer = context->tmp_buffer_stars;
                    buffer_size = sizeof(context->tmp_buffer_stars);
                    break;
                case FORKS_COMMAND:
                    data_buffer = context->tmp_buffer_forks;
                    buffer_size = sizeof(context->tmp_buffer_forks);
                    break;
                case OPENISSUES_COMMAND:
                    data_buffer = context->tmp_buffer_open_issues;
                    buffer_size = sizeof(context->tmp_buffer_open_issues);
                    break;
                case DESCRIPTIONS_COMMAND:
                    data_buffer = context->tmp_buffer_descriptions;
                    buffer_size = sizeof(context->tmp_buffer_descriptions);
                    break;
                default:
                    ESP_LOGE(TAG, "Invalid command");
                    data_buffer = zero_buffer;
                    buffer_size = sizeof(zero_buffer);
                    break;
                }

                total_written = 0;
                while (total_written < buffer_size) {
                    ESP_ERROR_CHECK(i2c_slave_write(handle, data_buffer + total_written, buffer_size - total_written, &write_len, 1000));
                    if (write_len == 0) {
                        ESP_LOGE(TAG, "Write error or timeout");
                        break;
                    }
                    total_written += write_len;
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    static i2c_slave_github_context_t context = {0};

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    context.event_queue = xQueueCreate(16, sizeof(i2c_slave_event_t));
    if (!context.event_queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    i2c_slave_config_t i2c_slv_config = {
        .i2c_port = I2C_SLAVE_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .slave_addr = ESP_SLAVE_ADDR,
        .send_buf_depth = 100,
        .receive_buf_depth = 100,
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &context.handle));
    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_slave_receive_cb,
        .on_request = i2c_slave_request_cb,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(context.handle, &cbs, &context));

    xTaskCreate(http_get_task, "http_get_task", 4096, &context, 20, NULL);
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 1024 * 4, &context, 10, NULL);
}
or is this overkill? i also found this example of soft_i2c_master.. semes more simple, what is "soft"? /* but then i also looked at an example of writing to a oledscreen for inspiration and se they used something called "i2c_new_master_bus" maybe this is mopre robust? 

my project is suposed, for now to be sending config and result between a esp controling a motor and a esp passing commands and a lots of other tasks. i writen a document of my project goal, but maybe my aproch is rong. maybe i hould aim for a more event based solution as the first example showes. here art my end goal. keep in minde i,have added crc to my serde_helper component. and the menu i have created. # I2C MotorController Component

## Purpose

This component wraps ESP-IDF's I2C functionality to create a clean and minimal API for communication between a master (main ESP) and a slave (motorcontroller ESP). It enables the master to send a control package to the motorcontroller and receive a response when the action is complete. The master can safely go into deep sleep and persist state using SD card, while the motorcontroller can wake up fresh and receive everything needed via I2C.


## Structs Used

### motorcontroller\_pkg\_t (from master to controller)

Used to initiate a lowering or raising cycle with complete context:
this is included by `buoye_structs.h` and does not need to be created here
```c
typedef struct {
    state_t STATE;                   // current operation mode: LOWERING or RISING
    int prev_working_time;          // duration (seconds) of last operation, used for estimating next time
    int rising_timeout_percent;     // factor to define timeout threshold for rising if home sensor not seen
    uint16_t prev_reported_depth;   // last measured depth (cm)
    uint16_t prev_end_depth;        // previous target depth (cm)
    uint16_t prev_estimated_cm_per_s; // last estimated lowering speed in cm/s (scaled e.g., 10000 => 10.000 cm/s)
    POLL_TYPE poll_type;            // type of polling (e.g., STATIC_DEPTH or CONTINUOUS)
    uint16_t end_depth;             // target depth for this operation (cm)
    uint16_t static_points[MAX_POINTS]; // depths at which to pause and sample (cm), terminated by 0
    uint16_t samples;               // sample count at each static point
    uint16_t static_poll_interval_s; // interval between static samples (seconds)

    // added for filter operation and timing
    double alpha;                   // alpha gain for alpha-beta filter
    double beta;                    // beta gain for alpha-beta filter
} motorcontroller_pkg_t;
```

Use `motorcontroller_pkg_init_default()` to initialize with defaults.

### motorcontroller\_response\_t (from controller back to master)

Returned when the motorcontroller completes its task:

```c
typedef struct {
    state_t STATE;                  // operation mode performed
    esp_err_t result;              // result code: ESP_OK, ESP_ERR_TIMEOUT, TENSION_ALARM, etc.
    int working_time;              // total time (seconds) taken for the elevation
    uint16_t estimated_cm_per_s;  // new speed estimate (scaled)
} motorcontroller_response_t;
```

Use `motorcontroller_response_init_default()` to zero out/initialize.

## Serialization Helpers (serde\_helper)

These are used internally by the I2C master/slave component to serialize/deserialize packages.

### Functions:
this is included by `serde_helper.h` and does not need to be created here
```c
esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len);
esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg);
esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len);
esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp);
```

All return `ESP_OK` or relevant error codes like `ESP_ERR_INVALID_ARG` or `ESP_ERR_INVALID_SIZE`.


## I2C Motorcontroller Idea

To clarify: the slave is the ESP32 motorcontroller, the master is the coordinator and just sends a `motorcontroller_pkg_t` to start the controller. When finished, the controller should call `i2c_motctrl_slave_send_response()`. In both scenarios, the other must wait for the response.

### Master Example

```c
void motorcontroller_task(void* pvArgs){
    ESP_ERROR_CHECK(i2c_motctrl_master_driver_is_installed()); // must be installed beforehand!
    motorcontroller_pkg_t pkg;
    setup_motorcontroller_pkg(&pkg);
    int retry = 5;
    esp_err_t err;
    for(int i = 0; i < retry; ++i){
        i2c_motctrl_master_wake_up_motorcontroller();
        vTaskDelay(pdMS_TO_TICKS(1000));
        err = i2c_motctrl_master_send_pkg(&pkg, 5);
        if (err == ESP_OK) break;
    }
    if (err != ESP_OK) goto end;

    motorcontroller_response_t resp;
    int wait_seconds = motctrl_master_calculate_wait_time(pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    err = i2c_motctrl_master_wait_response(&resp, wait_seconds);
    if (err != ESP_OK) goto end;

    update_motctrl_pkg(&pkg, &resp);
    save_motctrl_pkg(&pkg);
end:
    if (err != ESP_OK){
        xEventGroupSetBits(work_failed_event_group, MOTCTRL_FAILED_BIT);
    }
    xEventGroupSetBits(MOTCTRL_event_group, MOTCTRL_TASK_FINNISHED_BIT);
    ESP_LOGI_THREAD(TAG, "ferdig");
    motctrl_task_handle = NULL;
    vTaskDelete(NULL);
}
```

### Slave Example

```c
void app_main(void){
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI_THREAD(TAG, "Woke up by GPIO (EXT0) - manual mode");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ESP_LOGI_THREAD(TAG, "Woke up by GPIO (EXT1) - slave mode");
            ESP_ERROR_CHECK(i2c_innstall_slave_driver_cnfig());
            motorcontroller_pkg_t pkg;
            int timeout_sec = 20;
            ESP_ERROR_CHECK(i2c_motctrl_slave_wait_pkg(&pkg, timeout_sec));
            motorcontroller_response_t resp;
            start_and_wait_motctrl(&pkg, &resp);
            timeout_sec = 10;
            esp_err_t err;
            for (int i = 0; i < 5; ++i){
                err = i2c_motctrl_slave_send_response(&resp, timeout_sec);
                if (err == ESP_OK) break;
            }
            break;
        default:
            ESP_LOGI_THREAD(TAG, "Undefined or first boot");
            break;
    }
    esp_sleep_enable_timer_wakeup(UINT64_MAX);
}
```

The serilization function exist in serde_helper:
```c
#pragma once

#include "buoye_structs.h"
#include "esp_err.h"
#include <string.h>
#include "esp_crc.h"

#pragma pack(push,1)
typedef struct { 
    motorcontroller_pkg_t pkg;
    uint16_t crc;
} pkg_wire_t;

typedef struct { 
    motorcontroller_response_t resp;
    uint16_t crc;
} resp_wire_t;
#pragma pack(pop)

#define WIRE_PKG_SIZE   sizeof(pkg_wire_t)
#define WIRE_RESP_SIZE  sizeof(resp_wire_t)

// CRC calculation helpers
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg);
uint16_t calculate_resp_crc(const motorcontroller_response_t *resp);

esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len, uint16_t crc);
esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc);
esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len, uint16_t crc);
esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp, uint16_t *crc);
```

and implemented like this:
```c
#include "serde_helper.h"
#include "esp_crc.h"

// Use ESP-IDF's CRC16 implementation
uint16_t calculate_pkg_crc(const motorcontroller_pkg_t *pkg) {
    return esp_crc16_le(UINT16_MAX, (const uint8_t*)pkg, sizeof(motorcontroller_pkg_t));
}

uint16_t calculate_resp_crc(const motorcontroller_response_t *resp) {
    return esp_crc16_le(UINT16_MAX, (const uint8_t*)resp, sizeof(motorcontroller_response_t));
}


esp_err_t serialize_pkg(const motorcontroller_pkg_t *pkg, uint8_t *buf, size_t *out_len, uint16_t crc) {
    if (!pkg || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    
    pkg_wire_t wire_pkg = {
        .pkg = *pkg,
        .crc = crc
    };
    
    memcpy(buf, &wire_pkg, WIRE_PKG_SIZE);
    *out_len = WIRE_PKG_SIZE;
    return ESP_OK;
}

esp_err_t deserialize_pkg(const uint8_t *buf, size_t len, motorcontroller_pkg_t *pkg, uint16_t *crc) {
    if (!buf || !pkg || !crc) return ESP_ERR_INVALID_ARG;
    if (len < WIRE_PKG_SIZE) return ESP_ERR_INVALID_SIZE;
    
    pkg_wire_t wire_pkg;
    memcpy(&wire_pkg, buf, WIRE_PKG_SIZE);
    *pkg = wire_pkg.pkg;
    *crc = wire_pkg.crc;
    
    return ESP_OK;
}

esp_err_t serialize_resp(const motorcontroller_response_t *resp, uint8_t *buf, size_t *out_len, uint16_t crc) {
    if (!resp || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    
    resp_wire_t wire_resp = {
        .resp = *resp,
        .crc = crc
    };
    
    memcpy(buf, &wire_resp, WIRE_RESP_SIZE);
    *out_len = WIRE_RESP_SIZE;
    return ESP_OK;
}

esp_err_t deserialize_resp(const uint8_t *buf, size_t len, motorcontroller_response_t *resp, uint16_t *crc) {
    if (!buf || !resp || !crc) return ESP_ERR_INVALID_ARG;
    if (len < WIRE_RESP_SIZE) return ESP_ERR_INVALID_SIZE;
    
    resp_wire_t wire_resp;
    memcpy(&wire_resp, buf, WIRE_RESP_SIZE);
    *resp = wire_resp.resp;
    *crc = wire_resp.crc;
    
    return ESP_OK;
}
```


create cmake and the c/h file of the two components only. use the sugested Kconfig.projbuild file in the tekst above. cand use the helperfuncttion. the structs motorcontroller_response_t and motorcontroller_pkg_t are included from buoye_structs.h and the serilize and crc checking is included from serde_helper.h. this is the testing main we want working for now!

```c
#include <stdio.h>
#include "buoye_structs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "freertos/event_groups.h"

// #define TEST_AS_MASTER

#ifdef TEST_AS_MASTER
#include "i2c_motorcontroller_master.h"
#include "driver/i2c.h"
#else
#include "i2c_motorcontroller_slave.h"
#endif

static const char *TAG = "I2C_TEST";

void app_main(void) {
    #ifdef TEST_AS_MASTER
    /********** MASTER TEST **********/
    ESP_LOGI(TAG, "Starting I2C MASTER test");
    
    // Initialize I2C master driver
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MOTCTRL_SDA_GPIO,
        .scl_io_num = CONFIG_I2C_MOTCTRL_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_I2C_MOTCTRL_CLK_SPEED,
    };
    i2c_param_config(CONFIG_MOTCTRL_I2C_NUM, &conf);
    i2c_driver_install(CONFIG_MOTCTRL_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    
    // Create test package
    motorcontroller_pkg_t pkg;
    motorcontroller_pkg_init_default(&pkg);
    pkg.STATE = LOWERING;
    pkg.end_depth = 100;
    pkg.samples = 5;
    pkg.alpha = 0.7;
    pkg.beta = 0.3;
    
    ESP_LOGI(TAG, "Sending package:");
    print_pkg(&pkg);
    
    // Wake up slave
    ESP_LOGI(TAG, "Waking up slave...");
    i2c_motctrl_master_wake_up_motorcontroller();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Send package
    ESP_LOGI(TAG, "Sending package to slave...");
    esp_err_t err = i2c_motctrl_master_send_pkg(&pkg, 5);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Package send failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Calculate and wait for response
    int wait_time = motctrl_master_calculate_wait_time(
        pkg.STATE, pkg.prev_estimated_cm_per_s, pkg.rising_timeout_percent);
    ESP_LOGI(TAG, "Waiting for response (timeout: %d seconds)...", wait_time);
    
    motorcontroller_response_t resp;
    err = i2c_motctrl_master_wait_response(&resp, wait_time);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Response wait failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Received response:");
    print_resp(&resp);
    
    ESP_LOGI(TAG, "MASTER test complete!");
    
    #else
    /********** SLAVE TEST **********/
    ESP_LOGI(TAG, "Starting I2C SLAVE test");
    
    // Install slave driver
    esp_err_t err = i2c_innstall_slave_driver_cnfig();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Slave driver install failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Wait for package
    motorcontroller_pkg_t pkg;
    int timeout_sec = 20;
    ESP_LOGI(TAG, "Waiting for package (timeout: %d seconds)...", timeout_sec);
    err = i2c_motctrl_slave_wait_pkg(&pkg, timeout_sec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Package wait failed: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Received package:");
    print_pkg(&pkg);
    
    // Simulate work (2 seconds)
    ESP_LOGI(TAG, "Simulating work (2 seconds)...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Prepare response
    motorcontroller_response_t resp;
    motorcontroller_response_init_default(&resp);
    resp.STATE = pkg.STATE;
    resp.result = ESP_OK;
    resp.working_time = 120;
    resp.estimated_cm_per_s = 5000;
    
    ESP_LOGI(TAG, "Sending response:");
    print_resp(&resp);
    
    // Send response
    timeout_sec = 10;
    for (int i = 0; i < 5; ++i) {
        err = i2c_motctrl_slave_send_response(&resp, timeout_sec);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Response sent successfully");
            break;
        }
        ESP_LOGW(TAG, "Attempt %d: %s", i+1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send response after 5 attempts");
    }
    
    ESP_LOGI(TAG, "SLAVE test complete!");
    #endif
}
``` but maybe this should be more event based to avoid the master holding the buss all the time? but we are in a position where we can estimate how long the slave wil work by calculating the end depth times estimated speed, so maybe use this to wait until and the poll with a interwal of 1 second, the slva can run a event handler, and if its ready to give the result (finnished) it gives that, if not it tells him to come back? this would be more inspired by the first example shown. best pracis?
yes, so i started creating the master component: #pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "buoye_structs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C master event types
 */
typedef enum {
    I2C_MASTER_EVT_PKG_SENT,
    I2C_MASTER_EVT_RESP_RECEIVED,
    I2C_MASTER_EVT_TIMEOUT,
    I2C_MASTER_EVT_ERROR
} i2c_master_event_t;

/**
 * @brief I2C master event data
 */
typedef struct {
    i2c_master_event_t event;
    esp_err_t error_code;
    union {
        struct {
            motorcontroller_response_t response;
            bool crc_valid;
        } resp_data;
        struct {
            bool ack_received;
        } pkg_data;
    };
} i2c_master_event_data_t;

/**
 * @brief I2C master context structure
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    QueueHandle_t event_queue;
    TaskHandle_t poll_task_handle;
    EventGroupHandle_t operation_event_group;
    
    // Current operation state
    motorcontroller_pkg_t current_pkg;
    motorcontroller_response_t received_resp;
    bool operation_active;
    TickType_t operation_start_time;
    int expected_duration_sec;
    
    // Buffers for communication
    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];
    size_t tx_len;
    size_t rx_len;
} i2c_motctrl_master_ctx_t;

// Event group bits
#define MOTCTRL_MASTER_PKG_SENT_BIT        (1 << 0)
#define MOTCTRL_MASTER_RESP_RECEIVED_BIT   (1 << 1)
#define MOTCTRL_MASTER_TIMEOUT_BIT         (1 << 2)
#define MOTCTRL_MASTER_ERROR_BIT           (1 << 3)

/**
 * @brief Initialize I2C master for motor controller communication
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_init(void);

/**
 * @brief Deinitialize I2C master
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_deinit(void);

/**
 * @brief Send package to motor controller slave
 * 
 * @param pkg Package to send
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_send_pkg(const motorcontroller_pkg_t *pkg, int timeout_sec);

/**
 * @brief Wait for response from motor controller slave
 * 
 * @param resp Response structure to fill
 * @param timeout_sec Timeout in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_wait_response(motorcontroller_response_t *resp, int timeout_sec);

/**
 * @brief Wake up motor controller slave
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_wake_up_motorcontroller(void);

/**
 * @brief Calculate expected wait time for motor controller operation
 * 
 * @param state Operation state (LOWERING or RISING)
 * @param prev_estimated_cm_per_s Previous speed estimate
 * @param rising_timeout_percent Timeout percentage for rising
 * @return Expected wait time in seconds
 */
int motctrl_master_calculate_wait_time(state_t state, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent);

/**
 * @brief Check if motor controller is ready for new operation
 * 
 * @return true if ready, false otherwise
 */
bool i2c_motctrl_master_is_ready(void);

/**
 * @brief Cancel current operation
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motctrl_master_cancel_operation(void);

#ifdef __cplusplus
}
#endif 

and the c file
#include "i2c_motorcontroller_master.h"
#include "serde_helper.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

static const char *TAG = "i2c_motctrl_master";

static i2c_motctrl_master_ctx_t s_master_ctx = {0};

// Poll task for checking slave status
static void i2c_master_poll_task(void *pvParameters);

// Internal helper functions
static esp_err_t i2c_master_transmit_pkg(const motorcontroller_pkg_t *pkg);
static esp_err_t i2c_master_receive_response(motorcontroller_response_t *resp);
static esp_err_t i2c_master_check_slave_ready(bool *ready);

esp_err_t i2c_motctrl_master_init(void)
{
    if (s_master_ctx.bus_handle != NULL) {
        ESP_LOGW(TAG, "Master already initialized");
        return ESP_OK;
    }

    // Initialize I2C master bus
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_MOTCTRL_I2C_NUM,
        .scl_io_num = CONFIG_I2C_MOTCTRL_SCL_GPIO,
        .sda_io_num = CONFIG_I2C_MOTCTRL_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &s_master_ctx.bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add device to bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_I2C_MOTCTRL_ADDR,
        .scl_speed_hz = CONFIG_I2C_MOTCTRL_CLK_SPEED,
    };

    ret = i2c_master_bus_add_device(s_master_ctx.bus_handle, &dev_cfg, &s_master_ctx.dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_master_ctx.bus_handle);
        s_master_ctx.bus_handle = NULL;
        return ret;
    }

    // Create event queue
    s_master_ctx.event_queue = xQueueCreate(10, sizeof(i2c_master_event_data_t));
    if (s_master_ctx.event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        i2c_master_bus_rm_device(s_master_ctx.dev_handle);
        i2c_del_master_bus(s_master_ctx.bus_handle);
        s_master_ctx.bus_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Create operation event group
    s_master_ctx.operation_event_group = xEventGroupCreate();
    if (s_master_ctx.operation_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create operation event group");
        vQueueDelete(s_master_ctx.event_queue);
        i2c_master_bus_rm_device(s_master_ctx.dev_handle);
        i2c_del_master_bus(s_master_ctx.bus_handle);
        s_master_ctx.bus_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_motctrl_master_deinit(void)
{
    if (s_master_ctx.bus_handle == NULL) {
        return ESP_OK;
    }

    // Stop poll task if running
    if (s_master_ctx.poll_task_handle != NULL) {
        vTaskDelete(s_master_ctx.poll_task_handle);
        s_master_ctx.poll_task_handle = NULL;
    }

    // Clean up resources
    if (s_master_ctx.operation_event_group != NULL) {
        vEventGroupDelete(s_master_ctx.operation_event_group);
        s_master_ctx.operation_event_group = NULL;
    }

    if (s_master_ctx.event_queue != NULL) {
        vQueueDelete(s_master_ctx.event_queue);
        s_master_ctx.event_queue = NULL;
    }

    i2c_master_bus_rm_device(s_master_ctx.dev_handle);
    i2c_del_master_bus(s_master_ctx.bus_handle);
    s_master_ctx.bus_handle = NULL;
    s_master_ctx.dev_handle = NULL;

    ESP_LOGI(TAG, "I2C master deinitialized");
    return ESP_OK;
}

esp_err_t i2c_motctrl_master_send_pkg(const motorcontroller_pkg_t *pkg, int timeout_sec)
{
    if (s_master_ctx.bus_handle == NULL) {
        ESP_LOGE(TAG, "Master not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_master_ctx.operation_active) {
        ESP_LOGE(TAG, "Operation already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear event group
    xEventGroupClearBits(s_master_ctx.operation_event_group, 0xFF);

    // Store package for reference
    s_master_ctx.current_pkg = *pkg;
    s_master_ctx.operation_active = true;
    s_master_ctx.operation_start_time = xTaskGetTickCount();

    // Transmit package
    esp_err_t ret = i2c_master_transmit_pkg(pkg);
    if (ret != ESP_OK) {
        s_master_ctx.operation_active = false;
        return ret;
    }

    // Wait for acknowledgment
    EventBits_t bits = xEventGroupWaitBits(
        s_master_ctx.operation_event_group,
        MOTCTRL_MASTER_PKG_SENT_BIT | MOTCTRL_MASTER_ERROR_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    if (bits & MOTCTRL_MASTER_ERROR_BIT) {
        s_master_ctx.operation_active = false;
        return ESP_FAIL;
    }

    if (!(bits & MOTCTRL_MASTER_PKG_SENT_BIT)) {
        s_master_ctx.operation_active = false;
        ESP_LOGE(TAG, "Package send timeout");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Package sent successfully");
    return ESP_OK;
}

esp_err_t i2c_motctrl_master_wait_response(motorcontroller_response_t *resp, int timeout_sec)
{
    if (s_master_ctx.bus_handle == NULL) {
        ESP_LOGE(TAG, "Master not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_master_ctx.operation_active) {
        ESP_LOGE(TAG, "No operation in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate expected completion time
    s_master_ctx.expected_duration_sec = timeout_sec;

    // Start polling task
    BaseType_t task_ret = xTaskCreate(
        i2c_master_poll_task,
        "i2c_poll",
        2048,
        NULL,
        5,
        &s_master_ctx.poll_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create poll task");
        s_master_ctx.operation_active = false;
        return ESP_ERR_NO_MEM;
    }

    // Wait for response or timeout
    EventBits_t bits = xEventGroupWaitBits(
        s_master_ctx.operation_event_group,
        MOTCTRL_MASTER_RESP_RECEIVED_BIT | MOTCTRL_MASTER_TIMEOUT_BIT | MOTCTRL_MASTER_ERROR_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_sec * 1000)
    );

    // Stop polling task
    if (s_master_ctx.poll_task_handle != NULL) {
        vTaskDelete(s_master_ctx.poll_task_handle);
        s_master_ctx.poll_task_handle = NULL;
    }

    s_master_ctx.operation_active = false;

    if (bits & MOTCTRL_MASTER_ERROR_BIT) {
        ESP_LOGE(TAG, "Communication error while waiting for response");
        return ESP_FAIL;
    }

    if (bits & MOTCTRL_MASTER_TIMEOUT_BIT) {
        ESP_LOGE(TAG, "Response timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (bits & MOTCTRL_MASTER_RESP_RECEIVED_BIT) {
        *resp = s_master_ctx.received_resp;
        ESP_LOGI(TAG, "Response received successfully");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Unexpected timeout waiting for response");
    return ESP_ERR_TIMEOUT;
}

esp_err_t i2c_motctrl_master_wake_up_motorcontroller(void)
{
    // Configure wake-up pin if defined
    #ifdef CONFIG_MOTCTRL_WAKEUP_PIN
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_MOTCTRL_WAKEUP_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    // Send wake-up pulse
    gpio_set_level(CONFIG_MOTCTRL_WAKEUP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CONFIG_MOTCTRL_WAKEUP_PIN, 0);
    
    ESP_LOGI(TAG, "Wake-up signal sent");
    #else
    ESP_LOGW(TAG, "Wake-up pin not configured");
    #endif
    
    return ESP_OK;
}

int motctrl_master_calculate_wait_time(state_t state, uint16_t prev_estimated_cm_per_s, int rising_timeout_percent)
{
    int base_time = 60; // Default 1 minute
    
    if (prev_estimated_cm_per_s > 0) {
        // Estimate based on previous speed
        base_time = (s_master_ctx.current_pkg.end_depth * 100) / prev_estimated_cm_per_s;
    }
    
    if (state == RISING) {
        // Apply rising timeout percentage
        base_time = (base_time * (100 + rising_timeout_percent)) / 100;
    }
    
    // Add 10% buffer
    base_time = (base_time * 110) / 100;
    
    // Minimum 30 seconds, maximum 30 minutes
    if (base_time < 30) base_time = 30;
    if (base_time > 1800) base_time = 1800;
    
    return base_time;
}

bool i2c_motctrl_master_is_ready(void)
{
    return (s_master_ctx.bus_handle != NULL && !s_master_ctx.operation_active);
}

esp_err_t i2c_motctrl_master_cancel_operation(void)
{
    if (!s_master_ctx.operation_active) {
        return ESP_OK;
    }

    // Stop polling task
    if (s_master_ctx.poll_task_handle != NULL) {
        vTaskDelete(s_master_ctx.poll_task_handle);
        s_master_ctx.poll_task_handle = NULL;
    }

    s_master_ctx.operation_active = false;
    xEventGroupSetBits(s_master_ctx.operation_event_group, MOTCTRL_MASTER_ERROR_BIT);

    ESP_LOGI(TAG, "Operation cancelled");
    return ESP_OK;
}

// Internal helper functions
static esp_err_t i2c_master_transmit_pkg(const motorcontroller_pkg_t *pkg)
{
    uint16_t crc = calculate_pkg_crc(pkg);
    esp_err_t ret = serialize_pkg(pkg, s_master_ctx.tx_buffer, &s_master_ctx.tx_len, crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize package");
        return ret;
    }

    ret = i2c_master_transmit(s_master_ctx.dev_handle, s_master_ctx.tx_buffer, s_master_ctx.tx_len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit package: %s", esp_err_to_name(ret));
        xEventGroupSetBits(s_master_ctx.operation_event_group, MOTCTRL_MASTER_ERROR_BIT);
        return ret;
    }

    xEventGroupSetBits(s_master_ctx.operation_event_group, MOTCTRL_MASTER_PKG_SENT_BIT);
    return ESP_OK;
}

static esp_err_t i2c_master_receive_response(motorcontroller_response_t *resp)
{
    esp_err_t ret = i2c_master_receive(s_master_ctx.dev_handle, s_master_ctx.rx_buffer, WIRE_RESP_SIZE, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive response: %s", esp_err_to_name(ret));
        return ret;
    }

    uint16_t received_crc;
    ret = deserialize_resp(s_master_ctx.rx_buffer, WIRE_RESP_SIZE, resp, &received_crc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deserialize response");
        return ret;
    }

    // Verify CRC
    uint16_t calculated_crc = calculate_resp_crc(resp);
    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG, "CRC mismatch: received 0x%04X, calculated 0x%04X", received_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static esp_err_t i2c_master_check_slave_ready(bool *ready)
{
    // Send a simple probe to check if slave is ready
    uint8_t probe_data = 0x00;
    esp_err_t ret = i2c_master_transmit_receive(s_master_ctx.dev_handle, &probe_data, 1, &probe_data, 1, pdMS_TO_TICKS(100));
    
    if (ret == ESP_OK) {
        // Slave responded - check if it's ready (0x01) or busy (0x00)
        *ready = (probe_data == 0x01);
    } else {
        *ready = false;
    }
    
    return ret;
}

static void i2c_master_poll_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t poll_interval = pdMS_TO_TICKS(1000); // Poll every 1 second
    bool slave_ready = false;
    
    ESP_LOGI(TAG, "Starting poll task");
    
    while (s_master_ctx.operation_active) {
        vTaskDelayUntil(&last_wake_time, poll_interval);
        
        // Check if slave is ready
        esp_err_t ret = i2c_master_check_slave_ready(&slave_ready);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to check slave status: %s", esp_err_to_name(ret));
            continue;
        }
        
        if (slave_ready) {
            ESP_LOGI(TAG, "Slave is ready, requesting response");
            
            // Try to receive response
            ret = i2c_master_receive_response(&s_master_ctx.received_resp);
            if (ret == ESP_OK) {
                xEventGroupSetBits(s_master_ctx.operation_event_group, MOTCTRL_MASTER_RESP_RECEIVED_BIT);
                break;
            } else {
                ESP_LOGW(TAG, "Failed to receive response: %s", esp_err_to_name(ret));
            }
        }
        
        // Check for timeout
        TickType_t elapsed = (xTaskGetTickCount() - s_master_ctx.operation_start_time) / portTICK_PERIOD_MS;
        if (elapsed >= (s_master_ctx.expected_duration_sec * 1000)) {
            ESP_LOGW(TAG, "Operation timeout");
            xEventGroupSetBits(s_master_ctx.operation_event_group, MOTCTRL_MASTER_TIMEOUT_BIT);
            break;
        }
    }
    
    ESP_LOGI(TAG, "Poll task finished");
    s_master_ctx.poll_task_handle = NULL;
    vTaskDelete(NULL);
}
Continue with the code for i2c_motorcontroller_slave, the master looks good, but you didnt finnish the slave h file or started on the .c file! keep going with this style