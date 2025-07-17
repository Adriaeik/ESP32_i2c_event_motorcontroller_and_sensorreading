#include "outputs.h"
#include "esp_log.h"

static const char* TAG = "outputs";

// Lokale speil for status
static bool winch_up = false;
static bool winch_down = false;
#if CONFIG_USE_WINCH_RUNNING_LAMP
    static bool lamp_on = false;
#endif
static bool auto_lamp_on = false;
static bool alarm_lamp_on = false;
static bool outputs_initialized = false;

void outputs_init(void) {
    if (outputs_initialized) {
        ESP_LOGW(TAG, "Output system already initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing output system...");
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 
                        #if CONFIG_USE_WINCH_RUNNING_LAMP
                        (1ULL << GPIO_LAMP)       |
                        #endif // CONFIG_USE_WINCH_RUNNING_LAMP
                        #if CONFIG_USE_AUTO_LAMP
                        (1ULL << GPIO_AUTO_LAMP)  |
                        #endif // CONFIG_USE_AUTO_LAMP
                        #if CONFIG_USE_ALARM_LAMP
                        (1ULL << GPIO_ALARM_LAMP) |
                        #endif // CONFIG_USE_ALARM_LAMP
                        (1ULL << GPIO_WINCH_DOWN) |
                        (1ULL << GPIO_WINCH_UP),  
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    // Tving alle utgangar til AV uavhengig av speilverdi
    winch_down = true;
    outputs_set_winch_down(false);

    winch_up = true;
    outputs_set_winch_up(false);
#if CONFIG_USE_WINCH_RUNNING_LAMP
    lamp_on = true;
    outputs_set_lamp(false);
#endif

    auto_lamp_on = true;
    outputs_set_auto_lamp(false);

    alarm_lamp_on = true;
    outputs_set_alarm_lamp(false);
    #if CONFIG_USE_WINCH_RUNNING_LAMP
        ESP_LOGI(TAG, "GPIO %d konfigurert som WINCH_RUNNING_LAMP (OUTPUT)", GPIO_LAMP);
    #endif
    #if CONFIG_USE_AUTO_LAMP
        ESP_LOGI(TAG, "GPIO %d konfigurert som AUTO_LAMP (OUTPUT)", GPIO_AUTO_LAMP);
    #endif
    #if CONFIG_USE_ALARM_LAMP
        ESP_LOGI(TAG, "GPIO %d konfigurert som ALARM_LAMP (OUTPUT)", GPIO_ALARM_LAMP);
    #endif
    ESP_LOGI(TAG, "GPIO %d konfigurert som WINCH_DOWN (OUTPUT)", GPIO_WINCH_DOWN);
    ESP_LOGI(TAG, "GPIO %d konfigurert som WINCH_UP (OUTPUT)", GPIO_WINCH_UP);
    outputs_initialized = true;
    ESP_LOGI(TAG, "Output system initialized");
}

void outputs_deinit(void) {
    if (!outputs_initialized) {
        ESP_LOGD(TAG, "Output system not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Deinitializing output system...");
    
    // Turn off all outputs safely
    outputs_set_winch_down(false);
    outputs_set_winch_up(false);
    outputs_set_lamp(false);
    outputs_set_auto_lamp(false);
    outputs_set_alarm_lamp(false);
    
    // Reset state
    winch_up = false;
    winch_down = false;
#if CONFIG_USE_WINCH_RUNNING_LAMP
    lamp_on = false;
#endif
    auto_lamp_on = false;
    alarm_lamp_on = false;
    
    outputs_initialized = false;
    ESP_LOGI(TAG, "Output system deinitialized");
}

void outputs_set_winch_down(bool enable) {
    if (!outputs_initialized) return;
    
    if (enable && winch_up) {
        // Forrigling: slå av opp før du slår på ned
        outputs_set_winch_up(false);
    }

    if (enable != winch_down) {
        winch_down = enable;
        ESP_LOGI(TAG, "Winch Down: %s", enable ? "PÅ" : "AV");
        gpio_set_level(GPIO_WINCH_DOWN, !enable);
    }
}

void outputs_set_winch_up(bool enable) {
    if (!outputs_initialized) return;
    
    if (enable && winch_down) {
        // Forrigling: slå av ned før du slår på opp
        outputs_set_winch_down(false);
    }

    if (enable != winch_up) {
        winch_up = enable;
        ESP_LOGI(TAG, "Winch Up: %s", enable ? "PÅ" : "AV");
        gpio_set_level(GPIO_WINCH_UP, !enable);
    }
}

void outputs_set_lamp(bool enable) {
    if (!outputs_initialized) return;
    
    #if CONFIG_USE_WINCH_RUNNING_LAMP
    if (enable != lamp_on) {
        lamp_on = enable;
        ESP_LOGI(TAG, "Lamp: %s", enable ? "PÅ" : "AV");
    }
    gpio_set_level(GPIO_LAMP, enable);
    #else
    if (enable) {
        ESP_LOGW(TAG, "Lamp er ikke aktivert i konfigurasjonen.");
    } 
    #endif // CONFIG_USE_WINCH_RUNNING_LAMP
}

void outputs_set_auto_lamp(bool enable) {
    if (!outputs_initialized) return;
    
    #if CONFIG_USE_AUTO_LAMP
    if (enable != auto_lamp_on) {
        auto_lamp_on = enable;
        ESP_LOGI(TAG, "Auto Lamp: %s", enable ? "PÅ" : "AV");
    }
    gpio_set_level(GPIO_AUTO_LAMP, enable);
    #else
    if (enable) {
        ESP_LOGW(TAG, "Auto Lamp er ikke aktivert i konfigurasjonen.");
    }
    #endif // CONFIG_USE_AUTO_LAMP
}

void outputs_set_alarm_lamp(bool enable) {
    if (!outputs_initialized) return;
    
    #if CONFIG_USE_ALARM_LAMP
    if (enable != alarm_lamp_on) {
        alarm_lamp_on = enable;
        ESP_LOGI(TAG, "Alarm Lamp: %s", enable ? "PÅ" : "AV");
    }
    gpio_set_level(GPIO_ALARM_LAMP, enable);
    #else
    if (enable) {
        ESP_LOGW(TAG, "Alarm Lamp er ikke aktivert i konfigurasjonen.");
    }
    #endif // CONFIG_USE_ALARM_LAMP
}

bool outputs_winch_running(void) {
    return outputs_initialized && (winch_up || winch_down);
}