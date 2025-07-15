#include <math.h> // for floor()
#include "set_data.h"
#include "log_thread.h"
#include "esp_check.h"


static const char *TAG = "set_data";


esp_err_t get_sheared_config_default(sheared_config_t *c)
{
    *c = sheared_config_default();
    return ESP_OK;
}

// === TEMP HUB CONFIG ===
static sheared_config_t temp_hub_config;
esp_err_t set_temp_hub_config(const sheared_config_t *config) {
    temp_hub_config = *config;
    return ESP_OK;
}
esp_err_t get_temp_hub_config(sheared_config_t *config) {
    *config = temp_hub_config;
    return ESP_OK;
}


time_t controll_current_time_to_start_work(time_t last_time, int period_sec) {
    time_t now = time(NULL);

    // Dersom last_time ikkje er initialisert eller vi er altfor seint ute
    if (last_time == 0 || last_time + period_sec < now) {
        // Legg inn ein liten forsinkelse så begge får tid til å vekke seg og synkronisere
        time_t future_time = now + 10;
        ESP_LOGW_THREAD(TAG, "måtte finne opt tid: %s", ctime(&future_time));
        return future_time;
    }

    return last_time;
}
// muligens inkludere state slik at vi kan ha lågare periodetid ved init
esp_err_t update_work_time(state_t STATE, buoye_config_t *config,  int offset_sec) {
    if (!config) return ESP_ERR_INVALID_ARG;
    if (STATE == INIT) return update_work_time_on_init(config, offset_sec);
    
    config->current_time_to_start_work = controll_current_time_to_start_work(
        config->time_to_start_next_work, // forrige gang vi starta - blir no last-time
        config->sheared_config.periodic_time_sec
    );
    
    config->time_to_start_next_work = config->current_time_to_start_work + config->sheared_config.periodic_time_sec;
    
    return ESP_OK;
}

esp_err_t controll_current_time_on_init(time_t last_time, int offset_sec) {
    time_t now = time(NULL);
    // Dersom last_time ikkje er initialisert eller vi er altfor seint ute
    if (last_time != 0 && (last_time + offset_sec) < now) {
        ESP_LOGE_THREAD(TAG, "last_time: %s | now: %s", ctime(&last_time), ctime(&now));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t update_work_time_on_init(buoye_config_t *config, int offset_sec) {
    if (!config) return ESP_ERR_INVALID_ARG;
    time_t last_time = config->time_to_start_next_work;
    ESP_RETURN_ON_ERROR(controll_current_time_on_init(last_time, offset_sec), 
                            TAG, "Vi er i fortida -> ABORT!");
    config->current_time_to_start_work = controll_current_time_to_start_work(
                                            last_time, 
                                            offset_sec
                                        );
    config->time_to_start_next_work = config->current_time_to_start_work + offset_sec;
    return ESP_OK;
}
