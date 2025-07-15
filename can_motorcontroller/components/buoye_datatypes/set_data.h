#pragma once

#include "buoye_structs.h"
#include "esp_err.h"

esp_err_t get_sheared_config_default(sheared_config_t *c);

esp_err_t set_temp_hub_config(const sheared_config_t *config);
esp_err_t get_temp_hub_config(sheared_config_t *config);
esp_err_t update_work_time(state_t STATE, buoye_config_t *config,  int offset_sec);
esp_err_t update_work_time_on_init(buoye_config_t *config, int offset_sec);