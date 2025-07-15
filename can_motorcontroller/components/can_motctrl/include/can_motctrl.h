#ifndef MOTCTRL_H
#define MOTCTRL_H

#include "can_motctrl_common.h"
#include "can_serde_helper.h"
#include "can_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t send_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                             const can_fragment_list_t *frag_list);

esp_err_t receive_fragment_list(uint32_t start_id, uint32_t data_id, uint32_t end_id, 
                                can_fragment_list_t *frag_list, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_H