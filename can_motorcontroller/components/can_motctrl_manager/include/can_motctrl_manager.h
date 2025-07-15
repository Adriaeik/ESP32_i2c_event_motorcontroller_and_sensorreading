
// components/motctrl_manager/include/motctrl_manager.h
#ifndef MOTCTRL_MANAGER_H
#define MOTCTRL_MANAGER_H

#include "can_motctrl.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t start_worker(const motorcontroller_pkg_t *pkg, uint32_t timeout_ms, uint8_t retries);

esp_err_t wait_for_worker(motorcontroller_response_t *resp, uint32_t wait_offset_ms, uint32_t timeout_ms);

void manager_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_MANAGER_H