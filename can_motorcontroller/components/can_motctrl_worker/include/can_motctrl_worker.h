#ifndef MOTCTRL_WORKER_H
#define MOTCTRL_WORKER_H

#include "can_motctrl.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t receive_work_package(motorcontroller_pkg_t *pkg, uint32_t timeout_ms);

esp_err_t send_work_response(const motorcontroller_response_t *resp, uint32_t timeout_ms, uint8_t retries);

void worker_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // MOTCTRL_WORKER_H