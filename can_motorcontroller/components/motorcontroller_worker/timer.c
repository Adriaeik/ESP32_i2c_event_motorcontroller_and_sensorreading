#include "timer.h"
#include "freertos/task.h"

void timer_start(w_timer_t * t) {
    t->start_tick = xTaskGetTickCount();
}

bool timer_elapsed(const w_timer_t * t, uint32_t duration_ms) {
    TickType_t now = xTaskGetTickCount();
    return (now - t->start_tick) >= pdMS_TO_TICKS(duration_ms);
}

uint32_t timer_ms_since_start(const w_timer_t * t) {
    TickType_t now = xTaskGetTickCount();
    return (now - t->start_tick) * portTICK_PERIOD_MS;
}
uint32_t timer_get_start_time_ms(const w_timer_t *t) {
    // Gjev start­tida i ms rekna frå FreeRTOS-oppstart
    return t->start_tick * portTICK_PERIOD_MS;
}