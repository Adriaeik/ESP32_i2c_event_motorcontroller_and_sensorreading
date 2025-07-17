#ifndef TIMER_H
#define TIMER_H

#include "freertos/FreeRTOS.h"
#include <stdbool.h>

// Start ein ny w_timer_t og returner handtak
typedef struct {
    TickType_t start_tick;
} w_timer_t;

void timer_start(w_timer_t * t); // (re)starts the w_timer_t by updating the start_tick.
bool timer_elapsed(const w_timer_t * t, uint32_t duration_ms); // checks if the duration_ms is pased 
uint32_t timer_ms_since_start(const w_timer_t * t);

#endif // TIMER_H
