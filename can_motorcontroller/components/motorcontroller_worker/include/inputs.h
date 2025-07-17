#ifndef INPUTS_H
#define INPUTS_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "gpio_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void inputs_init(void);

// Basic input state getters - only 3 inputs now
bool inputs_get_winch_home(void);
bool inputs_get_winch_tension(void);
bool inputs_get_winch_auto(void);

// Edge detection functions for critical sensors
bool inputs_home_edge_detected(void);
bool inputs_tension_edge_detected(void);

// Controller integration functions
void inputs_set_controller_queue(QueueHandle_t queue);
const char* inputs_get_name(uint8_t index);

#endif // INPUTS_H