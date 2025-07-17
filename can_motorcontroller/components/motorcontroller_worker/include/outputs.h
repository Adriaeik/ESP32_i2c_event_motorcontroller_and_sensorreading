#ifndef OUTPUTS_H
#define OUTPUTS_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "gpio_config.h"

void outputs_init(void);

void outputs_set_winch_down(bool enable);
void outputs_set_winch_up(bool enable);
void outputs_set_lamp(bool enable);
void outputs_set_auto_lamp(bool enable);
void outputs_set_alarm_lamp(bool enable);

bool outputs_winch_running(void);

#endif // OUTPUTS_H
