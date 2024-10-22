#ifndef __SERVO_H
#define __SERVO_H

#include "structs.h"
#include "stm32l4xx_hal.h"

#define ARR 10000
#define PERIOD_MS 20

#define DEGREE_TO_MS(deg) (deg/90 + 1)

void servo_init(TIM_HandleTypeDef* timer_handle);

void servo_update(TIM_HandleTypeDef* timer_handle, struct TargetPosition* target_position);

#endif
