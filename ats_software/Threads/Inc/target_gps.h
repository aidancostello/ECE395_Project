#ifndef __TARGET_GPS_H
#define __TARGET_GPS_H

#include "structs.h"
#include "stm32l4xx_hal.h"
#include "pins.h"

#define SCALAR 10000

void target_gps_init();

void target_gps_update(UART_HandleTypeDef* uart_handle, struct GpsData* gps_data);

#endif
