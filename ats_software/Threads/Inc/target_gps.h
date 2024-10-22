#ifndef __TARGET_GPS_H
#define __TARGET_GPS_H

#include "structs.h"
#include "stm32l4xx_hal.h"
#include "pins.h"

void target_gps_init();

void target_gps_update(struct GpsData* gps_data);

#endif
