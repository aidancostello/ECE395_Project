#ifndef __SELF_GPS_H
#define __SELF_GPS_H

#include "structs.h"
#include "MAX_M8C.h"
#include "pins.h"

void self_gps_init(I2C_HandleTypeDef* i2c_handle);

void self_gps_update(I2C_HandleTypeDef* i2c_handle, struct GpsData* gps_data);

#endif
