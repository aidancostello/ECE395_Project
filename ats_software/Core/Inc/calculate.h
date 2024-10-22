#ifndef __CALCULATE_H
#define __CALCULATE_H

#include "structs.h"
#include <math.h>

#define TO_RADIANS(deg) (deg * M_PI / 180)
#define TO_DEGREES(rad) (rad * 180 / M_PI)
#define EARTH_RADIUS_M 6378137

void calculate_update(struct GpsData* gps_data, struct TargetPosition* target_position);

#endif
