#ifndef __CALCULATE_H
#define __CALCULATE_H

#include "structs.h"
#include <math.h>

#define EARTH_RADIUS_M 6378137
#define PI 3.14159265358979323846
#define TO_RADIANS(deg) (deg * PI / 180)
#define TO_DEGREES(rad) (rad * 180 / PI)


void calculate_update(struct GpsData* gps_data, struct TargetPosition* target_position);

#endif
