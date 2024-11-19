#include "calculate.h"

#ifdef LOG_CALCULATE
static uint8_t tick = 0;
#endif

void calculate_update(struct GpsData* gps_data, struct TargetPosition* target_position) {
	// copy out gps data
	osMutexAcquire(*(gps_data->mtx), portMAX_DELAY);
	double self_lat = TO_RADIANS(gps_data->self_lat);
	double self_lon = TO_RADIANS(gps_data->self_lon);
	double self_alt = gps_data->self_alt;
	double target_lat = TO_RADIANS(gps_data->target_lat);
	double target_lon = TO_RADIANS(gps_data->target_lon);
	double target_alt = gps_data->target_alt;
	osMutexRelease(*(gps_data->mtx));

	double delta_lon = target_lon-self_lon;
	double delta_lat = target_lat-self_lat;
	double delta_alt = target_alt-self_alt;

	// calculate rotation angle from north
	double rotation_angle = TO_DEGREES(atan2(sin(delta_lon)*cos(target_lat), cos(self_lat)*sin(target_lat) - sin(self_lat)*cos(target_lat)*cos(delta_lon)));

	// calculate elevation angle from horizontal
	double a = sin(delta_lat/2)*sin(delta_lat/2) + sin(delta_lon/2)*sin(delta_lon/2)*cos(self_lat)*cos(target_lat); 
  	double c = 2*atan2(sqrt(a), sqrt(1-a)); 
	double elevation_angle = TO_DEGREES(atan2(delta_alt, EARTH_RADIUS_M*c));

	// clip elevation angle to [0, 90]
	elevation_angle = elevation_angle > 90 ? 90 : elevation_angle;
	elevation_angle = elevation_angle < 0 ? 0 : elevation_angle;

	// make rotation angle always positive
	rotation_angle = rotation_angle < 0 ? rotation_angle + 360 : rotation_angle;

	// write target position data
	osMutexAcquire(*(target_position->mtx), portMAX_DELAY);
	target_position->rotation_angle = rotation_angle;
	target_position->elevation_angle = elevation_angle;
	osMutexRelease(*(target_position->mtx));

	#ifdef LOG_CALCULATE
	if (tick == 10) {
		log_transmit_buf("Self: ", 6);
		log_transmit_double(TO_DEGREES(self_lat), 3, ' ');
		log_transmit_double(TO_DEGREES(self_lon), 3, ' ');
		log_transmit_double(self_alt, 3, '\n');
		log_transmit_buf("Target: ", 8);
		log_transmit_double(TO_DEGREES(target_lat), 3, ' ');
		log_transmit_double(TO_DEGREES(target_lon), 3, ' ');
		log_transmit_double(target_alt, 3, '\n');
		log_transmit_buf("Computed Rotation: ", 19);
		log_transmit_double(rotation_angle, 3, ' ');
		log_transmit_buf("Computed Elevation: ", 20);
		log_transmit_double(elevation_angle, 3, '\n');
		tick = 0;
	} 
	else {
		tick++;
	}
	#endif

	return;
}
