#ifndef __STRUCTS_H
#define __STRUCTS_H

#include "cmsis_os.h"

struct GpsData {
	int32_t self_lat;
	int32_t self_lon;
	int32_t self_alt;
	int32_t target_lat;
	int32_t target_lon;
	int32_t target_alt;
	osMutexId_t* mtx;
};

struct TargetPosition {
	double rotation_angle;
	double elevation_angle;
	osMutexId_t* mtx;
};

struct EncoderPosition {
	double rotation_angle;
	osMutexId_t* mtx;
};

struct DataPointers {
	struct GpsData* gps_data;
	struct TargetPosition* target_position;
	struct EncoderPosition* encoder_position;
};


#endif
