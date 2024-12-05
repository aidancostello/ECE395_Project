#ifndef __STRUCTS_H
#define __STRUCTS_H

#include "cmsis_os.h"

struct TargetGpsDataRaw {
	int64_t target_lat;
	int64_t target_lon;
	int64_t target_alt;
};

struct GpsData {
	double self_lat;
	double self_lon;
	double self_alt;
	double target_lat;
	double target_lon;
	double target_alt;
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
