#ifndef __STRUCTS_H
#define __STRUCTS_H

#include "cmsis_os.h"

struct TargetGpsDataRaw {
	int64_t target_lat;		// degrees * SCALAR from target_gps.h
	int64_t target_lon;		// degrees * SCALAR from target_gps.h
	int64_t target_alt;		// meters * SCALAR from target_gps.h
};

struct GpsData {
	double self_lat;		// degrees
	double self_lon;		// degrees
	double self_alt;		// meters
	double target_lat;		// degrees
	double target_lon;		// degrees
	double target_alt;		// meters
	osMutexId_t* mtx;
};

struct TargetPosition {
	double rotation_angle;	// degrees
	double elevation_angle;	// degrees
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
