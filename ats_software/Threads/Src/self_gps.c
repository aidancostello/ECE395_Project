#include "self_gps.h"

void self_gps_init(I2C_HandleTypeDef* i2c_handle) {
	// TODO: uncomment once gps driver is tested
	// max_m8c_init(i2c_handle, GPS_RESET_BANK, GPS_RESET_PIN);
	return;
}

void self_gps_update(I2C_HandleTypeDef* i2c_handle, struct GpsData* gps_data) {
	// TODO: uncomment once gps driver is tested
	// // setup varaibles
	// struct max_m8c_gps_data raw_data = {0};
	// uint16_t bytes_avail = 0;

	// // request packet
	// if (max_m8c_request_nav_pvt(i2c_handle) != MAX_M8C_SUCCESS) {
	// 	return;
	// }

	// // wait for response
	// while (bytes_avail != UBX_HEADER_SIZE+UBX_CHECKSUM_SIZE+UBX_NAV_PVT_PAYLOAD_SIZE) {
	// 	max_m8c_available(i2c_handle, &bytes_avail);
	// }

	// // read response
	// if (max_m8c_read_nav_pvt(i2c_handle, &raw_data) != MAX_M8C_SUCCESS) {
	// 	return;
	// }

	// // update self gps data
	// osMutexAcquire(gps_data->mtx, 0);
	// gps_data->self_lat = raw_data.lat / (double)LAT_LON_SCALAR;
	// gps_data->self_lon = raw_data.lon / (double)LAT_LON_SCALAR;
	// gps_data->self_alt = raw_data.lon / (double)ALT_SCALAR;
	// osMutexRelease(gps_data->mtx);

	osMutexAcquire(*(gps_data->mtx), portMAX_DELAY);
	gps_data->self_lat = 40.115;
	gps_data->self_lon = -88.227778;
	gps_data->self_alt = 228.6;
	osMutexRelease(*(gps_data->mtx));

	return;
}
