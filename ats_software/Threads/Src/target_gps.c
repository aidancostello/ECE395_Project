#include "target_gps.h"

void target_gps_init() {
	// set RTS and CTS pins low as they are unused
	HAL_GPIO_WritePin(UART_RTS_BANK, UART_RTS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_CTS_BANK, UART_CTS_PIN, GPIO_PIN_RESET);
	return;
}

void target_gps_update(struct GpsData* gps_data) {
	// TODO: read from target gps over uart
	double target_lat = 0;
	double target_lon = 0;
	double target_alt = 0;

	// update self gps data
	osMutexAcquire(gps_data->mtx, 0);
	gps_data->target_lat = target_lat;
	gps_data->target_lon = target_lon;
	gps_data->target_alt = target_alt;
	osMutexRelease(gps_data->mtx);

	return;
}
