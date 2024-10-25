#include "target_gps.h"

void target_gps_init() {
	// set RTS and CTS pins low as they are unused
	HAL_GPIO_WritePin(UART_RTS_BANK, UART_RTS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_CTS_BANK, UART_CTS_PIN, GPIO_PIN_RESET);
	return;
}

void target_gps_update(UART_HandleTypeDef* uart_handle, struct GpsData* gps_data) {
	// read data
	// TODO: implement interrupt receive
	uint8_t buf[12];
	if (HAL_UART_Receive(uart_handle, buf, 12, 100) != HAL_OK) {
		return;
	}

	// little endian
	int32_t target_lat = (buf[3]<<24) | (buf[2]<<16) | (buf[1]<<8) | buf[0];
	int32_t target_lon = (buf[7]<<24) | (buf[6]<<16) | (buf[5]<<8) | buf[4];
	int32_t target_alt = (buf[11]<<24) | (buf[10]<<16) | (buf[9]<<8) | buf[8];

	// update self gps data
	osMutexAcquire(gps_data->mtx, 0);
	gps_data->target_lat = target_lat/(double)SCALAR;
	gps_data->target_lon = target_lon/(double)SCALAR;
	gps_data->target_alt = target_alt/(double)SCALAR;
	osMutexRelease(gps_data->mtx);

	return;
}
