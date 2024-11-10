#include "target_gps.h"

void target_gps_init() {
	// set RTS and CTS pins low as they are unused
	HAL_GPIO_WritePin(UART_RTS_BANK, UART_RTS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UART_CTS_BANK, UART_CTS_PIN, GPIO_PIN_RESET);
	return;
}

void target_gps_update(osMessageQueueId_t* uart_queue, struct GpsData* gps_data) {
	// early return if queue is empty
	if (osMessageQueueGetCount(*uart_queue) == 0) {
		return;
	}

	// get data from queue
	struct TargetGpsDataRaw temp;
	if (osMessageQueueGet(*uart_queue, &temp, NULL, 0) != osOK) {
		return;
	}

	// update self gps data
	osMutexAcquire(gps_data->mtx, 0);
	gps_data->target_lat = temp.target_lat / SCALAR;
	gps_data->target_lon = temp.target_lon / SCALAR;
	gps_data->target_alt = temp.target_alt / SCALAR;
	osMutexRelease(gps_data->mtx);

	return;
}
