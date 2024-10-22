#include "servo.h"

static void write_angle(TIM_HandleTypeDef* timer_handle, double angle) {
	double duty_cycle = (DEGREE_TO_MS(angle))/PERIOD_MS;
	timer_handle->Instance->CCR1 = (uint32_t)(duty_cycle*ARR);
}

void servo_init(TIM_HandleTypeDef* timer_handle) {
	HAL_TIM_PWM_Start(timer_handle, TIM_CHANNEL_1);
	write_angle(timer_handle, 0);
	return;
}

void servo_update(TIM_HandleTypeDef* timer_handle, struct TargetPosition* target_position) {
	// read target angle
	osMutexAcquire(target_position->mtx, 0);
	double elevation_angle = target_position->elevation_angle;
	osMutexRelease(target_position->mtx);

	write_angle(timer_handle, elevation_angle);

	return;
}
