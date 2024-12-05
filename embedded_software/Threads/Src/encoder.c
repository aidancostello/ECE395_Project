#include "encoder.h"

void encoder_init() {
	// TODO: init encoder
	return;
}

void encoder_update(struct EncoderPosition* encoder_position) {
	// TODO: read encoder values
	double rotation_angle = 0;

	// update rotation angle
	osMutexAcquire(*(encoder_position->mtx), portMAX_DELAY);
	encoder_position->rotation_angle = rotation_angle;
	osMutexRelease(*(encoder_position->mtx));

	return;
}
