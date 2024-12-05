#include "stepper.h"

static double current_angle;

void stepper_init() {
	// set reset, en, pwm high
	HAL_GPIO_WritePin(STEPPER_RESET_BANK, STEPPER_RESET_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEPPER_EN_BANK, STEPPER_EN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEPPER_PWM_BANK, STEPPER_PWM_PIN, GPIO_PIN_SET);
	// set decay, stck, dir, modes low
	HAL_GPIO_WritePin(STEPPER_STCK_BANK, STEPPER_STCK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_DIR_BANK, STEPPER_DIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_MODE1_BANK, STEPPER_MODE1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_MODE2_BANK, STEPPER_MODE2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_MODE3_BANK, STEPPER_MODE3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_DECAY_BANK, STEPPER_DECAY_PIN, GPIO_PIN_RESET);
	current_angle = 0;
	return;
}

void stepper_update(struct EncoderPosition* encoder_position, struct TargetPosition* target_position) {
	// TODO: implement read encoder angle
	// osMutexAcquire(encoder_position->mtx, 0);
	// double encoder_rotation_angle = encoder_position->rotation_angle;
	// osMutexRelease(encoder_position->mtx);

	// read target angle
	osMutexAcquire(*(target_position->mtx), portMAX_DELAY);
	double rotation_angle = target_position->rotation_angle;
	osMutexRelease(*(target_position->mtx));

	// determine step direction
	GPIO_PinState dir = GPIO_PIN_SET;
	double angle_diff = rotation_angle-current_angle;
	if (angle_diff < 0) {
		dir = GPIO_PIN_RESET;
		// make angle diff positive
		angle_diff *= -1;
	}

	// check opposite direction
	if (angle_diff > 180) {
        angle_diff = 360-angle_diff;
        dir = dir ^ (0x01);
	}

	// determine largest step possible
	uint16_t divisor = 1;
	uint8_t mode_bits = 0;
	for (; mode_bits < MICROSTEP_UPPER_BOUND; mode_bits++) {
		if ((double)DEGREES_PER_STEP/divisor <= angle_diff) {
			break;
		}
		divisor *= 2;
	}

	// close enough that we don't need to step
	if (mode_bits == MICROSTEP_UPPER_BOUND) {
		return;
	}

	#ifdef LOG_STEPPER
	log_print("Current: ");
	log_print_double(current_angle, 3);
	log_print(" | Target: ");
	log_print_double(rotation_angle, 3);
	log_print(" | Step Amount: ");
	log_print_double((double)DEGREES_PER_STEP/divisor, 3);
	log_print(" | Stepping -> dir: ");
	log_print_double((double)dir, 0);
	log_print(" mode1: ");
	log_print_double((double)(mode_bits&0x1), 0);
	log_print(" mode2: ");
	log_print_double((double)((mode_bits&0x2)>>1), 0);
	log_print(" mode3: ");
	log_print_double((double)((mode_bits&0x4)>>2), 0);
	log_print("\n");
	#endif

	// set direction
	HAL_GPIO_WritePin(STEPPER_DIR_BANK, STEPPER_DIR_PIN, dir);
	// set mode pins
	HAL_GPIO_WritePin(STEPPER_MODE1_BANK, STEPPER_MODE1_PIN, (mode_bits&0x1));
	HAL_GPIO_WritePin(STEPPER_MODE2_BANK, STEPPER_MODE2_PIN, (mode_bits&0x2)>>1);
	HAL_GPIO_WritePin(STEPPER_MODE3_BANK, STEPPER_MODE3_PIN, (mode_bits&0x4)>>2);
	//execute step
	HAL_GPIO_WritePin(STEPPER_STCK_BANK, STEPPER_STCK_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(STEPPER_STCK_BANK, STEPPER_STCK_PIN, GPIO_PIN_RESET);

	// update current angle
	current_angle = dir ? current_angle + (double)DEGREES_PER_STEP/divisor : current_angle - (double)DEGREES_PER_STEP/divisor;

	// constrain current angle to range [0, 359)
	current_angle = current_angle >= 360 ? current_angle-360 : current_angle;
	current_angle = current_angle < 0 ? current_angle+360 : current_angle;

	return;
}
