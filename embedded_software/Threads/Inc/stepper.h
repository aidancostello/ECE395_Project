#ifndef __STEPPER_H
#define __STEPPER_H

#include "structs.h"
#include "stm32l4xx_hal.h"
#include "pins.h"
#include "logging.h"
#include "log_config.h"

#define DEGREES_PER_STEP 1.8
#define MICROSTEP_UPPER_BOUND 6

void stepper_init();

void stepper_update(struct EncoderPosition* encoder_position, struct TargetPosition* target_position);

#endif
