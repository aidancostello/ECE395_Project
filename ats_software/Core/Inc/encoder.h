#ifndef __ENCODER_H
#define __ENCODER_H

#include "structs.h"

void encoder_init();

void encoder_update(struct EncoderPosition* encoder_position);

#endif
