#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"


#define ANGLE_MAX_ALPHA_DEGREE 	20.0

void logic_init(void);
void logic_loop(void);

int32_t clacAngle(uint8_t drv, float *angleIn);

#endif /* LOGIC_H_ */
