#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"

#define ANGLE_MAX_ALPHA_DEGREE 	20.0

typedef enum {AIL, FLP, JMP_AIL, JMP_FLP} MODE;

typedef struct sweep_s
{
	float 		Ta;
	uint32_t 	N;
	uint8_t		U;
	float			omegaStart;
	float			omegaEnd;
	char 			string[200];
	uint16_t 	len;
	uint32_t 	k;
  MODE      mode;
} sweep_t;


typedef struct data_s
{
	int32_t	posTarget[4];
	int32_t	posActual[4];
	int16_t torqueActual[4];
	int16_t velocityActual[4];
} data_t;



void logic_init(void);
void logic_loop(void);

float sat(float x);
int32_t clacAngle(uint8_t drv, float *angleIn);

#endif /* LOGIC_H_ */
