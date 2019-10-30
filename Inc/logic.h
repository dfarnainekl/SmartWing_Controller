#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"

#define ANGLE_MAX_ALPHA_DEGREE 	20.0

typedef enum
{
	AIL, FLP, JMP_AIL, JMP_FLP, SWING_INNER, SWING_OUTER
} MODE;

typedef struct sweep_s
{
	float Ta;
	uint32_t N;
	uint16_t U;
	float omegaStart;
	float omegaEnd;
	char string[200];
	uint16_t len;
	uint32_t k;
	float r;
	float t;
	MODE mode;
} sweep_t;

typedef struct data1_s
{
	// int32_t	posTarget[4];
	// int32_t	posActual[4];
	int16_t torqueActual[4];
	int16_t torqueTarget[4];
	int32_t velocityActual[4];
	int32_t velocityTarget[4];
	int32_t velocityIntegratorValue[4];

} data1_t;

typedef struct data2_s
{
	int32_t positionActual;
	int32_t positionTarget;
	int32_t positionIntegratorValue;
} data2_t;

typedef struct control_s
{
	float angleIn;
	float torqueTarget;

	float velocityTarget;
	float velocityActual;
	float velocityI;
	float velocityP;
	float velocityIntegratorValue;
	float velocityIntegratorLimit;
	float velocityError;

	float positionTarget;
	float positionActual;
	float positionI;
	float positionP;
	float positionIntegratorValue;
	float positionIntegratorLimit;
	float positionError;
} control_t;

typedef struct motor_s
{
	int32_t  torqueActual;		// only for logging
	int32_t  torqueTarget;		// written to tmc4671
	int32_t velocityActual;
	int32_t velocityTarget;		// only for logging
	int32_t positionTarget;		// only for logging
	int32_t positionActual;
	float angleBeta;
	float angleAlpha;
	float velocityBeta;
	float velocityAlpha;
	float torqueBeta;
	float torqueAlpha;
} motor_t;

void logic_init(void);
void logic_loop(void);

float sat(float x);
// int32_t clacAngle(uint8_t drv, float *angleIn);
void positionPI(uint8_t drv);
void velocityPI(uint8_t drv);

float clacAngle(uint8_t drv, float angleBeta);
float clacAngleVelocity(uint8_t drv, float angleBeta, float velocityBeta);
float calcTorque(uint8_t drv, float angleAlpha, float torqueAlpha);

#endif /* LOGIC_H_ */
