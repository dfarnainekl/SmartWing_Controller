#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"

#define ANGLE_MAX_ALPHA_DEGREE 	20.0
#define ORDER 2

typedef enum
{
	AIL, FLP, JMP_AIL, JMP_FLP, SWING_INNER, SWING_OUTER
} MODE;

typedef struct sweep_s
{
	float Ta;
	uint32_t N;
	float U;
	float omegaStart;
	float omegaEnd;
	char string[200];
	uint16_t len;
	uint32_t k;
	float r;
	float t;
	MODE mode;
	float out;
} sweep_t;

typedef struct data1_s
{
	int32_t torqueActual[2];
	int32_t torqueTarget[2];
	int32_t velocityActual[2];
	int32_t velocityTarget[2];
	int32_t velocityIntegratorValue[2];

	int32_t positionActual[2];
	int32_t positionTarget[2];
	int32_t positionIntegratorValue[2];
} data1_t;

typedef struct data2_s
{
	float torqueTarget[2];
	float velocityActual[2];
	float velocityTarget[2];
} data2_t;

typedef struct control_s
{
	float angleIn;
	float angleOut;

	float torqueTarget;
	float x[ORDER];
	float x_[ORDER];



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
	int32_t torqueActual;		// only for logging
	int32_t torqueTarget;		// written to tmc4671
	int32_t velocityActual;
	int32_t velocityTarget;		// only for logging
	int32_t positionTarget;		// only for logging
	int32_t positionActual;
} motor_t;

void logic_init(void);
void logic_loop(void);

void print_data(int32_t data , uint8_t* string);
void print_data2(float data , uint8_t* string);
float sat(float x);

void positionPI(uint8_t drv);
void velocityPI(uint8_t drv);
void velocityPICompensation(uint8_t drv);

int32_t clacAngle(uint8_t drv);
float calcTorque(uint8_t drv, float angleAlpha, float torqueAlpha);

#endif /* LOGIC_H_ */
