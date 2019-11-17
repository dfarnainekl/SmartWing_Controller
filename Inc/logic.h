#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"

#define ANGLE_MAX_ALPHA_DEGREE 	20.0
#define ORDER_VEL_FILT 8
#define ORDER_POS_FILT 2

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
	uint32_t len;
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
	float torqueTarget;
	float velocityActual;
	float velocityTarget;
	float positionActual;
	float positionTarget;
} data2_t;

typedef struct control_s
{
	float angleIn;
	float angleOut;

	float torqueTarget;
	float torqueTarget_Limited;

	float velocityTarget;
	float velocityActual;
	float velocityError;
	float velocityIntegratorValue;
	float bq_intermediate1;
	float bq_intermediate2;
	float bq_intermediate3;
	float bq_vel_delay1[2];
	float bq_vel_delay2[2];
	float bq_vel_delay3[2];
	float bq_vel_delay4[2];

	float positionTarget;
	float positionActual;
	float positionError;
	float bq_pos_delay1[2];

	//old:
	float velocityI;
	float velocityP;
	float velocityIntegratorLimit;
	float positionI;
	float positionP;
	float positionIntegratorLimit;
	float positionIntegratorValue;
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
// void velocityPICompensation(uint8_t drv);
// void positionPICompensation(uint8_t drv);
void positionPICompensation(uint8_t drv);
void velocityPICompensation(uint8_t drv);

float biquad(float in, float* coeffs, float gain, float* w_);

int32_t calcAngleTarget(uint8_t drv);
float calcAngleBetaAlpha(uint8_t drv, float angleBeta);
float clacAngleVelocityBetaAlpha(uint8_t drv, float angleBeta, float velocityBeta);
float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha);

#endif /* LOGIC_H_ */
