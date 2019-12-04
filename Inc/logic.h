#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "swdriver.h"
#include "tmc6200/TMC6200.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "tmc4671/TMC4671.h"
#include "as5047U.h"
#include "usart.h"
#include "tim.h"
#include "logic_helper.h"



#define ANGLE_MAX_ALPHA_DEGREE 	20.0


#define MODE_STOP				0
#define MODE_TORQUE				1
#define MODE_VELOCITY			2
#define MODE_POSITION			3
#define MODE_RCCONTROL			4
#define MODE_TORQUE_SWEEP		5
#define MODE_IDLE				6
#define MODE_CONTROL_TEST		7
#define MODE_VELOCITY_STEP		8
#define MODE_TORQUE_X			9


typedef enum
{
	LOW, MID, HIGH, SINE, STEP, TEST, M2, M3, M2M3
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
	float currentActual[2];
	float currentTarget[2];

	float velocityActual[2];
	float velocityTarget[2];

	float positionActual[2];
	float positionTarget[2];
} data1_t;

typedef struct data2_s
{
	float torqueTarget;
	float velocityActual;
	float velocityTarget;
	float positionActual;
	float positionTarget;
} data2_t;



void logic_init(void);
void logic_loop(void);



#endif /* LOGIC_H_ */
