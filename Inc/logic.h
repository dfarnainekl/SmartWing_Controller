#ifndef LOGIC_H_
#define LOGIC_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "swdriver.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "as5047U.h"
#include "usart.h"
#include "tim.h"
#include "logic_helper.h"



#define ANGLE_MAX_ALPHA_DEGREE 	20.0


#define MODE_STOP				0
#define MODE_TORQUE				1
#define MODE_TRAJTEST			2
#define MODE_POSITION			3
#define MODE_POSITION_INIT_M3	4
#define MODE_POSITION_INIT_M2	5
#define MODE_IDLE				6
#define MODE_RCCONTROL			7


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
	float phiIn[2];
	float phi[2];
	float alphaM[2];
	float phiEst[2];
	float omegaEst[2];
	float alphaEst[2];
	float alphaFrict[2];
	float iq[2];
} data1_t;

typedef struct data2_s
{
	float phiInLimited[2];
	float phiDes[2];
	float omegaDes[2];
	float alphaDes[2];
} data2_t;



void logic_init(void);
void logic_loop(void);



#endif /* LOGIC_H_ */
