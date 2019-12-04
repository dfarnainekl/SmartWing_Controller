#ifndef LOGIC_HELPER_H_
#define LOGIC_HELPER_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
//#include "logic.h"

#define TA			0.0005
#define DATA_N		8192
// #define DATA_N		1024
#define MATLAB      1

#define BQ_VEL 1
#define BQ_POS 1


#define J_M 	(28.44e-6) //Nm
#define J_W     (4340.95e-6) //Nm

typedef struct biquad_s
{
    float coeff[6];
    float gain;
    float w_[2];
} biquad_t;

typedef struct pid_controller_s
{
    float x[2];
    float limit;
    float A[2][2];
    float B[2];
    float C[2];
    float D;
    float kb;
} pid_controller_t;

typedef struct pi_controller_s
{
    float x;
    float limit;
    float A;
    float B;
    float C;
    float D;
} pi_controller_t;

typedef struct limiter_s
{
    float out_;
    float R;
} limiter_t;


typedef struct control_s
{
    float currentTargetBeta;
    float currentActualBeta;

    float velocityActualBeta;
    float velocityTargetBeta;
    float velocityErrorBeta;


    float positionActualBeta;
    float positionTargetBeta;
    float positionErrorBeta;

	//biquads
	biquad_t bqChainVel[BQ_VEL];
	biquad_t bqChainPos[BQ_POS];
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



float biquad(biquad_t* bq, float in);
void  biquadReset(biquad_t* bq);

float rateLimiter(limiter_t* limiter, float in);
void rateLimiterInit(limiter_t* limiter, float r, float out_);


float PIDControl(pid_controller_t* pid, float e);
float PIControl(pi_controller_t* pi, float e);
void PIControlReset(pi_controller_t* pi);
void PIControlSetup(pi_controller_t* pi, float a, float b, float c, float d, float limit, float x);


void velocityController(uint8_t drv);
void positionController(uint8_t drv);


float sat(float x);
void print_data_int(int32_t data , uint8_t* string);
void print_data_float(float data , uint8_t* string);


float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha);
float calcAngleBetaAlpha(uint8_t drv, float angleBeta);
float clacAngleVelocityBetaAlpha(uint8_t drv, float angleBeta, float velocityBeta);
float calcAngleTarget(uint8_t drv, float* angleIn);







#endif /* LOGIC_HELPER_H_ */
