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
    float a0, a1, a2, b0, b1, b2;
    float w1, w2;
    float gain;
} biquad_t;


typedef struct limiter_s
{
    float out_;
    float R;
} limiter_t;

typedef struct dob_s
{
    //intregrators
    float ePhi_int;
    float alphaFrict_int;
    float omegaEst_int;
    float phiEst_int;

    float alphaEst;
    float ePhi;

    //parameters
    float ke1;
    float ke2;
    float ke3;
    float keI;
}
dob_t;

typedef struct control_s
{
    float phi0;

    float phiIn;
    float phiInLimited;

    //Trajectory generator
    biquad_t bqTrajPhi;
    biquad_t bqTrajOmega;
    biquad_t bqTrajAlpha;
    biquad_t bqTrajPhi1;
    biquad_t bqTrajPhi2;
    biquad_t bqTrajOmega1;
    biquad_t bqPhi;

    limiter_t limTrajPhi;

    float phiDes;
    float omegaDes;
    float alphaDes;

    //Disturbance Observer
    float phi;
    float alphaM;
    dob_t dob;

    float phiEst;
    float omegaEst;
    float alphaEst;
    float alphaFrict;
    float ePhi;

    //feedback
    float kFB0;
    float kFB1;

    float CmEst;


    float iq;
} control_t;

typedef struct motor_s
{
	int32_t torqueActual;		// only for logging
	int32_t torqueTarget;		// written to tmc4671
	int32_t velocityActual;
	int32_t positionActual;
} motor_t;



float biquad(biquad_t* bq, float in);
void  biquadReset(biquad_t* bq, float init);
void  biquadInit(biquad_t* bq, float gain, float b0, float b1, float b2, float a0, float a1, float a2);


float rateLimiter(limiter_t* limiter, float in);
void  rateLimiterInit(limiter_t* limiter, float r, float out_);

void disturbanceObserverInit(control_t* ctrl, float ke1, float ke2, float ke3, float keI, float kFB0, float kFB1, float CmEst);
void disturbanceObserver(control_t* ctrl);
void disturbanceObserverResetCm(control_t* ctrl, float CmEst);
void disturbanceObserverResetPhi0(control_t* ctrl, float phi0);

float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha);
float calcAngleBetaAlpha(uint8_t drv, float angleBeta);
float clacAngleVelocityBetaAlpha(uint8_t drv, float angleBeta, float velocityBeta);
float calcAngleTarget(uint8_t drv, float* angleIn);

float sat(float x);
void print_data_int(int32_t data , uint8_t* string);
void print_data_float(float data , uint8_t* string);





// typedef struct pid_controller_s
// {
//     float x[2];
//     float limit;
//     float A[2][2];
//     float B[2];
//     float C[2];
//     float D;
//     float kb;
// } pid_controller_t;
//
// typedef struct pi_controller_s
// {
//     float x;
//     float limit;
//     float A;
//     float B;
//     float C;
//     float D;
// } pi_controller_t;
//
// float PIDControl(pid_controller_t* pid, float e);
// float PIControl(pi_controller_t* pi, float e);
// void PIControlReset(pi_controller_t* pi);
// void PIControlSetup(pi_controller_t* pi, float a, float b, float c, float d, float limit, float x);

#endif /* LOGIC_HELPER_H_ */
