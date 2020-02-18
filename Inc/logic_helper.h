#ifndef LOGIC_HELPER_H_
#define LOGIC_HELPER_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
#include "usart.h"
//#include <stdint.h>
//#include "logic.h"

#define TA			0.0005
#define DATA_N		8192
#define MATLAB      0


#define I_LIMIT     30.0
#define F_FB        1.0
#define F_OBS       100

// #define CM_M2       773.0006664650
// #define CM_M3       493.3781002056

#define CM_M2       773.0
#define CM_M3       493.0

// #define CM_M2       500.0
// #define CM_M3       300.0


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

    biquad_t bqQ1;
    biquad_t bqQ2;
    biquad_t bqQ3;
    biquad_t bqQ4;
    biquad_t bqFB;

    limiter_t limTrajPhi;

    float phiDes;
    float omegaDes;
    float alphaDes;

    //Disturbance Observer
    float phi;
    float alphaM;
    dob_t dob;

    float alphaM_1;
    float alphaM_2;
    float alphaFB;
    float factorFF;

    float phiEst;
    float omegaEst;
    float alphaEst;
    float alphaFrict;
    float ePhi;

    float fOBS;
    float fFB;

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

// void disturbanceObserverInit(control_t* ctrl, float omegaOBS, float omegaFB, float CmEst);
void disturbanceObserverInit(control_t* ctrl, float fOBS, float fFB, float CmEst);
void disturbanceObserver(control_t* ctrl);
void disturbanceObserverResetCm(control_t* ctrl, float CmEst);
void disturbanceObserverResetPhi0(control_t* ctrl, float phi0);

float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha);
float calcAngleBetaAlpha(uint8_t drv, float angleBeta);
float calcAngleTarget(uint8_t drv, float* angleIn);

float sat(float x);
void print_data_int(int32_t data , uint8_t* string);
void print_data_float(float data , uint8_t* string);



#endif /* LOGIC_HELPER_H_ */
