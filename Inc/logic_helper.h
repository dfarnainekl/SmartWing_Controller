#ifndef LOGIC_HELPER_H_
#define LOGIC_HELPER_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include "logic.h"

#define TA			0.0005
#define DATA_N		8192
#define MATLAB      1


#define J_M 	(28.44e-6) //Nm
#define J_W     (4340.95e-6) //Nm


void  initPIControl();
float PIControl(uint8_t drv, float error, float Ki, float Kp, float integratorValue, float IntegratorLimit);

float sat(float x);
void print_data(int32_t data , uint8_t* string);
void print_data2(float data , uint8_t* string);


float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha);
float calcAngleBetaAlpha(uint8_t drv, float angleBeta);
float clacAngleVelocityBetaAlpha(uint8_t drv, float angleBeta, float velocityBeta);
int32_t calcAngleTarget(uint8_t drv);


#endif /* LOGIC_HELPER_H_ */
