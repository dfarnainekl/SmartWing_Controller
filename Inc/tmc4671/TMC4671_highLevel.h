#ifndef TMC4671_TMC4671_HIGHLEVEL_H_
#define TMC4671_TMC4671_HIGHLEVEL_H_


#include "tmc4671/TMC4671.h"
#include "stm32h7xx_hal.h"
#include "swdriver.h"
#include "usart.h"
#include "as5047U.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>


void TMC4671_highLevel_init(uint8_t drv);
uint16_t TMC4671_getAdcRaw0(uint8_t drv);
uint16_t TMC4671_getAdcRaw1(uint8_t drv);

void TMC4671_highLevel_pwmOff(uint8_t drv);
void TMC4671_highLevel_pwmOn(uint8_t drv);
void TMC4671_highLevel_stoppedMode(uint8_t drv);
void TMC4671_highLevel_torqueMode(uint8_t drv);

void TMC4671_highLevel_setCurrentLimit(uint8_t drv, uint16_t torque_flux_limit);
void TMC4671_highLevel_setTorqueTarget(uint8_t drv, int16_t targetTorque);
void TMC4671_highLevel_setTorqueTargetA(uint8_t drv, float targetTorque);
void TMC4671_highLevel_setFluxTarget(uint8_t drv, int16_t targetFlux);

void TMC4671_highLevel_initEncoder(uint8_t drv);

char* TMC4671_highLevel_getStatus(uint8_t drv);
int16_t TMC4671_highLevel_getTorqueTarget(uint8_t drv);
int16_t TMC4671_highLevel_getTorqueActual(uint8_t drv);

int32_t TMC4671_highLevel_getVelocityActual(uint8_t drv);
int32_t TMC4671_highLevel_getPositionActual(uint8_t drv);
float TMC4671_highLevel_getPositionActualRad(uint8_t drv);


int16_t TMC4671_highLevel_getPhiE(uint8_t drv);
int16_t TMC4671_highLevel_getPhiM(uint8_t drv);

// void TMC4671_highLevel_positionMode_rampToZero(uint8_t drv);

#endif /* TMC4671_TMC4671_HIGHLEVEL_H_ */
