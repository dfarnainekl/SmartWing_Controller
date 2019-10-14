#ifndef TMC4671_TMC4671_HIGHLEVEL_H_
#define TMC4671_TMC4671_HIGHLEVEL_H_


#include <stdint.h>
#include <stdbool.h>

void TMC4671_highLevel_init(uint8_t drv);

void TMC4671_highLevel_pwmOff(uint8_t drv);
void TMC4671_highLevel_stoppedMode(uint8_t drv);
void TMC4671_highLevel_positionMode(uint8_t drv);
void TMC4671_highLevel_positionMode_fluxTorqueRamp(uint8_t drv);

void TMC4671_highLevel_setPosition(uint8_t drv, int32_t position);
void TMC4671_highLevel_setPosition_nonBlocking(uint8_t drv, int32_t position);

void TMC4671_highLevel_printOffsetAngle(uint8_t drv);
uint16_t TMC4671_getAdcRaw0(uint8_t drv);
uint16_t TMC4671_getAdcRaw1(uint8_t drv);
void TMC4671_highLevel_initEncoder(uint8_t drv);

void TMC4671_highLevel_initEncoder_new(uint8_t drv);
void TMC4671_highLevel_openLoopTest2(uint8_t drv);
void TMC4671_highLevel_positionMode2(uint8_t drv);
void TMC4671_highLevel_positionMode_rampToZero(uint8_t drv);

//set control parameters
void TMC4671_highLevel_setPositionFilter(uint8_t drv, bool status);
void TMC4671_highLevel_setCurrentLimit(uint8_t drv, uint16_t torque_flux_limit);
void TMC4671_highLevel_setIntegralPosition(uint8_t drv, uint16_t integral);

char* TMC4671_highLevel_getStatus(uint8_t drv);

//Get current values
int16_t TMC4671_highLevel_getTorqueTarget(uint8_t drv);
int16_t TMC4671_highLevel_getTorqueActual(uint8_t drv);
int16_t TMC4671_highLevel_getVelocityTarget(uint8_t drv);
int16_t TMC4671_highLevel_getVelocityActual(uint8_t drv);
int32_t TMC4671_highLevel_getPositionTarget(uint8_t drv);
int32_t TMC4671_highLevel_getPositionActual(uint8_t drv);

//Test functions
void TMC4671_highLevel_positionTest(uint8_t drv);
void TMC4671_highLevel_torqueTest(uint8_t drv);
void TMC4671_highLevel_openLoopTest(uint8_t drv);

#endif /* TMC4671_TMC4671_HIGHLEVEL_H_ */
