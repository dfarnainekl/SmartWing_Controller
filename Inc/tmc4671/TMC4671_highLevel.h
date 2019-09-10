#ifndef TMC4671_TMC4671_HIGHLEVEL_H_
#define TMC4671_TMC4671_HIGHLEVEL_H_


#include <stdint.h>


void TMC4671_highLevel_init(uint8_t drv);

void TMC4671_highLevel_pwmOff(uint8_t drv);
void TMC4671_highLevel_positionMode(uint8_t drv);
void TMC4671_highLevel_setPosition(uint8_t drv, int32_t position);
void TMC4671_highLevel_setPosition_nonBlocking(uint8_t drv, int32_t position);

void TMC4671_highLevel_printOffsetAngle(uint8_t drv);
void TMC4671_highLevel_initEncoder(uint8_t drv);

void TMC4671_highLevel_positionTest(uint8_t drv);
void TMC4671_highLevel_torqueTest(uint8_t drv);
void TMC4671_highLevel_openLoopTest(uint8_t drv);


#endif /* TMC4671_TMC4671_HIGHLEVEL_H_ */
