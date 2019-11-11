#ifndef TMC6200_HIGHLEVEL_H_
#define TMC6200_HIGHLEVEL_H_

#include <stdint.h>

#define WRITE 0x80
#define READ 0x00

void tmc6200_highLevel_init(uint8_t drv);
void tmc6200_highLeve_resetErrorFlags(uint8_t drv);

void spiSpeedSlow_set(uint8_t drv);
void spiSpeedSlow_reset(uint8_t drv);

#endif /* TMC6200_HIGHLEVEL_H_ */
