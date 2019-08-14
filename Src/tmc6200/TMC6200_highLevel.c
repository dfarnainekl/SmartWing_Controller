#include "tmc6200/TMC6200_highLevel.h"
#include "tmc6200/TMC6200.h"
#include "tmc6200/TMC6200_Fields.h"


void tmc6200_highLevel_init(uint8_t drv)
{
	tmc6200_writeInt(drv, TMC6200_GCONF, (3 << TMC6200_AMPLIFICATION_SHIFT)); // single line control: off, current amplification: 20
}

