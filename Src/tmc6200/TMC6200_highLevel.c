#include "tmc6200/TMC6200_highLevel.h"
#include "tmc6200/TMC6200.h"
#include "tmc6200/TMC6200_Fields.h"


void tmc6200_highLevel_init(uint8_t drv)
{
	tmc6200_writeInt(drv, TMC6200_GCONF, (3 << TMC6200_AMPLIFICATION_SHIFT)); // single line control: off, current amplification: 20
	tmc6200_writeInt(drv, TMC6200_DRV_CONF, (0 << TMC6200_DRVSTRENGTH_SHIFT)); // no BBM, 150C OT, low drive strength


}

void tmc6200_highLeve_resetErrorFlags(uint8_t drv)
{
	//reset status register if no error has occured
	if( tmc6200_readInt(drv, TMC6200_GSTAT) == 1 )
		tmc6200_writeInt(drv, TMC6200_GSTAT, (1 << TMC6200_RESET_SHIFT));		// clear reset bit
}
