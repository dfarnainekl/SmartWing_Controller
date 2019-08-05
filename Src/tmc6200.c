#include "tmc6200.h"
#include "swdriver.h"
#include "spi.h"

void tmc6200_init(uint8_t drv)
{
	uint8_t data[5];
	data[0] = WRITE | 0x00;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0b00110000; // single line control: off, current amplification: 20

	swdriver_setCsnDriver(drv, false);
	HAL_SPI_Transmit(swdriver[drv].SPI, data, 5, 10000000);
	swdriver_setCsnDriver(drv, true);
}

