#include "tmc6200/TMC6200.h"
#include "swdriver.h"
#include "spi.h"


// spi access
int tmc6200_readInt(uint8_t motor, uint8_t address)
{
	uint8_t txData[5];
	txData[0] = TMC_ADDRESS(address);
	txData[1] = 0;
	txData[2] = 0;
	txData[3] = 0;
	txData[4] = 0;

	uint8_t rxData[5];

	swdriver_setCsnDriver(motor, false);
	HAL_SPI_TransmitReceive(swdriver[motor].SPI, txData, rxData, 5, 10000000); //FIXME timeout
	swdriver_setCsnDriver(motor, true);

	return (int)((rxData[3] << 24) | (rxData[2] << 16) | (rxData[1] << 8) | (rxData[0] << 0));
}


void tmc6200_writeInt(uint8_t motor, uint8_t address, int value)
{
	uint8_t data[5];
	data[0] = address | TMC6200_WRITE_BIT;
	data[1] = 0xFF & (value>>24);
	data[2] = 0xFF & (value>>16);
	data[3] = 0xFF & (value>>8);
	data[4] = 0xFF & (value>>0);

	swdriver_setCsnDriver(motor, false);
	HAL_SPI_Transmit(swdriver[motor].SPI, data, 5, 10000000); //FIXME timeout
	swdriver_setCsnDriver(motor, true);
}
