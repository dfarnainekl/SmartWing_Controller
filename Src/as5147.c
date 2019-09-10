#include "as5147.h"
#include "swdriver.h"
#include "spi.h"


static void spiMode_set(uint8_t drv)
{
	swdriver[drv].SPI->Init.CLKPolarity = SPI_POLARITY_LOW;
	swdriver[drv].SPI->Init.CLKPhase = SPI_PHASE_2EDGE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_Delay(2);
}

static void spiMode_reset(uint8_t drv)
{
	swdriver[drv].SPI->Init.CLKPolarity = SPI_POLARITY_HIGH;
	swdriver[drv].SPI->Init.CLKPhase = SPI_PHASE_2EDGE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_Delay(2);
}

uint16_t as5147_getAngle(uint8_t drv) //returns 11bit value
{
	spiMode_set(drv);

	uint8_t txData[2];
	txData[0] = (1 << 7) | (1 << 6) | 0x3F; // parity 1, read, address upper 6 bits
	txData[1] = 0xFF; // address lower 6 bits

	swdriver_setCsnEncoder(drv, false);
	HAL_SPI_Transmit(swdriver[drv].SPI, txData, 2, 10000000); //FIXME timeout
	swdriver_setCsnEncoder(drv, true);
	HAL_Delay(2);

	txData[0] = 0;
	txData[1] = 0;
	uint8_t rxData[2];

	swdriver_setCsnEncoder(drv, false);
	HAL_SPI_TransmitReceive(swdriver[drv].SPI, txData, rxData, 2, 10000000); //FIXME timeout
	swdriver_setCsnEncoder(drv, true);

	spiMode_reset(drv);

	return ((((((uint16_t)rxData[0]) & 0x3F) << 8) | rxData[1]) >> 3);
}
