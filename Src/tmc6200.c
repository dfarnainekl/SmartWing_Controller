#include "tmc6200.h"
#include "stm32h7xx_hal.h"
#include "spi.h"

void tmc6200_init(uint8_t drv)
{
	uint8_t data[5];
	data[0] = WRITE | 0x00;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0b00110000; // single line control: off, current amplification: 20

	switch(drv)
	{
		case 0:
			HAL_GPIO_WritePin(DRV1_CSN_DRV_GPIO_Port, DRV1_CSN_DRV_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, data, 5, 10000000);
			HAL_GPIO_WritePin(DRV1_CSN_DRV_GPIO_Port, DRV1_CSN_DRV_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(DRV2_CSN_DRV_GPIO_Port, DRV2_CSN_DRV_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, data, 5, 10000000);
			HAL_GPIO_WritePin(DRV2_CSN_DRV_GPIO_Port, DRV2_CSN_DRV_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(DRV3_CSN_DRV_GPIO_Port, DRV3_CSN_DRV_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi3, data, 5, 10000000);
			HAL_GPIO_WritePin(DRV3_CSN_DRV_GPIO_Port, DRV3_CSN_DRV_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(DRV4_CSN_DRV_GPIO_Port, DRV4_CSN_DRV_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi4, data, 5, 10000000);
			HAL_GPIO_WritePin(DRV4_CSN_DRV_GPIO_Port, DRV4_CSN_DRV_Pin, GPIO_PIN_SET);
			break;
	}
}

