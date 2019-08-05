#ifndef SWDRIVER_H_
#define SWDRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"


typedef struct swdriver_s
{
	GPIO_TypeDef* EN_port;
	uint16_t      EN_pin;
	GPIO_TypeDef* STATUS_port;
	uint16_t      STATUS_pin;
	GPIO_TypeDef* FAULT_port;
	uint16_t      FAULT_pin;
	GPIO_TypeDef* CSN_CTR_port;
	uint16_t      CSN_CTR_pin;
	GPIO_TypeDef* CSN_DRV_port;
	uint16_t      CSN_DRV_pin;
	GPIO_TypeDef* CSN_ENC_port;
	uint16_t      CSN_ENC_pin;
	SPI_HandleTypeDef* SPI;
} swdriver_t;


extern swdriver_t swdriver[4];


//#define SWDRIVER_SPI(i) swdriver[i].SPI;


//TODO: make those inline, but correctly
void swdriver_setEnable(uint8_t drv, bool enabled);
void swdriver_setCsnController(uint8_t drv, bool enabled);
void swdriver_setCsnDriver(uint8_t drv, bool enabled);
void swdriver_setCsnEncoder(uint8_t drv, bool enabled);
bool swdriver_getStatus(uint8_t drv);
bool swdriver_getFault(uint8_t drv);

#endif /* SWDRIVER_H_ */
