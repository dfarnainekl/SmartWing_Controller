#ifndef SWDRIVER_H_
#define SWDRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"


#define DRV0_OFFSET_I0 38888
#define DRV0_OFFSET_I1 36260
#define DRV0_OFFSET_ENC_PHIM 0
#define DRV0_OFFSET_ENC_PHIE 0
#define DRV0_OFFSET_PHIM_PHIE (DRV0_OFFSET_ENC_PHIE - 7 * DRV0_OFFSET_ENC_PHIM)

#define DRV1_OFFSET_I0 38888
#define DRV1_OFFSET_I1 36260
#define DRV1_OFFSET_ENC_PHIM 0
#define DRV1_OFFSET_ENC_PHIE 0
#define DRV1_OFFSET_PHIM_PHIE (DRV0_OFFSET_ENC_PHIE - 7 * DRV0_OFFSET_ENC_PHIM)

#define DRV2_OFFSET_I0 38888
#define DRV2_OFFSET_I1 36260
#define DRV2_OFFSET_ENC_PHIM 0
#define DRV2_OFFSET_ENC_PHIE 0
#define DRV2_OFFSET_PHIM_PHIE (DRV0_OFFSET_ENC_PHIE - 7 * DRV0_OFFSET_ENC_PHIM)

#define DRV3_OFFSET_I0 38756 //FIXME offsets are changing!?
#define DRV3_OFFSET_I1 36109
#define DRV3_OFFSET_ENC_PHIM -15232 // -encoder zero angle (16bit) -15232
#define DRV3_OFFSET_ENC_PHIE -10752 // -encoder zero angle (16bit) -10752
#define DRV3_OFFSET_PHIM_PHIE 30336 //(DRV0_OFFSET_ENC_PHIE - (7 * DRV0_OFFSET_ENC_PHIM)) //FIXME this calculation does not work


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
	uint16_t ofs_i0;
	uint16_t ofs_i1;
	int16_t ofs_enc_phim;
	int16_t ofs_phim_phie;
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
