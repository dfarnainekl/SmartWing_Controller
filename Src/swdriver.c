#include "swdriver.h"
#include "main.h"


swdriver_t swdriver[4] = {{.EN_port = DRV1_EN_GPIO_Port, .EN_pin = DRV1_EN_Pin, .STATUS_port = DRV1_STATUS_GPIO_Port, .STATUS_pin = DRV1_STATUS_Pin, .FAULT_port = DRV1_FAULT_GPIO_Port, .FAULT_pin = DRV1_FAULT_Pin, .CSN_CTR_port = DRV1_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV1_CSN_CTR_Pin, .CSN_DRV_port = DRV1_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV1_CSN_DRV_Pin, .CSN_ENC_port = DRV1_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV1_CSN_ENC_Pin, .SPI = &hspi1, .ofs_i0 = DRV0_OFFSET_I0, .ofs_i1 = DRV0_OFFSET_I1, .ofs_enc_phim = DRV0_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV0_OFFSET_PHIM_PHIE},
		                  {.EN_port = DRV2_EN_GPIO_Port, .EN_pin = DRV2_EN_Pin, .STATUS_port = DRV2_STATUS_GPIO_Port, .STATUS_pin = DRV2_STATUS_Pin, .FAULT_port = DRV2_FAULT_GPIO_Port, .FAULT_pin = DRV2_FAULT_Pin, .CSN_CTR_port = DRV2_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV2_CSN_CTR_Pin, .CSN_DRV_port = DRV2_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV2_CSN_DRV_Pin, .CSN_ENC_port = DRV2_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV2_CSN_ENC_Pin, .SPI = &hspi2, .ofs_i0 = DRV1_OFFSET_I0, .ofs_i1 = DRV1_OFFSET_I1, .ofs_enc_phim = DRV1_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV1_OFFSET_PHIM_PHIE},
		                  {.EN_port = DRV3_EN_GPIO_Port, .EN_pin = DRV3_EN_Pin, .STATUS_port = DRV3_STATUS_GPIO_Port, .STATUS_pin = DRV3_STATUS_Pin, .FAULT_port = DRV3_FAULT_GPIO_Port, .FAULT_pin = DRV3_FAULT_Pin, .CSN_CTR_port = DRV3_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV3_CSN_CTR_Pin, .CSN_DRV_port = DRV3_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV3_CSN_DRV_Pin, .CSN_ENC_port = DRV3_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV3_CSN_ENC_Pin, .SPI = &hspi3, .ofs_i0 = DRV2_OFFSET_I0, .ofs_i1 = DRV2_OFFSET_I1, .ofs_enc_phim = DRV2_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV2_OFFSET_PHIM_PHIE},
		                  {.EN_port = DRV4_EN_GPIO_Port, .EN_pin = DRV4_EN_Pin, .STATUS_port = DRV4_STATUS_GPIO_Port, .STATUS_pin = DRV4_STATUS_Pin, .FAULT_port = DRV4_FAULT_GPIO_Port, .FAULT_pin = DRV4_FAULT_Pin, .CSN_CTR_port = DRV4_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV4_CSN_CTR_Pin, .CSN_DRV_port = DRV4_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV4_CSN_DRV_Pin, .CSN_ENC_port = DRV4_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV4_CSN_ENC_Pin, .SPI = &hspi4, .ofs_i0 = DRV3_OFFSET_I0, .ofs_i1 = DRV3_OFFSET_I1, .ofs_enc_phim = DRV3_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV3_OFFSET_PHIM_PHIE}};


void swdriver_setEnable(uint8_t drv, bool enabled)
{
	HAL_GPIO_WritePin(swdriver[drv].EN_port, swdriver[drv].EN_pin, enabled);
}

void swdriver_setCsnController(uint8_t drv, bool enabled)
{
	HAL_GPIO_WritePin(swdriver[drv].CSN_CTR_port, swdriver[drv].CSN_CTR_pin, enabled);
}

void swdriver_setCsnDriver(uint8_t drv, bool enabled)
{
	HAL_GPIO_WritePin(swdriver[drv].CSN_DRV_port, swdriver[drv].CSN_DRV_pin, enabled);
}

void swdriver_setCsnEncoder(uint8_t drv, bool enabled)
{
	HAL_GPIO_WritePin(swdriver[drv].CSN_ENC_port, swdriver[drv].CSN_ENC_pin, enabled);
}

bool swdriver_getStatus(uint8_t drv)
{
	return HAL_GPIO_ReadPin(swdriver[drv].STATUS_port, swdriver[drv].STATUS_pin);
}

bool swdriver_getFault(uint8_t drv)
{
	return HAL_GPIO_ReadPin(swdriver[drv].FAULT_port, swdriver[drv].FAULT_pin);
}
