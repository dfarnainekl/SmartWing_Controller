#include "swdriver.h"
#include "main.h"


swdriver_t swdriver[4] = {{.EN_port = DRV2_EN_GPIO_Port, .EN_pin = DRV2_EN_Pin, .STATUS_port = DRV2_STATUS_GPIO_Port, .STATUS_pin = DRV2_STATUS_Pin, .FAULT_port = DRV2_FAULT_GPIO_Port, .FAULT_pin = DRV2_FAULT_Pin, .CSN_CTR_port = DRV2_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV2_CSN_CTR_Pin, .CSN_DRV_port = DRV2_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV2_CSN_DRV_Pin, .CSN_ENC_port = DRV2_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV2_CSN_ENC_Pin, .SPI = &hspi2, .ofs_i0 = DRV1_OFFSET_I0, .ofs_i1 = DRV1_OFFSET_I1, .ofs_enc_phim = DRV1_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV1_OFFSET_PHIM_PHIE, .ofs_pos0 = DRV1_OFFSET_POS0},
					   	  {.EN_port = DRV2_EN_GPIO_Port, .EN_pin = DRV2_EN_Pin, .STATUS_port = DRV2_STATUS_GPIO_Port, .STATUS_pin = DRV2_STATUS_Pin, .FAULT_port = DRV2_FAULT_GPIO_Port, .FAULT_pin = DRV2_FAULT_Pin, .CSN_CTR_port = DRV2_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV2_CSN_CTR_Pin, .CSN_DRV_port = DRV2_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV2_CSN_DRV_Pin, .CSN_ENC_port = DRV2_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV2_CSN_ENC_Pin, .SPI = &hspi2, .ofs_i0 = DRV1_OFFSET_I0, .ofs_i1 = DRV1_OFFSET_I1, .ofs_enc_phim = DRV1_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV1_OFFSET_PHIM_PHIE, .ofs_pos0 = DRV1_OFFSET_POS0},
						  {.EN_port = DRV2_EN_GPIO_Port, .EN_pin = DRV2_EN_Pin, .STATUS_port = DRV2_STATUS_GPIO_Port, .STATUS_pin = DRV2_STATUS_Pin, .FAULT_port = DRV2_FAULT_GPIO_Port, .FAULT_pin = DRV2_FAULT_Pin, .CSN_CTR_port = DRV2_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV2_CSN_CTR_Pin, .CSN_DRV_port = DRV2_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV2_CSN_DRV_Pin, .CSN_ENC_port = DRV2_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV2_CSN_ENC_Pin, .SPI = &hspi2, .ofs_i0 = DRV1_OFFSET_I0, .ofs_i1 = DRV1_OFFSET_I1, .ofs_enc_phim = DRV1_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV1_OFFSET_PHIM_PHIE, .ofs_pos0 = DRV1_OFFSET_POS0},
						  {.EN_port = DRV2_EN_GPIO_Port, .EN_pin = DRV2_EN_Pin, .STATUS_port = DRV2_STATUS_GPIO_Port, .STATUS_pin = DRV2_STATUS_Pin, .FAULT_port = DRV2_FAULT_GPIO_Port, .FAULT_pin = DRV2_FAULT_Pin, .CSN_CTR_port = DRV2_CSN_CTR_GPIO_Port, .CSN_CTR_pin = DRV2_CSN_CTR_Pin, .CSN_DRV_port = DRV2_CSN_DRV_GPIO_Port, .CSN_DRV_pin = DRV2_CSN_DRV_Pin, .CSN_ENC_port = DRV2_CSN_ENC_GPIO_Port, .CSN_ENC_pin = DRV2_CSN_ENC_Pin, .SPI = &hspi2, .ofs_i0 = DRV1_OFFSET_I0, .ofs_i1 = DRV1_OFFSET_I1, .ofs_enc_phim = DRV1_OFFSET_ENC_PHIM, .ofs_phim_phie = DRV1_OFFSET_PHIM_PHIE, .ofs_pos0 = DRV1_OFFSET_POS0}};


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
