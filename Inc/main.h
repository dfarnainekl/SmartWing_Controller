/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_GREEN_Pin LL_GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin LL_GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOD
#define DRV2_CSN_DRV_Pin LL_GPIO_PIN_11
#define DRV2_CSN_DRV_GPIO_Port GPIOD
#define DRV2_CSN_ENC_Pin LL_GPIO_PIN_14
#define DRV2_CSN_ENC_GPIO_Port GPIOD
#define DRV2_CSN_CTR_Pin LL_GPIO_PIN_6
#define DRV2_CSN_CTR_GPIO_Port GPIOC
#define DRV2_STATUS_Pin LL_GPIO_PIN_12
#define DRV2_STATUS_GPIO_Port GPIOC
#define CAN1_STBY_Pin LL_GPIO_PIN_2
#define CAN1_STBY_GPIO_Port GPIOD
#define DRV2_FAULT_Pin LL_GPIO_PIN_4
#define DRV2_FAULT_GPIO_Port GPIOD
#define DRV2_EN_Pin LL_GPIO_PIN_7
#define DRV2_EN_GPIO_Port GPIOD
#define CAN2_STBY_Pin LL_GPIO_PIN_9
#define CAN2_STBY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef enum
{
	NOICE = 0,
	OOF,
	OOF_NO_NEW_DATA,
	OOF_CAN_INIT,
	OOF_CAN_TX_FULL,
	OOF_UNKNOWN_CMD,
	OOF_WRONG_CHANNEL_TYPE,
	OOF_NOT_IMPLEMENTED,
	OOF_SPI_TXP_FULL,
	OOF_SPI_RXP_FULL,
	OOF_SPI_NO_EOT,
	OOF_ADS_NO_RDY,
	OOF_ADS_INIT,
	OOF_TOO_LITTLE,
	OOF_TOO_MUCH,
	OOF_NOT_EQUAL,
	OOF_NO_OP
}Result_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
