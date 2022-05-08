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
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOD
#define DRV2_CSN_DRV_Pin GPIO_PIN_11
#define DRV2_CSN_DRV_GPIO_Port GPIOD
#define DRV2_CSN_ENC_Pin GPIO_PIN_14
#define DRV2_CSN_ENC_GPIO_Port GPIOD
#define DRV2_CSN_CTR_Pin GPIO_PIN_6
#define DRV2_CSN_CTR_GPIO_Port GPIOC
#define DRV2_STATUS_Pin GPIO_PIN_12
#define DRV2_STATUS_GPIO_Port GPIOC
#define CAN1_STBY_Pin GPIO_PIN_2
#define CAN1_STBY_GPIO_Port GPIOD
#define DRV2_FAULT_Pin GPIO_PIN_4
#define DRV2_FAULT_GPIO_Port GPIOD
#define DRV2_EN_Pin GPIO_PIN_7
#define DRV2_EN_GPIO_Port GPIOD
#define CAN2_STBY_Pin GPIO_PIN_9
#define CAN2_STBY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
