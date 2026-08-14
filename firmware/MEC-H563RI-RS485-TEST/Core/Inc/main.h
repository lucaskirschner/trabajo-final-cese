/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h5xx_hal.h"

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
#define BUTTON_Pin GPIO_PIN_14
#define BUTTON_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_9
#define RS485_RE_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_10
#define RS485_DE_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_12
#define RS485_TX_GPIO_Port GPIOA
#define GR_LED_Pin GPIO_PIN_4
#define GR_LED_GPIO_Port GPIOB
#define YE_LED_Pin GPIO_PIN_5
#define YE_LED_GPIO_Port GPIOB
#define RE_LED_Pin GPIO_PIN_8
#define RE_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
