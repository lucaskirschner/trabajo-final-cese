/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.h
  * Description        : FreeRTOS applicative header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __APP_FREERTOS_H
#define __APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Exported macro -------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */
extern osThreadId_t outputTaskHandle;
extern osThreadId_t userTaskHandle;
extern osThreadId_t inputTaskHandle;
extern osThreadId_t diagnosticsTaskHandle;
extern osThreadId_t radioTaskHandle;
extern osMutexId_t ioPortSpiMutexHandle;
extern osMutexId_t dinDataMutexHandle;
extern osMessageQueueId_t outputQueueHandle;
extern osMessageQueueId_t inputQueueHandle;
extern osMessageQueueId_t radioInputQueueHandle;
extern osMessageQueueId_t radioOutputQueueHandle;
extern osEventFlagsId_t doutFaultEventHandle;
extern osEventFlagsId_t dinEventHandle;
extern osEventFlagsId_t dinFaultEventHandle;
extern osEventFlagsId_t radioEventHandle;
extern osEventFlagsId_t doutEventHandle;
extern osEventFlagsId_t radioFaultEventHandle;

/* Exported function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Output_Task(void *argument);
void User_Task(void *argument);
void Input_Task(void *argument);
void Diagnostics_Task(void *argument);
void Radio_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#ifdef __cplusplus
}
#endif
#endif /* __APP_FREERTOS_H */
