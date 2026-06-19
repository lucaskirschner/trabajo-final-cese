/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
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

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_radio.h"
#include "app_user.h"
#include "app_din.h"
#include "app_dout.h"
#include "app_diagnostics.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for radioTask */
osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for userTask */
osThreadId_t userTaskHandle;
const osThreadAttr_t userTask_attributes = {
  .name = "userTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for dinTask */
osThreadId_t dinTaskHandle;
const osThreadAttr_t dinTask_attributes = {
  .name = "dinTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for doutTask */
osThreadId_t doutTaskHandle;
const osThreadAttr_t doutTask_attributes = {
  .name = "doutTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for diagnosticsTask */
osThreadId_t diagnosticsTaskHandle;
const osThreadAttr_t diagnosticsTask_attributes = {
  .name = "diagnosticsTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ioPortSpiMutex */
osMutexId_t ioPortSpiMutexHandle;
const osMutexAttr_t ioPortSpiMutex_attributes = {
  .name = "ioPortSpiMutex"
};
/* Definitions for dinDataMutex */
osMutexId_t dinDataMutexHandle;
const osMutexAttr_t dinDataMutex_attributes = {
  .name = "dinDataMutex"
};
/* Definitions for radioInputQueue */
osMessageQueueId_t radioInputQueueHandle;
const osMessageQueueAttr_t radioInputQueue_attributes = {
  .name = "radioInputQueue"
};
/* Definitions for radioOutputQueue */
osMessageQueueId_t radioOutputQueueHandle;
const osMessageQueueAttr_t radioOutputQueue_attributes = {
  .name = "radioOutputQueue"
};
/* Definitions for outOutputQueue */
osMessageQueueId_t outOutputQueueHandle;
const osMessageQueueAttr_t outOutputQueue_attributes = {
  .name = "outOutputQueue"
};
/* Definitions for radioEvent */
osEventFlagsId_t radioEventHandle;
const osEventFlagsAttr_t radioEvent_attributes = {
  .name = "radioEvent"
};
/* Definitions for radioFault */
osEventFlagsId_t radioFaultHandle;
const osEventFlagsAttr_t radioFault_attributes = {
  .name = "radioFault"
};
/* Definitions for dinFault */
osEventFlagsId_t dinFaultHandle;
const osEventFlagsAttr_t dinFault_attributes = {
  .name = "dinFault"
};
/* Definitions for doutFault */
osEventFlagsId_t doutFaultHandle;
const osEventFlagsAttr_t doutFault_attributes = {
  .name = "doutFault"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* creation of ioPortSpiMutex */
  ioPortSpiMutexHandle = osMutexNew(&ioPortSpiMutex_attributes);

  /* creation of dinDataMutex */
  dinDataMutexHandle = osMutexNew(&dinDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of radioInputQueue */
  radioInputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &radioInputQueue_attributes);
  /* creation of radioOutputQueue */
  radioOutputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &radioOutputQueue_attributes);
  /* creation of outOutputQueue */
  outOutputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &outOutputQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */
  /* creation of radioTask */
  radioTaskHandle = osThreadNew(radioTask, NULL, &radioTask_attributes);

  /* creation of userTask */
  userTaskHandle = osThreadNew(userTask, NULL, &userTask_attributes);

  /* creation of dinTask */
  dinTaskHandle = osThreadNew(dinTask, NULL, &dinTask_attributes);

  /* creation of doutTask */
  doutTaskHandle = osThreadNew(doutTask, NULL, &doutTask_attributes);

  /* creation of diagnosticsTask */
  diagnosticsTaskHandle = osThreadNew(diagnosticsTask, NULL, &diagnosticsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* creation of radioEvent */
  radioEventHandle = osEventFlagsNew(&radioEvent_attributes);

  /* creation of radioFault */
  radioFaultHandle = osEventFlagsNew(&radioFault_attributes);

  /* creation of dinFault */
  dinFaultHandle = osEventFlagsNew(&dinFault_attributes);

  /* creation of doutFault */
  doutFaultHandle = osEventFlagsNew(&doutFault_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */

  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_radioTask */
/**
* @brief Function implementing the radioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_radioTask */
void radioTask(void *argument)
{
  /* USER CODE BEGIN radioTask */
  app_radio_task(argument);
  /* USER CODE END radioTask */
}

/* USER CODE BEGIN Header_userTask */
/**
* @brief Function implementing the userTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_userTask */
void userTask(void *argument)
{
  /* USER CODE BEGIN userTask */
  app_user_task(argument);
  /* USER CODE END userTask */
}

/* USER CODE BEGIN Header_dinTask */
/**
* @brief Function implementing the dinTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dinTask */
void dinTask(void *argument)
{
  /* USER CODE BEGIN dinTask */
  app_din_task(argument);
  /* USER CODE END dinTask */
}

/* USER CODE BEGIN Header_doutTask */
/**
* @brief Function implementing the doutTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_doutTask */
void doutTask(void *argument)
{
  /* USER CODE BEGIN doutTask */
  /* Infinite loop */
  app_dout_task(argument);
  /* USER CODE END doutTask */
}

/* USER CODE BEGIN Header_diagnosticsTask */
/**
* @brief Function implementing the diagnosticsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_diagnosticsTask */
void diagnosticsTask(void *argument)
{
  /* USER CODE BEGIN diagnosticsTask */
  /* Infinite loop */
  app_diagnostics_task(argument);
  /* USER CODE END diagnosticsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

