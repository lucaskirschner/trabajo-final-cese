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
#include "app_dout.h"
#include "app_din.h"
#include "app_diagnostics.h"
#include "app_user.h"
#include "app_radio.h"

#include "swo.h"
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
/* Definitions for outputTask */
osThreadId_t outputTaskHandle;
const osThreadAttr_t outputTask_attributes = {
  .name = "outputTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for userTask */
osThreadId_t userTaskHandle;
const osThreadAttr_t userTask_attributes = {
  .name = "userTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for inputTask */
osThreadId_t inputTaskHandle;
const osThreadAttr_t inputTask_attributes = {
  .name = "inputTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for diagnosticsTask */
osThreadId_t diagnosticsTaskHandle;
const osThreadAttr_t diagnosticsTask_attributes = {
  .name = "diagnosticsTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for radioTask */
osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .priority = (osPriority_t) osPriorityLow,
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
/* Definitions for outputQueue */
osMessageQueueId_t outputQueueHandle;
const osMessageQueueAttr_t outputQueue_attributes = {
  .name = "outputQueue"
};
/* Definitions for inputQueue */
osMessageQueueId_t inputQueueHandle;
const osMessageQueueAttr_t inputQueue_attributes = {
  .name = "inputQueue"
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
/* Definitions for doutFaultEvent */
osEventFlagsId_t doutFaultEventHandle;
const osEventFlagsAttr_t doutFaultEvent_attributes = {
  .name = "doutFaultEvent"
};
/* Definitions for dinEvent */
osEventFlagsId_t dinEventHandle;
const osEventFlagsAttr_t dinEvent_attributes = {
  .name = "dinEvent"
};
/* Definitions for dinFaultEvent */
osEventFlagsId_t dinFaultEventHandle;
const osEventFlagsAttr_t dinFaultEvent_attributes = {
  .name = "dinFaultEvent"
};
/* Definitions for radioEvent */
osEventFlagsId_t radioEventHandle;
const osEventFlagsAttr_t radioEvent_attributes = {
  .name = "radioEvent"
};
/* Definitions for doutEvent */
osEventFlagsId_t doutEventHandle;
const osEventFlagsAttr_t doutEvent_attributes = {
  .name = "doutEvent"
};
/* Definitions for radioFaultEvent */
osEventFlagsId_t radioFaultEventHandle;
const osEventFlagsAttr_t radioFaultEvent_attributes = {
  .name = "radioFaultEvent"
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
  if (ioPortSpiMutexHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of outputQueue */
  outputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &outputQueue_attributes);
  /* creation of inputQueue */
  inputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &inputQueue_attributes);
  /* creation of radioInputQueue */
  radioInputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &radioInputQueue_attributes);
  /* creation of radioOutputQueue */
  radioOutputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &radioOutputQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if (outputQueueHandle == NULL)
  {
    Error_Handler();
  }

  if (inputQueueHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */
  /* creation of outputTask */
  outputTaskHandle = osThreadNew(Output_Task, NULL, &outputTask_attributes);

  /* creation of userTask */
  userTaskHandle = osThreadNew(User_Task, NULL, &userTask_attributes);

  /* creation of inputTask */
  inputTaskHandle = osThreadNew(Input_Task, NULL, &inputTask_attributes);

  /* creation of diagnosticsTask */
  diagnosticsTaskHandle = osThreadNew(Diagnostics_Task, NULL, &diagnosticsTask_attributes);

  /* creation of radioTask */
  radioTaskHandle = osThreadNew(Radio_Task, NULL, &radioTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (outputTaskHandle == NULL)
  {
    Error_Handler();
  }

  if (userTaskHandle == NULL)
  {
    Error_Handler();
  }

  if (inputTaskHandle == NULL)
  {
    Error_Handler();
  }

  if (diagnosticsTaskHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* creation of doutFaultEvent */
  doutFaultEventHandle = osEventFlagsNew(&doutFaultEvent_attributes);

  /* creation of dinEvent */
  dinEventHandle = osEventFlagsNew(&dinEvent_attributes);

  /* creation of dinFaultEvent */
  dinFaultEventHandle = osEventFlagsNew(&dinFaultEvent_attributes);

  /* creation of radioEvent */
  radioEventHandle = osEventFlagsNew(&radioEvent_attributes);

  /* creation of doutEvent */
  doutEventHandle = osEventFlagsNew(&doutEvent_attributes);

  /* creation of radioFaultEvent */
  radioFaultEventHandle = osEventFlagsNew(&radioFaultEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  if (doutFaultEventHandle == NULL)
  {
    Error_Handler();
  }

  if (dinEventHandle == NULL)
  {
    Error_Handler();
  }

  if (dinFaultEventHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_Output_Task */
/**
* @brief Function implementing the outputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Output_Task */
void Output_Task(void *argument)
{
  /* USER CODE BEGIN outputTask */
	app_dout_task(argument);
  /* USER CODE END outputTask */
}

/* USER CODE BEGIN Header_User_Task */
/**
* @brief Function implementing the userTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_User_Task */
void User_Task(void *argument)
{
  /* USER CODE BEGIN userTask */
	app_user_task(argument);
  /* USER CODE END userTask */
}

/* USER CODE BEGIN Header_Input_Task */
/**
* @brief Function implementing the inputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Input_Task */
void Input_Task(void *argument)
{
  /* USER CODE BEGIN inputTask */
	app_din_task(argument);
  /* USER CODE END inputTask */
}

/* USER CODE BEGIN Header_Diagnostics_Task */
/**
* @brief Function implementing the diagnosticsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Diagnostics_Task */
void Diagnostics_Task(void *argument)
{
  /* USER CODE BEGIN diagnosticsTask */
	app_diagnostics_task(argument);
  /* USER CODE END diagnosticsTask */
}

/* USER CODE BEGIN Header_Radio_Task */
/**
* @brief Function implementing the radioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Radio_Task */
void Radio_Task(void *argument)
{
  /* USER CODE BEGIN radioTask */
	app_radio_task(argument);
  /* USER CODE END radioTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

