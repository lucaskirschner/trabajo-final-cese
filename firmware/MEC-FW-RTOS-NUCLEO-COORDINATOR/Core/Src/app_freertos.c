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
#include "app_rs485.h"
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
/* Definitions for rs485Task */
osThreadId_t rs485TaskHandle;
const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
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
/* Definitions for rs485InputQueue */
osMessageQueueId_t rs485InputQueueHandle;
const osMessageQueueAttr_t rs485InputQueue_attributes = {
  .name = "rs485InputQueue"
};
/* Definitions for rs485OutputQueue */
osMessageQueueId_t rs485OutputQueueHandle;
const osMessageQueueAttr_t rs485OutputQueue_attributes = {
  .name = "rs485OutputQueue"
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
/* Definitions for rs485Event */
osEventFlagsId_t rs485EventHandle;
const osEventFlagsAttr_t rs485Event_attributes = {
  .name = "rs485Event"
};
/* Definitions for rs485Fault */
osEventFlagsId_t rs485FaultHandle;
const osEventFlagsAttr_t rs485Fault_attributes = {
  .name = "rs485Fault"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Error_Handler(void);
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
  if ((ioPortSpiMutexHandle == NULL) ||
      (dinDataMutexHandle == NULL))
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
  /* creation of radioInputQueue */
  radioInputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &radioInputQueue_attributes);
  /* creation of radioOutputQueue */
  radioOutputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &radioOutputQueue_attributes);
  /* creation of outOutputQueue */
  outOutputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &outOutputQueue_attributes);
  /* creation of rs485InputQueue */
  rs485InputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &rs485InputQueue_attributes);
  /* creation of rs485OutputQueue */
  rs485OutputQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &rs485OutputQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if ((radioInputQueueHandle == NULL) ||
      (radioOutputQueueHandle == NULL) ||
      (outOutputQueueHandle == NULL))
  {
    Error_Handler();
  }
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

  /* creation of rs485Task */
  rs485TaskHandle = osThreadNew(rs485Task, NULL, &rs485Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if ((radioTaskHandle == NULL) ||
      (userTaskHandle == NULL) ||
      (dinTaskHandle == NULL) ||
      (doutTaskHandle == NULL) ||
      (diagnosticsTaskHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* creation of radioEvent */
  radioEventHandle = osEventFlagsNew(&radioEvent_attributes);

  /* creation of radioFault */
  radioFaultHandle = osEventFlagsNew(&radioFault_attributes);

  /* creation of dinFault */
  dinFaultHandle = osEventFlagsNew(&dinFault_attributes);

  /* creation of doutFault */
  doutFaultHandle = osEventFlagsNew(&doutFault_attributes);

  /* creation of rs485Event */
  rs485EventHandle = osEventFlagsNew(&rs485Event_attributes);

  /* creation of rs485Fault */
  rs485FaultHandle = osEventFlagsNew(&rs485Fault_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  if ((radioEventHandle == NULL) ||
      (radioFaultHandle == NULL) ||
      (dinFaultHandle == NULL) ||
      (doutFaultHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_radioTask */
/**
* @brief Function implementing the radioTask thread.
*
* This task executes the radio application layer. It handles the MRF24J40
* service routine, including pending radio events, received data and queued
* transmissions.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_radioTask */
void radioTask(void *argument)
{
  /* USER CODE BEGIN radioTask */
  //app_radio_task(argument);
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END radioTask */
}

/* USER CODE BEGIN Header_userTask */
/**
* @brief Function implementing the userTask thread.
*
* This task executes the user application layer. It contains the application
* logic that uses the available input, output and communication services.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_userTask */
void userTask(void *argument)
{
  /* USER CODE BEGIN userTask */
  //app_user_task(argument);
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END userTask */
}

/* USER CODE BEGIN Header_dinTask */
/**
* @brief Function implementing the dinTask thread.
*
* This task executes the digital input application layer. It periodically
* updates the input state and provides the latest input data to the system.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dinTask */
void dinTask(void *argument)
{
  /* USER CODE BEGIN dinTask */
  //app_din_task(argument);
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END dinTask */
}

/* USER CODE BEGIN Header_doutTask */
/**
* @brief Function implementing the doutTask thread.
*
* This task executes the digital output application layer. It processes pending
* output commands and updates the output driver state.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_doutTask */
void doutTask(void *argument)
{
  /* USER CODE BEGIN doutTask */
  /* Infinite loop */
  //app_dout_task(argument);
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END doutTask */
}

/* USER CODE BEGIN Header_diagnosticsTask */
/**
* @brief Function implementing the diagnosticsTask thread.
*
* This task executes the diagnostics application layer. It monitors system
* fault flags and reports diagnostic information for the main application
* modules.
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_diagnosticsTask */
void diagnosticsTask(void *argument)
{
  /* USER CODE BEGIN diagnosticsTask */
  /* Infinite loop */
  //app_diagnostics_task(argument);
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END diagnosticsTask */
}

/* USER CODE BEGIN Header_rs485Task */
/**
* @brief Function implementing the rs485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rs485Task */
void rs485Task(void *argument)
{
  /* USER CODE BEGIN rs485Task */
  /* Infinite loop */
  app_rs485_task(argument);
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END rs485Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

