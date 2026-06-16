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

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */
  /* creation of radioTask */
  radioTaskHandle = osThreadNew(radioTask, NULL, &radioTask_attributes);

  /* creation of userTask */
  userTaskHandle = osThreadNew(userTask, NULL, &userTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* creation of radioEvent */
  radioEventHandle = osEventFlagsNew(&radioEvent_attributes);

  /* creation of radioFault */
  radioFaultHandle = osEventFlagsNew(&radioFault_attributes);

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

