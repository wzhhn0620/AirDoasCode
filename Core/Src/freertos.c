/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Mpu6050Task.h"
#include "UsartHandleTask.h"
#include "StepCtrlTask.h"
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
osThreadId USARTHandleHandle;
osThreadId MPU6050Handle;
osThreadId STEPCTRLHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Usart_Handle(void const * argument);
void Mpu6050_Measure(void const * argument);
void StepMotor_Ctrl(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of USARTHandle */
  osThreadDef(USARTHandle, Usart_Handle, osPriorityAboveNormal, 0, 128);
  USARTHandleHandle = osThreadCreate(osThread(USARTHandle), NULL);

  /* definition and creation of MPU6050 */
  osThreadDef(MPU6050, Mpu6050_Measure, osPriorityNormal, 0, 256);
  MPU6050Handle = osThreadCreate(osThread(MPU6050), NULL);

  /* definition and creation of STEPCTRL */
  osThreadDef(STEPCTRL, StepMotor_Ctrl, osPriorityBelowNormal, 0, 128);
  STEPCTRLHandle = osThreadCreate(osThread(STEPCTRL), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Usart_Handle */
/**
  * @brief  Function implementing the USARTHandle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Usart_Handle */
__weak void Usart_Handle(void const * argument)
{
  /* USER CODE BEGIN Usart_Handle */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Usart_Handle */
}

/* USER CODE BEGIN Header_Mpu6050_Measure */
/**
* @brief Function implementing the MPU6050 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mpu6050_Measure */
__weak void Mpu6050_Measure(void const * argument)
{
  /* USER CODE BEGIN Mpu6050_Measure */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Mpu6050_Measure */
}

/* USER CODE BEGIN Header_StepMotor_Ctrl */
/**
* @brief Function implementing the STEPCTRL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StepMotor_Ctrl */
__weak void StepMotor_Ctrl(void const * argument)
{
  /* USER CODE BEGIN StepMotor_Ctrl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StepMotor_Ctrl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

