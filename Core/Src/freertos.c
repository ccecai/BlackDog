/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "imu.h"
#include "led_flow_task.h"
#include "Screen.h"
#include "Subordinate_Desk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t standup_flag = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
fp32 gyro[3];
fp32 accel[3];
fp32 temp;

/* USER CODE END Variables */
osThreadId StartDebugTaskHandle;
osThreadId RemoteControlTaHandle;
osThreadId GIM_Output_LeftHandle;
osThreadId GIM6010InitHandle;
osThreadId GIM_Output_RighHandle;
osThreadId PIDHandle;
osThreadId PwmOutHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebug(void const * argument);
void RemoteControl(void const * argument);
void GIM_OutputLeftTask(void const * argument);
void GIM6010InitTask(void const * argument);
void GIMOutputrightTask(void const * argument);
void PID_Task(void const * argument);
void PwmOutTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of StartDebugTask */
  osThreadDef(StartDebugTask, StartDebug, osPriorityLow, 0, 128);
  StartDebugTaskHandle = osThreadCreate(osThread(StartDebugTask), NULL);

  /* definition and creation of RemoteControlTa */
  osThreadDef(RemoteControlTa, RemoteControl, osPriorityNormal, 0, 512);
  RemoteControlTaHandle = osThreadCreate(osThread(RemoteControlTa), NULL);

  /* definition and creation of GIM_Output_Left */
  osThreadDef(GIM_Output_Left, GIM_OutputLeftTask, osPriorityHigh, 0, 256);
  GIM_Output_LeftHandle = osThreadCreate(osThread(GIM_Output_Left), NULL);

  /* definition and creation of GIM6010Init */
  osThreadDef(GIM6010Init, GIM6010InitTask, osPriorityBelowNormal, 0, 256);
  GIM6010InitHandle = osThreadCreate(osThread(GIM6010Init), NULL);

  /* definition and creation of GIM_Output_Righ */
  osThreadDef(GIM_Output_Righ, GIMOutputrightTask, osPriorityHigh, 0, 256);
  GIM_Output_RighHandle = osThreadCreate(osThread(GIM_Output_Righ), NULL);

  /* definition and creation of PID */
  osThreadDef(PID, PID_Task, osPriorityAboveNormal, 0, 512);
  PIDHandle = osThreadCreate(osThread(PID), NULL);

  /* definition and creation of PwmOut */
  osThreadDef(PwmOut, PwmOutTask, osPriorityLow, 0, 256);
  PwmOutHandle = osThreadCreate(osThread(PwmOut), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

    osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

    vTaskResume(StartDebugTaskHandle);
    vTaskSuspend(GIM_Output_LeftHandle);
    vTaskSuspend(GIM_Output_RighHandle);
    vTaskSuspend(PIDHandle);
    vTaskSuspend(RemoteControlTaHandle);
    vTaskSuspend(GIM6010InitHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDebug */
/**
  * @brief  Function implementing the StartDebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */
    Mymain_Init();

    vTaskResume(GIM6010InitHandle);
  /* Infinite loop */
  for(;;)
  {
      osDelay(1000);
  }
  /* USER CODE END StartDebug */
}

/* USER CODE BEGIN Header_RemoteControl */
/**
* @brief Function implementing the RemoteControlTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteControl */
void RemoteControl(void const * argument)
{
  /* USER CODE BEGIN RemoteControl */
    remote_control_init();

    RC_ctrl_t *local_rc_ctrl;
    local_rc_ctrl = get_remote_control_point();
  /* Infinite loop */
  for(;;)
  {
      /*************
       *    遥控器只有左方拨档开关有用，最上方一档为蹲伏即上电状态，中间档为直立
       *    下方档为运动开关，右方摇杆为控制运动
       *
       ****************/
      Posture_Controller(local_rc_ctrl);
      usart_printf("%d,%d\n",Desk_Data[3],Desk_Data[1]);
      osDelay(1);
  }
  /* USER CODE END RemoteControl */
}

/* USER CODE BEGIN Header_GIM_OutputLeftTask */
/**
* @brief Function implementing the GIM_Output_Left thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIM_OutputLeftTask */
void GIM_OutputLeftTask(void const * argument)
{
  /* USER CODE BEGIN GIM_OutputLeftTask */
  /* Infinite loop */
  for(;;)
  {
      Leg_Output('l');

    osDelay(1);
  }
  /* USER CODE END GIM_OutputLeftTask */
}

/* USER CODE BEGIN Header_GIM6010InitTask */
/**
* @brief Function implementing the GIM6010Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIM6010InitTask */
void GIM6010InitTask(void const * argument)
{
  /* USER CODE BEGIN GIM6010InitTask */

//    Turn_AllMotor_to_USB();
    Motor_Init();

    Eight_PID_Init();
    ChangeGainOfPID(15.0f,5.0f,4.0f,0.1f);//输出化PID

    osDelay(1000);

    Read_beginPos(); //读取电机上电时候的初始位置

    vTaskResume(PIDHandle);
    vTaskResume(RemoteControlTaHandle);
    vTaskResume(GIM_Output_LeftHandle);
    vTaskResume(GIM_Output_RighHandle);

    vTaskSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GIM6010InitTask */
}

/* USER CODE BEGIN Header_GIMOutputrightTask */
/**
* @brief Function implementing the GIM_Output_Righ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GIMOutputrightTask */
void GIMOutputrightTask(void const * argument)
{
  /* USER CODE BEGIN GIMOutputrightTask */
  /* Infinite loop */
  for(;;)
  {
      Leg_Output('r');

    osDelay(1);
  }
  /* USER CODE END GIMOutputrightTask */
}

/* USER CODE BEGIN Header_PID_Task */
/**
* @brief Function implementing the PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_Task */
void PID_Task(void const * argument)
{
  /* USER CODE BEGIN PID_Task */
  /* Infinite loop */
  for(;;)
  {
      for(uint8_t id = 1;id < 9;id ++)
      {
          if(LieDown_flag == 0)
          {
              SetPoint(&AngleLoop[id], AngleWant_MotorX[id], id);
              PID_PosLocCalc(&AngleLoop[id], GIM6010[id].data_pos - begin_pos[id], id);
          }
          else if(LieDown_flag == 1)
          {
              SetPoint(&AngleLoop[id], AngleWant_MotorX[id], id);
              PID_PosLocCalc(&AngleLoop[id], GIM6010[id].data_pos, id);
          }
      }

      osDelay(2);
  }
  /* USER CODE END PID_Task */
}

/* USER CODE BEGIN Header_PwmOutTask */
/**
* @brief Function implementing the PwmOut thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PwmOutTask */
void PwmOutTask(void const * argument)
{
  /* USER CODE BEGIN PwmOutTask */
  /* Infinite loop */
  for(;;)
  {
      Process();

    osDelay(500);
  }
  /* USER CODE END PwmOutTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
