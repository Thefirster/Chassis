/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "Speed_decomposition.h"
#include "usart.h"
#include "lj_plus.h"
#include "gpio.h"
#include "pid_controller.h"
#include "Speed_decomposition.h"
#include "yuntai.h"
#include "quhuan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern wheel_Struct  wheel_final_v[5];  
extern M3508x_STA M3508[8];
extern handle_last SHOUBING;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int COUNT,Flag,Pin1_READ,Pin2_READ,Pin3_READ;float TIME;
extern coordinate_Struct Robot_Coordinate_system_Vel;
extern int Pin1_READ, Pin2_READ, Pin3_READ;
extern Action_data Action_Data;
extern LUJING_STU Lujing_Status; // 路径结构�?
int  Line_flag,qidong_flag;
float Angle, keep_x, keep_y;
uint8_t lj_flag = 0;
// 直线（目标x，目标y，要转的角度，V初，V末，加�?�区比例，减速区比例，w�?大，V�?大，停止区长度）
struct Lujing_Test
{
    float x;
    float y;
    float w;
    float v_start;
    float v_end;
    float a_up;
    float a_down;
    float w_max;
    float v_max;
    float stop_length;
};
struct Lujing_Test lujing_test = {
    .x = 0,//500,
    .y = 0,
    .w = 0,
    .v_start = 6000,
    .v_end = 0,
    .a_up = 0.2,
    .a_down = 0.5,
    .w_max = 100,
    .v_max = 10000,
    .stop_length = 20};
float CeShi[4] = {13000, 5000, 50, 6000};
float distance = 1000;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t Number;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId jigou_yuntaiHandle;
osThreadId dipanHandle;
osThreadId jigou_quhuanHandle;
osThreadId shoubingHandle;
osThreadId xianyuHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Jigou_yuntai(void const * argument);
void Dipan(void const * argument);
void Jigou_quhuan(void const * argument);
void Shoubing(void const * argument);
void Xianyu(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of jigou_yuntai */
  osThreadDef(jigou_yuntai, Jigou_yuntai, osPriorityIdle, 0, 128);
  jigou_yuntaiHandle = osThreadCreate(osThread(jigou_yuntai), NULL);

  /* definition and creation of dipan */
  osThreadDef(dipan, Dipan, osPriorityHigh, 0, 128);
  dipanHandle = osThreadCreate(osThread(dipan), NULL);

  /* definition and creation of jigou_quhuan */
  osThreadDef(jigou_quhuan, Jigou_quhuan, osPriorityIdle, 0, 128);
  jigou_quhuanHandle = osThreadCreate(osThread(jigou_quhuan), NULL);

  /* definition and creation of shoubing */
  osThreadDef(shoubing, Shoubing, osPriorityIdle, 0, 128);
  shoubingHandle = osThreadCreate(osThread(shoubing), NULL);

  /* definition and creation of xianyu */
  osThreadDef(xianyu, Xianyu, osPriorityIdle, 0, 128);
  xianyuHandle = osThreadCreate(osThread(xianyu), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */		
    for (;;)
    {
					if(SHOUBING.F_4 == 1)
					{
								yuntai.M3508_level += 0.00001 * SHOUBING.F_LX;
//			yuntai.M3508_pitch += 0.000000001 * SHOUBING.F_LY;
//			yuntai.RPM_begin = SHOUBING.F_5; 
//			yuntai.RPM_flag = SHOUBING.M_L;
			
					}
					yuntai.DM_flag = SHOUBING.F_6;
		}
  
    osDelay(1);
//				
//    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Jigou_yuntai */
/**
* @brief Function implementing the jigou_yuntai thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Jigou_yuntai */
void Jigou_yuntai(void const * argument)
{
  /* USER CODE BEGIN Jigou_yuntai */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Jigou_yuntai */
}

/* USER CODE BEGIN Header_Dipan */
/**
* @brief Function implementing the dipan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Dipan */
void Dipan(void const * argument)
{
  /* USER CODE BEGIN Dipan */
  /* Infinite loop */
  for(;;)
  {
		 OP();
		if (SHOUBING.F_1)
        {
           line(-3100, 0, 0, 6000, 0, 0.2, 0.2, 1000, 20000, 20);
            // Բ
//            circle(500, 500, 90, 230, 0, 500, DISABLE, 0.2, 0.5);
					while(SHOUBING.F_1)
					{
						 osDelay(1);
					}
        }
	  if (SHOUBING.F_2)
        {
           line(lujing_test.x, lujing_test.y, lujing_test.w, lujing_test.v_start, lujing_test.v_end, lujing_test.a_up, lujing_test.a_down, lujing_test.w_max, lujing_test.v_max, lujing_test.stop_length);
            // Բ
//            circle(500, 500, 90, 230, 0, 500, DISABLE, 0.2, 0.5);
            while(SHOUBING.F_2)
					{
						 osDelay(1);
					}
        }
		if (SHOUBING.F_3)
        {
          Robot_Coordinate_system_Vel.x=SHOUBING.F_RX;
  		    Robot_Coordinate_system_Vel.y=SHOUBING.F_RY;
  		    Robot_Coordinate_system_Vel.w=SHOUBING.F_LX;
          
					
					Lujing_Status.CoordinateSystem.Target_Position.w = Action_Data.angle_Z;
					Lujing_Status.CoordinateSystem.Target_Position.x = Action_Data.x;
					Lujing_Status.CoordinateSystem.Target_Position.y = Action_Data.y;
        }
		else if(SHOUBING.F_3 == 0)
		{
			 Keep_Robot_Position(Lujing_Status.CoordinateSystem.Target_Position.w, Lujing_Status.CoordinateSystem.Target_Position.x, Lujing_Status.CoordinateSystem.Target_Position.y);
		}
////        
//        Keep_Robot_Position(Lujing_Status.CoordinateSystem.Target_Position.w, Lujing_Status.CoordinateSystem.Target_Position.x, Lujing_Status.CoordinateSystem.Target_Position.y);
//        Robot_Coordinate_system_Vel.x=lujing_test.x;
//		    Robot_Coordinate_system_Vel.y=lujing_test.y;
//		    Robot_Coordinate_system_Vel.w=lujing_test.w;
        Robot_Wheel_Control();
        //		}
    osDelay(1);
  }
  /* USER CODE END Dipan */
}

/* USER CODE BEGIN Header_Jigou_quhuan */
/**
* @brief Function implementing the jigou_quhuan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Jigou_quhuan */
void Jigou_quhuan(void const * argument)
{
  /* USER CODE BEGIN Jigou_quhuan */
  /* Infinite loop */
  for(;;)
  {
		yuntai_control_all();   //ȡ������
    osDelay(1);
  }
  /* USER CODE END Jigou_quhuan */
}

/* USER CODE BEGIN Header_Shoubing */
/**
* @brief Function implementing the shoubing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoubing */
void Shoubing(void const * argument)
{
  /* USER CODE BEGIN Shoubing */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoubing */
}

/* USER CODE BEGIN Header_Xianyu */
/**
* @brief Function implementing the xianyu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Xianyu */
void Xianyu(void const * argument)
{
  /* USER CODE BEGIN Xianyu */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Xianyu */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
