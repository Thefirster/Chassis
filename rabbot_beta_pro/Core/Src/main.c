/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "M3508.h"
#include "handkey.h"
#include "motion_control.h"
#include "A1_motor.h"
#include "my_uart.h"
#include "auto_route.h"
#include "yuntai.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926f
#define RADIN_TO_REG 57.296f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
extern M3508x_STA M3508[8];//创建结构体实例
extern M3508x_STA M3508_2[8];
float Rpm_set=0;
extern MOTOR_recv motor_recv[2];
A1_data a1_data[2];
int c_flag=3;

extern uint8_t Gyro_RX_Data[30];


extern Action_data Action_Data;
extern float speed_level;
extern float kx_av,ky_av;
extern float radar_x,radar_y;
extern float a1_pos[2];
extern float spped_set_a1;
extern int child_block;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
//8.13514 -0.1382 5 16400
//7.29757261 -0.351111144 6 16100
//6.65611 -0.1893 8 16500
//9.4684 -0.1758 2 15150
//5.37451675 -0.19946 10 15150
//0.075206	0.18194
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
		//Usart5_Start();
    a1_init();         // a1电机参数初始化
    yuntai_init(); // 初始化云台pid
    Usart6_Start();    // 启动a1电机接收
    c_flag = 1;
    HAL_TIM_Base_Start_IT(&htim2);//0.5515
    HAL_Delay(20);//1397542
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_Delay(20);
    Gyro_Receive_Start();
    Handkey_Receive_Start();
    MaPan_Receive_Start();
    Radar_Receive_Start();
		unsigned char cmd[5] = {0XA5, 0X5A, 0X04, 0X00, 0XFB};
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_Delay(20);
		HAL_UART_Transmit_DMA(&huart3, cmd, 5);
    a1_set_mode(YAW_MOTOR_ID, 0);                        // 使能电机，0
    a1_set_mode(PITCH_MOTOR_ID, 0);

    HAL_Delay(10);
    Usart3_Start(); // 启动视觉接收


    Can1_Configure();
    Can2_Configure();
    HAL_Delay(100);
    keep_pid_init();
    PID_Path_Init();
    M3510_PID_MotorInit(M3508);
    M3510_PID_MotorInit(M3508_2);


    __HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);




  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); 
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    if (htim == &htim2) {

				m3508_motion_control(&M3508[0]);
				m3508_motion_control(&M3508[5]);
				
        CAN1_SetMotor_0_3(M3508);
        CAN1_SetMotor_4_7(M3508);
				if(!child_block){
        yuntai_control();
				}
				else 
				{
//					a1_set_mode(0,0);
//					a1_set_mode(1,0);
				}
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);
    }
    if (htim == &htim4) {
        if (c_flag == 1)
        {
            c_flag = 0;
            A1_Motor_Send_Data_yuntai(a1_data[1].w, a1_data[1].pos, a1_data[1].kp, a1_data[1].kw, a1_data[1].t, a1_data[1].id, a1_data[1].mode); // now+28.5
            a1_data[1].w_now = motor_recv[1].motor_recv_data.Mdata.W / 128.f / 9.0f;
            a1_data[1].angle = RADIN_TO_REG / 9.0f * motor_recv[1].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f;
        }
        else if (c_flag == 0)
        {
            c_flag = 1;
            A1_Motor_Send_Data_yuntai(a1_data[0].w, a1_data[0].pos, a1_data[0].kp, a1_data[0].kw, a1_data[0].t, a1_data[0].id, a1_data[0].mode); // now-8
            a1_data[0].w_now = motor_recv[0].motor_recv_data.Mdata.W / 128.f / 9.0f;
            a1_data[0].angle = RADIN_TO_REG / 9.0f * motor_recv[0].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f;
        }
//2ms发送一次，改 htim4.Init.Period = 2000-1;
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_UPDATE);
    }
		taskEXIT_CRITICAL_FROM_ISR(status_value);
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
