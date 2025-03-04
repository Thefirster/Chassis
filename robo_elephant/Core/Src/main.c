/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Speed_decomposition.h"
#include "pid_controller.h"
#include "lj_plus.h"
#include "vesc.h"
#include "DM_J4310.h"
#include "JY_ME02.h"
#include "M3508.h"
#include "mycan.h"
#include "myusart.h"
#include "pid.h"
#include "timer.h"
#include "vision.h"
#include "yuntai.h"
#include "quhuan.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Change_FLAG[3];
float original_speed=400;
int8_t Number;
extern int Pin1_READ,Pin2_READ,Pin3_READ;
extern M3508x_STA M3508[8];
extern uint8_t self_rec_data[24];
extern uint8_t receive_data;
float Time;
int cnt = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
// 15.51  1.71  -0.67
  /* USER CODE END 1 */

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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//	MaPan_Receive_Start();
	RC_CtrlData_init();
	Can1_Configure();
	Can2_Configure();
	HAL_Delay(100);
	M3510_PID_MotorInit(M3508);
	SHOUBING_Receive_Start();
	MaPan_Receive_Start();
	
//	Self_MaPan_Receive_Start();
//	__HAL_UART_ENABLE_IT(&huart5,UART_IT_RXNE);
//	HAL_UART_Receive_IT(&huart5,self_rec_data,26);
//  HAL_UART_Receive_IT(&huart5,&receive_data,1);
  PID_Path_Init();
	
	Anglesum_Max_Min_Init();
	DM_MotorStart(&hcan1, DM_CANID_1, MIT_MODE);
	
	yuntai_middle_level = M350x[level_eid].angleSum;
	Left_Down_Pos = M350x[m2006_eid].angleSum;
	
	yuntai.M3508_pitch = JY_Encoder[pitch_eid].angle;
	yuntai.M3508_level = yuntai_middle_level;
	yuntai.M2006_angle_send = Left_Down_Pos;
	
	pid_Init_All();
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	duoji(duoji_upright_state);
	
	HAL_TIM_Base_Start_IT(&htim2);
    ReadFlash(); // 上电读一次flash数据
//    WriteFlash();
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
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim == &htim2)//定时器2中断函数  0.2ms
	{
    Robot_Wheel_speed_Control_3508(original_speed,~Number);
		Robot_Wheel_Control_3508(Number);
		if(Number==0x07)Robot_Wheel_speed_Control_VESC();
//	M3508_Speed(M3508  ,1000);
//  M3508_Speed(M3508+1,1000);
//		M3508_Speed(M3508,1000);
//		Robot_Wheel_Control_3508(Number);
//		VESC_Set_RPM(2,1000,&hcan1);
//		M3508_Speed_Position(M3508,0);
//		M3508_Speed(M3508+2  ,1000);
//		CAN1_SetMotor_0_3(M3508);
		
    if(cnt == 0){
			Yuntai_Rotor_Control();  //VESC&DM
		  if(mtr.vel > 5)
			  yuntai.DM_vel = 20;
	  	else
		  	yuntai.DM_vel = 0;
	  	DM_MotorCtrl(&hcan1, DM_CANID_1, MIT_MODE , yuntai.DM_pos, yuntai.DM_vel, 3.5f, 0.2f, 0.31f);
			cnt++;
		}else if(cnt == 1){
			motor_control();
			M350x_TxMes_0_3(&hcan1, M350x);
			cnt = 0;
		}
		
	}
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

