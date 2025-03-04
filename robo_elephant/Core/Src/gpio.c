/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "Speed_decomposition.h"
#include "pid_controller.h"
extern wheel_Struct  wheel_final_v[5];  
extern M3508x_STA M3508[8];
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if(GPIO_Pin==GPIO_PIN_7)
//  {
//    if((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)&&((Number&0x01)==0))
//    {
//			wheel_final_v[1].Original_angle_encode=M3508[0].dAngle_Sum;
//			Number|=0x01;
////			EXTI->IMR&=~EXTI_LINE_2;	
//    }
//		Pin1_READ=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
//  }
//  else if(GPIO_Pin==GPIO_PIN_3)//上升和下降沿都捕捉
//  {
////    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==1)TIM6->CNT=0,Flag=1;
////		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==1&&Flag==0)COUNT=TIM6->CNT,TIME=COUNT/100,Flag=0;
//    if((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)==0)&&((Number&0x02)==0))
//    {
//			wheel_final_v[2].Original_angle_encode=M3508[1].dAngle_Sum;
//			Number|=0x02;
////			EXTI->IMR&=~EXTI_LINE_3;
//    }		
//		Pin2_READ=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
//  }
//	  else if(GPIO_Pin==GPIO_PIN_8)//上升和下降沿都捕捉
//  {
//    if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)==1)&&((Number&0x04)==0))
//    {
//			wheel_final_v[3].Original_angle_encode=M3508[2].dAngle_Sum;
//			Number|=0x04;
////			EXTI->IMR&=~EXTI_LINE_8;
//    }
//		Pin3_READ=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8);
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
//  }
//}
/* USER CODE END 2 */
