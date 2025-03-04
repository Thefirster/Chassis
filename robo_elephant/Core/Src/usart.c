/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "freertos.h"
#include "task.h"
#include "math.h"
#include "Speed_decomposition.h"
#include "string.h"

RC_Ctl_t RC_CtrlData1,
RC_CtrlData2;

static float ABS(float a)
{
	return a>0?a:-a;
}

/*码盘*/
Action_data Action_Data;
Action_data Self_Action_Data;
uint8_t rec_data[30];
float Angle_Z_last=0;//机器人上一时刻的角度
float angle_Z_temp_1,angle_Z_temp_2,angle_Z_temp;//一些计算角度的临时变量

/* USER CODE END 0 */

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_NORMAL;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG9     ------> USART6_RX
    PG14     ------> USART6_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG9     ------> USART6_RX
    PG14     ------> USART6_TX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9|GPIO_PIN_14);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void RC_CtrlData_init(void)
{
  		 RC_CtrlData2.rc.ch0 = 1024 ;
			 RC_CtrlData2.rc.ch1 = 1024;
			 RC_CtrlData2.rc.ch2 = 1024;
			 RC_CtrlData2.rc.ch3 = 1024;
}
void RemoteDataProcess(uint8_t *pData)
{
 if(pData == NULL)
 {
 return;
 }
	 RC_CtrlData1.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	 RC_CtrlData1.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	 RC_CtrlData1.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	 RC_CtrlData1.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;	 
	 RC_CtrlData1.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	 RC_CtrlData1.rc.s2 = ((pData[5] >> 4) & 0x0003);
	 RC_CtrlData1.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	 RC_CtrlData1.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	 RC_CtrlData1.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
	 RC_CtrlData1.mouse.press_l = pData[12];
	 RC_CtrlData1.mouse.press_r = pData[13];
	 RC_CtrlData1.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
 //your control code ….
 		if(RC_CtrlData1.mouse.x==0&&RC_CtrlData1.mouse.y==0&&RC_CtrlData1.mouse.z==0&&RC_CtrlData1.mouse.press_l==0&&RC_CtrlData1.mouse.press_r==0&&RC_CtrlData1.key.v==0)
		{
		 	 RC_CtrlData2.rc.ch0 = RC_CtrlData1.rc.ch0 ;
			 RC_CtrlData2.rc.ch1 = RC_CtrlData1.rc.ch1;
			 RC_CtrlData2.rc.ch2 = RC_CtrlData1.rc.ch2;
			 RC_CtrlData2.rc.ch3 = RC_CtrlData1.rc.ch3;	 
			 RC_CtrlData2.rc.s1 = RC_CtrlData1.rc.s1;
			 RC_CtrlData2.rc.s2 = RC_CtrlData1.rc.s2;
			 RC_CtrlData2.mouse.x = 0;
			 RC_CtrlData2.mouse.y = 0;
			 RC_CtrlData2.mouse.z = 0; 
			 RC_CtrlData2.mouse.press_l =0;
			 RC_CtrlData2.mouse.press_r = 0;
			 RC_CtrlData2.key.v =0;
//			 DMA_SetCurrDataCounter(DMA1_Stream5,18); 
		}
		else
    {
	     RC_CtrlData2.rc.ch0 = 1024 ;
			 RC_CtrlData2.rc.ch1 = 1024;
			 RC_CtrlData2.rc.ch2 = 1024;
			 RC_CtrlData2.rc.ch3 = 1024;
			 RC_CtrlData2.rc.s1 = RC_CtrlData1.rc.s1;
			 RC_CtrlData2.rc.s2 = RC_CtrlData1.rc.s2;
			 RC_CtrlData2.mouse.x = 0;
			 RC_CtrlData2.mouse.y = 0;
			 RC_CtrlData2.mouse.z = 0; 
			 RC_CtrlData2.mouse.press_l =0;
			 RC_CtrlData2.mouse.press_r = 0;
			 RC_CtrlData2.key.v =0;			 
		}
}
/********************************************手柄************************************************************************/
handle_last SHOUBING;
handle shark;
int len=0;
uint16_t CH[18] = {1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024}; 
//uint8_t sbus_rx_buffer[32];
	extern int8_t Number;

uint32_t val_flash[5][3]; 
float val[5][3]; // 调试数据
    
    
#define WRITE_START_ADDR ((uint32_t)0x08060000) // 数据起始地址



void ReadFlash() // 上电读一次flash数据
{
    memcpy(val_flash,(uint32_t *)WRITE_START_ADDR,sizeof(val_flash));
}

void WriteFlash() // 往flash里写数据
{
    uint8_t sect = 7; // 扇区7
    HAL_FLASH_Unlock();//解锁flash
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);//清除一些错误标志
    FLASH_Erase_Sector(sect,FLASH_VOLTAGE_RANGE_3);
    
    for(uint8_t i = 0; i < 5 ; i ++)
    {
        for(uint8_t j = 0; j < 3; j ++)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WRITE_START_ADDR + 12 * i + 4 * j, val_flash[i][j]);
        }
    }
    HAL_FLASH_Lock();
}

void SHOUBINGCHULI(uint8_t *pData,short *hData)
{
	if((pData == NULL)&&(hData == NULL))
 {
 return;
 }
    if(pData[0] == 0xAA && pData[1] == 0xAA) // 手柄控制帧头
    {
        SHOUBING.F_1 = pData[16];
        SHOUBING.F_2 = pData[17];
        SHOUBING.F_3 = pData[15];
        SHOUBING.F_4 = pData[11];
        SHOUBING.F_5 = pData[12];
        SHOUBING.F_6 = pData[13];
        SHOUBING.F_7 = pData[14];
        SHOUBING.F_8 = pData[10];
        
        SHOUBING.F_LX = -30*hData[2];
        SHOUBING.F_LY = -30*hData[1];
        SHOUBING.F_RX = -30*hData[4];
        SHOUBING.F_RY = -30*hData[3];
        
        if(pData[22] == 0)SHOUBING.M_L = 1;
        else if(pData[21] == 0)SHOUBING.M_L = 2;
        else if(pData[20] == 0)SHOUBING.M_L = 3;
        else if(pData[19] == 0)SHOUBING.M_L = 4;
        else if(pData[18] == 0)SHOUBING.M_L = 5;
        
        if(pData[27] == 0)SHOUBING.M_R = 1;
        else if(pData[26] == 0)SHOUBING.M_R = 2;
        else if(pData[25] == 0)SHOUBING.M_R = 3;
        else if(pData[24] == 0)SHOUBING.M_R = 4;
        else if(pData[23] == 0)SHOUBING.M_R = 5;
    }
    else if(pData[0] == 0x0D && pData[1] == 0x0A) // 串口屏控制帧头
    {
        if(pData[2] == 's' && pData[3] == 'v') // 保存调试数据到flash中
        {
            WriteFlash();
        }
        else
        {
            val_flash[pData[2]][pData[3]] = *(uint32_t*)&pData[4];
            val[pData[2]][pData[3]] = *(int32_t*)&val_flash[pData[2]][pData[3]] / 1000.0f;
            
        }
        
    }
	
}



float get_lcd_val(uint8_t id)
{
    uint8_t row = id / 10;
    uint8_t col = id % 10;
    if(val[row][col] == 0xFFFFFFFF)
        return 0;
    return val[row][col];
}

uint8_t test[32];
void UART2_IDLE_Callback(UART_HandleTypeDef *huart)
{
		uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     //判断一帧数据是否接收完毕
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);     //清空IDLE标志位
		(void)USART2->SR;      //清空SR寄存器
		(void)USART2->DR;     //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF2_6);  //清空DMA传输完成标志位
		HAL_UART_DMAStop(huart);
		
		HAL_UART_Receive_DMA(huart, shark.a, 32);
        if(shark.a[0] == 0x0D && shark.a[1] == 0x0A)
        {
            memcpy(test, shark.a, 32);
        }
		SHOUBINGCHULI(shark.a,shark.b);
//		RemoteDataProcess(sbus_rx_buffer);		
//   if(RC_CtrlData2.rc.s1==1)Number=0x07;		
	}
		taskEXIT_CRITICAL_FROM_ISR(status_value);
}
void SHOUBING_Receive_Start(void)
{
		HAL_UART_Receive_DMA(&huart2,shark.a,32);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
}
/****************************************************码盘********************************************************************/
//配置的时候dma配置在串口前面
void MaPan_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rec_data,30);
}

void UART1_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  //临界段代码保护
	
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     //判断一帧数据是否接收完毕
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);     //清空IDLE标志位
		(void)USART1->SR;      //清空SR寄存器
		(void)USART1->DR;     //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF2_6);  //清空DMA传输完成标志位
		HAL_UART_DMAStop(huart);
		
		Mapan_Data_Process();//码盘接收数据处理函数
			
		HAL_UART_Receive_DMA(huart, rec_data, 30);		//再次使能接收
	}
	
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

static float get_float_from_4u8(unsigned char *p)
{
	float a;
	unsigned char *r;
	r=(unsigned char*)&a;
	*r=p[0];
	r++;
	*r=p[1];
	r++;
	*r=p[2];
	r++;
	*r=p[3];
	return(a);
}
float delt_y=153;
extern float Time;
float speed,X_jilu;
void Mapan_Data_Process(void)//码盘接收数据处理函数
{
	if((rec_data[0]==0X0D)&&(rec_data[1]==0X0A)/*&&(rec_data[26]==0X0A)&&(rec_data[27]==0X0D)*/)//帧头帧尾判断
	{
		float angle_template = get_float_from_4u8(&rec_data[2]);
		
		if(Angle_Z_last<angle_template)//一种情况
			angle_Z_temp_1=angle_template-Angle_Z_last,//逆时针
			angle_Z_temp_2=angle_template-Angle_Z_last-360;
		else
			angle_Z_temp_1=angle_template-Angle_Z_last,//顺时针
			angle_Z_temp_2=360+angle_template-Angle_Z_last;
		//无论顺时针转还是逆时针转，都是取小的那个角度
		angle_Z_temp=(ABS(angle_Z_temp_1))>(ABS(angle_Z_temp_2))? angle_Z_temp_2:angle_Z_temp_1;
		Angle_Z_last=angle_template;
		Action_Data.angle_Z=Action_Data.angle_Z+angle_Z_temp;
		
//		Action_Data.y= -get_float_from_4u8(&rec_data[18])+delt_y*cos(Action_Data.angle_Z/180.0f*PI)-delt_y;
//		Action_Data.x=-get_float_from_4u8(&rec_data[14])-delt_y*sin(Action_Data.angle_Z/180.0f*PI);
		Action_Data.y= get_float_from_4u8(&rec_data[18]);
		Action_Data.x= get_float_from_4u8(&rec_data[14]);
		Action_Data.w= get_float_from_4u8(&rec_data[22])+0.8f;
		speed=(Action_Data.x-X_jilu)/(Time*0.001f);
		X_jilu=Action_Data.x,Time=0;
				//码盘逆正顺负
	}
}
/****************************************************自制码盘********************************************************************/
uint8_t self_rec_data[46];
void Self_MaPan_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart5,self_rec_data,46);
}
void Self_Mapan_Data_Process(void)
{
//  if((rec_data[0]==0X0D)&&(rec_data[1]==0X0A))
//	{
		Self_Action_Data.angle_Z=get_float_from_4u8(&self_rec_data[4]);
	  Self_Action_Data.x=get_float_from_4u8(&self_rec_data[8]);
		Self_Action_Data.y=get_float_from_4u8(&self_rec_data[12]);
		Self_Action_Data.w=get_float_from_4u8(&self_rec_data[16]);
//	}
}
void UART5_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  //临界段代码保护
	
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     //判断一帧数据是否接收完毕
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);     //清空IDLE标志位
		(void)UART5->SR;      //清空SR寄存器
		(void)UART5->DR;     //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF0_4);  //清空DMA传输完成标志位
		HAL_UART_DMAStop(huart);
		
		Self_Mapan_Data_Process();//码盘接收数据处理函数
			
		HAL_UART_Receive_DMA(huart, self_rec_data, 46);		//再次使能接收
	}
	
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}
 /************************普通串口中断示例 *********************************/
//uint8_t receive_data ;
//int i,state;
//float x,y;
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//	if(huart->Instance==UART5)//如果是串口1
//	{		 
//    HAL_UART_Receive_IT(&huart5,&receive_data,1); //接收数据 放在receive_data
//		if(state==0)
//		{
//		  if(receive_data==0x0d)
//				state=1;
//		}
//		else if(state==1)
//		{
//		  if(receive_data==0x0a)state=2;
//			else state=0;	
//		}
//		else if(state==2)
//		{
//		  if(i<24)self_rec_data[i++]=receive_data;
//			else state=0,i=0;
//		}
//		Self_Action_Data.angle_Z=get_float_from_4u8(&self_rec_data[0]);
//	  Self_Action_Data.x= -get_float_from_4u8(&self_rec_data[12])*10+delt_y*sin(Self_Action_Data.angle_Z/180.0f*PI);
//		Self_Action_Data.y= -get_float_from_4u8(&self_rec_data[16])*10-delt_y*cos(Self_Action_Data.angle_Z/180.0f*PI)+delt_y;
//		Self_Action_Data.w= get_float_from_4u8(&self_rec_data[20]);
//	  x= -get_float_from_4u8(&self_rec_data[12])*10;
//		y= -get_float_from_4u8(&self_rec_data[16])*10;	  
//	}
// }
// 
// /*vofa????*/
//float tempFloat[4];
//uint8_t tempData[20];

//void sending()
//{

////    if(send_ms_time >= 0.02)
////    {
////        send_ms_time = 0;
////    }
////    else 
////    {
////        return ;
////    }
//    /*????*/
//    tempFloat[0] = (float)Action_Data.x; // send_world_x
//    tempFloat[1] = (float)Action_Data.y; // send_world_y
//    tempFloat[2] = (float)Action_Data.angle_Z;
//    tempFloat[3] = (float)Action_Data.w;

//    memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));

//    tempData[16] = 0x00;
//    tempData[17] = 0x00;
//    tempData[18] = 0x80;
//    tempData[19] = 0x7f;

//    HAL_UART_Transmit(&huart5, tempData, 20,0xffff);
//    
//}

/* USER CODE END 1 */
