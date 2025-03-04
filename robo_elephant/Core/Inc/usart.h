/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
__packed struct
	 {
	 uint16_t ch0;
	 uint16_t ch1;
	 uint16_t ch2;
	 uint16_t ch3;
	 uint8_t s1;
	 uint8_t s2;
	 }rc;
	 
 __packed struct
	 {
	 int16_t x;
	 int16_t y;
	 int16_t z;
	 uint8_t press_l;
	 uint8_t press_r;
	 }mouse;
	 
__packed struct
	 {
	 uint16_t v;
	 }key;
	 
}RC_Ctl_t;
/*����*/
typedef struct
{
	float angle_Z;
	float angle_y;
	float angle_x;
	float x;
	float y;
	float w;
} Action_data;

typedef struct
{
	//���ֱ�����ߵ��ֱ����ұ�
	int F_1;
	int F_2;
	int F_3;
	int F_4;
	int F_5;
	int F_6;
	int F_7;
	int F_8;
	
	//�ֱ���ߵ�ҡ��
	float F_RX;
	float F_RY;
	//�ֱ���ߵ�ҡ��
	float F_LX;
	float F_LY;
	//�ֱ������ģʽ
	int M_L ;
	int M_R ;
} handle_last;


typedef union 
{
	uint8_t a[32];
	short b[16];
}handle;


typedef union 
{
	uint8_t a[40];
	float b[10];
}xike;
/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void SHOUBING_Receive_Start(void);
void UART4_IDLE_Callback(UART_HandleTypeDef *huart);
void RC_CtrlData_init(void);


void MaPan_Receive_Start(void);
void Self_MaPan_Receive_Start(void);
void Mapan_Data_Process(void);
void UART1_IDLE_Callback(UART_HandleTypeDef *huart);
void UART2_IDLE_Callback(UART_HandleTypeDef *huart);
void UART5_IDLE_Callback(UART_HandleTypeDef *huart);
void sending(void);
float get_lcd_val(uint8_t id);
void WriteFlash(void); // ��flash��д����
void ReadFlash(void); // �ϵ��һ��flash����
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

