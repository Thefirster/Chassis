#ifndef _HANDKEY_H_
#define _HANDKEY_H_
#include "stm32f4xx.h"

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

typedef struct
{
	float angle_Z;
	float angle_y;
	float angle_x;
	float x;
	float y;
	float w;
	float x_0;
	float y_0;
	float zero_x;
	float zero_y;
	float one_x;
	float one_y;
	float x_tmp;
	float y_tmp;
	float zero_w;
	float w_0;
	float x_reset;
	float y_reset;
	float x_set;
	float y_set;
} Action_data;

typedef struct
{
	int num;
	int xiaodou_ram[50];
	int data_set;
	
}Key_xiaodou;

typedef union
{
	float data_chg[5];
	uint8_t data_rec[20];
} data_chg;
/*GYRO_DATA*/
typedef struct
{
	float angle_pitch;//¸©Ñö½Ç
	float angle_yaw;//Æ«º½½Ç
	float angle_roll;//ºá¹ö½Ç
//	float x;
//	float y;
//	float w;
} Gyro_Data;

void Handkey_Receive_Start(void);
void Gyro_Receive_Start(void);
void RemoteDataProcess(uint8_t *pData);
void Gyro_Data_Process(void);
float get_float_from_4u8(unsigned char *p);
void Radar_Receive_Start(void);
void UART2_IDLE_Callback(UART_HandleTypeDef *huart);
void UART5_IDLE_Callback(UART_HandleTypeDef *huart);
void Mapan_Data_Process(void);
void UART1_IDLE_Callback(UART_HandleTypeDef *huart);
void UART6_IDLE_Callback(UART_HandleTypeDef *huart);
void USART3_IDLE_Callback(UART_HandleTypeDef *huart);
void MaPan_Receive_Start(void);
float dataasic_to_float(uint8_t data1,uint8_t data2,uint8_t data3);
void zero_reset(void);
float get_float_from_4u8_radar(unsigned char *p,int i);
void Radar_Data_Process(void);
void ceju_dataprocess(uint8_t *pData);
#endif
