#ifndef _ROTOR_H_
#define _ROTOR_H_

#include "main.h"
#include "pid.h"
#include "mycan.h"
#include "pid.h"
#include "JY_ME02.h"
#include "timer.h"

//
typedef struct   
{
	int16_t speed;          //电机当前转速
	int16_t angleNow;       //现在的角度
	int16_t angleLast;      //上一次的角度
	float angleSum;         //机械角度之和
	float angleSumLast;     //上次角度之和
	int16_t dAngle;         //角度差
	         
	int16_t electric;       //传给电机的电流值
	
  PID M_Speed_PoPID;   //位置式PID
	PID M_Speed_InPID;   //增量式PID
	float poitionOutput;       //位置式PID输出
	float incremenOutput;      //增量式PID输出
	float cascadeOutput;       //串级PID输出
}M350D_STA;
//
extern M350D_STA M350x[8];

void M350x_RecData(CAN_RxHeaderTypeDef *pHeader, uint8_t *RxCanData, M350D_STA *M350x);  
void M350x_TxMes_0_3(CAN_HandleTypeDef *hcanx, M350D_STA *M350x); 
void M350x_TxMes_4_7(CAN_HandleTypeDef *hcanx, M350D_STA *M350x);
void pid_clear(M350D_STA *M350x);                                      
float Scope_Max_Min(float NowSpeed, float MaxSpeed);
void pid_Init(M350D_STA *M350x, float Pkp, float Pki, float Pkd, float Ikp, float Iki, float Ikd);
void pid_Init_All(void);
void M_Calculate_IncrementalPID_Test(M350D_STA* M350x, float Speed);
void M_Calculate_PositionalPID_Test(M350D_STA *M350x, float Angle);
void M_Calculate_CascadePID_Test(M350D_STA *M350x, float Angle, float SpeedMax);
volatile void M_Init_Stable(M350D_STA *M350x, volatile float *SendAngle);
void M_Calculate_CascadePID_Vel(M350D_STA *M350x, float Vel);
void M_Encoder_Cascade_PID(M350D_STA *M350x, JY_Can_Receive *JY, float Angle, float SpeedMax);

#endif
