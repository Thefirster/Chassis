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
	int16_t speed;          //�����ǰת��
	int16_t angleNow;       //���ڵĽǶ�
	int16_t angleLast;      //��һ�εĽǶ�
	float angleSum;         //��е�Ƕ�֮��
	float angleSumLast;     //�ϴνǶ�֮��
	int16_t dAngle;         //�ǶȲ�
	         
	int16_t electric;       //��������ĵ���ֵ
	
  PID M_Speed_PoPID;   //λ��ʽPID
	PID M_Speed_InPID;   //����ʽPID
	float poitionOutput;       //λ��ʽPID���
	float incremenOutput;      //����ʽPID���
	float cascadeOutput;       //����PID���
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
