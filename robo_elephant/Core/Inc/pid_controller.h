#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__
// #include "stm32f4xx_hal.h"
#include "can.h"
typedef struct
{
    float KP;
    float KI;
    float KD;

    float InitKp;
    float InitKi;
    float InitKd;

    float Error[3];
    float Output;
    float Input;
    float target;
    float Err;
    float Last_Err;
    float DeadBand;
    float ITerm;
    float Pout;
    float Dout;
    float Last_Dout;
    float Iout;
    float Last_Measure;
    float Measure;
    float Last_Output;
    int youhua[6];
    int PID_Complete_Flag;
    float k;
    float Max_out;
		
		
    
} PID_Struct;

typedef struct
{
    int16_t Angle; // 转子机械角度
    int16_t Speed; // 转子转速
    int16_t Temperature;
    int8_t Currency;
    int16_t MAX_OUT_JILU;

    PID_Struct Position_S_PID_STruct;
    PID_Struct Speed_P_PID_STruct;

    PID_Struct Single_Speed_PID_STruct;
    int32_t dAngle_Sum; // 累积角度
    int16_t Angle_Old;  // 前一次的机械角度
    int16_t Out;
    int16_t Out0;
} M3508x_STA; //    M3508  DE ZHUANGTAI

void Can_3508_Receive(CAN_RxHeaderTypeDef Can_Rx, uint8_t *Data, M3508x_STA *M350x);
void M3510_Angle_Calculate(M3508x_STA *M350x);
void M3508_Speed_Position(M3508x_STA *M350x, int32_t Angle);
void M3508_Speed(M3508x_STA *M350x, int32_t speed);
void M3510_PID_MotorInit(M3508x_STA *M350x);
void CAN1_SetMotor_0_3(M3508x_STA *M350x);
void CAN1_SetMotor_4_7(M3508x_STA *M350x);
void CAN2_SetMotor_0_3(M3508x_STA *M350x);
void CAN2_SetMotor_4_7(M3508x_STA *M350x);
void PID_Calculate(PID_Struct *PID_Parameter, float A, float B,
                   float Maxout, float IntegralLimit,
                   float Derivative_Filtering_Coefficient, float Output_Filtering_Coefficient);

#endif
