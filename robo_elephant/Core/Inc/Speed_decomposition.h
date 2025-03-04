#ifndef __SPEED_DECOMPOSITION_H__
#define __SPEED_DECOMPOSITION_H__
#include "can.h"
#define PI 3.1415926535f
#define HASSIS_Struct_o1_length		1							
#define HASSIS_Struct_o2_length  	1 
#define HASSIS_Struct_o3_length		1								
#define HASSIS_Struct_o4_length   1
#define HASSIS_Struct_o1_angle    PI*1/3
#define HASSIS_Struct_o2_angle    PI*2/3
#define HASSIS_Struct_o3_angle    0
//#define HASSIS_Struct_o4_angle    PI/4
typedef struct
{
	float x;
	float y;
	float resultant_v;
	float absolute_angle;//与前进方向夹角 ，逆时针从0到360，【0,360）  ，即轮子的绝对式角度
	float angle_sum;  //    polar_angle积分，（负无穷，正无穷）       ， 即轮子的增量式角度
	float angle_gap;  //轮子与目标速度角度的差值，也就是要转的角度
	float v_angle; //目标合速度的角度
	float Original_angle_encode;//轮子在零位时的编码器的值
} wheel_Struct;

//typedef struct
//{
//	float x;//x位置
//	float y;//y位置
//	float Angular_velocity;//绕坐标原点旋转角速度
//} coordinitioate_Struct;//坐标结构体


void gain_absolute_angle(wheel_Struct *wheel_v,int id);
void gain_gap_angle(wheel_Struct *wheel_v);
void Robot_Wheel_Control(void);
void Robot_Wheel_Control_3508(int8_t number);
void Robot_Wheel_speed_Control_3508(float Speed,int8_t number);
void Robot_Wheel_speed_Control_VESC(void);
#endif
