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
	float absolute_angle;//��ǰ������н� ����ʱ���0��360����0,360��  �������ӵľ���ʽ�Ƕ�
	float angle_sum;  //    polar_angle���֣�������������       �� �����ӵ�����ʽ�Ƕ�
	float angle_gap;  //������Ŀ���ٶȽǶȵĲ�ֵ��Ҳ����Ҫת�ĽǶ�
	float v_angle; //Ŀ����ٶȵĽǶ�
	float Original_angle_encode;//��������λʱ�ı�������ֵ
} wheel_Struct;

//typedef struct
//{
//	float x;//xλ��
//	float y;//yλ��
//	float Angular_velocity;//������ԭ����ת���ٶ�
//} coordinitioate_Struct;//����ṹ��


void gain_absolute_angle(wheel_Struct *wheel_v,int id);
void gain_gap_angle(wheel_Struct *wheel_v);
void Robot_Wheel_Control(void);
void Robot_Wheel_Control_3508(int8_t number);
void Robot_Wheel_speed_Control_3508(float Speed,int8_t number);
void Robot_Wheel_speed_Control_VESC(void);
#endif
