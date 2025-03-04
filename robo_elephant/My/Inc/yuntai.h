#ifndef __YUNTAI_H
#define __YUNTAI_H

#include "stm32f4xx.h"
#include "M3508.h"
#include "DM_J4310.h"
#include "VESC.h"
#include "myusart.h"
#include "vision.h"
#include "JY_ME02.h"

#define ONE_ROUND_DM -14.9

//id号
enum eid_yuntai{
	level_eid = 0,
	pitch_eid = 2,
	m2006_eid = 1,
	
//	level_encoder = 0,
	pitch_encoder = 1,
};
//2006限速
enum m2006_max_min{
//	SpeedMaxLevel = 1000,
	SpeedMaxPitch = 1000,
	
//	level_max = 320,
//	level_min = 260,
	
	pitch_max = 285,
	pitch_min = 230,
};

//DM控制数据汇总
typedef struct{
	float KP; 
  float KD;
  float send_pos;
  float send_vel;
  float send_pos_cnt;
  float tff;
}DM_Data_Init;
//M3508数据汇总
typedef struct{
	float level_send_angle;  
  float pitch_send_angle;
  float anglesum_level_max;
  float anglesum_level_min;
  float anglesum_pitch_max;
  float anglesum_pitch_min;
}M3508_Data_Init;

typedef struct{
	float M3508_level;
	float M3508_pitch;
	float RPM;
	float DM_pos;
	float DM_vel;
	float DM_cnt;
	float anglesum_pitch_max;
  float anglesum_pitch_min;
	float M2006_angle_send;
	
	uint8_t begin_flag;
	uint8_t mode_flag;
	uint8_t DM_flag;
	uint8_t RPM_begin;
	uint8_t RPM_flag;
}yuntai_control;

extern yuntai_control yuntai;
//extern M3508_Data_Init M3508_Data;
//extern DM_Data_Init DM;

void Yuntai_Rotor_Control(void);
void Vision_Control(void);
volatile void limit_a_b(volatile float *control_num, volatile float max_num, volatile float min_num);
void Anglesum_Max_Min_Init(void);
void motor_control(void);

#endif
