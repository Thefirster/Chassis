#ifndef _YUNTAI_H
#define _YUNTAI_H

#include "stdint.h"

#define YAW_MOTOR_ID (1)
#define PITCH_MOTOR_ID (0)

typedef struct
{
    float yaw;
    float pitch;
} Yuntai;
typedef struct
{
    float x;
    float y;
    float z;
} VisionData;//Ω” ’£¨‘∆Ã®

void yuntai_set_target_pitch(float x);
void yuntai_set_target_yaw(float x);

void Usart3_Start(void);
void vision_data_process(void);

void yuntai_control(void);
void yuntai_anotc(void);
void mode_change(void);
void yuntai_init(void);
void yuntai_start_path(uint8_t id, float end_pos, float T);
void yuntai_manual(uint8_t id);
float radar_run(int id_zhu);
void yuntai_shoot_select(float yaw_target,float pitch_target);
#endif
