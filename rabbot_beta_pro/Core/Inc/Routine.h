#ifndef _ROUTINE_H_
#define _ROUTINE_H_

#include "stm32f4xx.h"
typedef struct
{
    float x;         // xλ��/�ٶ�
    float y;         // yλ��/�ٶ�
    float w;         // ������ԭ����ת�Ƕ�/���ٶ�
} coordinate_Struct; // ����ṹ��

typedef struct
{
    float Rho;
    float Theta;
} Polar_coordinates_Struct; // ������ṹ��

typedef enum
{
    Robot_LineType_Cycle_Clockwise,    // ˳ʱ��Բ��
    Robot_LineType_Cycle_Anticlockwise // ��ʱ��Բ��
} CIRCLE_TYPE_ENU;

typedef struct
{

    struct // ״̬�ṹ��
    {
        FunctionalState NewState;    // ��־λ�����ڼ���һЩ��ʼ����
        FunctionalState Work_Start;  // ��־λ�������ж�·���Ƿ����
        CIRCLE_TYPE_ENU circle_tpye; // ѡ��˳ʱ��Բ������ʱ��Բ��
        FunctionalState yanqiexian;  // �Ƿ������ߣ�enableΪ������
    } Flag;

    struct // ����ϵ�ṹ��
    {
        coordinate_Struct Zero_Point;      // ��������ϵ��㣨0,0��
        coordinate_Struct Start_Position;  // ����λ��(��������ϵ)
        coordinate_Struct Target_Position; // Ŀ��λ��(��������ϵ)

        coordinate_Struct Line_NowPos;         // ������ϵ�µ�ǰλ��
        coordinate_Struct Line_TargetPos;      // ������ϵ��Ŀ��λ��
        coordinate_Struct Line_TargetVel;      // ������ϵ��Ŀ���ٶ�
        coordinate_Struct World_HeartPos;      // Բ��
        Polar_coordinates_Struct Polar_NowPos; // ������ϵ�»����˵�ǰλ�ã���ǰλ����Բ������Ϊ���ᣩ
        coordinate_Struct Polar_TargetVel;     // ������ϵ�»������ٶ�
    } CoordinateSystem;
    struct // �����ṹ��
    {
        float Slow_length;               // ������
        float Stop_length;               // ֹͣ��
        float Speedup_Length;            // ������
        float Start_Speed;               // ��ʼ�ٶ�
        float Max_Speed;                 // ����ٶ�
        float End_Speed;                 // ĩ���ٶ�
        float Slow_accelerated_speed;    // ���ټ��ٶ�
        float Speedup_accelerated_speed; // ���ټ��ٶ�

        float Slow_angle;                  // ���ٽǶ���
        float Stop_angle;                  // ֹͣ�Ƕ���
        float Speedup_angle;               // ���ٽǶ���
        float Start_w_Speed;               // �������ٶ�
        float Max_w_Speed;                 // �����ٶ�
        float End_w_Speed;                 // ĩ�˽��ٶ�
        float Slow_accelerated_w_speed;    // ���ٽǼ��ٶ�
        float Speedup_accelerated_w_speed; // ���ٽǼ��ٶ�
        float angle_rotate_Sum;            // ��ǰ��ת�Ƕ�
			  float angle_rotate_last_Sum;            // ��ǰ��ת�Ƕ�

        float Target_Angle_Sum; // Ŀ���ۼ�Բ�Ľ�
        float Now_Angle_Sum;    // ��ǰ�ۼ�Բ�Ľ�
        float R;                // �뾶

        float speed_up_rate;    // �����������
        float speed_down_rate;  // �����������
        float wspeed_up_rate;   // ���ٶȼ����������
        float wspeed_down_rate; // ���ٶȼ����������
        float distance;         // ��Ŀ���ľ���
    } Parameter;

} ROUTINE_STU;
// ֱ�ߣ�Ŀ��x��Ŀ��y��Ҫת�ĽǶȣ�V����Vĩ��������������������������w���V���ֹͣ�����ȣ�
void line(float Target_x, float Target_y, float rotate_Sum, int Start_Speed, int End_Speed, float up_rate, float down_rate, int Max_w, int speed_max, float stop_length);
// Բ����Ŀ��x��Ŀ��y��Բ�Ľ�(����)��V����Vĩ��V����Ƿ����У�������������������������
void circle(float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, FunctionalState yanqie, float up_rate, float down_rate);
void Keep_Robot_Position(float Angle, float X, float Y);
void PID_Path_Init(void);
#endif

