#ifndef _ROUTINE_H_
#define _ROUTINE_H_

#include "stm32f4xx.h"
typedef struct
{
    float x;         // x位置/速度
    float y;         // y位置/速度
    float w;         // 绕坐标原点旋转角度/角速度
} coordinate_Struct; // 坐标结构体

typedef struct
{
    float Rho;
    float Theta;
} Polar_coordinates_Struct; // 极坐标结构体

typedef enum
{
    Robot_LineType_Cycle_Clockwise,    // 顺时针圆弧
    Robot_LineType_Cycle_Anticlockwise // 逆时针圆弧
} CIRCLE_TYPE_ENU;

typedef struct
{

    struct // 状态结构体
    {
        FunctionalState NewState;    // 标志位，用于计算一些初始变量
        FunctionalState Work_Start;  // 标志位，用于判断路径是否结束
        CIRCLE_TYPE_ENU circle_tpye; // 选择顺时针圆弧、逆时针圆弧
        FunctionalState yanqiexian;  // 是否沿切线，enable为沿切线
    } Flag;

    struct // 坐标系结构体
    {
        coordinate_Struct Zero_Point;      // 世界坐标系零点（0,0）
        coordinate_Struct Start_Position;  // 启动位置(世界坐标系)
        coordinate_Struct Target_Position; // 目标位置(世界坐标系)

        coordinate_Struct Line_NowPos;         // 线坐标系下当前位置
        coordinate_Struct Line_TargetPos;      // 线坐标系下目标位置
        coordinate_Struct Line_TargetVel;      // 线坐标系下目标速度
        coordinate_Struct World_HeartPos;      // 圆心
        Polar_coordinates_Struct Polar_NowPos; // 极坐标系下机器人当前位置（当前位置与圆心连线为极轴）
        coordinate_Struct Polar_TargetVel;     // 极坐标系下机器人速度
    } CoordinateSystem;
    struct // 参数结构体
    {
        float Slow_length;               // 减速区
        float Stop_length;               // 停止区
        float Speedup_Length;            // 加速区
        float Start_Speed;               // 起始速度
        float Max_Speed;                 // 最大速度
        float End_Speed;                 // 末端速度
        float Slow_accelerated_speed;    // 减速加速度
        float Speedup_accelerated_speed; // 加速加速度

        float Slow_angle;                  // 减速角度区
        float Stop_angle;                  // 停止角度区
        float Speedup_angle;               // 加速角度区
        float Start_w_Speed;               // 启动角速度
        float Max_w_Speed;                 // 最大角速度
        float End_w_Speed;                 // 末端角速度
        float Slow_accelerated_w_speed;    // 减速角加速度
        float Speedup_accelerated_w_speed; // 加速角加速度
        float angle_rotate_Sum;            // 当前自转角度
			  float angle_rotate_last_Sum;            // 当前自转角度

        float Target_Angle_Sum; // 目标累加圆心角
        float Now_Angle_Sum;    // 当前累加圆心角
        float R;                // 半径

        float speed_up_rate;    // 加速区间比例
        float speed_down_rate;  // 减速区间比例
        float wspeed_up_rate;   // 角速度加速区间比例
        float wspeed_down_rate; // 角速度减速区间比例
        float distance;         // 到目标点的距离
    } Parameter;

} ROUTINE_STU;
// 直线（目标x，目标y，要转的角度，V初，V末，加速区比例，减速区比例，w最大，V最大，停止区长度）
void line(float Target_x, float Target_y, float rotate_Sum, int Start_Speed, int End_Speed, float up_rate, float down_rate, int Max_w, int speed_max, float stop_length);
// 圆弧（目标x，目标y，圆心角(正负)，V初，V末，V最大，是否沿切，加速区比例，减速区比例）
void circle(float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, FunctionalState yanqie, float up_rate, float down_rate);
void Keep_Robot_Position(float Angle, float X, float Y);
void PID_Path_Init(void);
#endif

