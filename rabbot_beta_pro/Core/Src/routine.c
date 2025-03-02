#include "math.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Routine.h"
#include "main.h"
#include "M3508.h"
#include "handkey.h"
#include "motion_control.h"
/*************************/

// 角度制转换为弧度制系数
#define CHANGE_TO_RADIAN (0.01745329251994f)
// 弧度制转换为角度制系数
#define CHANGE_TO_ANGLE (57.29577951308232f)
#define PI (3.1415926)
extern float Body_x, Body_y, World_x, World_y;//解算的世界坐标
float Angle_Aim = 0;    // 目标角度值，中间变量
float Angle_offset = 0; // 剩余要转的角度，中间变量
float Line_angle = 0;   // 线坐标系与世界坐标系夹角
float Angle_dis = 0;    // 角度转动距离，中间变量
extern float vx_set,vy_set,w_set;
/*************************/

/*************************/
coordinate_Struct World_Coordinate_system_NowPos;    // 世界坐标系下当前位置
coordinate_Struct World_Coordinate_system_TargetVel; // 世界坐标系下目标速度
coordinate_Struct Robot_Coordinate_system_TargetVel; // 车身坐标系下目标速度
coordinate_Struct Robot_Coordinate_system_Vel;       // 路径计算出的车身速度，用于车轮速度分配
coordinate_Struct Zero_Point = {0, 0};               // 世界坐标系零点

ROUTINE_STU Routine_Status; // 路径结构体
Gyro_Data Gyro_Rotine_Data;
/*************************/
static float ABS(float a)
{
    return a > 0 ? a : -a;
}

PID_Struct Line_AdjustPID; // 直线法向PId调节
PID_Struct Line_StopPID;   // 直线停止区切向PId调节
PID_Struct Line_AnglePID;  // 直线角度PId调节
PID_Struct Cir_AdjustPID;  // 圆弧法向PId调节
PID_Struct Cir_StopPID;    // 圆弧停止区切向PId调节
PID_Struct Cir_AnglePID;   // 圆弧角度PId调节
PID_Struct Keep_X_PID;     // 保持x方向位置
PID_Struct Keep_Y_PID;     // 保持y方向位置
PID_Struct Keep_W_PID;     // 保持角度
/*********************************************************************************
 * @name 	CalculateLineAngle
 * @brief	计算两点矢量方向角度
 * @param	pointStart:起始点：
 * @param	pointEnd:终止点;
 * @retval	两点矢量方向角度 -180~180
 *********************************************************************************/
float CalculateLineAngle(coordinate_Struct pointStart, coordinate_Struct pointEnd)
{
    float a = 0.0f;
    float b = 0.0f;

    a = pointEnd.y - pointStart.y;
    b = pointEnd.x - pointStart.x;
    // atan2f范围可以包含-180到180
    return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
 * @name 	CalculatePoint2Pointdistance
 * @brief	计算点到点的距离
 * @param	point1 起始点
 * @param	point2 结束点
 * @retval   返回两点之间的距离
 *********************************************************************************/
float CalculatePoint2Pointdistance(coordinate_Struct point1, coordinate_Struct point2)
{
    float dis;
    dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
    return dis;
}

/**
 * @brief: 坐标系旋转变换
 * @param  *Coordinitioate_To_Convert 要转换的坐标系
 * @param  angle 坐标系夹角，逆时针旋转为正
 * @retval: Coordinitioate_Result 转换后的坐标系
 */
coordinate_Struct coordinate_transformation(coordinate_Struct *Coordinitioate_To_Convert, coordinate_Struct *CoordinateSystem, float angle)
{
    double rad = angle * CHANGE_TO_RADIAN;
    coordinate_Struct Coordinitioate_Result;
    Coordinitioate_Result.y = +(Coordinitioate_To_Convert->y - CoordinateSystem->y) * cos(rad) - (Coordinitioate_To_Convert->x - CoordinateSystem->x) * sin(rad);
    Coordinitioate_Result.x = +(Coordinitioate_To_Convert->y - CoordinateSystem->y) * sin(rad) + (Coordinitioate_To_Convert->x - CoordinateSystem->x) * cos(rad);
    Coordinitioate_Result.w = Coordinitioate_To_Convert->w;
    return Coordinitioate_Result;
}

float fitting_function(float x)
{
    float v_rate;
    v_rate = -2.9138f * x * x * x * x + 8.042f * x * x * x - 7.9895f * x * x + 3.7571f * x + 0.1f;
    return v_rate;
}

float fitting_function2(float x)
{
    float v_rate;
    x = 1 - x;
    v_rate = -2.9138f * x * x * x * x + 8.042f * x * x * x - 7.9895f * x * x + 3.7571f * x + 0.1f;
    return v_rate;
}

static void PID_Clear(PID_Struct *PID)
{
    PID->Output = 0;
    PID->Last_Err = 0;
    PID->ITerm = 0;
}

static void PId_Calculate(PID_Struct *PId, float measure, float target)
{

    PId->Measure = measure;
    PId->target = target;
    PId->Err = PId->target - PId->Measure;

    if (ABS(PId->Err) < PId->DeadBand)
		{
        PId->Output = 0;
		    PId->ITerm = 0;
		}
    else
    {
        PId->Output = PId->KP * PId->Err  + PId->KI * PId->ITerm + PId->KD * (PId->Err - PId->Last_Err) + (PId->Err > 0 ? PId->Forward : -PId->Forward);
			  if(ABS(PId->Err) > 1){PId->ITerm += PId->Err;}
        if (PId->ITerm > 30000)
            PId->ITerm = 30000;
        else if (PId->ITerm < -30000)
            PId->ITerm = -30000;
        PId->Last_Err = PId->Err;
    }
}

float Robot_Control_Line(ROUTINE_STU *Status)
{
    

    /* 0. 计算初始参数，if里面一段路径只运行一次 */
    if (Status->Flag.NewState == ENABLE)
    {
        Status->Flag.NewState = DISABLE;

        /* 线坐标系与世界坐标系夹角 */
        Line_angle = CalculateLineAngle(Status->CoordinateSystem.Start_Position, Status->CoordinateSystem.Target_Position);

        /* 计算线坐标系下的目标位置 */
        Status->CoordinateSystem.Line_TargetPos = coordinate_transformation(&Status->CoordinateSystem.Target_Position, &Status->CoordinateSystem.Start_Position, Line_angle);

        /* plus 计算到目标点的距离*/
        Status->Parameter.distance = Status->CoordinateSystem.Line_TargetPos.x;

        /* 计算减速区大小 */
        Status->Parameter.Slow_length = Status->Parameter.speed_down_rate * Status->Parameter.distance;
        /* 计算加速区大小 */
        Status->Parameter.Speedup_Length = Status->Parameter.speed_up_rate * Status->Parameter.distance;

        /* 0.2 计算角速度的加速区减速区，用于控制角度均匀变化 */
        /* 计算减速区大小 */
        Status->Parameter.Slow_angle = Status->Parameter.wspeed_down_rate * Status->Parameter.angle_rotate_Sum;
        /* 计算加速区大小 */
        Status->Parameter.Speedup_angle = Status->Parameter.wspeed_up_rate * Status->Parameter.angle_rotate_Sum;

        /* 0.3 计算目标角度 */
        /* 记录路径初始角度，当前路径初始角度为上一段路径的目标角度 */
        Status->CoordinateSystem.Start_Position.w = Status->CoordinateSystem.Target_Position.w;

        /* 计算当前路径目标角度，目标角度为初始角度+要转的角度 */
        Angle_Aim = Status->CoordinateSystem.Target_Position.w + Status->Parameter.angle_rotate_Sum;

        /* 更新下一段路径的初始角度 */
        Status->CoordinateSystem.Target_Position.w = Angle_Aim;
//				Status->Parameter.angle_rotate_last_Sum = Status->Parameter.angle_rotate_Sum;

        /* 0.4 清PId避免上一段路径的误差积累 */
        PID_Clear(&Line_AdjustPID);
        PID_Clear(&Line_StopPID);
        PID_Clear(&Line_AnglePID);
    }

    /* 更新世界坐标系下当前机器人的坐标 */
    World_Coordinate_system_NowPos.x = World_x;
    World_Coordinate_system_NowPos.y = World_y;

    /* 更新线坐标系下当前机器人的坐标 */
    Status->CoordinateSystem.Line_NowPos = coordinate_transformation(&World_Coordinate_system_NowPos, &Status->CoordinateSystem.Start_Position, Line_angle);

    /* 1. 计算期望角速度 */
    Angle_offset = ABS(Angle_Aim - Gyro_Rotine_Data.angle_yaw);

    Angle_dis = ABS(Gyro_Rotine_Data.angle_yaw - Status->CoordinateSystem.Start_Position.w);
    /* 1.1 当前角度在加速区内，计算目标角速度 */
    if (Angle_dis < Status->Parameter.Speedup_angle)
    {
        Robot_Coordinate_system_Vel.w =
            Status->Parameter.Start_w_Speed +                                      // 初速度
            fitting_function(Angle_dis / Status->Parameter.Speedup_angle) *        // 加速比例
                (Status->Parameter.Max_w_Speed - Status->Parameter.Start_w_Speed); // 速度变化量
    }
    /* 1.2 如果不在加速区内，并且在减速区外，角速度为最大角速度 */
    else if (Angle_dis >= Status->Parameter.Speedup_angle && Angle_offset > Status->Parameter.Slow_angle)
    {
        Robot_Coordinate_system_Vel.w = Status->Parameter.Max_w_Speed;
    }
    /* 1.3 如果在减速区内停止区外 */
    else if (Angle_offset <= Status->Parameter.Slow_angle && Angle_offset > Status->Parameter.Stop_angle)
    {
        Robot_Coordinate_system_Vel.w =
            Status->Parameter.Max_w_Speed -                                 // 最大速度
            fitting_function(Angle_offset / Status->Parameter.Slow_angle) * // 减速比例
                (Status->Parameter.Max_w_Speed);                            // 速度变化量
    }
    /* 1.4末尾速度为零，则需要在直线结尾精确定位 */
//    else if (Angle_offset < Status->Parameter.Stop_angle) //(fabs(Status->Parameter.End_w_Speed) < 0.001f && Status->Parameter.Stop_angle != 0)
//    {
//        PId_Calculate(&Line_AnglePID, Angle_offset, 0);
//        Robot_Coordinate_system_Vel.w = Line_AnglePID.Output;
//    }

    /* 1.5 顺时针旋转则速度取反 */
    if (Status->Parameter.angle_rotate_Sum < 0)
    {
        Robot_Coordinate_system_Vel.w *= -1;
    }

    /* 1.6 判断路径是否走完 */
    if (Angle_offset < 0.5f)
    {
        Status->Parameter.angle_rotate_Sum = 0; // 清零，以后不再运行自转部分了
    }

    /* 2. 线坐标系下XY期望速度计算 */

    /* 2.1 切向速度计算 */

    /* 2.1.1如果在加速区 */
    if ((Status->CoordinateSystem.Line_NowPos.x) < Status->Parameter.Speedup_Length)
    {
        Status->CoordinateSystem.Line_TargetVel.x = // 期望速度
            Status->Parameter.Start_Speed +
            fitting_function(Status->CoordinateSystem.Line_NowPos.x / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
    }
    /* 2.1.2 如果在减速区外，切向速度为正常速度 */
    else if ((Status->CoordinateSystem.Line_NowPos.x) <= (Status->CoordinateSystem.Line_TargetPos.x - Status->Parameter.Slow_length))
    {
        Status->CoordinateSystem.Line_TargetVel.x = Status->Parameter.Max_Speed;
    }
    /* 2.1.3如果在减速区内 */
    else if ((Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x) > Status->Parameter.Stop_length && (Status->CoordinateSystem.Line_NowPos.x + Status->Parameter.Slow_length) >= Status->CoordinateSystem.Line_TargetPos.x)
    {
        Status->CoordinateSystem.Line_TargetVel.x = // 期望速度
            Status->Parameter.Max_Speed -
            fitting_function2((Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x) / Status->Parameter.Slow_length) *
                (Status->Parameter.Max_Speed);
    }
    /* 2.1.4末尾速度为零，则需要在直线结尾精确定位 */
//    else if (fabs(Status->CoordinateSystem.Target_Position.w) < 0.001f && Status->Parameter.Stop_length != 0)
//    {
//        PId_Calculate(&Line_StopPID, Status->CoordinateSystem.Line_NowPos.x, Status->CoordinateSystem.Line_TargetPos.x);
//        Status->CoordinateSystem.Line_TargetVel.x = Line_StopPID.Output;
//    }
//    /* 2.1.5末尾速度不为零，速度等于末端速度 */
//    else if (fabs(Status->Parameter.Stop_length) < 0.001F)
//    {
//        Status->CoordinateSystem.Line_TargetVel.x = Status->CoordinateSystem.Line_TargetVel.x;
//    }

    /* 2.2 法向速度计算 */
    if (ABS(Status->CoordinateSystem.Line_NowPos.x - Status->CoordinateSystem.Line_TargetPos.x) > 10) // 快到直线结尾时不调节法向速度，避免出现轮子抖动
    {
        PId_Calculate(&Line_AdjustPID, Status->CoordinateSystem.Line_NowPos.y, 0);

        /* 前馈调节 */
        if (ABS(Status->CoordinateSystem.Line_NowPos.y - Status->CoordinateSystem.Line_TargetPos.y) > 1.5f)
        {
            if (Line_AdjustPID.Output > 0)
                Line_AdjustPID.Output += 3;
            else if (Line_AdjustPID.Output < 0)
                Line_AdjustPID.Output -= 3;
        }

        Status->CoordinateSystem.Line_TargetVel.y = Line_AdjustPID.Output;
    }
    else
        Status->CoordinateSystem.Line_TargetVel.y = 0;

//    /* 当线坐标系y方向偏差过大时减小x方向速度(待测试) */
//    if (ABS(Status->CoordinateSystem.Line_TargetVel.y) > 1000)
//    {
//        Status->CoordinateSystem.Line_TargetVel.x *= 0.5f;
//    }
    /* 3. 将线坐标系下的速度转化到车身坐标系*/
    /* 将线坐标系下的速度转换成世界坐标系下速度 */
    World_Coordinate_system_TargetVel = coordinate_transformation(&Status->CoordinateSystem.Line_TargetVel, &Status->CoordinateSystem.Zero_Point, -Line_angle);

    /* 将世界坐标系下速度转换成车身坐标系下速度 */
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Gyro_Rotine_Data.angle_yaw);

    /* 自转结束后保持车身角度 */
    if (Angle_offset < 0.5f || Status->Parameter.angle_rotate_Sum == 0)
    {
        /* 保持车身角度 */
        PId_Calculate(&Line_AnglePID, Gyro_Rotine_Data.angle_yaw, Status->CoordinateSystem.Target_Position.w);
        Robot_Coordinate_system_Vel.w = Line_AnglePID.Output;
    }

    /* 4. 判断路径是否走完 */
    if (Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x < 10)
    {
        Status->CoordinateSystem.Target_Position.w = Angle_Aim;
        Status->Flag.Work_Start = DISABLE;
    }

    return (Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x);
}

// 老圆弧路径
float Robot_Control_Circle(ROUTINE_STU *Status)
{
    static float Slope_angle;                            // 极坐标速度坐标系与世界坐标系夹角
    static float Angle_last;                             // 机器人在目标线坐标系下走过的角度、上一时刻的角度
    static float angle_temp_1, angle_temp_2, angle_temp; // 一些计算角度的临时变量
    static float arc_remain;                             // 与目标点的距离差值

    /* 0 计算初始参数，if里面一段路径只运行一次 */
    if (Status->Flag.NewState == ENABLE)
    {
        Status->Flag.NewState = DISABLE;

        // 更新起始角度
        Status->CoordinateSystem.Start_Position.w = Status->CoordinateSystem.Target_Position.w;

        Status->Parameter.distance = ABS(Status->Parameter.Target_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R;

        // 计算减速区大小
        Status->Parameter.Slow_length = Status->Parameter.speed_down_rate * Status->Parameter.distance;
        // 计算加速区大小
        Status->Parameter.Speedup_Length = Status->Parameter.speed_up_rate * Status->Parameter.distance;
        // PId误差归零
        PID_Clear(&Cir_AdjustPID);
        PID_Clear(&Cir_StopPID);
        PID_Clear(&Cir_AnglePID);

        // 计算路径起始点在极坐标系下的坐标
        Status->CoordinateSystem.Polar_NowPos.Rho = CalculatePoint2Pointdistance(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);
        Status->CoordinateSystem.Polar_NowPos.Theta = CalculateLineAngle(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);

        // 更新Angle_last
        Angle_last = Status->CoordinateSystem.Polar_NowPos.Theta;
        Status->Parameter.Now_Angle_Sum = 0;
    }

    /* 1 计算当前机器人在极坐标系下坐标 */
    // 更新世界坐标系坐标
    World_Coordinate_system_NowPos.x = World_x;
    World_Coordinate_system_NowPos.y = World_y;

    // 更新极坐标
    Status->CoordinateSystem.Polar_NowPos.Rho = CalculatePoint2Pointdistance(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);
    Status->CoordinateSystem.Polar_NowPos.Theta = CalculateLineAngle(Status->CoordinateSystem.World_HeartPos, World_Coordinate_system_NowPos);

    /* 2 计算机器人已经走过的圆心角 */
    // 首先计算与目标点的角度差值，计算方法借鉴了M3508无刷电机计算角度的方法
    if (Angle_last < Status->CoordinateSystem.Polar_NowPos.Theta) // 一种情况
    {
        angle_temp_1 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last; // 逆时针
        angle_temp_2 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last - 2 * PI;
    }
    else
    {
        angle_temp_1 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last; // 顺时针
        angle_temp_2 = 2 * PI + Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last;
    }
    // 无论顺时针转还是逆时针转，都是取小的那个角度
    angle_temp = (ABS(angle_temp_1)) > (ABS(angle_temp_2)) ? angle_temp_2 : angle_temp_1;
    Status->Parameter.Now_Angle_Sum += angle_temp;
    Angle_last = Status->CoordinateSystem.Polar_NowPos.Theta; // 当前所在位置极角变为last
    // 再计算与目标点的距离差值
    arc_remain = ABS((Status->Parameter.Target_Angle_Sum - Status->Parameter.Now_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R);

    /* 3 计算极坐标速度坐标系下的切向速度 */
    // 如果在加速区
    if (ABS(Status->Parameter.Now_Angle_Sum) * Status->Parameter.R < Status->Parameter.Speedup_Length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // 期望速度
            Status->Parameter.Start_Speed +
            fitting_function(Status->Parameter.Now_Angle_Sum * Status->Parameter.R / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
    }
    // 如果没在减速区外，切向速度为最大速度
    else if (arc_remain >= Status->Parameter.Slow_length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = Status->Parameter.Max_Speed;
    }
    // 如果在减速区内，停止区外
    else if (arc_remain >= Status->Parameter.Stop_length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // 期望速度
            Status->Parameter.Max_Speed -
            fitting_function(arc_remain / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed);
    }
    // 末尾速度为零，则需要在直线结尾精确定位
    else if (fabs(Status->CoordinateSystem.Target_Position.w) < 0.001f && Status->Parameter.Stop_length != 0)
    {
        PId_Calculate(&Cir_StopPID, arc_remain, 0);
        Status->CoordinateSystem.Polar_TargetVel.x = -Cir_StopPID.Output;
    }
    else if (fabs(Status->Parameter.Stop_length) < 0.001F)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = Status->CoordinateSystem.Polar_TargetVel.x;
    }

//    // 顺时针旋转则切向速度取反
//    if (Status->Flag.circle_tpye == Robot_LineType_Cycle_Clockwise)
//    {
//        Status->CoordinateSystem.Polar_TargetVel.x *= -1;
//    }

    /* 4 计算极坐标速度坐标系下的法向向速度 */
    // 法向速度PID调节
    PId_Calculate(&Cir_AdjustPID, Status->CoordinateSystem.Polar_NowPos.Rho, Status->Parameter.R);
    Status->CoordinateSystem.Polar_TargetVel.y = -Cir_AdjustPID.Output;

    /* 5 计算角速度 */
    // 如果沿切线：更新目标角度
    if (Status->Flag.yanqiexian)
    {
        Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Now_Angle_Sum;
    }

    // PID保持车身角度
    PId_Calculate(&Cir_AnglePID, Gyro_Rotine_Data.angle_yaw, Status->CoordinateSystem.Target_Position.w);
    Robot_Coordinate_system_Vel.w = Cir_AnglePID.Output;

    /* 当极坐标速度坐标系法向方向偏差过大时减小切向方向速度(待测试) */
    if (ABS(Status->CoordinateSystem.Polar_TargetVel.y) > 1000)
    {
        Status->CoordinateSystem.Polar_TargetVel.x *= 0.5f;
    }
    /* 6 将极坐标速度坐标系下的速度转化到车身坐标系*/
    // 计算极坐标速度坐标系与世界坐标系夹角
    Slope_angle = Status->CoordinateSystem.Polar_NowPos.Theta - 90.0f;
    /* 将极坐标速度坐标系的速度转换成世界坐标系下速度 */
    World_Coordinate_system_TargetVel = coordinate_transformation(&Status->CoordinateSystem.Polar_TargetVel, &Status->CoordinateSystem.Zero_Point, -Slope_angle);

    /* 将世界坐标系下速度转换成车身坐标系下速度 */
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Gyro_Rotine_Data.angle_yaw);

    // 判断路径是否走完
    if (arc_remain < 5)
    {
        if (Status->Flag.yanqiexian)
            Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Target_Angle_Sum;
        else
            Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w;
        Status->Flag.Work_Start = DISABLE;
    }

    return (arc_remain);
}
// 直线（目标x，目标y，要转的角度，V初，V末，加速区比例，减速区比例，w最大，V最大，停止区长度）
void line(float Target_x, float Target_y, float rotate_Sum, int Start_Speed, int End_Speed, float up_rate, float down_rate, int Max_w, int speed_max, float Stop_length)
{
    Routine_Status.Flag.NewState = ENABLE;
    Routine_Status.Flag.Work_Start = ENABLE;

    Routine_Status.CoordinateSystem.Start_Position.x = World_x; // 开始位置
    Routine_Status.CoordinateSystem.Start_Position.y = World_y;
    Routine_Status.CoordinateSystem.Target_Position.x = Target_x; // 目标位置
    Routine_Status.CoordinateSystem.Target_Position.y = Target_y;

    /* 速度 */
    Routine_Status.Parameter.Start_Speed = Start_Speed; // 启动速度，不要为0
    Routine_Status.Parameter.Max_Speed = speed_max;     // 最大速度
    Routine_Status.Parameter.End_Speed = End_Speed;     // 路径末端速度

    Routine_Status.Parameter.speed_up_rate = up_rate;     // 加速区比例
    Routine_Status.Parameter.speed_down_rate = down_rate; // 减速区比例
    Routine_Status.Parameter.Stop_length = Stop_length;   // 停止区长度

    /* 角度 */
    Routine_Status.Parameter.angle_rotate_Sum = rotate_Sum; // 自转角度，逆时针为正

    Routine_Status.Parameter.Start_w_Speed = 40;  // 启动角速度
    Routine_Status.Parameter.Max_w_Speed = Max_w; // 最大角速度
    Routine_Status.Parameter.End_w_Speed = 0;     // 路径末端角速度

		Routine_Status.Parameter.wspeed_up_rate = 0.2;
		Routine_Status.Parameter.wspeed_down_rate = 0.2;
    Routine_Status.Parameter.Stop_angle = 0.5;                     // 停止区长度(角度)
    while (Routine_Status.Flag.Work_Start)
    {
        Robot_Control_Line(&Routine_Status);
        motion_resolve(Robot_Coordinate_system_Vel.x,Robot_Coordinate_system_Vel.y,Robot_Coordinate_system_Vel.w);
        osDelay(2);
    }
}

// 圆弧（目标x，目标y，圆心角(正负)，V初，V末，V最大，是否沿切，加速区比例，减速区比例）
void circle(float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, FunctionalState yanqie, float up_rate, float down_rate)
{
    Routine_Status.Flag.Work_Start = ENABLE;
    Routine_Status.Flag.NewState = ENABLE;

    float gama, R, ygxl, d, c_x, c_y;
    float rad = angle * CHANGE_TO_RADIAN;
    d = sqrt((Target_x - World_x) * (Target_x - World_x) + (Target_y - World_y) * (Target_y - World_y));
    R = d / 2.0f / sinf(rad / 2.0f);
    if (angle >= 0) // 顺时针
    {
        if ((Target_y - World_y) >= 0)
        {
            gama = acosf((Target_x - World_x) / d);
        }
        else if ((Target_y - World_y) < 0)
        {
            gama = acosf((World_x - Target_x) / d) + PI / 2;
        }
        ygxl = gama - (PI / 2 - rad / 2);

        c_x = World_x + R * cos(ygxl);
        c_y = World_y + R * sin(ygxl);
        Routine_Status.Flag.circle_tpye = Robot_LineType_Cycle_Clockwise; // 顺时针
    }
    else if (angle < 0) // 逆时针
    {
        if ((Target_y - World_y) >= 0)
        {
            gama = acosf((Target_x - World_x) / d);
        }
        else if ((Target_y - World_y) < 0)
        {
            gama = acosf((World_x - Target_x) / d) + PI / 2;
        }
        ygxl = gama + (PI / 2 - rad / 2);

        c_x = World_x + R * cos(ygxl);
        c_y = World_y + R * sin(ygxl);
        Routine_Status.Flag.circle_tpye = Robot_LineType_Cycle_Anticlockwise; // 逆时针
    }

    Routine_Status.CoordinateSystem.Start_Position.x = Routine_Status.CoordinateSystem.Target_Position.x; // 开始位置
    Routine_Status.CoordinateSystem.Start_Position.y = Routine_Status.CoordinateSystem.Target_Position.y;
    Routine_Status.CoordinateSystem.Target_Position.x = Target_x; // 目标位置
    Routine_Status.CoordinateSystem.Target_Position.y = Target_y;

    Routine_Status.Parameter.Start_Speed = start_v; // 开始速度
    Routine_Status.Parameter.Max_Speed = v_max;     // 最大速度
    Routine_Status.Parameter.End_Speed = end_v;     // 路径末端速度

    Routine_Status.Parameter.speed_up_rate = up_rate;     // 加速区间比例
    Routine_Status.Parameter.speed_down_rate = down_rate; // 减速区间比例
    Routine_Status.Parameter.Stop_length = 20;            // 停止区长度
    // Routine_Status.Flag.circle_tpye = type;                       // 选择顺时针还是逆时针
    Routine_Status.Flag.yanqiexian = yanqie;                // 是否沿切线
    Routine_Status.Parameter.Target_Angle_Sum = angle;      // 圆心角
    Routine_Status.Parameter.R = R;                         // 半径
    Routine_Status.CoordinateSystem.World_HeartPos.x = c_x; // 圆心
    Routine_Status.CoordinateSystem.World_HeartPos.y = c_y;

    while (Routine_Status.Flag.Work_Start)
    {
        Robot_Control_Circle(&Routine_Status);
		motion_resolve(vx_set,vy_set,w_set);
        HAL_Delay(2);
    }
}

void Keep_Robot_Position(float Angle, float X, float Y)
{
    // 保持车身角度
	Keep_X_PID.KP = 10*ABS(Keep_X_PID.Err*Keep_X_PID.Err*Keep_X_PID.Err)/(ABS(Keep_X_PID.Err*Keep_X_PID.Err)+ABS(Keep_Y_PID.Err*Keep_Y_PID.Err));
	Keep_Y_PID.KP = 10*ABS(Keep_Y_PID.Err*Keep_Y_PID.Err*Keep_Y_PID.Err)/(ABS(Keep_X_PID.Err*Keep_X_PID.Err)+ABS(Keep_Y_PID.Err*Keep_Y_PID.Err));
	Keep_W_PID.KP = 10*ABS(Keep_W_PID.Err*Keep_W_PID.Err*Keep_W_PID.Err)/(ABS(Keep_X_PID.Err*Keep_X_PID.Err)+ABS(Keep_Y_PID.Err*Keep_Y_PID.Err)+ABS(Keep_X_PID.Err));

	PId_Calculate(&Keep_W_PID, Gyro_Rotine_Data.angle_yaw, Angle);
    if (Keep_W_PID.Output > 5000)
        Keep_W_PID.Output = 5000;
    if (Keep_W_PID.Output < -5000)
        Keep_W_PID.Output = -5000;
    World_Coordinate_system_TargetVel.w = Keep_W_PID.Output;

    PId_Calculate(&Keep_X_PID, World_x, X);
    if (Keep_X_PID.Output > 5000)
        Keep_X_PID.Output = 5000;
    if (Keep_X_PID.Output < -5000)
        Keep_X_PID.Output = -5000;
    World_Coordinate_system_TargetVel.x = Keep_X_PID.Output;

    PId_Calculate(&Keep_Y_PID, World_y, Y);
    if (Keep_Y_PID.Output > 5000)
        Keep_Y_PID.Output = 5000;
    if (Keep_Y_PID.Output < -5000)
        Keep_Y_PID.Output = -5000;
    World_Coordinate_system_TargetVel.y = Keep_Y_PID.Output;
		
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Zero_Point, Gyro_Rotine_Data.angle_yaw);
}

void PID_Init(PID_Struct *pid,float p,float i,float d,float Dead_Band,float forward)
{
   pid->KP=p;
	 pid->KI=i;
	 pid->KD=d;
	 pid->DeadBand=Dead_Band;
	 pid->Forward=forward;
	
}

void PID_Path_Init(void)
{
    PID_Init(&Keep_W_PID, 5, 0.01, 0, 0.5, 30);//300,250,250
    PID_Init(&Keep_X_PID, 5, 0.01, 0, 1, 30);
    PID_Init(&Keep_Y_PID, 5, 0.01, 0, 1, 30);

    PID_Init(&Line_AnglePID, 5, 0.001, 0, 0.1, 250);
    PID_Init(&Line_StopPID, 5, 0.01, 0, 1, 250);
    PID_Init(&Line_AdjustPID, 5, 0.01, 0, 1, 0);
}
