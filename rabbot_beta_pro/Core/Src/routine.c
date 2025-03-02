#include "math.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Routine.h"
#include "main.h"
#include "M3508.h"
#include "handkey.h"
#include "motion_control.h"
/*************************/

// �Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN (0.01745329251994f)
// ������ת��Ϊ�Ƕ���ϵ��
#define CHANGE_TO_ANGLE (57.29577951308232f)
#define PI (3.1415926)
extern float Body_x, Body_y, World_x, World_y;//�������������
float Angle_Aim = 0;    // Ŀ��Ƕ�ֵ���м����
float Angle_offset = 0; // ʣ��Ҫת�ĽǶȣ��м����
float Line_angle = 0;   // ������ϵ����������ϵ�н�
float Angle_dis = 0;    // �Ƕ�ת�����룬�м����
extern float vx_set,vy_set,w_set;
/*************************/

/*************************/
coordinate_Struct World_Coordinate_system_NowPos;    // ��������ϵ�µ�ǰλ��
coordinate_Struct World_Coordinate_system_TargetVel; // ��������ϵ��Ŀ���ٶ�
coordinate_Struct Robot_Coordinate_system_TargetVel; // ��������ϵ��Ŀ���ٶ�
coordinate_Struct Robot_Coordinate_system_Vel;       // ·��������ĳ����ٶȣ����ڳ����ٶȷ���
coordinate_Struct Zero_Point = {0, 0};               // ��������ϵ���

ROUTINE_STU Routine_Status; // ·���ṹ��
Gyro_Data Gyro_Rotine_Data;
/*************************/
static float ABS(float a)
{
    return a > 0 ? a : -a;
}

PID_Struct Line_AdjustPID; // ֱ�߷���PId����
PID_Struct Line_StopPID;   // ֱ��ֹͣ������PId����
PID_Struct Line_AnglePID;  // ֱ�߽Ƕ�PId����
PID_Struct Cir_AdjustPID;  // Բ������PId����
PID_Struct Cir_StopPID;    // Բ��ֹͣ������PId����
PID_Struct Cir_AnglePID;   // Բ���Ƕ�PId����
PID_Struct Keep_X_PID;     // ����x����λ��
PID_Struct Keep_Y_PID;     // ����y����λ��
PID_Struct Keep_W_PID;     // ���ֽǶ�
/*********************************************************************************
 * @name 	CalculateLineAngle
 * @brief	��������ʸ������Ƕ�
 * @param	pointStart:��ʼ�㣺
 * @param	pointEnd:��ֹ��;
 * @retval	����ʸ������Ƕ� -180~180
 *********************************************************************************/
float CalculateLineAngle(coordinate_Struct pointStart, coordinate_Struct pointEnd)
{
    float a = 0.0f;
    float b = 0.0f;

    a = pointEnd.y - pointStart.y;
    b = pointEnd.x - pointStart.x;
    // atan2f��Χ���԰���-180��180
    return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
 * @name 	CalculatePoint2Pointdistance
 * @brief	����㵽��ľ���
 * @param	point1 ��ʼ��
 * @param	point2 ������
 * @retval   ��������֮��ľ���
 *********************************************************************************/
float CalculatePoint2Pointdistance(coordinate_Struct point1, coordinate_Struct point2)
{
    float dis;
    dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
    return dis;
}

/**
 * @brief: ����ϵ��ת�任
 * @param  *Coordinitioate_To_Convert Ҫת��������ϵ
 * @param  angle ����ϵ�нǣ���ʱ����תΪ��
 * @retval: Coordinitioate_Result ת���������ϵ
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
    

    /* 0. �����ʼ������if����һ��·��ֻ����һ�� */
    if (Status->Flag.NewState == ENABLE)
    {
        Status->Flag.NewState = DISABLE;

        /* ������ϵ����������ϵ�н� */
        Line_angle = CalculateLineAngle(Status->CoordinateSystem.Start_Position, Status->CoordinateSystem.Target_Position);

        /* ����������ϵ�µ�Ŀ��λ�� */
        Status->CoordinateSystem.Line_TargetPos = coordinate_transformation(&Status->CoordinateSystem.Target_Position, &Status->CoordinateSystem.Start_Position, Line_angle);

        /* plus ���㵽Ŀ���ľ���*/
        Status->Parameter.distance = Status->CoordinateSystem.Line_TargetPos.x;

        /* �����������С */
        Status->Parameter.Slow_length = Status->Parameter.speed_down_rate * Status->Parameter.distance;
        /* �����������С */
        Status->Parameter.Speedup_Length = Status->Parameter.speed_up_rate * Status->Parameter.distance;

        /* 0.2 ������ٶȵļ����������������ڿ��ƽǶȾ��ȱ仯 */
        /* �����������С */
        Status->Parameter.Slow_angle = Status->Parameter.wspeed_down_rate * Status->Parameter.angle_rotate_Sum;
        /* �����������С */
        Status->Parameter.Speedup_angle = Status->Parameter.wspeed_up_rate * Status->Parameter.angle_rotate_Sum;

        /* 0.3 ����Ŀ��Ƕ� */
        /* ��¼·����ʼ�Ƕȣ���ǰ·����ʼ�Ƕ�Ϊ��һ��·����Ŀ��Ƕ� */
        Status->CoordinateSystem.Start_Position.w = Status->CoordinateSystem.Target_Position.w;

        /* ���㵱ǰ·��Ŀ��Ƕȣ�Ŀ��Ƕ�Ϊ��ʼ�Ƕ�+Ҫת�ĽǶ� */
        Angle_Aim = Status->CoordinateSystem.Target_Position.w + Status->Parameter.angle_rotate_Sum;

        /* ������һ��·���ĳ�ʼ�Ƕ� */
        Status->CoordinateSystem.Target_Position.w = Angle_Aim;
//				Status->Parameter.angle_rotate_last_Sum = Status->Parameter.angle_rotate_Sum;

        /* 0.4 ��PId������һ��·���������� */
        PID_Clear(&Line_AdjustPID);
        PID_Clear(&Line_StopPID);
        PID_Clear(&Line_AnglePID);
    }

    /* ������������ϵ�µ�ǰ�����˵����� */
    World_Coordinate_system_NowPos.x = World_x;
    World_Coordinate_system_NowPos.y = World_y;

    /* ����������ϵ�µ�ǰ�����˵����� */
    Status->CoordinateSystem.Line_NowPos = coordinate_transformation(&World_Coordinate_system_NowPos, &Status->CoordinateSystem.Start_Position, Line_angle);

    /* 1. �����������ٶ� */
    Angle_offset = ABS(Angle_Aim - Gyro_Rotine_Data.angle_yaw);

    Angle_dis = ABS(Gyro_Rotine_Data.angle_yaw - Status->CoordinateSystem.Start_Position.w);
    /* 1.1 ��ǰ�Ƕ��ڼ������ڣ�����Ŀ����ٶ� */
    if (Angle_dis < Status->Parameter.Speedup_angle)
    {
        Robot_Coordinate_system_Vel.w =
            Status->Parameter.Start_w_Speed +                                      // ���ٶ�
            fitting_function(Angle_dis / Status->Parameter.Speedup_angle) *        // ���ٱ���
                (Status->Parameter.Max_w_Speed - Status->Parameter.Start_w_Speed); // �ٶȱ仯��
    }
    /* 1.2 ������ڼ������ڣ������ڼ������⣬���ٶ�Ϊ�����ٶ� */
    else if (Angle_dis >= Status->Parameter.Speedup_angle && Angle_offset > Status->Parameter.Slow_angle)
    {
        Robot_Coordinate_system_Vel.w = Status->Parameter.Max_w_Speed;
    }
    /* 1.3 ����ڼ�������ֹͣ���� */
    else if (Angle_offset <= Status->Parameter.Slow_angle && Angle_offset > Status->Parameter.Stop_angle)
    {
        Robot_Coordinate_system_Vel.w =
            Status->Parameter.Max_w_Speed -                                 // ����ٶ�
            fitting_function(Angle_offset / Status->Parameter.Slow_angle) * // ���ٱ���
                (Status->Parameter.Max_w_Speed);                            // �ٶȱ仯��
    }
    /* 1.4ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ */
//    else if (Angle_offset < Status->Parameter.Stop_angle) //(fabs(Status->Parameter.End_w_Speed) < 0.001f && Status->Parameter.Stop_angle != 0)
//    {
//        PId_Calculate(&Line_AnglePID, Angle_offset, 0);
//        Robot_Coordinate_system_Vel.w = Line_AnglePID.Output;
//    }

    /* 1.5 ˳ʱ����ת���ٶ�ȡ�� */
    if (Status->Parameter.angle_rotate_Sum < 0)
    {
        Robot_Coordinate_system_Vel.w *= -1;
    }

    /* 1.6 �ж�·���Ƿ����� */
    if (Angle_offset < 0.5f)
    {
        Status->Parameter.angle_rotate_Sum = 0; // ���㣬�Ժ���������ת������
    }

    /* 2. ������ϵ��XY�����ٶȼ��� */

    /* 2.1 �����ٶȼ��� */

    /* 2.1.1����ڼ����� */
    if ((Status->CoordinateSystem.Line_NowPos.x) < Status->Parameter.Speedup_Length)
    {
        Status->CoordinateSystem.Line_TargetVel.x = // �����ٶ�
            Status->Parameter.Start_Speed +
            fitting_function(Status->CoordinateSystem.Line_NowPos.x / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
    }
    /* 2.1.2 ����ڼ������⣬�����ٶ�Ϊ�����ٶ� */
    else if ((Status->CoordinateSystem.Line_NowPos.x) <= (Status->CoordinateSystem.Line_TargetPos.x - Status->Parameter.Slow_length))
    {
        Status->CoordinateSystem.Line_TargetVel.x = Status->Parameter.Max_Speed;
    }
    /* 2.1.3����ڼ������� */
    else if ((Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x) > Status->Parameter.Stop_length && (Status->CoordinateSystem.Line_NowPos.x + Status->Parameter.Slow_length) >= Status->CoordinateSystem.Line_TargetPos.x)
    {
        Status->CoordinateSystem.Line_TargetVel.x = // �����ٶ�
            Status->Parameter.Max_Speed -
            fitting_function2((Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x) / Status->Parameter.Slow_length) *
                (Status->Parameter.Max_Speed);
    }
    /* 2.1.4ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ */
//    else if (fabs(Status->CoordinateSystem.Target_Position.w) < 0.001f && Status->Parameter.Stop_length != 0)
//    {
//        PId_Calculate(&Line_StopPID, Status->CoordinateSystem.Line_NowPos.x, Status->CoordinateSystem.Line_TargetPos.x);
//        Status->CoordinateSystem.Line_TargetVel.x = Line_StopPID.Output;
//    }
//    /* 2.1.5ĩβ�ٶȲ�Ϊ�㣬�ٶȵ���ĩ���ٶ� */
//    else if (fabs(Status->Parameter.Stop_length) < 0.001F)
//    {
//        Status->CoordinateSystem.Line_TargetVel.x = Status->CoordinateSystem.Line_TargetVel.x;
//    }

    /* 2.2 �����ٶȼ��� */
    if (ABS(Status->CoordinateSystem.Line_NowPos.x - Status->CoordinateSystem.Line_TargetPos.x) > 10) // �쵽ֱ�߽�βʱ�����ڷ����ٶȣ�����������Ӷ���
    {
        PId_Calculate(&Line_AdjustPID, Status->CoordinateSystem.Line_NowPos.y, 0);

        /* ǰ������ */
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

//    /* ��������ϵy����ƫ�����ʱ��Сx�����ٶ�(������) */
//    if (ABS(Status->CoordinateSystem.Line_TargetVel.y) > 1000)
//    {
//        Status->CoordinateSystem.Line_TargetVel.x *= 0.5f;
//    }
    /* 3. ��������ϵ�µ��ٶ�ת������������ϵ*/
    /* ��������ϵ�µ��ٶ�ת������������ϵ���ٶ� */
    World_Coordinate_system_TargetVel = coordinate_transformation(&Status->CoordinateSystem.Line_TargetVel, &Status->CoordinateSystem.Zero_Point, -Line_angle);

    /* ����������ϵ���ٶ�ת���ɳ�������ϵ���ٶ� */
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Gyro_Rotine_Data.angle_yaw);

    /* ��ת�����󱣳ֳ���Ƕ� */
    if (Angle_offset < 0.5f || Status->Parameter.angle_rotate_Sum == 0)
    {
        /* ���ֳ���Ƕ� */
        PId_Calculate(&Line_AnglePID, Gyro_Rotine_Data.angle_yaw, Status->CoordinateSystem.Target_Position.w);
        Robot_Coordinate_system_Vel.w = Line_AnglePID.Output;
    }

    /* 4. �ж�·���Ƿ����� */
    if (Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x < 10)
    {
        Status->CoordinateSystem.Target_Position.w = Angle_Aim;
        Status->Flag.Work_Start = DISABLE;
    }

    return (Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x);
}

// ��Բ��·��
float Robot_Control_Circle(ROUTINE_STU *Status)
{
    static float Slope_angle;                            // �������ٶ�����ϵ����������ϵ�н�
    static float Angle_last;                             // ��������Ŀ��������ϵ���߹��ĽǶȡ���һʱ�̵ĽǶ�
    static float angle_temp_1, angle_temp_2, angle_temp; // һЩ����Ƕȵ���ʱ����
    static float arc_remain;                             // ��Ŀ���ľ����ֵ

    /* 0 �����ʼ������if����һ��·��ֻ����һ�� */
    if (Status->Flag.NewState == ENABLE)
    {
        Status->Flag.NewState = DISABLE;

        // ������ʼ�Ƕ�
        Status->CoordinateSystem.Start_Position.w = Status->CoordinateSystem.Target_Position.w;

        Status->Parameter.distance = ABS(Status->Parameter.Target_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R;

        // �����������С
        Status->Parameter.Slow_length = Status->Parameter.speed_down_rate * Status->Parameter.distance;
        // �����������С
        Status->Parameter.Speedup_Length = Status->Parameter.speed_up_rate * Status->Parameter.distance;
        // PId������
        PID_Clear(&Cir_AdjustPID);
        PID_Clear(&Cir_StopPID);
        PID_Clear(&Cir_AnglePID);

        // ����·����ʼ���ڼ�����ϵ�µ�����
        Status->CoordinateSystem.Polar_NowPos.Rho = CalculatePoint2Pointdistance(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);
        Status->CoordinateSystem.Polar_NowPos.Theta = CalculateLineAngle(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);

        // ����Angle_last
        Angle_last = Status->CoordinateSystem.Polar_NowPos.Theta;
        Status->Parameter.Now_Angle_Sum = 0;
    }

    /* 1 ���㵱ǰ�������ڼ�����ϵ������ */
    // ������������ϵ����
    World_Coordinate_system_NowPos.x = World_x;
    World_Coordinate_system_NowPos.y = World_y;

    // ���¼�����
    Status->CoordinateSystem.Polar_NowPos.Rho = CalculatePoint2Pointdistance(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);
    Status->CoordinateSystem.Polar_NowPos.Theta = CalculateLineAngle(Status->CoordinateSystem.World_HeartPos, World_Coordinate_system_NowPos);

    /* 2 ����������Ѿ��߹���Բ�Ľ� */
    // ���ȼ�����Ŀ���ĽǶȲ�ֵ�����㷽�������M3508��ˢ�������Ƕȵķ���
    if (Angle_last < Status->CoordinateSystem.Polar_NowPos.Theta) // һ�����
    {
        angle_temp_1 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last; // ��ʱ��
        angle_temp_2 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last - 2 * PI;
    }
    else
    {
        angle_temp_1 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last; // ˳ʱ��
        angle_temp_2 = 2 * PI + Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last;
    }
    // ����˳ʱ��ת������ʱ��ת������ȡС���Ǹ��Ƕ�
    angle_temp = (ABS(angle_temp_1)) > (ABS(angle_temp_2)) ? angle_temp_2 : angle_temp_1;
    Status->Parameter.Now_Angle_Sum += angle_temp;
    Angle_last = Status->CoordinateSystem.Polar_NowPos.Theta; // ��ǰ����λ�ü��Ǳ�Ϊlast
    // �ټ�����Ŀ���ľ����ֵ
    arc_remain = ABS((Status->Parameter.Target_Angle_Sum - Status->Parameter.Now_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R);

    /* 3 ���㼫�����ٶ�����ϵ�µ������ٶ� */
    // ����ڼ�����
    if (ABS(Status->Parameter.Now_Angle_Sum) * Status->Parameter.R < Status->Parameter.Speedup_Length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // �����ٶ�
            Status->Parameter.Start_Speed +
            fitting_function(Status->Parameter.Now_Angle_Sum * Status->Parameter.R / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
    }
    // ���û�ڼ������⣬�����ٶ�Ϊ����ٶ�
    else if (arc_remain >= Status->Parameter.Slow_length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = Status->Parameter.Max_Speed;
    }
    // ����ڼ������ڣ�ֹͣ����
    else if (arc_remain >= Status->Parameter.Stop_length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // �����ٶ�
            Status->Parameter.Max_Speed -
            fitting_function(arc_remain / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed);
    }
    // ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ
    else if (fabs(Status->CoordinateSystem.Target_Position.w) < 0.001f && Status->Parameter.Stop_length != 0)
    {
        PId_Calculate(&Cir_StopPID, arc_remain, 0);
        Status->CoordinateSystem.Polar_TargetVel.x = -Cir_StopPID.Output;
    }
    else if (fabs(Status->Parameter.Stop_length) < 0.001F)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = Status->CoordinateSystem.Polar_TargetVel.x;
    }

//    // ˳ʱ����ת�������ٶ�ȡ��
//    if (Status->Flag.circle_tpye == Robot_LineType_Cycle_Clockwise)
//    {
//        Status->CoordinateSystem.Polar_TargetVel.x *= -1;
//    }

    /* 4 ���㼫�����ٶ�����ϵ�µķ������ٶ� */
    // �����ٶ�PID����
    PId_Calculate(&Cir_AdjustPID, Status->CoordinateSystem.Polar_NowPos.Rho, Status->Parameter.R);
    Status->CoordinateSystem.Polar_TargetVel.y = -Cir_AdjustPID.Output;

    /* 5 ������ٶ� */
    // ��������ߣ�����Ŀ��Ƕ�
    if (Status->Flag.yanqiexian)
    {
        Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Now_Angle_Sum;
    }

    // PID���ֳ���Ƕ�
    PId_Calculate(&Cir_AnglePID, Gyro_Rotine_Data.angle_yaw, Status->CoordinateSystem.Target_Position.w);
    Robot_Coordinate_system_Vel.w = Cir_AnglePID.Output;

    /* ���������ٶ�����ϵ������ƫ�����ʱ��С�������ٶ�(������) */
    if (ABS(Status->CoordinateSystem.Polar_TargetVel.y) > 1000)
    {
        Status->CoordinateSystem.Polar_TargetVel.x *= 0.5f;
    }
    /* 6 ���������ٶ�����ϵ�µ��ٶ�ת������������ϵ*/
    // ���㼫�����ٶ�����ϵ����������ϵ�н�
    Slope_angle = Status->CoordinateSystem.Polar_NowPos.Theta - 90.0f;
    /* ���������ٶ�����ϵ���ٶ�ת������������ϵ���ٶ� */
    World_Coordinate_system_TargetVel = coordinate_transformation(&Status->CoordinateSystem.Polar_TargetVel, &Status->CoordinateSystem.Zero_Point, -Slope_angle);

    /* ����������ϵ���ٶ�ת���ɳ�������ϵ���ٶ� */
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Gyro_Rotine_Data.angle_yaw);

    // �ж�·���Ƿ�����
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
// ֱ�ߣ�Ŀ��x��Ŀ��y��Ҫת�ĽǶȣ�V����Vĩ��������������������������w���V���ֹͣ�����ȣ�
void line(float Target_x, float Target_y, float rotate_Sum, int Start_Speed, int End_Speed, float up_rate, float down_rate, int Max_w, int speed_max, float Stop_length)
{
    Routine_Status.Flag.NewState = ENABLE;
    Routine_Status.Flag.Work_Start = ENABLE;

    Routine_Status.CoordinateSystem.Start_Position.x = World_x; // ��ʼλ��
    Routine_Status.CoordinateSystem.Start_Position.y = World_y;
    Routine_Status.CoordinateSystem.Target_Position.x = Target_x; // Ŀ��λ��
    Routine_Status.CoordinateSystem.Target_Position.y = Target_y;

    /* �ٶ� */
    Routine_Status.Parameter.Start_Speed = Start_Speed; // �����ٶȣ���ҪΪ0
    Routine_Status.Parameter.Max_Speed = speed_max;     // ����ٶ�
    Routine_Status.Parameter.End_Speed = End_Speed;     // ·��ĩ���ٶ�

    Routine_Status.Parameter.speed_up_rate = up_rate;     // ����������
    Routine_Status.Parameter.speed_down_rate = down_rate; // ����������
    Routine_Status.Parameter.Stop_length = Stop_length;   // ֹͣ������

    /* �Ƕ� */
    Routine_Status.Parameter.angle_rotate_Sum = rotate_Sum; // ��ת�Ƕȣ���ʱ��Ϊ��

    Routine_Status.Parameter.Start_w_Speed = 40;  // �������ٶ�
    Routine_Status.Parameter.Max_w_Speed = Max_w; // �����ٶ�
    Routine_Status.Parameter.End_w_Speed = 0;     // ·��ĩ�˽��ٶ�

		Routine_Status.Parameter.wspeed_up_rate = 0.2;
		Routine_Status.Parameter.wspeed_down_rate = 0.2;
    Routine_Status.Parameter.Stop_angle = 0.5;                     // ֹͣ������(�Ƕ�)
    while (Routine_Status.Flag.Work_Start)
    {
        Robot_Control_Line(&Routine_Status);
        motion_resolve(Robot_Coordinate_system_Vel.x,Robot_Coordinate_system_Vel.y,Robot_Coordinate_system_Vel.w);
        osDelay(2);
    }
}

// Բ����Ŀ��x��Ŀ��y��Բ�Ľ�(����)��V����Vĩ��V����Ƿ����У�������������������������
void circle(float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, FunctionalState yanqie, float up_rate, float down_rate)
{
    Routine_Status.Flag.Work_Start = ENABLE;
    Routine_Status.Flag.NewState = ENABLE;

    float gama, R, ygxl, d, c_x, c_y;
    float rad = angle * CHANGE_TO_RADIAN;
    d = sqrt((Target_x - World_x) * (Target_x - World_x) + (Target_y - World_y) * (Target_y - World_y));
    R = d / 2.0f / sinf(rad / 2.0f);
    if (angle >= 0) // ˳ʱ��
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
        Routine_Status.Flag.circle_tpye = Robot_LineType_Cycle_Clockwise; // ˳ʱ��
    }
    else if (angle < 0) // ��ʱ��
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
        Routine_Status.Flag.circle_tpye = Robot_LineType_Cycle_Anticlockwise; // ��ʱ��
    }

    Routine_Status.CoordinateSystem.Start_Position.x = Routine_Status.CoordinateSystem.Target_Position.x; // ��ʼλ��
    Routine_Status.CoordinateSystem.Start_Position.y = Routine_Status.CoordinateSystem.Target_Position.y;
    Routine_Status.CoordinateSystem.Target_Position.x = Target_x; // Ŀ��λ��
    Routine_Status.CoordinateSystem.Target_Position.y = Target_y;

    Routine_Status.Parameter.Start_Speed = start_v; // ��ʼ�ٶ�
    Routine_Status.Parameter.Max_Speed = v_max;     // ����ٶ�
    Routine_Status.Parameter.End_Speed = end_v;     // ·��ĩ���ٶ�

    Routine_Status.Parameter.speed_up_rate = up_rate;     // �����������
    Routine_Status.Parameter.speed_down_rate = down_rate; // �����������
    Routine_Status.Parameter.Stop_length = 20;            // ֹͣ������
    // Routine_Status.Flag.circle_tpye = type;                       // ѡ��˳ʱ�뻹����ʱ��
    Routine_Status.Flag.yanqiexian = yanqie;                // �Ƿ�������
    Routine_Status.Parameter.Target_Angle_Sum = angle;      // Բ�Ľ�
    Routine_Status.Parameter.R = R;                         // �뾶
    Routine_Status.CoordinateSystem.World_HeartPos.x = c_x; // Բ��
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
    // ���ֳ���Ƕ�
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
