#include "lj_plus.h"
#include "usart.h"
#include "Speed_decomposition.h"
#include "pid_controller.h"
#include "math.h"
#include "cmsis_os.h"

// �Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN (0.01745329251994f)
// ������ת��Ϊ�Ƕ���ϵ��
#define CHANGE_TO_ANGLE (57.29577951308232f)
#define pi (3.1415926)

float watch[10]={0};
int i_sum=1;
float mis_error=0;
coordinate_Struct World_Coordinate_system_NowPos;    // ��������ϵ�µ�ǰλ��
coordinate_Struct World_Coordinate_system_TargetVel; // ��������ϵ��Ŀ���ٶ�
coordinate_Struct Robot_Coordinate_system_TargetVel; // ��������ϵ��Ŀ���ٶ�
coordinate_Struct Robot_Coordinate_system_Vel;       // ·��������ĳ����ٶȣ����ڳ����ٶȷ���
coordinate_Struct Zero_Point = {0, 0};               // ��������ϵ���
extern Action_data Action_Data;

LUJING_STU Lujing_Status; // ·���ṹ��

PID_TypeDef Line_AdjustPID; // ֱ�߷���pid����
PID_TypeDef Line_StopPID;   // ֱ��ֹͣ������pid����
PID_TypeDef Line_AnglePID;  // ֱ�߽Ƕ�pid����
PID_TypeDef Cir_AdjustPID;  // Բ������pid����
PID_TypeDef Cir_StopPID;    // Բ��ֹͣ������pid����
PID_TypeDef Cir_AnglePID;   // Բ���Ƕ�pid����
PID_TypeDef Keep_X_PID;     // ����x����λ��
PID_TypeDef Keep_Y_PID;     // ����y����λ��
PID_TypeDef Keep_W_PID;     // ���ֽǶ�

static float ABS(float a)
{
    return a > 0 ? a : -a;
}

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

static void PID_Clear(PID_TypeDef *PID)
{
    PID->Output = 0;
    PID->Last_Err = 0;
    PID->ITerm = 0;
}

static void Pid_Calculate(PID_TypeDef *pid, float measure, float target)
{

    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;
//	//��̬KP,KD����
//	  pid->Kp_Err = pid->Err;
//    pid->Kd_Err = pid->Err - pid->Last_Err;
//	  pid->Kp = pid->Kp_kp * (pid->Kp_Err * pid->Kp_Err)
//                        +pid->Kp_ki
//                        +pid->Kp_kd * (pid->Kp_Err - pid->Kp_Last_Err);
//    pid->Kd = pid->Kd_kp * (pid->Kd_Err * pid->Kd_Err)
//                        + pid->Kd_ki
//                        + pid->Kd_kd * (pid->Kd_Err - pid->Kd_Last_Err);
//    pid->Kp_Last_Err = pid->Kp_Err;
//    pid->Kd_Last_Err = pid->Kd_Err;

    if (ABS(pid->Err) < pid->dead_band)
        pid->Output = 0;
    else
    {
        pid->Output = pid->Kp * pid->Err  + pid->Ki * pid->ITerm + pid->Kd * (pid->Err - pid->Last_Err) + (pid->Err > 0 ? pid->Forward : -pid->Forward);
        pid->ITerm += pid->Err;
        if (pid->ITerm > 30000)
            pid->ITerm = 30000;
        else if (pid->ITerm < -30000)
            pid->ITerm = -30000;
        pid->Last_Err = pid->Err;
    }
}
// ֱ��·��
float Robot_Control_Line(LUJING_STU *Status)
{
    static float Angle_Aim = 0;    // Ŀ��Ƕ�ֵ���м����
    static float Angle_offset = 0; // ʣ��Ҫת�ĽǶȣ��м����
    static float Line_angle = 0;   // ������ϵ����������ϵ�н�
    static float Angle_dis = 0;    // �Ƕ�ת�����룬�м����

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
        Status->Parameter.Slow_angle = Status->Parameter.wspeed_up_rate * Status->Parameter.angle_rotate_Sum;
        /* �����������С */
        Status->Parameter.Speedup_angle = Status->Parameter.wspeed_up_rate * Status->Parameter.angle_rotate_Sum;

        /* 0.3 ����Ŀ��Ƕ� */
        /* ��¼·����ʼ�Ƕȣ���ǰ·����ʼ�Ƕ�Ϊ��һ��·����Ŀ��Ƕ� */
        Status->CoordinateSystem.Start_Position.w = Status->CoordinateSystem.Target_Position.w;

        /* ���㵱ǰ·��Ŀ��Ƕȣ�Ŀ��Ƕ�Ϊ��ʼ�Ƕ�+Ҫת�ĽǶ� */
        Angle_Aim = Status->CoordinateSystem.Target_Position.w + Status->Parameter.angle_rotate_Sum;

        /* ������һ��·���ĳ�ʼ�Ƕ� */
        Status->CoordinateSystem.Target_Position.w = Angle_Aim;

        /* 0.4 ��pid������һ��·���������� */
        PID_Clear(&Line_AdjustPID);
        PID_Clear(&Line_StopPID);
        PID_Clear(&Line_AnglePID);
    }

    /* ������������ϵ�µ�ǰ�����˵����� */
    World_Coordinate_system_NowPos.x = Action_Data.x;
    World_Coordinate_system_NowPos.y = Action_Data.y;

    /* ����������ϵ�µ�ǰ�����˵����� */
    Status->CoordinateSystem.Line_NowPos = coordinate_transformation(&World_Coordinate_system_NowPos, &Status->CoordinateSystem.Start_Position, Line_angle);

    /* 1. �����������ٶ� */
    Angle_offset = ABS(Angle_Aim - Action_Data.angle_Z);

    Angle_dis = ABS(Action_Data.angle_Z - Status->CoordinateSystem.Start_Position.w);
    /* 1.1 ��ǰ�Ƕ��ڼ������ڣ�����Ŀ����ٶ� */
    if (Angle_dis < Status->Parameter.Speedup_angle)
    {
        Robot_Coordinate_system_Vel.w =
            Status->Parameter.Start_w_Speed +                                      // ���ٶ�
            fitting_function(Angle_dis / Status->Parameter.Speedup_angle) *        // ���ٱ���
                (Status->Parameter.Max_w_Speed - Status->Parameter.Start_w_Speed); // �ٶȱ仯��
    }
    /* 1.2 ������ڼ������ڣ������ڼ������⣬���ٶ�Ϊ�����ٶ� */
    else if (Angle_offset >= Status->Parameter.Slow_angle)
    {
        Robot_Coordinate_system_Vel.w = Status->Parameter.Max_w_Speed;
    }
    /* 1.3 ����ڼ�������ֹͣ���� */
    else if (Angle_offset >= Status->Parameter.Stop_angle)
    {
        Robot_Coordinate_system_Vel.w =
            Status->Parameter.Max_w_Speed -                                 // ����ٶ�
            fitting_function(Angle_offset / Status->Parameter.Slow_angle) * // ���ٱ���
                (Status->Parameter.Max_w_Speed);                            // �ٶȱ仯��
    }
    /* 1.4ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ */
    else if (fabs(Status->Parameter.End_w_Speed) < 0.001f && Status->Parameter.Stop_angle != 0)
    {
        Pid_Calculate(&Line_AnglePID, Angle_offset, 0);
        Robot_Coordinate_system_Vel.w = Line_AnglePID.Output;
    }

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
    else if (fabs(Status->CoordinateSystem.Target_Position.w) < 0.001f && Status->Parameter.Stop_length != 0)
    {
        Pid_Calculate(&Line_StopPID, Status->CoordinateSystem.Line_NowPos.x, Status->CoordinateSystem.Line_TargetPos.x);
        Status->CoordinateSystem.Line_TargetVel.x = Line_StopPID.Output;
    }
    /* 2.1.5ĩβ�ٶȲ�Ϊ�㣬�ٶȵ���ĩ���ٶ� */
    else if (fabs(Status->Parameter.Stop_length) < 0.001F)
    {
        Status->CoordinateSystem.Line_TargetVel.x = Status->CoordinateSystem.Line_TargetVel.x;
    }

    /* 2.2 �����ٶȼ��� */
    if (ABS(Status->CoordinateSystem.Line_NowPos.x - Status->CoordinateSystem.Line_TargetPos.x) > 10) // �쵽ֱ�߽�βʱ�����ڷ����ٶȣ�����������Ӷ���
    {
        Pid_Calculate(&Line_AdjustPID, Status->CoordinateSystem.Line_NowPos.y, 0);

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

    /* ��������ϵy����ƫ�����ʱ��Сx�����ٶ�(������) */
    if (ABS(Status->CoordinateSystem.Line_TargetVel.y) > 1000)
    {
        Status->CoordinateSystem.Line_TargetVel.x *= 0.5f;
    }
    /* 3. ��������ϵ�µ��ٶ�ת������������ϵ*/
    /* ��������ϵ�µ��ٶ�ת������������ϵ���ٶ� */
    World_Coordinate_system_TargetVel = coordinate_transformation(&Status->CoordinateSystem.Line_TargetVel, &Status->CoordinateSystem.Zero_Point, -Line_angle);

    /* ����������ϵ���ٶ�ת���ɳ�������ϵ���ٶ� */
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Action_Data.angle_Z);

    /* ��ת�����󱣳ֳ���Ƕ� */
    if (Angle_offset < 0.5f || Status->Parameter.angle_rotate_Sum == 0)
    {
        /* ���ֳ���Ƕ� */
        Pid_Calculate(&Line_AnglePID, Action_Data.angle_Z, Status->CoordinateSystem.Target_Position.w);
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
float Robot_Control_Circle(LUJING_STU *Status)
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
        Status->Parameter.Slow_length = Status->Parameter.speed_down_rate * Status->Parameter.Target_Angle_Sum;
        // �����������С
        Status->Parameter.Speedup_Length = Status->Parameter.speed_up_rate * Status->Parameter.Target_Angle_Sum;
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
    World_Coordinate_system_NowPos.x = Action_Data.x;
    World_Coordinate_system_NowPos.y = Action_Data.y;

    // ���¼�����
    Status->CoordinateSystem.Polar_NowPos.Rho = CalculatePoint2Pointdistance(Status->CoordinateSystem.World_HeartPos, Status->CoordinateSystem.Start_Position);
    Status->CoordinateSystem.Polar_NowPos.Theta = CalculateLineAngle(Status->CoordinateSystem.World_HeartPos, World_Coordinate_system_NowPos);

    /* 2 ����������Ѿ��߹���Բ�Ľ� */
    // ���ȼ�����Ŀ���ĽǶȲ�ֵ�����㷽�������M3508��ˢ�������Ƕȵķ���
    if (Lujing_Status.Flag.circle_tpye == Robot_LineType_Cycle_Anticlockwise) // һ�����
    {
        angle_temp_1 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last; // ��ʱ��
        angle_temp_2 = Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last - 2 * PI;
    }
    else
    {
        angle_temp_1 = -(Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last); // ˳ʱ��
        angle_temp_2 = 2 * PI + Status->CoordinateSystem.Polar_NowPos.Theta - Angle_last;
    }
    // ����˳ʱ��ת������ʱ��ת������ȡС���Ǹ��Ƕ�
    angle_temp = (ABS(angle_temp_1)) > (ABS(angle_temp_2)) ? angle_temp_2 : angle_temp_1;
    Status->Parameter.Now_Angle_Sum += angle_temp;
		watch[1]=Status->Parameter.Now_Angle_Sum;
    Angle_last = Status->CoordinateSystem.Polar_NowPos.Theta; // ��ǰ����λ�ü��Ǳ�Ϊlast
    // �ټ�����Ŀ���ľ����ֵ
    arc_remain = ABS(Status->Parameter.Target_Angle_Sum - Status->Parameter.Now_Angle_Sum) ;

    /* 3 ���㼫�����ٶ�����ϵ�µ������ٶ� */
    // ����ڼ�����
    if (ABS(Status->Parameter.Now_Angle_Sum)  < Status->Parameter.Speedup_Length)
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // �����ٶ�
            Status->Parameter.Start_Speed +
            fitting_function(Status->Parameter.Now_Angle_Sum  / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
			watch[0]= Status->CoordinateSystem.Polar_TargetVel.x;
    }
    // ���û�ڼ������⣬�����ٶ�Ϊ����ٶ�
    else if (ABS(Status->Parameter.Now_Angle_Sum) > Status->Parameter.Speedup_Length && ABS(Status->Parameter.Now_Angle_Sum) < (Status->Parameter.Target_Angle_Sum - Status->Parameter.Stop_length))
    {
        Status->CoordinateSystem.Polar_TargetVel.x = Status->Parameter.Max_Speed;
			watch[0]= Status->CoordinateSystem.Polar_TargetVel.x;
    }
    // ����ڼ������ڣ�ֹͣ����
    else if (ABS(Status->Parameter.Now_Angle_Sum) >= (Status->Parameter.Target_Angle_Sum - Status->Parameter.Stop_length))
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // �����ٶ�
            Status->Parameter.Max_Speed -
            fitting_function2(arc_remain / Status->Parameter.Stop_length) *
                (Status->Parameter.Max_Speed);
			watch[0]= Status->CoordinateSystem.Polar_TargetVel.x;
    }
		if (Lujing_Status.Flag.circle_tpye == Robot_LineType_Cycle_Anticlockwise)
		{
				Status->CoordinateSystem.Polar_TargetVel.x *= -1;
		}
    // ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ
//    else if (fabs(Status->CoordinateSystem.Target_Position.w) < 0.001f && Status->Parameter.Stop_length != 0)
//    {
//        Pid_Calculate(&Cir_StopPID, arc_remain, 0);
//        Status->CoordinateSystem.Polar_TargetVel.x = -Cir_StopPID.Output;
//    }
//    else if (fabs(Status->Parameter.Stop_length) < 0.001F)
//    {
//        Status->CoordinateSystem.Polar_TargetVel.x = Status->CoordinateSystem.Polar_TargetVel.x;
//    }

//    // ˳ʱ����ת�������ٶ�ȡ��
//    if (Status->Flag.circle_tpye == Robot_LineType_Cycle_Clockwise)
//    {
//        Status->CoordinateSystem.Polar_TargetVel.x *= -1;
//    }

    /* 4 ���㼫�����ٶ�����ϵ�µķ������ٶ� */
    // �����ٶ�PID����
    Pid_Calculate(&Cir_AdjustPID, Status->CoordinateSystem.Polar_NowPos.Rho, Status->Parameter.R);
    Status->CoordinateSystem.Polar_TargetVel.y = -Cir_AdjustPID.Output;

    /* 5 ������ٶ� */
    // ��������ߣ�����Ŀ��Ƕ�
    if (Status->Flag.yanqiexian)
    {
        Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Now_Angle_Sum;
    }

    // PID���ֳ���Ƕ�
    Pid_Calculate(&Cir_AnglePID, Action_Data.angle_Z, Status->CoordinateSystem.Target_Position.w);
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
    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Action_Data.angle_Z);

    // �ж�·���Ƿ�����
    if (ABS(Status->Parameter.Now_Angle_Sum)>=ABS(Status->Parameter.Target_Angle_Sum))
    {
        if (Status->Flag.yanqiexian)
            Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Target_Angle_Sum;
        else
            Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w;
        Status->Flag.Work_Start = DISABLE;
    }

    return (arc_remain);
}

// ��ʾ���㷽ʽʹ�ã������ο�
float Robot_Control_Circle_plus(LUJING_STU *Status)
{

    float sinxita, cosxita;
    sinxita = (Action_Data.y - Status->CoordinateSystem.World_HeartPos.y) / Status->Parameter.R;
    cosxita = (Action_Data.x - Status->CoordinateSystem.World_HeartPos.x) / Status->Parameter.R;
    World_Coordinate_system_TargetVel.x = sinxita * Status->Parameter.Max_Speed;
    World_Coordinate_system_TargetVel.y = -1 * cosxita * Status->Parameter.Max_Speed;
    return 0;
}

// ֱ�ߣ�Ŀ��x��Ŀ��y��Ҫת�ĽǶȣ�V����Vĩ��������������������������w���V���ֹͣ�����ȣ�
void line(float Target_x, float Target_y, float rotate_Sum, int Start_Speed, int End_Speed, float up_rate, float down_rate, int Max_w, int speed_max, float Stop_length)
{
    Lujing_Status.Flag.NewState = ENABLE;
    Lujing_Status.Flag.Work_Start = ENABLE;

    Lujing_Status.CoordinateSystem.Start_Position.x = Action_Data.x; // ��ʼλ��
    Lujing_Status.CoordinateSystem.Start_Position.y = Action_Data.y;
    Lujing_Status.CoordinateSystem.Target_Position.x = Target_x; // Ŀ��λ��
    Lujing_Status.CoordinateSystem.Target_Position.y = Target_y;

    /* �ٶ� */
    Lujing_Status.Parameter.Start_Speed = Start_Speed; // �����ٶȣ���ҪΪ0
    Lujing_Status.Parameter.Max_Speed = speed_max;     // ����ٶ�
    Lujing_Status.Parameter.End_Speed = End_Speed;     // ·��ĩ���ٶ�

    Lujing_Status.Parameter.speed_up_rate = up_rate;     // ����������
    Lujing_Status.Parameter.speed_down_rate = down_rate; // ����������
    Lujing_Status.Parameter.Stop_length = Stop_length;   // ֹͣ������

    /* �Ƕ� */
    Lujing_Status.Parameter.angle_rotate_Sum = rotate_Sum; // ��ת�Ƕȣ���ʱ��Ϊ��

    Lujing_Status.Parameter.Start_w_Speed = 40;  // �������ٶ�
    Lujing_Status.Parameter.Max_w_Speed = Max_w; // �����ٶ�
    Lujing_Status.Parameter.End_w_Speed = 0;     // ·��ĩ�˽��ٶ�

    Lujing_Status.Parameter.Speedup_accelerated_w_speed = 100000; // ���������ٶ�
    Lujing_Status.Parameter.Slow_accelerated_w_speed = 10000;     // �������Ӽ��ٶ�
    Lujing_Status.Parameter.Stop_angle = 0.5;                     // ֹͣ������(�Ƕ�)
    while (Lujing_Status.Flag.Work_Start)
    {
        Robot_Control_Line(&Lujing_Status);
        Robot_Wheel_Control();
				
        osDelay(2);
    }
}

// Բ����Ŀ��x��Ŀ��y��Բ�Ľ�(����)��V����Vĩ��V����Ƿ����У�������������������������
void circle(float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, float up_rate, float down_rate)
{
    Lujing_Status.Flag.Work_Start = ENABLE;
    Lujing_Status.Flag.NewState = ENABLE;
			Target_x+=Action_Data.x;
			Target_y+=Action_Data.y;
			if(angle<0){
				angle=360+angle;
				Lujing_Status.Flag.circle_tpye = Robot_LineType_Cycle_Anticlockwise;
			}
			else Lujing_Status.Flag.circle_tpye = Robot_LineType_Cycle_Clockwise;
	
    float gama, R, ygxl, d, c_x, c_y;
    float rad ;
		rad= angle * CHANGE_TO_RADIAN;
    d = sqrt((Target_x - Action_Data.x) * (Target_x - Action_Data.x) + (Target_y - Action_Data.y) * (Target_y - Action_Data.y));
    R = d / 2.0f / sinf(rad / 2.0f);
		
        if ((Target_y - Action_Data.y) >= 0)
        {
            gama = acosf((Target_x - Action_Data.x) / d);
        }
        else if ((Target_y - Action_Data.y) < 0)
        {
            gama = acosf((Action_Data.x - Target_x) / d) + pi;
        }
        ygxl = gama - (pi / 2 - rad / 2);

        c_x = Action_Data.x + R * cos(ygxl);
        c_y = Action_Data.y + R * sin(ygxl);
         // ˳ʱ��

//    else if (angle < 0) // ��ʱ��
//    {
//        if ((Target_y - Action_Data.y) >= 0)
//        {
//            gama = acosf((Target_x - Action_Data.x) / d);
//        }
//        else if ((Target_y - Action_Data.y) < 0)
//        {
//            gama = acosf((Action_Data.x - Target_x) / d) + pi / 2;
//        }
//        ygxl = gama + (pi / 2 - rad / 2);

//        c_x = Action_Data.x + R * cos(ygxl);
//        c_y = Action_Data.y + R * sin(ygxl);
//        Lujing_Status.Flag.circle_tpye = Robot_LineType_Cycle_Anticlockwise; // ��ʱ��
//    }

    Lujing_Status.CoordinateSystem.Start_Position.x = Action_Data.x; // ��ʼλ��
    Lujing_Status.CoordinateSystem.Start_Position.y = Action_Data.y;
    Lujing_Status.CoordinateSystem.Target_Position.x = Target_x; // Ŀ��λ��
    Lujing_Status.CoordinateSystem.Target_Position.y = Target_y;

    Lujing_Status.Parameter.Start_Speed = start_v; // ��ʼ�ٶ�
    Lujing_Status.Parameter.Max_Speed = v_max;     // ����ٶ�
    Lujing_Status.Parameter.End_Speed = end_v;     // ·��ĩ���ٶ�

    Lujing_Status.Parameter.speed_up_rate = up_rate;     // �����������
    Lujing_Status.Parameter.speed_down_rate = down_rate; // �����������
    Lujing_Status.Parameter.Stop_length = 20;            // ֹͣ������
    // Lujing_Status.Flag.circle_tpye = type;                       // ѡ��˳ʱ�뻹����ʱ��
    //Lujing_Status.Flag.yanqiexian = yanqie;                // �Ƿ�������
		if(Lujing_Status.Flag.circle_tpye == Robot_LineType_Cycle_Anticlockwise)
			Lujing_Status.Parameter.Target_Angle_Sum =ABS(360- angle); 
		else Lujing_Status.Parameter.Target_Angle_Sum = angle;      // Բ�Ľ�
    Lujing_Status.Parameter.R = R;                         // �뾶
    Lujing_Status.CoordinateSystem.World_HeartPos.x = c_x; // Բ��
    Lujing_Status.CoordinateSystem.World_HeartPos.y = c_y;
	
    while (Lujing_Status.Flag.Work_Start)
    {
        Robot_Control_Circle(&Lujing_Status);
        //Robot_Wheel_Control();
//				motion_debug(c_x,c_y,R,i_sum);
//			i_sum++;
//			if(i_sum ==110)i_sum=0;
			Robot_Wheel_Control();
			mis_error=sqrt((Action_Data.x-c_x)*(Action_Data.x-c_x)+(Action_Data.y-c_y)*(Action_Data.y-c_y))-R;
        HAL_Delay(2);
    }
}

void motion_debug(float x0,float y0,float r,int i)
{
	float x,y;
	
	y=y0+r*sin((45+0.9*i)*CHANGE_TO_RADIAN);
	x=-x0-r*cos((45+0.9*i)*CHANGE_TO_RADIAN);
	
	Action_Data.x=-x;
	Action_Data.y=y;
}
	

void Keep_Robot_Position(float Angle, float X, float Y)
{
    // ���ֳ���Ƕ�
	if(ABS(Keep_X_PID.Err) > 50 ) {Keep_X_PID.Kp = 4000/Keep_X_PID.Err;}
	else if (ABS(Keep_X_PID.Err) > 25 ){Keep_X_PID.Kp = 5;}
	else {Keep_X_PID.Kp = 5*ABS(Keep_X_PID.Err);}
	
	if(ABS(Keep_Y_PID.Err) > 50 ) {Keep_Y_PID.Kp = 4000/Keep_Y_PID.Err;}
	else if (ABS(Keep_Y_PID.Err) > 25 ){Keep_Y_PID.Kp = 5;}
	else {Keep_Y_PID.Kp = 5*ABS(Keep_Y_PID.Err);}
	
	if(ABS(Keep_X_PID.Err)*ABS(Keep_X_PID.Err)+ABS(Keep_Y_PID.Err)*ABS(Keep_Y_PID.Err) > 5000) {Keep_W_PID.Kp = 0.5;}
	else if (ABS(Keep_X_PID.Err)*ABS(Keep_X_PID.Err)+ABS(Keep_Y_PID.Err)*ABS(Keep_Y_PID.Err) > 1250 ){Keep_W_PID.Kp = 1;}
	else {Keep_W_PID.Kp = 5;}
//	if (ABS(Keep_X_PID.Err) > 20 ){Keep_X_PID.Kp = 4000/Keep_X_PID.Err;}
//	  Keep_X_PID.Kp = 5*ABS(Keep_X_PID.Err);
//	
//	if (ABS(Keep_Y_PID.Err) > 20 ){Keep_Y_PID.Kp = 4000/Keep_Y_PID.Err;}
//    Keep_Y_PID.Kp = 5*ABS(Keep_Y_PID.Err);
//	
//	if(ABS(Keep_X_PID.Err)*ABS(Keep_X_PID.Err)+ABS(Keep_Y_PID.Err)*ABS(Keep_Y_PID.Err) > 200) {Keep_W_PID.Kp = 0.1;}
//	else {Keep_W_PID.Kp = 5*Keep_W_PID.Err*Keep_W_PID.Err;}
		Pid_Calculate(&Keep_W_PID, Action_Data.angle_Z, Angle);
    //	Pid_Calculate(&Keep_W_PID,Action_Data.w,Angle,1);
    if (Keep_W_PID.Output > 5000)
        Keep_W_PID.Output = 5000;
    if (Keep_W_PID.Output < -5000)
        Keep_W_PID.Output = -5000;
    World_Coordinate_system_TargetVel.w = Keep_W_PID.Output;

    Pid_Calculate(&Keep_X_PID, Action_Data.x, X);
    if (Keep_X_PID.Output > 5000)
        Keep_X_PID.Output = 5000;
    if (Keep_X_PID.Output < -5000)
        Keep_X_PID.Output = -5000;
    World_Coordinate_system_TargetVel.x = Keep_X_PID.Output;

    Pid_Calculate(&Keep_Y_PID, Action_Data.y, Y);
    if (Keep_Y_PID.Output > 5000)
        Keep_Y_PID.Output = 5000;
    if (Keep_Y_PID.Output < -5000)
        Keep_Y_PID.Output = -5000;
    World_Coordinate_system_TargetVel.y = Keep_Y_PID.Output;
		
//	if( ABS(Keep_X_PID.Err) > 20 | ABS(Keep_Y_PID.Err) > 20)
//	{
//    Pid_Calculate(&Keep_W_PID, Action_Data.angle_Z, Angle);
//    //	Pid_Calculate(&Keep_W_PID,Action_Data.w,Angle,1);
//    if (Keep_W_PID.Output > 30000)
//        Keep_W_PID.Output = 30000;
//    if (Keep_W_PID.Output < -30000)
//        Keep_W_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.w = Keep_W_PID.Output;

//    Pid_Calculate(&Keep_X_PID, Action_Data.x, X);
//    if (Keep_X_PID.Output > 30000)
//        Keep_X_PID.Output = 30000;
//    if (Keep_X_PID.Output < -30000)
//        Keep_X_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.x = Keep_X_PID.Output;

//    Pid_Calculate(&Keep_Y_PID, Action_Data.y, Y);
//    if (Keep_Y_PID.Output > 30000)
//        Keep_Y_PID.Output = 30000;
//    if (Keep_Y_PID.Output < -30000)
//        Keep_Y_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.y = Keep_Y_PID.Output;

////		World_Coordinate_system_TargetVel.x= 0;
//		World_Coordinate_system_TargetVel.w= 0;
//	}
//	if( ABS(Keep_X_PID.Err) < 20 && ABS(Keep_X_PID.Err) > 5)
//	{
//    Pid_Calculate(&Keep_W_PID, Action_Data.angle_Z, Angle);
//    //	Pid_Calculate(&Keep_W_PID,Action_Data.w,Angle,1);
//    if (Keep_W_PID.Output > 30000)
//        Keep_W_PID.Output = 30000;
//    if (Keep_W_PID.Output < -30000)
//        Keep_W_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.w = Keep_W_PID.Output;

//    Pid_Calculate(&Keep_X_PID, Action_Data.x, X);
//    if (Keep_X_PID.Output > 30000)
//        Keep_X_PID.Output = 30000;
//    if (Keep_X_PID.Output < -30000)
//        Keep_X_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.x = Keep_X_PID.Output;

//    Pid_Calculate(&Keep_Y_PID, Action_Data.y, Y);
//    if (Keep_Y_PID.Output > 30000)
//        Keep_Y_PID.Output = 30000;
//    if (Keep_Y_PID.Output < -30000)
//        Keep_Y_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.y = Keep_Y_PID.Output;

//		World_Coordinate_system_TargetVel.y= 0;
//		World_Coordinate_system_TargetVel.w= 0;
//	}
//	if( ABS(Keep_Y_PID.Err) < 20 && ABS(Keep_Y_PID.Err) > 5 && ABS(Keep_X_PID.Err) < 5) 
//	{
//		 
//		 Pid_Calculate(&Keep_W_PID, Action_Data.angle_Z, Angle);
//    //	Pid_Calculate(&Keep_W_PID,Action_Data.w,Angle,1);
//    if (Keep_W_PID.Output > 30000)
//        Keep_W_PID.Output = 30000;
//    if (Keep_W_PID.Output < -30000)
//        Keep_W_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.w = Keep_W_PID.Output;

//    Pid_Calculate(&Keep_X_PID, Action_Data.x, X);
//    if (Keep_X_PID.Output > 30000)
//        Keep_X_PID.Output = 30000;
//    if (Keep_X_PID.Output < -30000)
//        Keep_X_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.x = Keep_X_PID.Output;

//    Pid_Calculate(&Keep_Y_PID, Action_Data.y, Y);
//    if (Keep_Y_PID.Output > 30000)
//        Keep_Y_PID.Output = 30000;
//    if (Keep_Y_PID.Output < -30000)
//        Keep_Y_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.y = Keep_Y_PID.Output;

//		World_Coordinate_system_TargetVel.x= 0;
//		World_Coordinate_system_TargetVel.w= 0;
//	}
//	if( ABS(Keep_X_PID.Err) < 5 && ABS(Keep_Y_PID.Err) < 5)
//	{
//	  Pid_Calculate(&Keep_W_PID, Action_Data.angle_Z, Angle);
//    //	Pid_Calculate(&Keep_W_PID,Action_Data.w,Angle,1);
//    if (Keep_W_PID.Output > 30000)
//        Keep_W_PID.Output = 30000;
//    if (Keep_W_PID.Output < -30000)
//        Keep_W_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.w = Keep_W_PID.Output;
//		
//		Pid_Calculate(&Keep_X_PID, Action_Data.x, X);
//    if (Keep_X_PID.Output > 30000)
//        Keep_X_PID.Output = 30000;
//    if (Keep_X_PID.Output < -30000)
//        Keep_X_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.x = Keep_X_PID.Output;

//    Pid_Calculate(&Keep_Y_PID, Action_Data.y, Y);
//    if (Keep_Y_PID.Output > 30000)
//        Keep_Y_PID.Output = 30000;
//    if (Keep_Y_PID.Output < -30000)
//        Keep_Y_PID.Output = -30000;
//    World_Coordinate_system_TargetVel.y = Keep_Y_PID.Output;
//		
//		World_Coordinate_system_TargetVel.x= 0;
//		World_Coordinate_system_TargetVel.y= 0;
//	}
	
    //	World_Coordinate_system_Velocity.x=0;
    //	World_Coordinate_system_Velocity.y=0;
    //	Robot_Coordinate_system.Angular_velocity=0;
    // ����XY�����ٶ�
    //	Keep_X_PID.target=X;
    //	Keep_X_PID.Input=Action_Data.x;
    //	PID_Calculate(&keep_Slow_PID);
    ////	if(Slow_PID.Output>500)Slow_PID.Output=500;
    ////	else if(Slow_PID.Output<-500)Slow_PID.Output=-500;
    //	World_Coordinate_system_Velocity.x=keep_Slow_PID.Output;
    //
    //	Keep_Y_PID.target=Y;
    //	Keep_Y_PID.Input=Action_Data.y;
    //	PID_Calculate(&keep_Adjust_PID);
    //	World_Coordinate_system_Velocity.y=keep_Adjust_PID.Output;

    Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Zero_Point, Action_Data.angle_Z);
}

//void PID_Init(PID_TypeDef *pid, float pp, float Pi, float pd, float i,float dp,float di,float dd,float Dead_Band, float forward)
//{
//    pid->Kp_kp = pp;
//    pid->Kp_ki = Pi;
//    pid->Kp_kd = pd;
//	  pid->Kd_kp = dp;
//    pid->Kd_ki = di;
//    pid->Kd_kd = dd;
//	  pid->Ki = i;
//    pid->dead_band = Dead_Band;
//    pid->Forward = forward;
//}
void PID_Init(PID_TypeDef *pid,float p,float i,float d,float Dead_Band,float forward)
{
   pid->Kp=p;
	 pid->Ki=i;
	 pid->Kd=d;
	 pid->dead_band=Dead_Band;
	 pid->Forward=forward;
	
}

void PID_Path_Init(void)
{
    PID_Init(&Keep_W_PID, 5, 0.06, 0, 0.5, 30);//300,250,250
    PID_Init(&Keep_X_PID, 5, 0.01, 0, 1, 30);
    PID_Init(&Keep_Y_PID, 5, 0.01, 0, 1, 30);
//    PID_Init(&Keep_W_PID,0.05, 5, 0, 0.06, 0.000, 1, 0, 0.5, 30);
//    PID_Init(&Keep_X_PID, 0.05, 5, 0, 0.01, 0.000, 1, 0, 1, 30);
//    PID_Init(&Keep_Y_PID, 0.05, 5, 0, 0.01, 0.000, 1, 0, 1, 30);
//    PID_Init(&Line_AnglePID, 5, 0.001, 0, 0.1, 250);
//    PID_Init(&Line_StopPID, 5, 0.01, 0, 1, 250);
//    PID_Init(&Line_AdjustPID, 5, 0.01, 0, 1, 0);

//    PID_Init(&Cir_AdjustPID, 5, 0.001, 0, 0.1, 250);
//    PID_Init(&Cir_StopPID, 5, 0.01, 0, 1, 250);
//    PID_Init(&Cir_AdjustPID, 5, 0.01, 0, 1, 0);
}
