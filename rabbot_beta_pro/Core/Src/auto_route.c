#include "auto_route.h"
#include "motion_control.h"
#include "handkey.h"
#include "M3508.h"
#include "math.h"
#include "M3508.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"






// �Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN (0.01745329251994f)
// ������ת��Ϊ�Ƕ���ϵ��
#define CHANGE_TO_ANGLE (57.29577951308232f)
#define pi (3.1415926)
#define PI (3.1415926)
extern Action_data Action_Data;

PID_Struct keep_x;
PID_Struct keep_y;
PID_Struct keep_w;
extern float vy_set,vx_set,vw_set,w_set;
extern float vx_0,vy_0;
//extern float Action_Data.angle_Z;
LUJING_STU Lujing_Status;
extern int EMR_stop;
coordinate_Struct World_Coordinate_system_NowPos;    // ��������ϵ�µ�ǰλ��
coordinate_Struct World_Coordinate_system_TargetVel; // ��������ϵ��Ŀ���ٶ�
coordinate_Struct Robot_Coordinate_system_TargetVel; // ��������ϵ��Ŀ���ٶ�
coordinate_Struct Robot_Coordinate_system_Vel; // ·��������ĳ����ٶȣ����ڳ����ٶȷ���
coordinate_Struct action_data_now;
coordinate_Struct Zero_Point = {0, 0};               // ��������ϵ���
coordinate_Struct keep_point;
PID_Struct Cir_AdjustPID;  // Բ������pid����
PID_Struct Cir_StopPID;    // Բ��ֹͣ������pid����
PID_Struct Cir_AnglePID; 
PID_TypeDef Keep_X_PID;     // ����x����λ��
PID_TypeDef Keep_Y_PID;     // ����y����λ��
PID_TypeDef Keep_W_PID;     // ���ֽǶ�
PID_TypeDef Keep_X_PID1;     // ����x����λ��
PID_TypeDef Keep_Y_PID1;     // ����y����λ��
PID_TypeDef Keep_W_PID1;     // ���ֽǶ�
PID_TypeDef Line_AdjustPID; // ֱ�߷���pid����
PID_TypeDef Line_StopPID;   // ֱ��ֹͣ������pid����
PID_TypeDef Line_AnglePID;  // ֱ�߽Ƕ�pid����
extern float watch[10];

static void PID_Clear(PID_TypeDef *PID)
{
    PID->Output = 0;
    PID->Last_Err = 0;
    PID->ITerm = 0;
}

float mis_error;
float CalculatePoint2Pointdistance(coordinate_Struct point1, coordinate_Struct point2)
{
    float dis;
    dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
    return dis;
}
float CalculateLineAngle(coordinate_Struct pointStart, coordinate_Struct pointEnd)
{
    float a = 0.0f;
    float b = 0.0f;

    a = pointEnd.y - pointStart.y;
    b = pointEnd.x - pointStart.x;
    // atan2f��Χ���԰���-180��180
    return (atan2f(a, b) * CHANGE_TO_ANGLE);
}
static float ABS(float a)
{
    return a > 0 ? a : -a;
}
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
    v_rate = -2.7855f * x * x * x * x + 7.1212f * x * x * x - 6.7544f * x * x + 3.7167f * x + 0.1106f;

    return v_rate/1.414f;
}

float fitting_function2(float x)
{
    float v_rate;
    x = 1 - x;
        v_rate = -6.8344f * x * x * x * x + 16.973f * x * x * x - 14.882f * x * x + 5.7219f * x + 0.0117f;

    return v_rate/1.414f;
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

    if ((ABS(pid->Err) < pid->dead_band&&pid->dead_band!=0))
		{
        pid->Output = 0;
		    pid->ITerm = 0;
		}
    else
    {
						  if(ABS(pid->Err) > 0.1){pid->ITerm += pid->Err;}
        if (pid->ITerm > 300000)
            pid->ITerm = 300000;
        else if (pid->ITerm < -300000)
            pid->ITerm = -300000;
        pid->Output = pid->Kp * pid->Err  + pid->Ki * pid->ITerm + pid->Kd * (pid->Err - pid->Last_Err) + (pid->Err > 0 ? pid->Forward : -pid->Forward);

        pid->Last_Err = pid->Err;
    }
}
void PID_Path_Init(void)
{
    PID_Init(&Keep_W_PID, 50,0,5, 0.5, 0);//20, 0.01, 0
    PID_Init(&Keep_X_PID, 25, 0, 2, 0.5, 0);
    PID_Init(&Keep_Y_PID, 25, 0, 2, 0.5, 0);//5,0p.01,0
	
		PID_Init(&Line_AnglePID, 25, 0, 5, 0, 0);
    PID_Init(&Line_StopPID, 20, 0.01, 5, 1, 0);
    PID_Init(&Line_AdjustPID, 10, 0.01, 2, 1, 0);
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
void PID_Init(PID_TypeDef *pid,float p,float i,float d,float Dead_Band,float forward)
{
   pid->Kp=p;
	 pid->Ki=i;
	 pid->Kd=d;
	 pid->dead_band=Dead_Band;
	 pid->Forward=forward;
	
}

void Keep_Position(float X, float Y, float W)
{
	//Keep_X_PID1
//	Keep_X_PID.Kp = 3*ABS(Keep_X_PID.Err*Keep_X_PID.Err)/(ABS(Keep_X_PID.Err*Keep_X_PID.Err)+ABS(Keep_Y_PID.Err*Keep_Y_PID.Err)+ABS(Keep_W_PID.Err*Keep_W_PID.Err));
//	Keep_Y_PID.Kp = 3*ABS(Keep_Y_PID.Err*Keep_Y_PID.Err)/(ABS(Keep_X_PID.Err*Keep_X_PID.Err)+ABS(Keep_Y_PID.Err*Keep_Y_PID.Err)+ABS(Keep_W_PID.Err*Keep_W_PID.Err));
	//Keep_W_PID.Kp = 3*ABS(Keep_W_PID.Err*Keep_W_PID.Err)/(ABS(Keep_X_PID.Err*Keep_X_PID.Err)+ABS(Keep_Y_PID.Err*Keep_Y_PID.Err)+ABS(Keep_W_PID.Err*Keep_W_PID.Err));
//	keep_x.Input=Action_Data.x;
//	keep_y.Input=-Action_Data.y;
//	keep_w.Input=Action_Data.angle_Z;
//	
//	keep_x.target=X;
//	keep_y.target=Y;
//	keep_w.target=W;
	float vx_1,vy_1,vw_1,vsum;
	Pid_Calculate(&Keep_X_PID,Action_Data.x,X);
	Pid_Calculate(&Keep_Y_PID,Action_Data.y,Y);
	Pid_Calculate(&Keep_W_PID,Action_Data.angle_Z,W);
	if(fabs(Keep_X_PID.Err)<0.5)vx_1=0;
	else{
	if(Keep_X_PID.Output>3000)vx_1=3000;
			else if(Keep_X_PID.Output<-3000)vx_1= -3000;
			else vx_1=(int16_t)Keep_X_PID.Output;
	}
	
	if(fabs(Keep_Y_PID.Err)<0.5)vy_1=0;
	{
	if(Keep_Y_PID.Output>3000)vy_1=3000;
			else if(Keep_Y_PID.Output<-3000)vy_1= -3000;
			else vy_1=(int16_t)Keep_Y_PID.Output;
	}
	if(fabs(Keep_W_PID.Err)<0.5)vw_1=0;
	{
	if(Keep_W_PID.Output>3000)vw_1=3000;
			else if(Keep_W_PID.Output<-3000)vw_1= -3000;
			else vw_1=(int16_t)Keep_W_PID.Output;
	}
	
	

//	vsum=sqrt(vx_1*vx_1+vy_1*vy_1+vw_1*vw_1);
//	if(vsum>3000)
//	{
//		vx_1=vx_1/vsum*3000;
//		vy_1=vy_1/vsum*3000;
//		vw_1=vw_1/vsum*3000;
//	}

	
//	          vx_set=vy_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)+vx_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
//            vy_set=vy_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)-vx_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);

	vx_set=-vy_1*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)+vx_1*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
	vy_set=-vy_1*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)-vx_1*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
	
	vw_set=vw_1;
	
	motion_resolve(-vx_set,vy_set,vw_set);
}


void keep_pid_init()
{
	keep_x.KP=10;
	keep_x.KI=0;
	keep_x.KD=0;
	keep_y.KP=10;
	keep_y.KI=0;
	keep_y.KD=0;
	keep_w.KP=10;
	keep_w.KI=0;
	keep_w.KD=0;
	
	Cir_AdjustPID.KP=70;
	Cir_AdjustPID.KI=0.3;
	Cir_AdjustPID.KD=40;
	
	
	Cir_AdjustPID.DeadBand=0;
	Cir_AdjustPID.Forward=0;
	Cir_StopPID.KP=10;
	Cir_StopPID.KI=0.01;
	Cir_StopPID.KD=4;
	Cir_AnglePID.KP=20;
	Cir_AnglePID.KI=0.01;
	Cir_AnglePID.DeadBand=0.1;
	Cir_AnglePID.KD=0;
}
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
//        PID_Clear(&Cir_AdjustPID);
//        PID_Clear(&Cir_StopPID);
//        PID_Clear(&Cir_AnglePID);

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
    Status->CoordinateSystem.Polar_NowPos.Rho = CalculatePoint2Pointdistance(Status->CoordinateSystem.World_HeartPos, World_Coordinate_system_NowPos);
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
		//watch[1]=Status->Parameter.Now_Angle_Sum;
    Angle_last = Status->CoordinateSystem.Polar_NowPos.Theta; // ��ǰ����λ�ü��Ǳ�Ϊlast
    // �ټ�����Ŀ���ľ����ֵ
    arc_remain = ABS(Status->Parameter.Target_Angle_Sum - Status->Parameter.Now_Angle_Sum) ;

    /* 3 ���㼫�����ٶ�����ϵ�µ������ٶ� */
    // ����ڼ�����
    if (ABS(Status->Parameter.Now_Angle_Sum)  < ABS(Status->Parameter.Speedup_Length))
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // �����ٶ�
            Status->Parameter.Start_Speed +
            fitting_function(Status->Parameter.Now_Angle_Sum  / Status->Parameter.Speedup_Length) *
                (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
			//watch[0]= Status->CoordinateSystem.Polar_TargetVel.x;
    }
    // ���û�ڼ������⣬�����ٶ�Ϊ����ٶ�
    else if (ABS(Status->Parameter.Now_Angle_Sum) > ABS(Status->Parameter.Speedup_Length) && ABS(Status->Parameter.Now_Angle_Sum) < ABS(Status->Parameter.Target_Angle_Sum - Status->Parameter.Slow_length))
    {
        Status->CoordinateSystem.Polar_TargetVel.x = Status->Parameter.Max_Speed;
			//watch[0]= Status->CoordinateSystem.Polar_TargetVel.x;
    }
    // ����ڼ������ڣ�ֹͣ����
    else if (ABS(Status->Parameter.Now_Angle_Sum) >= ABS(Status->Parameter.Target_Angle_Sum - Status->Parameter.Slow_length))
    {
        Status->CoordinateSystem.Polar_TargetVel.x = // �����ٶ�
            Status->Parameter.Max_Speed -
            fitting_function2(arc_remain / Status->Parameter.Slow_length) * (Status->Parameter.Max_Speed-Status->Parameter.End_Speed);
			//watch[0]= Status->CoordinateSystem.Polar_TargetVel.x;
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
	//Cir_AdjustPID.Forward=50.0f/1000.0f*Status->Parameter.Max_Speed;
//		Cir_AdjustPID.Forward=20.0f;
//	Cir_AdjustPID.KP=40.0f/1000.0f*Status->Parameter.Max_Speed;
//	Cir_AdjustPID.KI=0.3f/1000.0f*Status->Parameter.Max_Speed;
//	Cir_AdjustPID.KD=10.0f/1000.0f*Status->Parameter.Max_Speed;
    /* 4 ���㼫�����ٶ�����ϵ�µķ������ٶ� */
    // �����ٶ�PID����
    PID_Calculate(&Cir_AdjustPID, Status->CoordinateSystem.Polar_NowPos.Rho, Status->Parameter.R);
   Status->CoordinateSystem.Polar_TargetVel.y = Cir_AdjustPID.Output;
if(Status->CoordinateSystem.Polar_TargetVel.y>10000)Status->CoordinateSystem.Polar_TargetVel.y=10000;
else if(Status->CoordinateSystem.Polar_TargetVel.y<-10000)Status->CoordinateSystem.Polar_TargetVel.y=-10000;
    /* 5 ������ٶ� */
			if (Lujing_Status.Flag.circle_tpye == Robot_LineType_Cycle_Anticlockwise)
		{
				Status->CoordinateSystem.Polar_TargetVel.x = -Status->CoordinateSystem.Polar_TargetVel.x;
			//Status->CoordinateSystem.Polar_TargetVel.y =0;
		}
    // ��������ߣ�����Ŀ��Ƕ�
    if (Status->Flag.yanqiexian)
    {
        Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Now_Angle_Sum;
    }

    // PID���ֳ���Ƕ�
    PID_Calculate(&Cir_AnglePID, Action_Data.angle_Z, Status->CoordinateSystem.Target_Position.w);
    Robot_Coordinate_system_Vel.w = Cir_AnglePID.Output;

    /* ���������ٶ�����ϵ������ƫ�����ʱ��С�������ٶ�(������) */
//    if (ABS(Status->CoordinateSystem.Polar_TargetVel.y) > 1000)
//    {
//        Status->CoordinateSystem.Polar_TargetVel.x *= 0.5f;
//    }
    /* 6 ���������ٶ�����ϵ�µ��ٶ�ת������������ϵ*/
    // ���㼫�����ٶ�����ϵ����������ϵ�н�
    Slope_angle = Status->CoordinateSystem.Polar_NowPos.Theta - 90.0f;
    /* ���������ٶ�����ϵ���ٶ�ת������������ϵ���ٶ� */
    World_Coordinate_system_TargetVel = coordinate_transformation(&Status->CoordinateSystem.Polar_TargetVel, &Status->CoordinateSystem.Zero_Point, -Slope_angle);

    /* ����������ϵ���ٶ�ת���ɳ�������ϵ���ٶ� */
    //Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Action_Data.angle_Z);
	vx_0=World_Coordinate_system_TargetVel.x;
	vy_0=-World_Coordinate_system_TargetVel.y;
	vw_set=Robot_Coordinate_system_Vel.w;
	
	vx_set=vy_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)+vx_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
	vy_set=vy_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)-vx_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
	
    // �ж�·���Ƿ�����
    if (ABS(Status->Parameter.Now_Angle_Sum)>=(ABS(Status->Parameter.Target_Angle_Sum)-0.5f))
    {
        if (Status->Flag.yanqiexian)
            Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w + Status->Parameter.Target_Angle_Sum;
        else
            Status->CoordinateSystem.Target_Position.w = Status->CoordinateSystem.Start_Position.w;
        Status->Flag.Work_Start = DISABLE;
    }

    return (arc_remain);
}

// Բ����Ŀ��x��Ŀ��y��Բ�Ľ�(����)��V����Vĩ��V����Ƿ����У�������������������������
void circle(float start_x,float start_y,float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, float up_rate, float down_rate)
{
    Lujing_Status.Flag.Work_Start = ENABLE;
    Lujing_Status.Flag.NewState = ENABLE;
//			Target_x+=Action_Data.x;
//			Target_y+=Action_Data.y;
			if(angle<0){
				angle=360+angle;
				Lujing_Status.Flag.circle_tpye = Robot_LineType_Cycle_Anticlockwise;
			}
			else Lujing_Status.Flag.circle_tpye = Robot_LineType_Cycle_Clockwise;
	
    float gama, R, ygxl, d, c_x, c_y;
    float rad ;
		rad= angle * CHANGE_TO_RADIAN;
    d = sqrt((Target_x - start_x) * (Target_x - start_x) + (Target_y - start_y) * (Target_y - start_y));
    R = d / 2.0f / sinf(rad / 2.0f);
		
        if ((Target_y - start_y) >= 0)
        {
            gama = acosf((Target_x - start_x) / d);
        }
        else if ((Target_y - start_y) < 0)
        {
            gama = acosf((start_x - Target_x) / d) + pi;
        }
        ygxl = gama - (pi / 2 - rad / 2);

        c_x = start_x + R * cos(ygxl);
        c_y = start_y + R * sin(ygxl);

    Lujing_Status.CoordinateSystem.Start_Position.x = start_x; // ��ʼλ��
    Lujing_Status.CoordinateSystem.Start_Position.y = start_y;
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

		//Cir_AdjustPID.DeadBand=0.1;
	
    while (Lujing_Status.Flag.Work_Start&&EMR_stop)
    {
      Robot_Control_Circle(&Lujing_Status);
			motion_resolve(-vx_set,vy_set,vw_set);// vesc�ٶȼӸ���
			mis_error=sqrt((Action_Data.x-c_x)*(Action_Data.x-c_x)+(Action_Data.y-c_y)*(Action_Data.y-c_y))-R;
      osDelay(2);
    }
}

 float Angle_Aim = 0;    // Ŀ��Ƕ�ֵ���м����
  float Angle_offset = 0; // ʣ��Ҫת�ĽǶȣ��м����
 float Line_angle = 0;   // ������ϵ����������ϵ�н�
 float Angle_dis = 0;    // �Ƕ�ת�����룬�м����
float vx_0,vy_0;
float Robot_Control_Line(LUJING_STU *Status)
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
        Status->Parameter.Slow_angle = Status->Parameter.wspeed_down_rate * ABS(Status->Parameter.angle_rotate_Sum-Action_Data.angle_Z);
        /* �����������С */
        Status->Parameter.Speedup_angle = Status->Parameter.wspeed_up_rate * ABS(Status->Parameter.angle_rotate_Sum-Action_Data.angle_Z);

        /* 0.3 ����Ŀ��Ƕ� */
        /* ��¼·����ʼ�Ƕȣ���ǰ·����ʼ�Ƕ�Ϊ��һ��·����Ŀ��Ƕ� */
        Status->CoordinateSystem.Start_Position.w = Action_Data.angle_Z;

        /* ���㵱ǰ·��Ŀ��Ƕȣ�Ŀ��Ƕ�Ϊ��ʼ�Ƕ�+Ҫת�ĽǶ� */
        Angle_Aim = Status->Parameter.angle_rotate_Sum;

        /* ������һ��·���ĳ�ʼ�Ƕ� */
        //Status->CoordinateSystem.Target_Position.w = Angle_Aim;
//				Status->Parameter.angle_rotate_last_Sum = Status->Parameter.angle_rotate_Sum;

        /* 0.4 ��pid������һ��·���������� */
        PID_Clear(&Line_AdjustPID);
        PID_Clear(&Line_StopPID);
        PID_Clear(&Line_AnglePID);
    }

    /* ������������ϵ�µ�ǰ�����˵����� */
    World_Coordinate_system_NowPos.x = Action_Data.x;
    World_Coordinate_system_NowPos.y = Action_Data.y;
    World_Coordinate_system_NowPos.w = Action_Data.angle_Z;
    /* ����������ϵ�µ�ǰ�����˵����� */
    Status->CoordinateSystem.Line_NowPos = coordinate_transformation(&World_Coordinate_system_NowPos, &Status->CoordinateSystem.Start_Position, Line_angle);
	if (Status->CoordinateSystem.Line_NowPos.x < 0)
		Status->CoordinateSystem.Line_NowPos.x = 0;
    /* 1. �����������ٶ� */
    Angle_offset = ABS(Angle_Aim - Action_Data.angle_Z);

    Angle_dis = ABS(Action_Data.angle_Z- Status->CoordinateSystem.Start_Position.w);
    /* 1.1 ��ǰ�Ƕ��ڼ������ڣ�����Ŀ����ٶ� */
//    if (Angle_dis < Status->Parameter.Speedup_angle)
//    {
//        Status->CoordinateSystem.Line_TargetVel.w =
//            Status->Parameter.Start_w_Speed +                                      // ���ٶ�
//            fitting_function(Angle_dis / Status->Parameter.Speedup_angle) *        // ���ٱ���
//                (Status->Parameter.Max_w_Speed - Status->Parameter.Start_w_Speed); // �ٶȱ仯��
//    }
//    /* 1.2 ������ڼ������ڣ������ڼ������⣬���ٶ�Ϊ�����ٶ� */
//    else if (Angle_dis > Status->Parameter.Speedup_angle && Angle_offset > Status->Parameter.Slow_angle&&Angle_dis <Angle_Aim - Status->Parameter.Slow_angle)
//    {
//       Status->CoordinateSystem.Line_TargetVel.w= Status->Parameter.Max_w_Speed;
//    }
//    /* 1.3 ����ڼ�������ֹͣ���� */
//    else if (Angle_offset < Status->Parameter.Slow_angle && Angle_offset > Status->Parameter.Stop_angle)
//    {
//        Status->CoordinateSystem.Line_TargetVel.w =
//            Status->Parameter.Max_w_Speed -                                 // ����ٶ�
//            fitting_function2(Angle_offset / Status->Parameter.Slow_angle) * // ���ٱ���
//                (Status->Parameter.Max_w_Speed);       
//watch[0]=fitting_function2(Angle_offset / Status->Parameter.Slow_angle);		// �ٶȱ仯��
//    }
    /* 1.4ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ */
//    else if (Angle_offset < Status->Parameter.Stop_angle||Angle_Aim<=0.5) //(fabs(Status->Parameter.End_w_Speed) < 0.001f && Status->Parameter.Stop_angle != 0)
//    {
        Pid_Calculate(&Line_AnglePID, Action_Data.angle_Z, Angle_Aim);
				if(Line_AnglePID.Output>Status->Parameter.Max_w_Speed)          Status->CoordinateSystem.Line_TargetVel.w= Status->Parameter.Max_w_Speed;
				else if(Line_AnglePID.Output<-Status->Parameter.Max_w_Speed)    Status->CoordinateSystem.Line_TargetVel.w=-Status->Parameter.Max_w_Speed;
				else Status->CoordinateSystem.Line_TargetVel.w = Line_AnglePID.Output;
//    }

    /* 1.5 ˳ʱ����ת���ٶ�ȡ�� */
//    if (Status->Parameter.angle_rotate_Sum < 0)
//    {
//        Status->CoordinateSystem.Line_TargetVel.w *= -1;
//    }

    /* 1.6 �ж�·���Ƿ����� */
//    if (Angle_offset < 0.5f)
//    {
//        Status->Parameter.angle_rotate_Sum = 0; // ���㣬�Ժ���������ת������
//    }

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
                (Status->Parameter.Max_Speed - Status->Parameter.End_Speed);
    }
    /* 2.1.4ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ */

	else{
		Pid_Calculate(&Line_StopPID, Status->CoordinateSystem.Line_NowPos.x, Status->CoordinateSystem.Line_TargetPos.x);
		Status->CoordinateSystem.Line_TargetVel.x=Line_StopPID.Output;
	}

    /* 2.2 �����ٶȼ��� */
    if (ABS(Status->CoordinateSystem.Line_NowPos.x - Status->CoordinateSystem.Line_TargetPos.x) > 10) // �쵽ֱ�߽�βʱ�����ڷ����ٶȣ�����������Ӷ���
    {
        Pid_Calculate(&Line_AdjustPID, Status->CoordinateSystem.Line_NowPos.y, 0);
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
    //Robot_Coordinate_system_Vel = coordinate_transformation(&World_Coordinate_system_TargetVel, &Status->CoordinateSystem.Zero_Point, Action_Data.angle_Z);

    /* ��ת�����󱣳ֳ���Ƕ� */
//    if (Angle_offset < 0.5f || Status->Parameter.angle_rotate_Sum == 0)
//    {
//        /* ���ֳ���Ƕ� */
//        Pid_Calculate(&Line_AnglePID,Action_Data.angle_Z, Status->CoordinateSystem.Target_Position.w);
//        Robot_Coordinate_system_Vel.w = Line_AnglePID.Output;
//    }
	vx_0=World_Coordinate_system_TargetVel.x;
	vy_0=-World_Coordinate_system_TargetVel.y;
	vw_set=World_Coordinate_system_TargetVel.w;
	vx_set=vy_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)+vx_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
	vy_set=vy_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)-vx_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
    /* 4. �ж�·���Ƿ����� */
    if ((Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x < Lujing_Status.Parameter.x_break)&&(Angle_offset <=Lujing_Status.Parameter.angle_break))
    {
        Status->CoordinateSystem.Target_Position.w = Angle_Aim;
        Status->Flag.Work_Start = DISABLE;
    }

    return (Status->CoordinateSystem.Line_TargetPos.x - Status->CoordinateSystem.Line_NowPos.x);
}



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
	Lujing_Status.Parameter.Stop_angle = 2.0f; 
	
	Lujing_Status.Parameter.wspeed_up_rate = 0.1;
	Lujing_Status.Parameter.wspeed_down_rate = 0.1;
	
	
	Lujing_Status.Parameter.x_break = 30.0f;
	Lujing_Status.Parameter.angle_break = 5.0f;

		
		
                        // ֹͣ������(�Ƕ�)
    while (Lujing_Status.Flag.Work_Start&&EMR_stop)
    {
        Robot_Control_Line(&Lujing_Status);
        motion_resolve(-vx_set,vy_set,vw_set);//VESC�����������vx�Ӹ���
        osDelay(2);
    }
	keep_point.x = Lujing_Status.CoordinateSystem.Target_Position.x;
	
}




void lujing_set_break_condition(LUJING_STU * lujing, float x_break, float angle_break)
{	
	lujing->Parameter.x_break = x_break;
	lujing->Parameter.angle_break = angle_break;
}

//void line_timer(float speed_vx,float speed_vy,int time)
//{
//	float target_vx,target_vy,target_vw;
//	Pid_Calculate(&Line_AnglePID, Action_Data.angle_Z, Angle_Aim);
//  Status->CoordinateSystem.Line_TargetVel.w = Line_AnglePID.Output;
//	
//	
//	
//}