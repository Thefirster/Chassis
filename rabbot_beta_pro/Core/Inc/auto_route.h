#include "stm32f4xx.h"
void Keep_Position(float X, float Y, float W);
void keep_pid_init(void);


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
		float x_break;                   // x��������·��

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
		float angle_break;                // �Ƕ�����·������

        float Target_Angle_Sum; // Ŀ���ۼ�Բ�Ľ�
        float Now_Angle_Sum;    // ��ǰ�ۼ�Բ�Ľ�
        float R;                // �뾶

        float speed_up_rate;    // �����������
        float speed_down_rate;  // �����������
        float wspeed_up_rate;   // ���ٶȼ����������
        float wspeed_down_rate; // ���ٶȼ����������
        float distance;         // ��Ŀ���ľ���
    } Parameter;

} LUJING_STU;
typedef struct PID_TypeDef
{
    float Target;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Err;
    float Last_Err;
    float Last_Last_Err;
    float Forward;
    float dead_band;
    float Output, ITerm;
	
	  float Kp_kp;
	  float Kp_ki;
	  float Kp_kd; 
	  float Kp_Err;
		float Kp_Last_Err;
	  float Kp_max;
	   
		
	  float Kd_kp;
	  float Kd_ki;
	  float Kd_kd;  
		float Kd_Err;
		float Kd_Last_Err;
	
} PID_TypeDef;
void PID_Path_Init(void);
void PID_Init(PID_TypeDef *pid,float p,float i,float d,float Dead_Band,float forward);

void circle(float start_x,float start_y,float Target_x, float Target_y, float angle, int start_v, int end_v, int v_max, float up_rate, float down_rate);
float Robot_Control_Circle(LUJING_STU *Status);
void keep_pid_init(void);
void Keep_Position(float X, float Y, float W);
float fitting_function2(float x);
float fitting_function(float x);
coordinate_Struct coordinate_transformation(coordinate_Struct *Coordinitioate_To_Convert, coordinate_Struct *CoordinateSystem, float angle);
float CalculateLineAngle(coordinate_Struct pointStart, coordinate_Struct pointEnd);
float CalculatePoint2Pointdistance(coordinate_Struct point1, coordinate_Struct point2);
void line(float Target_x, float Target_y, float rotate_Sum, int Start_Speed, int End_Speed, float up_rate, float down_rate, int Max_w, int speed_max, float Stop_length);
void lujing_set_break_condition(LUJING_STU * lujing, float x_break, float angle_break);
float Robot_Control_Line(LUJING_STU *Status);

static void PID_Clear(PID_TypeDef *PID);
