#include "yuntai.h"
#include "stdint.h"
#include "A1_motor.h"
#include "ANO_TC.h"
#include "usart.h"
#include "math.h"
#include "cmsis_os.h"
#define L_radar_x 0.212f
#define L_radar_y 0.073f
#define Xita  1.239173f

Yuntai yuntai_target; // 速度位置模式下云台目标偏航与俯仰角(弧度制)
extern uint8_t vision_recv[12];
float Send2A1;
float yaw_kp = 0.1;
float yaw_kw = 0.5;
extern int path_flag;
float kalman_data_x = 0; // 经滤波后的x坐标
float ProcessNiose_Q = 1;
float MeasureNoise_R = 100;
/* ----------------------------------视觉通信----------------------------------- */
float final=0;
	double delta_x,delta_y,result,delta_yaw,xita,target,x0,y0;
float a=0.25;
float watch_w[500];
extern float a1_pos[2];
extern A1_data a1_data[2];
extern float radar_x,radar_y;
extern float init_pos_p,init_pos;
float Zhuzi_id_x_y[11][2];
// 一阶滤波
float firstOrderFilter(float in_data)
{
	final = a*in_data + (1-a)*final;	
	return(final);
}



VisionData vision_data;

void Usart3_Start() // 视觉通信
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, vision_recv, 12);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

uint8_t rsv_flag;
void vision_data_process()
{
    static float x = 0;
    static float y = 0;
    static float z = 0;
    if (vision_recv[0] == 0xAC) // 帧头
    {	
		
        x = vision_recv[1] * 255 + vision_recv[2];
        if (x != 0) // 0为错误码
        {
            if (vision_recv[3] == 1) // 代表负数
                vision_data.x = -x * 0.01745329251994f * 0.001f;
            else
                vision_data.x = x * 0.01745329251994f * 0.001f;
        }
	
    }
		else  if (vision_recv[0] == 0xAD) // 帧头
    {	
		
        y = vision_recv[1] * 255 + vision_recv[2];
        if (y != 0) // 0为错误码
        {
            if (vision_recv[3] == 1) // 代表负数
                vision_data.y = -y * 0.01745329251994f * 0.001f;
            else
                vision_data.y = y * 0.01745329251994f * 0.001f;
        }
	
    }
//    kalman_data_x = firstOrderFilter(vision_data.x);
}

/* ----------------------------------视觉追踪pid----------------------------------- */

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Target;
    float Measure;
    float Err;
    float Last_Err;
    float Last_Last_Err;

    float p_out;
    float i_out;
    float d_out;
    float Output;

} PID_TypeDef;

PID_TypeDef yuntai_yaw_speed_pid;
PID_TypeDef yuntai_yaw_position_pid;
PID_TypeDef yuntai_yaw_offest_pid;



static void limit(float *out, float max)
{
    if (*out > max)
        *out = max;
    if (*out < -max)
        *out = -max;
}

static void PID_Calculate_Incremental(PID_TypeDef *pid, float measure, float target)
{
    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;

    pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
    pid->i_out = pid->Ki * pid->Err;
    pid->d_out = pid->Kd * (pid->Err - 2 * pid->Last_Err + pid->Last_Last_Err);
    
    pid->Output += pid->p_out + pid->i_out + pid->d_out;
    pid->Last_Last_Err = pid->Last_Err;
    pid->Last_Err = pid->Err;
}

float integral_limit = 1000;
static void PID_Calculate_Positional(PID_TypeDef *pid, float measure, float target)
{
    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;
    pid->p_out = pid->Kp * pid->Err;
    pid->i_out = pid->Ki * pid->Last_Last_Err;
    pid->d_out = pid->Kd * (pid->Err - pid->Last_Err);
    pid->Output = pid->Kp * pid->Err + pid->Ki * pid->Last_Last_Err + pid->Kd * (pid->Err - pid->Last_Err);
    pid->Last_Err = pid->Err;
    pid->Last_Last_Err += pid->Err;

    limit(&pid->Last_Last_Err, integral_limit);
}

/* ----------------------------------滤波----------------------------------- */
typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
    float H;
}kalman;

kalman kal;

void kalmanCreate(kalman *p,float T_Q,float T_R)
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}

float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}



typedef enum // 状态机
{
    Start = 0,
    PositionMode,
    SpeedMode,
    VisionMode,
    PathMode,
		ManualMode,
} YuntaiMode;
YuntaiMode yuntai_mode = PositionMode;
YuntaiMode yuntai_last_mode = Start;

typedef struct
{
    FunctionalState executable;
    
    float start_rad;
    float end_rad;
    
    float target_rad;
    float target_w;
    
    float start_time;
    float T;
}Yuntai_Path;

Yuntai_Path s_path[2];

// 三次多项式插值测试
void yuntai_path(uint8_t id, float start, float end, float tm, float t)
{
    if (t>tm)
    {
        s_path[id].executable = DISABLE;
        return;//
    }
    
    s_path[id].target_rad = start + 3.0f/(tm*tm)*(end-start)*t*t - 2.0f/(tm*tm*tm)*(end - start)*t*t*t;
    s_path[id].target_w = 6.0f/(tm*tm)*(end-start)*t - 6.0f/(tm*tm*tm)*(end - start)*t*t;
}

void yuntai_manual(uint8_t id)
{
	   //s_path[id].executable = ENABLE; 
//		a1_data[0].mode=10;
//	a1_data[1].mode=10;
	
		yuntai_mode = ManualMode;
}

uint32_t time_now = 0;
void yuntai_start_path(uint8_t id, float end_pos, float T)//弧度，到达目标位置的时间
{
    s_path[id].start_time = time_now;
    s_path[id].start_rad = a1_get_position(id);
    s_path[id].end_rad = end_pos;
    s_path[id].T = T;
    s_path[id].executable = ENABLE;    
    yuntai_mode = PathMode;
}
/* ----------------------------------云台控制逻辑----------------------------------- */
extern float end_pos_1,end_pos_2;
// 设置目标角度(传入输出轴弧度)
void yuntai_set_target_pitch(float x)
{
    yuntai_target.pitch = x;
		end_pos_2=x;
}

void yuntai_set_target_yaw(float x)
{
    yuntai_target.yaw = x;
	end_pos_1=x;
}

// 云台偏航角串级pid计算，输入输出轴目标弧度
float pos_now, w_now;
float yaw_speed_limit = 10; // 速度限幅
float yaw_torque_limit = 1;    // 力矩限幅
void yuntai_yaw_speed_angle_calc()
{
    pos_now = a1_get_position(YAW_MOTOR_ID);
    w_now = a1_get_speed(YAW_MOTOR_ID);

    PID_Calculate_Positional(&yuntai_yaw_position_pid, pos_now, yuntai_target.yaw);
    limit(&yuntai_yaw_position_pid.Output, yaw_speed_limit);

    PID_Calculate_Incremental(&yuntai_yaw_speed_pid, w_now, yuntai_yaw_position_pid.Output);
    limit(&yuntai_yaw_speed_pid.Output, yaw_torque_limit);
}

uint8_t number = 15;
// 云台偏航角串级pid计算，输入为视觉像素偏差量
float yuntai_yaw_speed_vision_calc()
{
//    float test_x = (float)vision_data.x;
//    inter_FIR(&test_x, &kalman_data_x, FRAME_READ_LENGTH);
    kalman_data_x = KalmanFilter(&kal, vision_data.x);
//    kalman_data_x = firstOrderFilter(vision_data.x);
    w_now = a1_get_speed(YAW_MOTOR_ID);

    PID_Calculate_Positional(&yuntai_yaw_offest_pid, kalman_data_x, 0);
    limit(&yuntai_yaw_offest_pid.Output, yaw_speed_limit);

    PID_Calculate_Incremental(&yuntai_yaw_speed_pid, w_now, yuntai_yaw_offest_pid.Output);
    limit(&yuntai_yaw_speed_pid.Output, yaw_torque_limit);

    return yuntai_yaw_speed_pid.Output;
}

void pid_clear()
{
    yuntai_yaw_offest_pid.Last_Err = 0;
    yuntai_yaw_offest_pid.Last_Last_Err = 0;
    yuntai_yaw_offest_pid.Output = 0;

    yuntai_yaw_speed_pid.Last_Err = 0;
    yuntai_yaw_speed_pid.Last_Last_Err = 0;
    yuntai_yaw_speed_pid.Output = 0;
}


float min = 40;
float speed = 0.1;
extern float yuntai_x;
extern float yuntai_y;
extern float a1_speed[2];
extern int pos_block[2];

int aa=0;
void mode_change()
{
		yuntai_mode=PositionMode;
}
float pitch_kp=0.7,pitch_kw=0.5;
//uint8_t send_flag = 0;
//uint8_t vision_send_data = 0; // -2 -1 0 1 2
void yuntai_control() // 放在tim2里2ms执行一次
{
    switch (yuntai_mode)
    {
    case PositionMode: // 保持位置
    {
        if (yuntai_last_mode != yuntai_mode)
        {
            a1_set_kp_kw(YAW_MOTOR_ID, yaw_kp, yaw_kw);
            a1_set_kp_kw(PITCH_MOTOR_ID, pitch_kp, pitch_kw);
            yuntai_set_target_yaw(a1_get_position(YAW_MOTOR_ID)); // 把当前位置定做目标位置
            yuntai_set_target_pitch(a1_get_position(PITCH_MOTOR_ID));
        }
        //a1_set_kp_kw(PITCH_MOTOR_ID, pitch_kp, pitch_kw);
				if(fabs(yuntai_target.yaw-a1_get_position(YAW_MOTOR_ID))<1&&fabs(yuntai_target.pitch-a1_get_position(PITCH_MOTOR_ID))<0.1){
        a1_set_pos(YAW_MOTOR_ID, yuntai_target.yaw);
        a1_set_pos(PITCH_MOTOR_ID, yuntai_target.pitch);}
				else
				{
				    yuntai_set_target_yaw(a1_get_position(YAW_MOTOR_ID)); // 把当前位置定做目标位置
            yuntai_set_target_pitch(a1_get_position(PITCH_MOTOR_ID));
				}
//        a1_set_w(PITCH_MOTOR_ID,target_w);
        break;
    }
    case SpeedMode: // 速度控制
    {
        kalman_data_x = KalmanFilter(&kal, vision_data.x);
        if (yuntai_last_mode != yuntai_mode)
        {
            a1_set_kp_kw(YAW_MOTOR_ID, 0, yaw_kw);
            a1_set_kp_kw(PITCH_MOTOR_ID, 0, yaw_kw);
        }
        if (kalman_data_x > min)
        {
            a1_set_w(YAW_MOTOR_ID, -speed);
        }
        else if (kalman_data_x < -min)
        {
            a1_set_w(YAW_MOTOR_ID, speed);
        }
        else
            a1_set_w(YAW_MOTOR_ID, 0);       
        break;
    }
    case VisionMode: // 用来视觉定位，力矩控制
    {
        if (yuntai_last_mode != yuntai_mode)
        {
//            a1_set_kp_kw(YAW_MOTOR_ID, 0, 0);
//            a1_set_kp_kw(PITCH_MOTOR_ID, yaw_kp, yaw_kw);
//            pid_clear(); // 清除累计误差
            a1_set_kp_kw(YAW_MOTOR_ID, yaw_kp, yaw_kw);
            a1_set_kp_kw(PITCH_MOTOR_ID, yaw_kp, yaw_kw);
            yuntai_set_target_yaw(a1_get_position(YAW_MOTOR_ID)); // 把当前位置定做目标位置
            yuntai_set_target_pitch(a1_get_position(PITCH_MOTOR_ID));
        }
        
//        a1_set_torque(YAW_MOTOR_ID, yuntai_yaw_speed_vision_calc());
//        a1_set_pos(PITCH_MOTOR_ID, yuntai_target.pitch);
        break;
    }
    case PathMode: // 用来做动作
    {
        for(uint8_t i = 0; i < 2; i ++)
        {
            if(s_path[i].executable == ENABLE)
            {
                float t = time_now / 1000.0f - s_path[i].start_time / 1000.0f;
                yuntai_path(i, s_path[i].start_rad, s_path[i].end_rad, s_path[i].T, t);
                if (i == YAW_MOTOR_ID)
                {
                    yuntai_target.yaw = s_path[YAW_MOTOR_ID].target_rad; // 更新目标位置
                    a1_set_pos(YAW_MOTOR_ID, yuntai_target.yaw); 
                    a1_set_w(YAW_MOTOR_ID, s_path[YAW_MOTOR_ID].target_w);
                }   
                if (i == PITCH_MOTOR_ID)
                {
                    yuntai_target.pitch = s_path[PITCH_MOTOR_ID].target_rad;
                    a1_set_pos(PITCH_MOTOR_ID, yuntai_target.pitch); 
                    a1_set_w(PITCH_MOTOR_ID, s_path[PITCH_MOTOR_ID].target_w);   
                }
            }
        }
				break;
    }
		case ManualMode:
		{
			    for(uint8_t i = 0; i < 2; i ++)
        {
								if (i == YAW_MOTOR_ID)
                {
										if(pos_block[1]){
											a1_set_kp_kw(YAW_MOTOR_ID, 0.1, yaw_kw);
											if(fabs(yuntai_target.yaw-a1_get_position(YAW_MOTOR_ID))<1)
											a1_set_pos(YAW_MOTOR_ID, yuntai_target.yaw);
											else  yuntai_set_target_yaw(a1_get_position(YAW_MOTOR_ID));
										}
										else a1_set_kw_w(YAW_MOTOR_ID,3,a1_speed[1]);
                }   
                if (i == PITCH_MOTOR_ID)
                { 
										if(a1_pos[0]<=(init_pos_p-0.51))
										{
												if(a1_speed[0]>0)
											{
											a1_set_kw_w(PITCH_MOTOR_ID,15,a1_speed[0]);
											}
											else{
											a1_set_kp_kw(PITCH_MOTOR_ID, 0.7, yaw_kw);
											yuntai_set_target_pitch(init_pos_p-0.51);
											a1_set_pos(PITCH_MOTOR_ID, init_pos_p-0.51);
											}
										}
										else if(pos_block[0]){
											a1_set_kp_kw(PITCH_MOTOR_ID, 0.7, yaw_kw);
											if(fabs(yuntai_target.pitch-a1_get_position(PITCH_MOTOR_ID))<0.2)
											a1_set_pos(PITCH_MOTOR_ID, yuntai_target.pitch);
											else  yuntai_set_target_pitch(a1_get_position(PITCH_MOTOR_ID));
										}
										else if(a1_pos[0]>(init_pos_p-0.51))
										a1_set_kw_w(PITCH_MOTOR_ID,15,a1_speed[0]);
                }     
				}
				break;
		}
    default:
        break;
    }
    yuntai_last_mode = yuntai_mode;
    time_now += 2;
}





void yuntai_init()
{
    kalmanCreate(&kal,ProcessNiose_Q,MeasureNoise_R);
    
    yuntai_yaw_speed_pid.Kp = 0.14;
    yuntai_yaw_speed_pid.Ki = 0.0007;
    yuntai_yaw_speed_pid.Kd = 0.01;

    yuntai_yaw_position_pid.Kp = 0;
    yuntai_yaw_position_pid.Ki = 0;
    yuntai_yaw_position_pid.Kd = 0;
    
    yuntai_yaw_offest_pid.Kp = 0.003;
    yuntai_yaw_offest_pid.Ki = 0;
    yuntai_yaw_offest_pid.Kd = 0.08;
}

// 串口上位机，调参用

float radar_run(int id_zhu)
{


	
//	result = 3.1415926f/2 - atan2(delta_y,delta_x);//返回弧度值

//	delta_yaw = a1_get_position(YAW_MOTOR_ID) + A1_Start_Pos_Yaw;//偏航角弧度值	
//	delta_yaw = 3.1415926f/2 - a1_get_position(YAW_MOTOR_ID);
//		result = fabsf(a1_get_position(YAW_MOTOR_ID));if(result < 3.1415926f/2)	
	target = Xita - a1_get_position(YAW_MOTOR_ID) - 0.543966591f;
//	else target = adelta_yaw - 3.1415926f/2 - xita;
		
	x0 = radar_x + sqrt(L_radar_x * L_radar_x + L_radar_y * L_radar_y) * sinf(target);
	y0 = radar_y - sqrt(L_radar_x * L_radar_x + L_radar_y * L_radar_y) * cosf(target);		

	delta_x = Zhuzi_id_x_y[id_zhu - 1][0] - x0;
	delta_y = Zhuzi_id_x_y[id_zhu - 1][1] - y0;
	
	delta_yaw = atan2(delta_x,delta_y);
	Send2A1 = delta_yaw;
//	if(delta_x>0 && delta_y>0) Send2A1 = delta_yaw;
//	else if (delta_x<0 && delta_y>0) Send2A1 = delta_yaw;
//	else if (delta_x<0 && delta_y<0) Send2A1 = -3.1415926f + delta_yaw;
//	else if (delta_x>0 && delta_y<0) Send2A1 = 3.1415926f + delta_yaw;
	
	return Send2A1 ;
}

void yuntai_shoot_select(float yaw_target,float pitch_target)
{
	path_flag=1;
	yuntai_start_path(YAW_MOTOR_ID, init_pos+yaw_target, 0.5);
	yuntai_start_path(PITCH_MOTOR_ID,init_pos_p + pitch_target, 0.5);
	osDelay(500);
	yuntai_set_target_yaw(init_pos+yaw_target);
	yuntai_set_target_pitch(init_pos_p+pitch_target);
	path_flag=0;
}
