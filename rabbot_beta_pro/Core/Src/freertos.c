/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "M3508.h"
#include "motion_control.h"
#include "handkey.h"
#include "math.h"
#include "A1_motor.h"
#include "my_uart.h"
#include "auto_route.h"
#include "ANO_TC.h"
#include "yuntai.h"
//2023-04-12 01:06
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//int m_state = 0;
#define PI 3.1415926f
#define RADIN_TO_REG 57.296f
#define AUTO_STATE 3
#define MANUAL_STATE 2
#define CHANGE_TO_RADIAN (0.01745329251994f)
int test = 0;//瀹炴垬璋冭瘯
int flag_test=0;//鍗曠數鏈鸿皟璇?
float x_set,y_set,w_set,h0_set,h1_set;
float vw_0;
float vx_set,vy_set,vw_set;
float pos_set=0;
float speed_set=10;
int test_flag_pro=0;
float speed_set_w;
extern RC_Ctl_t CtrlData;
int (*hook)(int x, int y);
int GPIO_ID_STATE[4];
extern M3508x_STA M3508[8];
float RPM=0;
float pos_7[5]= {0};
float pos_5[5]= {0};
extern A1_data a1_data[2];
float spped_set_a1=5000;
int a1_flag=0;
int State_flag=0;
extern MOTOR_recv motor_recv[2];
int32_t test_pos = 0;
extern float Rpm_set;
float x,y,z;
extern int16_t handkey_direction[4];
extern float speed_level;
int motion_flag=0;
float c_x,c_y,c_w;
float data2send[5];
extern LUJING_STU Lujing_Status;
extern float data_yaw;
extern Action_data Action_Data;
extern float mis_error;
extern PID_Struct Cir_AdjustPID;
extern coordinate_Struct World_Coordinate_system_TargetVel;
extern int reset_flag;
int shoot_manual=0;
float a1_pos[2];
int wtf=0;
float yuntai_x=0;

float yuntai_y=0;

extern LUJING_STU Lujing_Status;
int jiahuan_state=1;
int tuihuan_state=0;
int yahuan_state=1;
extern PID_Struct keep_x;
extern PID_Struct keep_y;
extern PID_Struct keep_w;
/*-----------云台参数-----------*/
uint8_t flag_yuntai = 0;

float end_pos_1,end_pos_2 = 0;
uint8_t send_flag = 0;
uint8_t vision_send_data = 0; // -2 -1 0 1 2

float vx_last,vy_last;
float vx_pre,vy_pre;
float speed_up_rate=20.f;
float speed_down_rate=20.f;
float radar_flag=0;
extern float radar_start_x,radar_start_y;
extern float radar_last_x,radar_last_y;
extern float mapan_start_x,mapan_start_y;
extern float kx,ky;
extern float radar_sum,kx_av,ky_av;
extern float radar_x,radar_y;
extern float vx_0,vy_0;
extern VisionData vision_data;
int GPIO_state;
int fangxiang;
extern PID_TypeDef Keep_X_PID;     // 保持x方向位置
extern PID_TypeDef Keep_Y_PID;     // 保持y方向位置
extern PID_TypeDef Keep_W_PID;     // 保持角度

float x_break_test = 10.0f;
float angle_break_test = 0.5f;
int shoot_flag=0;
static void motion_3(void);
static void motion_2(void);
static void motion_1(void);
static void motion_4(void);
static void motion_5(void);
static void motion_6(void);
float a1_pos_set[2];
uint8_t Screen[32];
int chuan_i;
int cd_change=0;
extern int page;
int weidong_setx,weidong_sety;
extern float Angle_Z_last;
int out_flag;
float init_pos;
float init_pos_p;
int path_flag;
int huan_list;
int child_block=0;
int shoot_pre_cp=0;
int motion_reset=0;
int motion_set=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 娉ㄥ唽娑堟伅 */
enum {
    OPER_TYPE_ADD = 0,   /*  */
    OPER_TYPE_MINUS,     /*  */
    OPER_TYPE_MULTI,     /*  */
    OPER_TYPE_DEV,	     /*  */
    OPER_TYPE_MAX
};

typedef struct handle_cb {
    int type;
    int (*handle)(int a, int b);   // HOOK_Task
} HANDLE_CB;
union
{
    uint8_t abcd[28];
    float zhuan2u8[7];
} Screen_Send;
/*
娉ㄥ唽閽╁瓙锛岃繖閲屾樉寮忓湴鎶婃秷鎭被鍨嬪拰閽╁瓙鍑芥暟瀵瑰簲鍏崇郴浣撶幇鍑烘潵浜嗭紝
handle_cb閲岄潰鐨則ype鍙互涓嶈鐨勶紝娉ㄦ剰缁存姢濂芥敞鍐屾秷鎭〃涓?
姣忎釜鏋氫妇鍊煎厛鍚庨『搴忓拰涓嬮潰閽╁瓙鍑芥暟琛ㄤ腑鐨勫厛鍚庨『搴忎竴鑷达紝鑳藉搴斾笂
灏辫浜?
*/
HANDLE_CB g_handle [] = {
//	{ OPER_TYPE_ADD, add },
//	{ OPER_TYPE_MINUS, minus },
//	{ OPER_TYPE_MULTI, multi },
    { OPER_TYPE_DEV, NULL}
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId wait_stateHandle;
osThreadId motion_stateHandle;
osThreadId zero_stateHandle;
osThreadId motor_controlHandle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern M3508x_STA M3508[8];
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Wait_State(void const * argument);
void Motion_State(void const * argument);
void Zero_State(void const * argument);
void Motor_Control(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of wait_state */
  osThreadDef(wait_state, Wait_State, osPriorityIdle, 0, 256);
  wait_stateHandle = osThreadCreate(osThread(wait_state), NULL);

  /* definition and creation of motion_state */
  osThreadDef(motion_state, Motion_State, osPriorityIdle, 0, 256);
  motion_stateHandle = osThreadCreate(osThread(motion_state), NULL);

  /* definition and creation of zero_state */
  osThreadDef(zero_state, Zero_State, osPriorityHigh, 0, 256);
  zero_stateHandle = osThreadCreate(osThread(zero_state), NULL);

  /* definition and creation of motor_control */
  osThreadDef(motor_control, Motor_Control, osPriorityIdle, 0, 256);
  motor_controlHandle = osThreadCreate(osThread(motor_control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Wait_State */
/**

* @brief Function implementing the wait_state thread.
* @param argument: x_set锛寉_set锛寁x_set锛寁y_set锛孋trlData.rc.ch1~3(鍒嗗埆涓哄墠鍚庛?佸乏鍙炽?佽嚜杞?)
* @retval None
*/
float arfa_o,arfa_i;
int motion_list=0;
extern int ceju_data;
float a1_speed_boost;
int speed_select=0;
/* USER CODE END Header_Wait_State */
void Wait_State(void const * argument)
{
  /* USER CODE BEGIN Wait_State */
    /* Infinite loop */
    for(;;)
    {
				arfa_o=sinf(arfa_i);
        if(reset_flag^wtf)
        {
            wtf=!wtf;
            zero_reset();
        }
        data2send[0] = ceju_data;
        data2send[1] = handkey_direction[1];
        data2send[2] = handkey_direction[0];
        data2send[3] = handkey_direction[2];		
        data2send[4] = handkey_direction[3];
        Usart_Send_To_Show32(&huart5,data2send);
				Screen[0]=0xAA;
				Screen[1]=0xAA;
				if(page==1){
					if(!weidong_setx&&motion_list<4)
				Screen_Send.zhuan2u8[0]=66666;
					else if(!weidong_setx)Screen_Send.zhuan2u8[0]=77777;
					if(!weidong_sety&&motion_list<4)
				Screen_Send.zhuan2u8[1]=66666;	
					else if(!weidong_sety)Screen_Send.zhuan2u8[1]=77777;
				Screen_Send.zhuan2u8[2]=a1_pos[0];
				Screen_Send.zhuan2u8[3]=a1_pos[1];
				Screen_Send.zhuan2u8[4]=ceju_data;			
				Screen_Send.zhuan2u8[5]=spped_set_a1;	
						Screen_Send.zhuan2u8[6]=child_block;	
			
				}
				else if(page==2){
				Screen_Send.zhuan2u8[0]=Action_Data.x;
				Screen_Send.zhuan2u8[1]=Action_Data.y;	
				Screen_Send.zhuan2u8[2]=Action_Data.angle_Z;
				Screen_Send.zhuan2u8[3]=motion_list;
				Screen_Send.zhuan2u8[4]=huan_list;			
				Screen_Send.zhuan2u8[5]=spped_set_a1;
					Screen_Send.zhuan2u8[6]=child_block;	
				}
				else if(page==3){
				Screen_Send.zhuan2u8[0]=1;
				Screen_Send.zhuan2u8[1]=2;	
				Screen_Send.zhuan2u8[2]=3;
				Screen_Send.zhuan2u8[3]=4;
				Screen_Send.zhuan2u8[4]=5;			
				Screen_Send.zhuan2u8[5]=6;
				//Screen_Send.zhuan2u8[6]=Action_Data.y;
				}
				else{
				Screen_Send.zhuan2u8[0]=1;
				Screen_Send.zhuan2u8[1]=2;	
				Screen_Send.zhuan2u8[2]=3;
				Screen_Send.zhuan2u8[3]=4;
				Screen_Send.zhuan2u8[4]=5;			
				Screen_Send.zhuan2u8[5]=6;
				//Screen_Send.zhuan2u8[6]=Action_Data.y;
				}
				for(chuan_i=0; chuan_i<28; chuan_i++)
				{
						Screen[chuan_i+2]=Screen_Send.abcd[chuan_i];
				}
				if(speed_select)a1_speed_boost=10;
				else a1_speed_boost=50;
        HAL_UART_Transmit(&huart2,Screen, 32, 0xFFF); 
				if(shoot_manual){
					if(handkey_direction[2]>30)spped_set_a1+=a1_speed_boost;
					else if(handkey_direction[2]<-30)spped_set_a1-=a1_speed_boost;
					osDelay(25);
				}

        osDelay(5);
    }
  /* USER CODE END Wait_State */
}

/* USER CODE BEGIN Header_Motion_State */
/**
* @brief Function implementing the motion_state thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motion_State */
void Motion_State(void const * argument)
{
  /* USER CODE BEGIN Motion_State */
    /* Infinite loop */
    for(;;)
    {
			if(!child_block){
        if(State_flag==MANUAL_STATE) {
            x_set = handkey_direction[3];
            y_set =-handkey_direction[2];
						w_set = handkey_direction[1]*3.f;
            speed_set=1*5000;
            speed_set_w=50.0f;
            speed_up_rate=30.0f;
            vx_pre=x_set/sqrt(y_set*y_set+x_set*x_set+1)*speed_set*fabs(x_set)/1000.f;
            vy_pre=y_set/sqrt(y_set*y_set+x_set*x_set+1)*speed_set*fabs(y_set)/1000.f;
            if(vx_pre-vx_last>=speed_up_rate)
            {
                vx_0+=speed_up_rate;
            }
            else if(vx_pre-vx_last<=-speed_up_rate)
            {
                vx_0-=speed_up_rate;
            }
            else vx_0=vx_0;

            if(vy_pre-vy_last>=speed_up_rate)
            {
                vy_0+=speed_up_rate;
            }
            else if(vy_pre-vy_last<=-speed_up_rate)
            {
                vy_0-=speed_up_rate;
            }
            else vy_0=vy_0;

						if (vw_0 < w_set)
						{
							vw_0 += speed_set_w;
							if(vw_0 > w_set)
								vw_0 = w_set;
						}
						else
						{
							vw_0 -= speed_set_w;
							if(vw_0 < w_set)
								vw_0 = w_set;
						}
            vx_last=vx_0;
            vy_last=vy_0;
						if(!weidong_setx&&motion_list<4){Action_Data.zero_x=-Action_Data.x_0;
							Action_Data.zero_w=-Action_Data.w_0;
						}
						if(!weidong_sety&&motion_list<4){Action_Data.zero_y=-Action_Data.y_0;
							Action_Data.zero_w=-Action_Data.w_0;
						}
						else if(!weidong_sety&&motion_list>4){Action_Data.zero_x=-Action_Data.x_0;
							Action_Data.zero_y=-Action_Data.y_0;
							Action_Data.zero_w=-Action_Data.w_0-90.f;}
						if(!shoot_manual){
            vx_set=vy_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)+vx_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
            vy_set=vy_0*cosf(-Action_Data.angle_Z*CHANGE_TO_RADIAN)-vx_0*sinf(-Action_Data.angle_Z*CHANGE_TO_RADIAN);
							vx_set=vx_0;
							vy_set=vy_0;
							vw_set = vw_0;
    
							motion_resolve(-vx_set,vy_set,vw_set);
						}
						Lujing_Status.CoordinateSystem.Target_Position.x=Action_Data.x;
						Lujing_Status.CoordinateSystem.Target_Position.y=Action_Data.y;
						Lujing_Status.CoordinateSystem.Target_Position.w=Action_Data.angle_Z;					
        }

	
        else if(State_flag==AUTO_STATE) {
					
            if(motion_flag==1)
            {
							if((Action_Data.x!=0||Action_Data.y!=0)&&Action_Data.x<5&&Action_Data.y<5){
								circle(0,0,-95,1534.8,-10.4,0,10000,10000,0.2,0.2);
								circle(-95,1534.8,-1286.5,3830,-37.4,10000,10000,10000,0.1,0.2);
								circle(-1286.5,3830,-1903.3,4265,-17.35,10000,10000,10000,0.1,0.2);
								circle(-1903.3,4265,-3584.5,4846.3,-14.9,10000,5000,10000,0.1,0.5);
								circle(-3584.5,4846.3,-5177.5,4170.8,-73.6,5000,2000,5000,0.1,0.4);
								line(-5177.5,2470.8,0,5000,3000,0.12,0.15,5000,6000,20);
								line(-5177.5,1970.8,0,3000,0,0.1,0.2,10000,5000,20);
								jiahuan_state=0;
								motion_list++;
								vx_0=0;
								vy_0=0;
								vw_set=0;
							}
							else child_block=1;
							motion_flag = 0; 
            }
            else if(motion_flag==2)
            {
							if(Screen_Send.zhuan2u8[0]==66666&&Screen_Send.zhuan2u8[1]==66666){
								
								line(1500,200,0,0,2000,0.1,0.2,1000,5000,20);
								circle(Action_Data.x,Action_Data.y,2000,-1335,60,2000,0,5000,0.1,0.2); 
								//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
								yahuan_state=1;
								jiahuan_state=1;
								motion_list++;
								vx_set=0;
								vy_set=0;
								vw_set=0;
								
							}
							else child_block=1;
							motion_flag = 0;
   
            }
            else if(motion_flag==3)
            {
							
							catch_down(-106500);
							motion_list++;
							motion_flag = 0;
						}
						else if(motion_flag==4)
						{
							position_set(&M3508[0],M3508[0].dAngle_Sum,10);
							line(1665,Action_Data.y,0,0,0,0.12,0.15,1000,1000,20);
							motion_list++;
							motion_flag = 0;
        
            }
            else if(motion_flag==5)
            {
							//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
							yahuan_state=0;
							osDelay(500);
							line(2000,-1335,0,0,0,0.12,0.15,1000,1000,20);
							catch_up(106500);
							motion_list++;
							motion_flag = 0;
						}
						else if(motion_flag==6)
						{
							position_set(&M3508[0],M3508[0].dAngle_Sum,10);
							//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
							yahuan_state=1;
							//jiahuan_state=0;
							osDelay(500);
								jiahuan_state=1;
							osDelay(300);
								catch_down(-76500);
								motion_flag=0;
								motion_list++;
						}
						else if(motion_flag==7){
								position_set(&M3508[0],M3508[0].dAngle_Sum,10);
								jiahuan_state=0;
								osDelay(300);
								jiahuan_state=1;
								osDelay(300);
								jiahuan_state=0;
								osDelay(300);
								jiahuan_state=1;
								osDelay(300);
								jiahuan_state=0;
								motion_set=0;
							motion_list++;
							motion_flag=0;
            }//bie ji
						
						else if(motion_flag==8)
            {	
							line(2000,-4000,-90,0,2000,0.1,0.2,2000,5000,20);
							line(200,-4300,-90,2000,0,0.1,0.2,1000,5000,20);
							motion_list++;
							motion_flag=0;
            }
							else if(motion_flag==9)
            {	
							if(Screen_Send.zhuan2u8[1]==77777){
							line(-50,Action_Data.y-(1170-ceju_data),-90,0,0,0.2,0.2,1000,2000,20);
							
							motion_list++;
							}
							else child_block=1;
							motion_flag=0;
		
						}
						else if(motion_flag==10)
						{
							catch_down(-30000);
							motion_list++;
							motion_flag=0;
						}
						else if(motion_flag==11)//zhe li zhuyi meiyou xianxiang ,kandao jiangdao zuixiamian jiu an
						{
							position_set(&M3508[0],M3508[0].dAngle_Sum,10);
							motion_list++;
							motion_flag=0;
						}
						else if(motion_flag==12)
            {	
							//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
							if(huan_list==0&&jiahuan_state==0){
								tuihuan_state=0;
								path_flag=1;
								
								yuntai_start_path(PITCH_MOTOR_ID, init_pos_p, 0.5);
								osDelay(500);
								yuntai_set_target_pitch(init_pos_p);
								yuntai_start_path(YAW_MOTOR_ID, init_pos, 1);
								osDelay(1000);
								yuntai_set_target_yaw(init_pos);
								path_flag=0;
								shoot_pre_cp=0;
								
								//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
								jiahuan_state=1;
								osDelay(300);
								catch_up(106500);
											motion_list++;
		
							}
									else child_block=1;
								motion_flag=0;
						}
							else if (motion_flag==13){
							
								position_set(&M3508[0],M3508[0].dAngle_Sum,10);
								//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
								yahuan_state=1;
								osDelay(500);
								catch_down(-106500);
								motion_list++;
							motion_flag=0;
							}
							else if(motion_flag==14)
							{
								position_set(&M3508[0],M3508[0].dAngle_Sum,10);
								jiahuan_state=0;
								osDelay(300);
								jiahuan_state=1;
								osDelay(300);
								jiahuan_state=0;
								osDelay(300);
								jiahuan_state=1;
								osDelay(300);
								jiahuan_state=0;
								motion_flag=0;
								motion_list++;
							}
					
//							if(motion_set){
//								if(motion_flag==0){
//								jiahuan_state=1;
//								catch_down(-76500);
//								osDelay(500);
//								position_set(&M3508[0],M3508[0].dAngle_Sum,10);
//								jiahuan_state=0;
//								osDelay(300);
//								jiahuan_state=1;
//								osDelay(300);
//								jiahuan_state=0;
//								osDelay(300);
//								jiahuan_state=1;
//								osDelay(300);
//								jiahuan_state=0;
//								motion_set=0;
//								}
//							}
//							if(motion_reset){
//								if(motion_flag==0)
//								{
//									motion_list=0;
//									Action_Data.zero_x=-Action_Data.x_0;
//									Action_Data.zero_y=-Action_Data.y_0;
//									Action_Data.zero_w=-Action_Data.w_0;
//									motion_reset=0;
//								}
//							}
								
								
            
						if(!weidong_setx&&motion_list<4){Action_Data.zero_x=-Action_Data.x_0;
							Action_Data.zero_w=-Action_Data.w_0;
						}
						if(!weidong_sety&&motion_list<4){Action_Data.zero_y=-Action_Data.y_0;
							Action_Data.zero_w=-Action_Data.w_0;
						}
						else if(!weidong_sety&&motion_list>4){Action_Data.zero_x=-Action_Data.x_0;
							Action_Data.zero_w=-Action_Data.w_0-90.f;}
//						if(shoot_flag){
//							VESC_Set_HardBreak_Current(1,10000,&hcan2);
//							VESC_Set_HardBreak_Current(2,10000,&hcan2);
//							VESC_Set_HardBreak_Current(3,10000,&hcan2);
//							VESC_Set_HardBreak_Current(4,10000,&hcan2);
//						}
//							else
            Keep_Position(Lujing_Status.CoordinateSystem.Target_Position.x,Lujing_Status.CoordinateSystem.Target_Position.y,Lujing_Status.CoordinateSystem.Target_Position.w );
						}	
        }
			
			else Keep_Position(Action_Data.x,Action_Data.y,Action_Data.angle_Z);

 //motion_resolve(-vx_set,vy_set,vw_set);
        osDelay(2);
    }
  /* USER CODE END Motion_State */
}

/* USER CODE BEGIN Header_Zero_State */
float a1_speed[2];
int pos_block[2];
/**
* @brief Function implementing the zero_state thread.
* @param argument: Not used
* @retval None
*/

extern uint8_t rsv_flag;
int weidong_flag;

extern int number_input;
int t1,t2,t3,t4;
int number_last;
extern int shoot_select;
/* USER CODE END Header_Zero_State */
void Zero_State(void const * argument)
{
  /* USER CODE BEGIN Zero_State */
    /* Infinite loop */
    for(;;)
    {
							if(shoot_flag)
				{
					VESC_Set_RPM(1,spped_set_a1,&hcan1);
					VESC_Set_RPM(2,spped_set_a1,&hcan1);
				}
				a1_pos[0]=a1_get_position(PITCH_MOTOR_ID);
        a1_pos[1]=a1_get_position(YAW_MOTOR_ID);
				weidong_flag=HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15);
				weidong_setx=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9);
				weidong_sety=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,yahuan_state);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,jiahuan_state);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,tuihuan_state);
				if(shoot_select&&!child_block)
				{
							switch(number_input)
					{
						case 1:
							test_flag_pro=1;break;
						case 2:
							a1_flag=13;break;
						case 4:
							a1_flag=14;break;
						case 5:
							a1_flag=15;break;
						case 6:
							a1_flag=6;break;
						case 7:
							a1_flag=7;break;
						case 8:
							a1_flag=8;break;
						case 10:
							a1_flag=10;break;
						case 11:
							if(number_input!=number_last)speed_select=!speed_select;break;							
						case 12:
							a1_flag=12;break;
						default:break;	
					}
				}
				else if(!child_block)
				{
				switch(number_input)
				{
					case 1:
						jiahuan_state=1;break;
					case 2:
						tuihuan_state=1;break;
					case 3:
						yahuan_state=1;break;
					case 4:
						jiahuan_state=0;break;
					case 5:
						tuihuan_state=0;break;
					case 6:
						yahuan_state=0;break;

					
					
					default:break;
						
				}
			}
				number_last=number_input;
				if(!path_flag){
				if(shoot_manual)
				{
					
						yuntai_manual(0);
					
						if(handkey_direction[1]>100)a1_speed[0]=0.3;
					else if(handkey_direction[1]<-100)a1_speed[0]=-0.3;
						else a1_speed[0]=0;
						if(handkey_direction[0]>100)a1_speed[1]=1.5;
					else if(handkey_direction[0]<-100)a1_speed[1]=-1.5;
						else a1_speed[1]=0;
						if(a1_speed[0]==0){
							
							
							if(pos_block[0]==0){
							pos_block[0]=1;
								a1_data[0].mode=10;
							yuntai_set_target_pitch(a1_get_position(PITCH_MOTOR_ID));
							a1_set_kp_kw(0,0.2,0.5);}
						}
						else pos_block[0]=0;
						if(a1_speed[1]==0){
							if(pos_block[1]==0){
							pos_block[1]=1;
							a1_data[1].mode=10;
							a1_set_kp_kw(1,0.1,0.5);

							yuntai_set_target_yaw(a1_get_position(YAW_MOTOR_ID));
							}
						
						}
						else pos_block[1]=0;
					}
				
				else if(!path_flag)mode_change();
				}
				osDelay(1);
    }
  /* USER CODE END Zero_State */
}

/* USER CODE BEGIN Header_Motor_Control */
/**
* @brief Function implementing the motor_control thread.
* @param argument: Not used
* @retval None
*/

float m3508_5_speed_set=0;
float lift_up_step=19;
int middle_right=0;
int radar_flag_test;
extern float Send2A1;
int zhuzi_id;
float init_pos;

/* USER CODE END Header_Motor_Control */
void Motor_Control(void const * argument)
{
  /* USER CODE BEGIN Motor_Control */
    /* Infinite loop */
    for(;;)
    {

        if(test_flag_pro==1) // 抬升送环复位
				{
						if(lift_move(-2000.f,HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_15))==0)	
						{test_flag_pro=0;lift_step(8.1);huan_list=0;tuihuan_state=0;
						}
				}
				if(test_flag_pro==2) 
				{
					lift_step(lift_up_step);
					test_flag_pro=0;
				}
				if(test_flag_pro==3)
				{
					jiahuan_state=1;
					yahuan_state=0;
					//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
					osDelay(500);
					catch_down(-106500);
				
					test_flag_pro=0;
				}

				if(test_flag_pro==5)//shoot
				{
					if(huan_list<10){
					lift_step(16.12); // 云台送环抬升高度16
					osDelay(500);
					tuihuan_state=0;
					osDelay(600);
					tuihuan_state=1;
					huan_list++;
					test_flag_pro=0;}
					else child_block=1;
				}
				if(test_flag_pro==6)
				{
					lift_step(8.1); // 云台送环初始抬升距离
					//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);
					test_flag_pro=0;
				}
				if(test_flag_pro==7)
				{

					jiahuan_state=1;
					yahuan_state=0;
					osDelay(500);
					catch_up(106500);
					test_flag_pro=0;
				}


        if(a1_flag==1)
        {
					if(init_pos==0&&init_pos_p==0){
					a1_data[1].mode=10;
					path_flag=1;
					init_pos=a1_get_position(YAW_MOTOR_ID);
					init_pos_p=a1_get_position(PITCH_MOTOR_ID);
				yuntai_start_path(YAW_MOTOR_ID, init_pos-0.285f, 0.5);
					osDelay(500);
				yuntai_set_target_yaw(init_pos-0.285f); // 记录电机初始值，防止电机上电抖动
				yuntai_set_target_pitch(init_pos_p);
				a1_data[0].mode=10;
				a1_data[1].mode=10;
					path_flag=0;}//6.85463524 -0.128811777
					a1_flag=0;//0.165030763 0.257623553
        }//8.32529736 -0.154378116
				else if(a1_flag==2)//9.64673615 -0.144748122
        {
					if(motion_list<1)child_block=1;
					else{
					if(fabs(a1_get_position(PITCH_MOTOR_ID)-init_pos_p)<0.2&&fabs(a1_get_position(YAW_MOTOR_ID)-init_pos)<0.5){
					a1_data[1].mode=10;
					path_flag=1;
				yuntai_start_path(YAW_MOTOR_ID, init_pos, 0.5);
					osDelay(500);
				yuntai_set_target_yaw(init_pos); // 记录电机初始值，防止电机上电抖动
				yuntai_set_target_pitch(init_pos_p);
				a1_data[0].mode=10;
				a1_data[1].mode=10;
					path_flag=0;}
				}
					a1_flag=0;
        }
				 else if(a1_flag==3)
        {
					//if(!jiahuan_state){
				yuntai_set_target_yaw(a1_get_position(YAW_MOTOR_ID)); // 记录电机初始值，防止电机上电抖动
				yuntai_set_target_pitch(a1_get_position(PITCH_MOTOR_ID));
				a1_data[0].mode=10;
				a1_data[1].mode=10;
					path_flag=0;
					//a1_flag=0;
        }
				else if(a1_flag==6)
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(7.22236,-0.51);
					spped_set_a1=15800;}
					else  child_block=1;
					a1_flag=0;
				}
				else if(a1_flag==7)
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(5.444,-0.48166);
					spped_set_a1=11150;}
					//7.18708277 -0.129621372  8 16500
					else child_block=1;
					//0.548909426 0.239258364  
					a1_flag=0;
				}
				//15900 7.81955 0.3202 6
				else if(a1_flag==8)
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(6.58091,-0.37123);
					spped_set_a1=17200;}
					else child_block=1;
					a1_flag=0;
				}
				else if(a1_flag==10)
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(5.29930,-0.3814);
					spped_set_a1=15150;}
					else child_block=1;
					a1_flag=0;
				}

				else if(a1_flag==12)
				{
					if(!jiahuan_state&&!tuihuan_state){
					path_flag=1;
					yuntai_start_path(YAW_MOTOR_ID, init_pos+7, 1);
					osDelay(1000);
					yuntai_set_target_yaw(init_pos+7);
					yuntai_start_path(PITCH_MOTOR_ID, init_pos_p, 0.5);
					osDelay(500);
					yuntai_set_target_pitch(init_pos_p);
					path_flag=0;
					shoot_pre_cp=1;
					tuihuan_state=1;
					}
					else child_block=1;
					a1_flag=0;
				}
				else if(a1_flag==13)//2
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(9.39319,-0.35774);
						spped_set_a1=15150;//15150
					}
					else child_block=1;
					a1_flag=0;
				}
				else if(a1_flag==14)//4
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(8.95823,-0.51);
					spped_set_a1=9500;
					}
					else child_block=1;
					a1_flag=0;
				}
				else if(a1_flag==15)//5
				{
					if(!jiahuan_state&&shoot_pre_cp){
				yuntai_shoot_select(8.05993,-0.32014);
					spped_set_a1=16400;}
					else child_block=1;
					a1_flag=0;
				}

        osDelay(1);
    }
  /* USER CODE END Motor_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
