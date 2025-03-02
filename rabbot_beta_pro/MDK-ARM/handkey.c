#include "usart.h"
#include "handkey.h"
#include "freertos.h"
#include "task.h"
#include "math.h"
#include "iso646.h"
#include "my_uart.h"
#include "A1_motor.h"
#include "yuntai.h"
#define PI 3.1415926f
#define II 500
#define CHANGE_TO_RADIAN (0.01745329251994f)
int16_t handkey_pos[4];
int16_t handkey_direction[4];
RC_Ctl_t CtrlData;
uint8_t rcbuff[40];
uint8_t Gyro_RX_Data[30];
Gyro_Data Gyro_Data_RX;
Action_data Action_Data;
Action_data data_zero;
Action_data Self_Action_Data;
uint8_t rec_data[30];
uint8_t radar[30];
uint8_t camer[30];
data_chg data_CHG;
extern uint8_t ceju_datarc[8];
extern A1_data a1_data[2];
extern int motion_flag;
float radar_x_avr[100];
float radar_y_avr[100];
float speed_level;
float Angle_Z_last=0;//»úÆ÷ÈËÉÏÒ»Ê±¿ÌµÄ½Ç¶È
float angle_Z_temp_1,angle_Z_temp_2,angle_Z_temp;
int EMR_stop=1;
int reset_flag=0;
float angle_zero=0;
float angle_set=0;
float radar_x,radar_y = 0;
extern int State_flag;
extern float yuntai_x;
extern float yuntai_y;

float bata = -(2 * PI / 360.f * 45);
union Gyro_test
{
	float gyrotest[7];
	uint8_t final_data[28];
}Gyro;
static uint8_t i=0;
uint8_t datasum = 0;
float data_yaw;
float data_acc_x;
float data_acc_y;
float data_acc_z;
float data_gyro_x;
float data_gyro_y;
float data_gyro_z;

float data_roll;
float data_pitch;
float data_yaw;
	float radar_x_tmp,radar_y_tmp;
static struct
	{
		uint8_t Receive_Val[50];
		uint8_t SUM;
		float RPY[9];
	} posture;
extern MOTOR_recv motor_recv[2];
extern USART1_RECEIVETYPE Usart1Type1;
	
	int sum_i=0;
	float action_x_reset,action_y_reset;
	

void Radar_Data_Process(void)
{
	if(radar[0]==0x0A and radar[1]== 0x0D and radar[10]== 0x0D and radar[11]==0x0A )
	{
		
		radar_x_tmp = get_float_from_4u8(&radar[2]);
		radar_y_tmp = get_float_from_4u8(&radar[6]);
		if(radar_x_tmp !=0 && radar_y_tmp != 0)
		{
			radar_x = radar_x_tmp;
			radar_y = radar_y_tmp;
		}
	}

}
	
void Gyro_Data_Process(void)
{
            if(posture.Receive_Val[0] == 0x68 
             &&posture.Receive_Val[1] == 0x1F
             &&posture.Receive_Val[2] == 0x00
             &&posture.Receive_Val[3] == 0x84)
            {
                for(i = 4;i<31;i++)
                {
                    posture.SUM += posture.Receive_Val[i];
                }
                datasum = 0x1F+0x84+posture.SUM;
                if(datasum == posture.Receive_Val[31])
                {
//                         data_roll  = dataasic_to_float(posture.Receive_Val[0+4],posture.Receive_Val[1+4],posture.Receive_Val[2+4]);
//                         data_pitch = dataasic_to_float(posture.Receive_Val[3+4],posture.Receive_Val[4+4],posture.Receive_Val[5+4]);
													angle_set    = dataasic_to_float(posture.Receive_Val[6+4],posture.Receive_Val[7+4],posture.Receive_Val[8+4]); //data_yaw
													data_yaw = angle_set-angle_zero;
						 data_gyro_z  = dataasic_to_float(posture.Receive_Val[24+4],posture.Receive_Val[25+4],posture.Receive_Val[26+4]);
                }
                posture.SUM= 0;
            }
}

float dataasic_to_float(uint8_t data1,uint8_t data2,uint8_t data3)
{
   float data_num=0;
	 float turn = 1;
   if((data1 & 0x10) == 0x10) //对data1的高四位进行判断，从而分析出其正负
	 {
	     turn = -1;
	 }
	 data1 = data1 & 0x0F;
	 data_num =turn *( data1*100+ (data2>>4) *10 +   (data2 & 0x0F)+ (data3>>4) * 0.1 +   (data2 &0xF)*0.01);
	return data_num;
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : Handkey_Receive_Start
  ** @brief         : 手柄开始接收
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : DJI手柄和通讯模块连接之后，通讯模块的蓝色灯将亮起
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void MaPan_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rec_data,30);
}

void Handkey_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	HAL_UART_Receive_DMA(&huart2,rcbuff,40);	
}
void Radar_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	HAL_UART_Receive_DMA(&huart3,radar,30);	
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : Gyro_Receive_Start
  ** @brief         : 陀螺仪开始接收
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void Gyro_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	HAL_UART_Receive_DMA(&huart6,data_CHG.data_rec,20);
}
uint32_t times_yuntai = 0;
uint32_t err_times_yuntai = 0;
static void Usart1_DataProcess(uint8_t *pData) // 78字节
{
    // motor_recv = (MOTOR_recv)Usart1Type1.RX_pData;
    // memcpy(&motor_recv, Usart1Type1.RX_pData, 78);
    times_yuntai++;
    int num = 0;
    // MOTOR_recv recv;
    if (pData[0] == 0xFE && pData[1] == 0xEE)
    {
        if (pData[2] == 0)
            num = 0;
        else
            num = 1;
        (&motor_recv[num])->motor_recv_data.head.start[0] = pData[0];
        (&motor_recv[num])->motor_recv_data.head.start[1] = pData[1];
        (&motor_recv[num])->motor_recv_data.head.motorID = pData[2];
        (&motor_recv[num])->motor_recv_data.head.reserved = pData[3];
        (&motor_recv[num])->motor_recv_data.Mdata.mode = pData[4];
        (&motor_recv[num])->motor_recv_data.Mdata.ReadBit = pData[5];
        (&motor_recv[num])->motor_recv_data.Mdata.Temp = pData[6];
        (&motor_recv[num])->motor_recv_data.Mdata.MError = pData[7];
        (&motor_recv[num])->motor_recv_data.Mdata.Read.u8[0] = pData[8];
        (&motor_recv[num])->motor_recv_data.Mdata.Read.u8[1] = pData[9];
        (&motor_recv[num])->motor_recv_data.Mdata.Read.u8[2] = pData[10];
        (&motor_recv[num])->motor_recv_data.Mdata.Read.u8[3] = pData[11];
        (&motor_recv[num])->motor_recv_data.Mdata.T = (pData[12] | pData[13] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.W = (pData[14] | pData[15] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.LW = get_float_from_4u8(&pData[16]);
        (&motor_recv[num])->motor_recv_data.Mdata.W2 = (pData[20] | pData[21] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.LW2 = get_float_from_4u8(&pData[22]);
        (&motor_recv[num])->motor_recv_data.Mdata.Acc = (pData[26] | pData[27] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.OutAcc = (pData[28] | pData[29] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Pos = (pData[30] | pData[31] << 8 | pData[32] << 16 | pData[33] << 24);
        (&motor_recv[num])->motor_recv_data.Mdata.Pos2 = (pData[34] | pData[35] << 8 | pData[36] << 16 | pData[37] << 24);
        (&motor_recv[num])->motor_recv_data.Mdata.gyro[0] = (pData[38] | pData[39] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.gyro[1] = (pData[40] | pData[41] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.gyro[2] = (pData[42] | pData[43] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.acc[0] = (pData[44] | pData[45] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.acc[1] = (pData[46] | pData[47] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.acc[2] = (pData[48] | pData[49] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Fgyro[0] = (pData[50] | pData[51] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Fgyro[1] = (pData[52] | pData[53] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Fgyro[2] = (pData[54] | pData[55] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Facc[0] = (pData[56] | pData[57] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Facc[1] = (pData[58] | pData[59] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Facc[2] = (pData[60] | pData[61] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Fmag[0] = (pData[62] | pData[63] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Fmag[1] = (pData[64] | pData[65] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Fmag[2] = (pData[66] | pData[67] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Ftemp = pData[68];
        (&motor_recv[num])->motor_recv_data.Mdata.Force16 = (pData[69] | pData[70] << 8);
        (&motor_recv[num])->motor_recv_data.Mdata.Force8 = pData[71];
        (&motor_recv[num])->motor_recv_data.Mdata.FError = pData[72];
        (&motor_recv[num])->motor_recv_data.Mdata.Res[0] = pData[73];
        (&motor_recv[num])->motor_recv_data.CRCdata.u8[0] = pData[74];
        (&motor_recv[num])->motor_recv_data.CRCdata.u8[1] = pData[75];
        (&motor_recv[num])->motor_recv_data.CRCdata.u8[2] = pData[76];
        (&motor_recv[num])->motor_recv_data.CRCdata.u8[3] = pData[77];

        //		if(pData[2]==0)motor_recv[0]=recv;
        //		else if(pData[2]==1)motor_recv[1]=recv;
    }

}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART6)
    {
        err_times_yuntai++;
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        Usart6_Start();
        //        USART6_error_flag = 1; // USART 异常初始化标志位
    }
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : UART4_IDLE_Callback
  ** @brief         : 空闲中断回调函数
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void UART1_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  //ÁÙ½ç¶Î´úÂë±£»¤
	
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     //ÅÐ¶ÏÒ»Ö¡Êý¾ÝÊÇ·ñ½ÓÊÕÍê±Ï
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);     //Çå¿ÕIDLE±êÖ¾Î»
		(void)USART1->SR;      //Çå¿ÕSR¼Ä´æÆ÷
		(void)USART1->DR;     //Çå¿ÕDR¼Ä´æÆ÷
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF2_6);  //Çå¿ÕDMA´«ÊäÍê³É±êÖ¾Î»
		HAL_UART_DMAStop(huart);
		
		Mapan_Data_Process();//ÂëÅÌ½ÓÊÕÊý¾Ý´¦Àíº¯Êý
			
		HAL_UART_Receive_DMA(huart, rec_data, 30);		//ÔÙ´ÎÊ¹ÄÜ½ÓÊÕ
	}
	
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

void UART6_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  //ÁÙ½ç¶Î´úÂë±£»¤
	
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     //ÅÐ¶ÏÒ»Ö¡Êý¾ÝÊÇ·ñ½ÓÊÕÍê±Ï
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);     //Çå¿ÕIDLE±êÖ¾Î»
		(void)USART6->SR;      //Çå¿ÕSR¼Ä´æÆ÷
		(void)USART6->DR;     //Çå¿ÕDR¼Ä´æÆ÷
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF2_6);  //Çå¿ÕDMA´«ÊäÍê³É±êÖ¾Î»
		HAL_UART_DMAStop(huart);
        Usart1_DataProcess(Usart1Type1.RX_pData);
        Usart6_Start();		
		HAL_UART_Receive_DMA(huart, rec_data, 30);		//ÔÙ´ÎÊ¹ÄÜ½ÓÊÕ
	}
	
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

void UART2_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  
	
  if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);    
		(void)USART2->SR;      
		(void)USART2->DR;        
    
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF1_5);   

		HAL_UART_AbortReceive(huart);
		RemoteDataProcess(rcbuff);
		HAL_UART_Receive_DMA(huart,rcbuff,40);	
	}

	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : UART5_IDLE_Callback
  ** @brief         : 空闲中断回调函数
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void UART5_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  
	
if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);    
		(void)UART5->SR;      
		(void)UART5->DR;        
    
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_DMEIF0_4);   

		HAL_UART_AbortReceive(huart);
		//ceju_dataprocess(ceju_datarc);
		HAL_UART_Receive_DMA(huart,ceju_datarc,8);	
	}
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}
uint8_t vision_recv[12];

void USART3_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();  
	
if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))     
  {
		__HAL_UART_CLEAR_IDLEFLAG(huart);    
		(void)USART3->SR;      
		(void)USART3->DR;        
    
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF1_5);   

		HAL_UART_AbortReceive(huart);
		ceju_dataprocess(ceju_datarc);
		HAL_UART_Receive_DMA(huart,ceju_datarc,8);	  
	}
	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : RemoteDataProcess
  ** @brief         : None
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
Key_xiaodou key_xiaodou[24];
int xiaodou(Key_xiaodou *key_xd,int key_data,int self_data)//an jian xiao dou 
{
		key_xd->xiaodou_ram[key_xd->num]=key_data;
		if(key_xd->num>0)
		{
			if(key_xd->xiaodou_ram[key_xd->num]==key_xd->xiaodou_ram[key_xd->num-1])key_xd->num++;
			else key_xd->num=0;
			if(key_xd->num==10){key_xd->num=0;
			return key_data;}
		}
		else key_xd->num++;
		return self_data;

}

int key_INPUT[7]={0};
int key_input_last[7];
int number_input;

//static union{
//	char act_0[4];
//	uint8_t act0[4];
//}Act;
char ACT_00[5]="ACT0";
float radar_start_x,radar_start_y;
float radar_last_x,radar_last_y;
float mapan_start_x,mapan_start_y;
float kx,ky;
extern int a1_flag;
float radar_sum,kx_av,ky_av;
extern float radar_flag;
extern int shoot_flag;
extern int shoot_manual;
extern int middle_right;
extern int flag_yuntai;
int ii = 0;
uint8_t error_num = 0;
extern int test_flag_pro;
int page=1
	;
int plus_last,reduce_last,plus,reduce;
extern float spped_set_a1;
uint8_t uart_tx[50];
extern int motion_list;
int key_select[5];
int shoot_select;
int num_last;
int ceju_data;
void ceju_dataprocess(uint8_t *pData)
{
	if(pData[0]==0xB4&&pData[1]==0x69&&pData[2]!=0x84)
	{
		ceju_data=(int32_t)pData[3]<<24|(int32_t)pData[4]<<16|(int32_t)pData[5]<<8|(int32_t)pData[6];
	}
}
int once_block=0;
int unblock_input[6];
int unblock_input_num;
extern int child_block;
int key_4_block;
void RemoteDataProcess(uint8_t *pData)
{  
	// 手柄有概率所有数据都跳0xaa
	if(pData[0]==0xAA && pData[1]==0xAA && pData[30]==0xDD && pData[31]==0xDD)
{
	if (pData[10] != 0 && pData[10] != 1)error_num++;
  handkey_pos[1] = (int16_t)pData[3]<<8 |pData[2] ;//654-568右纵
  handkey_pos[0] = (int16_t)pData[5]<<8 |pData[4] ;//694-500右横
  handkey_pos[3] = (int16_t)pData[7]<<8 |pData[6] ;//657-566左纵
  handkey_pos[2] = (int16_t)pData[9]<<8 |pData[8] ;//675-549左横
	handkey_direction[0]=(handkey_direction[0]>0)?1000*handkey_pos[2]/690:1000*handkey_pos[2]/690;//左纵
	handkey_direction[1]=(handkey_direction[1]>0)?1000*handkey_pos[3]/690:1000*handkey_pos[3]/690;//左横
	handkey_direction[2]=(handkey_direction[2]>0)?1000*handkey_pos[0]/690:1000*handkey_pos[0]/690;//右纵
	handkey_direction[3]=(handkey_direction[3]>0)?1000*handkey_pos[1]/690:1000*handkey_pos[1]/690;//右横
		for(int i=0;i<10;i++)
		uart_tx[i]=pData[i];
		for(i=10;i<14;i++)
		{
			for(int j=0;j<8;j++)
			{
		    uart_tx[(i-10)*8+10+j]=(pData[i]>>j)&0x01;//41
			}
		}
		uart_tx[42]=uart_tx[43]=pData[31];

		if(handkey_direction[2]==0&&handkey_direction[3]==0)
		{
				if(key_INPUT[0]==1)
			{
				State_flag=3;
			}
				else State_flag=2;
		}
		else {State_flag=2;EMR_stop=0;}

		key_INPUT[0]=xiaodou(&key_xiaodou[0],uart_tx[16],key_INPUT[0]);
		
		key_INPUT[1]=xiaodou(&key_xiaodou[1],uart_tx[17],key_INPUT[1]);
		key_INPUT[2]=xiaodou(&key_xiaodou[2],uart_tx[15],key_INPUT[2]);
		key_INPUT[3]=xiaodou(&key_xiaodou[3],uart_tx[13],key_INPUT[3]);
		key_INPUT[4]=xiaodou(&key_xiaodou[4],uart_tx[11],key_INPUT[4]);
		key_INPUT[5]=xiaodou(&key_xiaodou[5],uart_tx[10],key_INPUT[5]);
		key_INPUT[6]=xiaodou(&key_xiaodou[6],uart_tx[12],key_INPUT[6]);
		EMR_stop = key_INPUT[0];
		if(pData[29]!=0)
		page=pData[29];
		if(key_input_last[1]!=key_INPUT[1]){
			if(key_INPUT[1] == 1)
			{
						switch(motion_list)
						{
							case 0:motion_flag=1;break;
							case 1:motion_flag=2;break;
							case 2:motion_flag=3;break;
							case 3:motion_flag=4;break;
							case 4:motion_flag=5;break;
							case 5:motion_flag=6;break;
							case 6:motion_flag=7;break;
							case 7:motion_flag=8;break;
							case 8:motion_flag=9;break;
							case 9:motion_flag=10;break;
							case 10:motion_flag=11;break;
							case 11:motion_flag=12;break;
							case 12:motion_flag=13;break;
							case 13:motion_flag=14;break;
							case 14:motion_flag=15;break;
							case 15:motion_flag=16;break;
							case 16:motion_flag=17;break;
							case 17:motion_flag=18;break;
							default: motion_flag=0;break;
						}
			}
		}
		else motion_flag=0;
		key_input_last[1]=key_INPUT[1];
		key_select[0]=xiaodou(&key_xiaodou[18],uart_tx[23],key_select[0]);
		key_select[1]=xiaodou(&key_xiaodou[19],uart_tx[24],key_select[1]);
		key_select[2]=xiaodou(&key_xiaodou[20],uart_tx[25],key_select[2]);
		key_select[3]=xiaodou(&key_xiaodou[21],uart_tx[34],key_select[3]);
		key_select[4]=xiaodou(&key_xiaodou[22],uart_tx[35],key_select[4]);
		
		if(key_input_last[2]!=key_INPUT[2]){
			if(key_INPUT[2] == 1)
			{
					
				test_flag_pro=5;

					//test_flag_pro=5;
			}
		}
		key_input_last[2]=key_INPUT[2];

		if(key_input_last[4]!=key_INPUT[4]){
			if(key_INPUT[4] == 1&&motion_list==0&&once_block==0)
			{
						a1_flag=1;
				once_block=1;
			}
			else if(key_INPUT[4] == 0)
			{
						a1_flag=2;
			}
		}
		if(motion_list==1||motion_list==2)a1_flag=2;
		key_input_last[4]=key_INPUT[4];

		shoot_flag=key_INPUT[5];
		shoot_manual=key_INPUT[3];
		shoot_select=key_INPUT[6];
		if(xiaodou(&key_xiaodou[7],uart_tx[26],(number_input==1))){number_input=1;}
		else if(xiaodou(&key_xiaodou[8],uart_tx[30],(number_input==2))){number_input=2;}
		else if(xiaodou(&key_xiaodou[9],uart_tx[36],(number_input==3))){number_input=3;}
		else if(xiaodou(&key_xiaodou[10],uart_tx[27],(number_input==4)||key_4_block)){
		number_input=4*!key_4_block;}
		else if(xiaodou(&key_xiaodou[11],uart_tx[31],(number_input==5))){number_input=5;}
		else if(xiaodou(&key_xiaodou[12],uart_tx[37],(number_input==6))){number_input=6;}
		else if(xiaodou(&key_xiaodou[13],uart_tx[28],(number_input==7))){number_input=7;}
		else if(xiaodou(&key_xiaodou[14],uart_tx[32],(number_input==8))){number_input=8;}
		else if(xiaodou(&key_xiaodou[15],uart_tx[38],(number_input==9))){number_input=9;}
		else if(xiaodou(&key_xiaodou[16],uart_tx[29],(number_input==10)))
		{
			number_input=10;

		}
		//else if(uart_tx[33]){number_input=11;}
		else if(xiaodou(&key_xiaodou[23],uart_tx[33],(number_input==11))){number_input=11;}
		else if(xiaodou(&key_xiaodou[17],uart_tx[39],(number_input==12)))
		{
			number_input=12;
		}
		else {
									plus=0;
							reduce=0;
						number_input=0;
			key_4_block=0;
		}
		if(child_block){
			if(num_last!=number_input&&number_input!=0)
			{
				unblock_input[unblock_input_num]=number_input;
				unblock_input_num++;
				switch(unblock_input_num){
					case 1:
						unblock_input_num=unblock_input_num*!(unblock_input[0]-4);break;
					case 2:
						unblock_input_num=unblock_input_num*!(unblock_input[1]-4);break;
					case 3:
						unblock_input_num=unblock_input_num*!(unblock_input[2]-4);break;
					default:
						unblock_input_num=0;break;
				}
			}
		}
		if(unblock_input_num==3)
{
					child_block=0;
					unblock_input_num=0;
					number_input=0;
					key_4_block=1;
//						a1_set_mode(0,10);
//					a1_set_mode(1,10);
}
		reduce_last=reduce;
		plus_last=plus;
		num_last=number_input;
	}

}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : get_float_from_4u8
  ** @brief         : 4 uint_8 2 float
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
float get_float_from_4u8_radar(unsigned char *p,int i)
{
	float a;
	unsigned char *r;
	r=(unsigned char*)&a;*r=p[i];
	r++;*r=p[i+1];
	r++;*r=p[i+2];
	r++;*r=p[i+3];
	return(a);
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : Mapan_Data_Process
  ** @brief         : 码盘接收数据处理函数
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : phead + 4 * (4 + 1) + ptail
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
float delt_y=153;
float Time;
float speed_handkey,X_jilu;
float deta_x,deta_y;
float x_last,y_last;
float length_mapan;
void Mapan_Data_Process(void)
{
	if((rec_data[0]==0X0D)&&(rec_data[1]==0X0A)/*&&(rec_data[26]==0X0A)&&(rec_data[27]==0X0D)*/)//Ö¡Í·Ö¡Î²ÅÐ¶Ï
	{
		float angle_template = get_float_from_4u8(&rec_data[2]);
		
		if(Angle_Z_last<angle_template)
			angle_Z_temp_1=angle_template-Angle_Z_last,
			angle_Z_temp_2=angle_template-Angle_Z_last-360;
		else
			angle_Z_temp_1=angle_template-Angle_Z_last,
			angle_Z_temp_2=360+angle_template-Angle_Z_last;

		angle_Z_temp=(fabs(angle_Z_temp_1))>(fabs(angle_Z_temp_2))? angle_Z_temp_2:angle_Z_temp_1;
		Angle_Z_last=angle_template;
		Action_Data.w_0=Action_Data.w_0+angle_Z_temp;
		Action_Data.angle_Z=Action_Data.w_0+Action_Data.zero_w;
		Action_Data.y_tmp= get_float_from_4u8(&rec_data[18]);
		Action_Data.x_tmp= get_float_from_4u8(&rec_data[14]);
		Action_Data.x_0 = Action_Data.x_tmp * cos(bata) - Action_Data.y_tmp * sin(bata);
		Action_Data.y_0 = Action_Data.x_tmp * sin(bata) + Action_Data.y_tmp * cos(bata);	
		deta_x=Action_Data.x_0-x_last;
		deta_y=Action_Data.y_0-y_last;
		
		Action_Data.x=Action_Data.x_0+Action_Data.zero_x;
		Action_Data.y=Action_Data.y_0+Action_Data.zero_y;

		Action_Data.x_reset=Action_Data.x*cos(Action_Data.zero_w*CHANGE_TO_RADIAN)-Action_Data.y*sin(Action_Data.zero_w*CHANGE_TO_RADIAN);
		Action_Data.y_reset=Action_Data.y*cos(Action_Data.zero_w*CHANGE_TO_RADIAN)+Action_Data.x*sin(Action_Data.zero_w*CHANGE_TO_RADIAN);
		if(Action_Data.x_reset!=0)Action_Data.x=Action_Data.x_reset;
		if(Action_Data.y_reset!=0)Action_Data.y=Action_Data.y_reset;
		Action_Data.w= get_float_from_4u8(&rec_data[22])+0.8f;

		speed_handkey=(Action_Data.x-X_jilu)/(Time*0.001f);
		X_jilu=Action_Data.x,Time=0;

	}
}

void zero_reset()
{
		Action_Data.zero_x=Action_Data.x_0;
		Action_Data.zero_y=Action_Data.y_0;
		angle_zero=angle_set;
}
	
