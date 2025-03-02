#include "A1_motor.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "stdint.h"

#define PI 3.1415926f

MOTOR_send motor_s;
MOTOR_recv motor_recv[2];
extern A1_data a1_data[2];
float test_t = 0;
//接收缓存区 	
uint8_t RS485_RX_BUF[78];  	//接收缓冲,最大64个字节.
//接收到的数据长度
uint8_t RS485_RX_CNT=0;  

uint8_t res = 0;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART1)
//    {
//        if(RS485_RX_CNT<78)
//        {
//            RS485_RX_BUF[RS485_RX_CNT]=res;		//记录接收到的值
//            RS485_RX_CNT++;						//接收数据增加1 
//        } 
//     HAL_UART_Receive_IT(&huart1, (uint8_t *)&res, 1);
//        
//    }											 
//}
enum motor_command_type
{
    TORQUE,
    POSITION,
    VELOCITY
};


uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t bits = 0;
	uint32_t i = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	
	for (i = 0; i < len; i++)
	{
			xbit = (uint32_t)1 << 31;
			data = ptr[i];
		for (bits = 0; bits < 32; bits++)
		{
			if (CRC32 & 0x80000000)
			{
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			}
			else
				CRC32 <<= 1;
			if (data & xbit)
				CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}
	}
	return CRC32;
}

int modify_data(MOTOR_send* motor)
{
	motor->motor_send_data.head.start[0] = 0xFE;
	motor->motor_send_data.head.start[1] = 0xEE;
	motor->motor_send_data.head.motorID = motor->id;
	
	motor->motor_send_data.Mdata.mode = motor ->mode;
	
    motor->motor_send_data.Mdata.ModifyBit = 0xff;
   // motor->motor_send_data.Mdata.ReadBit = 0x01;
    
	motor->motor_send_data.Mdata.T = (int16_t)((motor ->T)*256);
	motor->motor_send_data.Mdata.W = (int16_t)((motor ->W)*128);
	motor->motor_send_data.Mdata.Pos = (motor ->Pos)*16384/6.2832f;
	motor->motor_send_data.Mdata.K_P = (int16_t)((motor ->K_P)*2048);
	motor->motor_send_data.Mdata.K_W = (int16_t)((motor ->K_W)*1024);
	
	motor->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)motor,7);
	
	motor ->hex_len = 34;

	return 0;
}



void A1_Motor_Send_Data(float W, float Pos, float k_p, float k_w,int id,int mode)
{
    motor_s.id = id;                 //motor ID
    motor_s.mode = mode;              //switch to servo mode
    motor_s.T = test_t;            //Nm, T<255.9
    motor_s.W = W;               //rad/s, W<511.9
    motor_s.Pos = Pos;          //rad, Pos<131071.9
    motor_s.K_P = k_p;        //K_P<31.9     value should around 0.1
    motor_s.K_W = k_w;        //K_W<63.9     value should around 3

    modify_data(&motor_s);
    
		RS485_TX_EN(GPIO_PIN_SET);
    
    HAL_UART_Transmit(&huart6 ,(uint8_t *)&motor_s, 34, 0xFFF);
    //HAL_UART_Transmit_DMA(&huart1 ,(uint8_t *)&motor_s, 34);
    RS485_TX_EN(GPIO_PIN_RESET);
}

void A1_Motor_Send_Data_yuntai(float W, float Pos, float k_p, float k_w, float t, int id, int mode)
{
    motor_s.id = id;          // motor ID
    motor_s.mode = mode;      // switch to servo mode
    motor_s.T = t;            // Nm, T<255.9
    motor_s.W = W * 9.0f;     // rad/s, W<511.9
    motor_s.Pos = Pos * 9.0f; // rad, Pos<131071.9
    motor_s.K_P = k_p;        // K_P<31.9     value should around 0.1
    motor_s.K_W = k_w;        // K_W<63.9     value should around 3

    modify_data(&motor_s);

    RS485_TX_EN(GPIO_PIN_SET);

    HAL_UART_Transmit(&huart6, (uint8_t *)&motor_s, 34, 0xFFF);
    // HAL_UART_Transmit_DMA(&huart1 ,(uint8_t *)&motor_s, 34);
    RS485_TX_EN(GPIO_PIN_RESET);
}

void a1_init()
{
    a1_data[0].id = 0;
    a1_data[0].mode = 0;
    a1_data[0].kw = 0; // 5
    a1_data[0].kp = 0; // 0.05
    a1_data[0].pos = 0;
    a1_data[0].w = 0;

    a1_data[1].id = 1;
    a1_data[1].mode = 0;
    a1_data[1].kw = 5;
    a1_data[1].kp = 0.05;
    a1_data[1].pos = 0;
    a1_data[1].w = 0;
}

// 返回输出轴弧度
float a1_get_position(uint8_t id)
{
    return motor_recv[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f;
}

// 返回输出轴角速度
float a1_get_speed(uint8_t id)
{
    return motor_recv[id].motor_recv_data.Mdata.W / 128.0f / 9.0f;
}

// 等待电机回传数据
void a1_waiting_recv(uint8_t id)
{
    while (motor_recv[id].motor_recv_data.Mdata.Pos == 0)
    {
        HAL_Delay(2);
    }
}

// 设置电机mode
void a1_set_mode(uint8_t id, uint8_t x)
{
    a1_data[id].mode = x;
}

void a1_set_kp_kw(uint8_t id, float kp, float kw)
{
    a1_data[id].kp = kp;
    a1_data[id].kw = kw;
		a1_data[id].w = 0;
		//a1_data[id].pos = 0;
	//a1_data[id]. = kw;
}
void a1_set_kw_w(uint8_t id,float kw,float w)
{
    a1_data[id].kp = 0;
    a1_data[id].kw = kw;
		a1_data[id].w = w;
		a1_data[id].pos = 0;		
}

void a1_set_torque(uint8_t id, float x)
{
    if (x > 255)
        a1_data[id].t = 255;
    else if (x < -255)
        a1_data[id].t = -255;
    else
        a1_data[id].t = x;
}

// 输出轴旋转弧度
void a1_set_pos(uint8_t id, float rad)
{
    a1_data[id].pos = rad;
}

// 输出轴旋转角速度
void a1_set_w(uint8_t id, float w)
{
    a1_data[id].w = w;
}

