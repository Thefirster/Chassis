#include "my_uart.h"
#include "usart.h"
#include "A1_motor.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
USART1_RECEIVETYPE Usart1Type1;
static void Usart1_DataProcess(uint8_t* pData);
uint8_t ceju_datarc[8];
extern MOTOR_recv motor_recv[2];

//void Usart1_Start(void)
//{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart1, Usart1Type1.RX_pData, RX_LEN1);
//}
//void Usart6_Start(void)
//{
//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart6, Usart1Type1.RX_pData, RX_LEN1);
//}
void Usart4_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart4, Usart1Type1.RX_pData, RX_LEN1);
}
//void Usart5_Start(void)
//{
//	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart5, Usart1Type1.RX_pData, RX_LEN1);
//}

uint32_t times = 0;
uint32_t err_times = 0;
//void Usart6Receive_IDLE(void)
//{
//	uint32_t temp;
//	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
//		HAL_UART_DMAStop(&huart6);
//		temp = ((DMA_Stream_TypeDef*)huart6.hdmarx->Instance)->NDTR;
//		Usart1Type1.RX_Size = RX_LEN1 - temp;

//		//数据处理
//		Usart1_DataProcess(Usart1Type1.RX_pData);
//		
//		HAL_UART_Receive_DMA(&huart6, Usart1Type1.RX_pData, RX_LEN1);
//		Usart1Type1.RX_flag = 1;
//	}
//}
void Usart6_Start()
{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6,Usart1Type1.RX_pData,RX_LEN1);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
}

void Usart5_Start()
{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,ceju_datarc,8);
	__HAL_UART_ENABLE_IT(&huart5,DMA_IT_HT);
}
uint8_t USART6_error_flag = 0;

void float2u8Arry(uint8_t *u8Arry, float *floatdata, int key)
{
    uint8_t farray[4];
    *(float *)farray = *floatdata;
    if (key == 1)
    {
        u8Arry[3] = farray[0];
        u8Arry[2] = farray[1];
        u8Arry[1] = farray[2];
        u8Arry[0] = farray[3];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
        u8Arry[2] = farray[2];
        u8Arry[3] = farray[3];
    }
}

float get_float_from_4u8(unsigned char *p)
{
	float a;
	unsigned char *r;
	r=(unsigned char*)&a;
	*r=p[0];
	r++;
	*r=p[1];
	r++;
	*r=p[2];
	r++;
	*r=p[3];
	return(a);
}	
static void Usart1_DataProcess(uint8_t* pData)//78字节
{
	//motor_recv = (MOTOR_recv)Usart1Type1.RX_pData;
	//memcpy(&motor_recv, Usart1Type1.RX_pData, 78);
	times++;
	int num=0;
	//MOTOR_recv recv;
	if(pData[0] == 0xFE && pData[1] == 0xEE)
	{
		if(pData[2]==0)num=0;
		else num=1;
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
	if(USART6_error_flag == 1) //串口中断错误处理
  {
    __HAL_UART_DISABLE(&huart6);
    __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);                           
    memset(Usart1Type1.RX_pData, 0, RX_LEN1);
		__HAL_UART_ENABLE(&huart6);
		HAL_UART_Receive_DMA(&huart6, Usart1Type1.RX_pData, RX_LEN1);
    USART6_error_flag = 0;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); 
    if(huart->Instance==USART6)
    {
        HAL_UART_DMAStop(huart);
		Usart1_DataProcess(Usart1Type1.RX_pData);
		Usart6_Start();
		
    }
		taskEXIT_CRITICAL_FROM_ISR(status_value);
}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//  if(huart->Instance == USART6)
//  {
//		err_times++;
//    __HAL_UART_CLEAR_OREFLAG(&huart6);
//    USART6_error_flag = 1;//USART 异常初始化标志位
//  }
//}

