#include "myusart.h"

/****
IDLE中断+DMA接收
HAL_UART_RxCpltCallback   按照协议接受完数据才进入中断
  HAL_UART_Receive_IT 重新使能串口中断，在main前使用一次
  HAL_UART_Transmit_DMA 串口发送DMA  HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
  HAL_UART_Receive_DMA 使能重新接收DMA HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
HAL_UARTEx_RxEventCallback 当所收发的数据量比较大或者需要频繁进入收发中断中，采用IDLE中断+DMA接收的方式收发数据
****/

//Data_Rx vision_data_receive;
//Data_Tx vision_data_send = {
//	.Txbuffer[0] = 0x0a,
//	.Txbuffer[1] = 0x0e,
//	.Txbuffer[BUFFSIZE + 2] = 0x0e,
//	.Txbuffer[BUFFSIZE + 3] = 0x0f,
//};

////接收初始化
////void uart_start_receive(void)
////{
////	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, vision_data_receive.Rxbuffer, 4+BUFFSIZE);//此处修改最大接收数据个数
////	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
////}
////当接受完一个数据帧，就会进入该回调函数
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  
//{
//	static int cnt = 0;
//	if(huart == &huart1)
//	{
//		HAL_UART_DMAStop(huart);//暂停轮询模式
//		if((vision_data_receive.Rxbuffer[0]==0x0a)&&(vision_data_receive.Rxbuffer[1]==0x0d)&&
//			 (vision_data_receive.Rxbuffer[BUFFSIZE+2]==0x0d)&&(vision_data_receive.Rxbuffer[BUFFSIZE+3]==0x0a)){
//				 for(cnt = 0;cnt<BUFFSIZE;cnt++){
//					 vision_data_receive.Receive.ProcessedBuff[cnt]=vision_data_receive.Rxbuffer[cnt+2];
//				 }
//			 } 
//		HAL_UART_Transmit_DMA(&huart1, vision_data_receive.Rxbuffer, sizeof(vision_data_receive.Rxbuffer)); //XCOM在每个传输数据后面都会加上0x0d,0x0a
//		uart_start_receive(); //使能接收
//	}
//} 
////发送函数
//void vision_send_data(double send_flag)
//{
//	static int cnt2 = 0;
//	vision_data_send.Send.SendDouble = send_flag;
//  for(cnt2 = 2;cnt2 < BUFFSIZE + 2;cnt2++){
//		  vision_data_send.Txbuffer[cnt2] = vision_data_send.Send.ProcessedBuff[cnt2 - 2];
//	} 	
//	HAL_UART_Transmit_IT(&huart1, vision_data_send.Txbuffer, BUFFSIZE+4); 
//}
