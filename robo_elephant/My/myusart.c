#include "myusart.h"

/****
IDLE�ж�+DMA����
HAL_UART_RxCpltCallback   ����Э����������ݲŽ����ж�
  HAL_UART_Receive_IT ����ʹ�ܴ����жϣ���mainǰʹ��һ��
  HAL_UART_Transmit_DMA ���ڷ���DMA  HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
  HAL_UART_Receive_DMA ʹ�����½���DMA HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
HAL_UARTEx_RxEventCallback �����շ����������Ƚϴ������ҪƵ�������շ��ж��У�����IDLE�ж�+DMA���յķ�ʽ�շ�����
****/

//Data_Rx vision_data_receive;
//Data_Tx vision_data_send = {
//	.Txbuffer[0] = 0x0a,
//	.Txbuffer[1] = 0x0e,
//	.Txbuffer[BUFFSIZE + 2] = 0x0e,
//	.Txbuffer[BUFFSIZE + 3] = 0x0f,
//};

////���ճ�ʼ��
////void uart_start_receive(void)
////{
////	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, vision_data_receive.Rxbuffer, 4+BUFFSIZE);//�˴��޸����������ݸ���
////	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
////}
////��������һ������֡���ͻ����ûص�����
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  
//{
//	static int cnt = 0;
//	if(huart == &huart1)
//	{
//		HAL_UART_DMAStop(huart);//��ͣ��ѯģʽ
//		if((vision_data_receive.Rxbuffer[0]==0x0a)&&(vision_data_receive.Rxbuffer[1]==0x0d)&&
//			 (vision_data_receive.Rxbuffer[BUFFSIZE+2]==0x0d)&&(vision_data_receive.Rxbuffer[BUFFSIZE+3]==0x0a)){
//				 for(cnt = 0;cnt<BUFFSIZE;cnt++){
//					 vision_data_receive.Receive.ProcessedBuff[cnt]=vision_data_receive.Rxbuffer[cnt+2];
//				 }
//			 } 
//		HAL_UART_Transmit_DMA(&huart1, vision_data_receive.Rxbuffer, sizeof(vision_data_receive.Rxbuffer)); //XCOM��ÿ���������ݺ��涼�����0x0d,0x0a
//		uart_start_receive(); //ʹ�ܽ���
//	}
//} 
////���ͺ���
//void vision_send_data(double send_flag)
//{
//	static int cnt2 = 0;
//	vision_data_send.Send.SendDouble = send_flag;
//  for(cnt2 = 2;cnt2 < BUFFSIZE + 2;cnt2++){
//		  vision_data_send.Txbuffer[cnt2] = vision_data_send.Send.ProcessedBuff[cnt2 - 2];
//	} 	
//	HAL_UART_Transmit_IT(&huart1, vision_data_send.Txbuffer, BUFFSIZE+4); 
//}
