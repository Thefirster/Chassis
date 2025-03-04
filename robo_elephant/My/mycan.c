#include "mycan.h"

////can≥ı ºªØ
//HAL_StatusTypeDef Can1_Configure(void)  
//{
//	CAN_FilterTypeDef  sFilterConfig;
//	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0x0000;
//	sFilterConfig.FilterIdLow = 0x0000;
//	sFilterConfig.FilterMaskIdHigh = 0x0000;
//	sFilterConfig.FilterMaskIdLow = 0x0000;
//	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//	sFilterConfig.FilterActivation = ENABLE;
//	sFilterConfig.SlaveStartFilterBank = 0;
//	sFilterConfig.FilterBank = 0;
//  
//	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
//	HAL_CAN_Start(&hcan1);
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//  return HAL_OK;
//}





