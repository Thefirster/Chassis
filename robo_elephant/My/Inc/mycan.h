#ifndef __MYCAN_H
#define __MYCAN_H

#include "main.h"
#include "can.h"
#include "M3508.h"
#include "DM_J4310.h"
#include "string.h"
#include "JY_ME02.h"

HAL_StatusTypeDef Can1_Configure(void);
void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, CAN_HandleTypeDef *hcan); //VESCÊý¾Ý·¢ËÍ

#endif
