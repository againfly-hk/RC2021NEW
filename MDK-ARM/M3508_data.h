#ifndef __M3508_DATA_H__
#define __M3508_DATA_H__
#include "movement_struct.h"
#include "main.h"
extern void motor_12_code(uint8_t i);
extern void motor_03_code(uint8_t i);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif 
