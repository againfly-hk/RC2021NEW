#include "CAN_receive.h"
extern CAN_HandleTypeDef hcan1;
#define CHASSIS_CAN hcan1
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
static CAN_TxHeaderTypeDef  portal_frame_tx_message;
static uint8_t              portal_frame_can_send_data[8];
		
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
		
void CAN_cmd_portal_frame(int16_t left, int16_t right,int16_t drawer)
{
    uint32_t send_mail_box;
    portal_frame_tx_message.StdId = 0x1FF;
    portal_frame_tx_message.IDE = CAN_ID_STD;
    portal_frame_tx_message.RTR = CAN_RTR_DATA;
    portal_frame_tx_message.DLC = 0x08;
    portal_frame_can_send_data[0] = (left>> 8);
    portal_frame_can_send_data[1] = left;
    portal_frame_can_send_data[2] = (right >> 8);
    portal_frame_can_send_data[3] = right;
	  portal_frame_can_send_data[4] = (drawer >> 8);
    portal_frame_can_send_data[5] = drawer;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &portal_frame_tx_message,portal_frame_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
