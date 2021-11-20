#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "movement_struct.h"
#include "main.h"

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,	
	  CAN_PORTAL_FRAME_LEFT_ID=0x205,
	  CAN_PORTAL_FRAME_RIGHT_ID=0x206,
	  CAN_DRAWER_ID=0X207
} can_msg_id_e;//CAN_3508µç»úID

extern void CAN_cmd_portal_frame(int16_t left, int16_t right,int16_t drawer);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
