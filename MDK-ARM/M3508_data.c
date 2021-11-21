#include "M3508_data.h"
#include "CAN_receive.h"
#include "main.h"
#include "pid.h"
#include "frame.h"
#include "Callback.h"
#include "math.h"

extern CAN_HandleTypeDef hcan1;
extern pid_type_def frame_pid[2];
extern int frame_high;
motor_measure_t motor_chassis[7];
code motor_3508[4];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		int8_t i;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch(rx_header.StdId)
    {
			{
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_PORTAL_FRAME_LEFT_ID:
        case CAN_PORTAL_FRAME_RIGHT_ID:			
				i=rx_header.StdId-0x201;
				get_motor_measure(&motor_chassis[i],rx_data);//0,1,2,3 motor 4,5 frame							
				break;
       }
       default:
       {
				 break;
       }	 
    }//reveive data to calculata
		if(i==4){
			if(frame_high==0)	frame_reset();
			else	frame_code_control(i);
		}
		else if((i==1)||(i==2))	motor_12_code(i);
		else if((i==0)||(i==3)) motor_03_code(i);
}

void motor_12_code(uint8_t i)
{
	if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))	motor_3508[i].delta=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
	else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))	motor_3508[i].delta=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
	else	motor_3508[i].delta=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
	
  motor_3508[i].last_value=motor_3508[i].change;
	motor_3508[i].change+=motor_3508[i].delta;
//	if((motor_3508[i].last_value<0x7fffffff)&&(motor_3508[i].last_value>0x70000000)&&(motor_3508[i].change<0xffffff))	motor_3508[i].cnt++;
//	if((motor_3508[i].last_value<-0x7fffffff)&&(motor_3508[i].last_value>-0x70000000)&&(motor_3508[i].change>-0xffffff))	motor_3508[i].cnt--;
}

void motor_03_code(uint8_t i)
{
				if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))	motor_3508[i].delta=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
				else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))	motor_3508[i].delta=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
				else	motor_3508[i].delta=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
        motor_3508[i].last_value=motor_3508[i].change;	
				motor_3508[i].change-=motor_3508[i].delta;
}
