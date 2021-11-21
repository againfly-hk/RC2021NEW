#include "frame.h"
#include "pid.h"
#include "CAN_receive.h"
#include "stdlib.h"
#include "arm_math.h"
#define frame_left_kp 0.40
#define frame_left_ki 0.01
#define frame_left_kd 40
#define frame_left_max_out  3200
#define frame_left_max_iout 600

#define frame_right_kp 0.40
#define frame_right_ki 0.01
#define frame_right_kd 40
#define frame_right_max_out  3200
#define frame_right_max_iout 600

extern motor_measure_t motor_chassis[7];
	
pid_type_def frame_pid[2];
int frame_delta[2];
int frame_change[2];
int frame_high=0;
float kflag=0.4;
float v1_control=0;
float v2_control=0;

void frame_pid_init()
{
  float pid[3]={frame_left_kp,frame_left_ki,frame_left_kd};
	PID_init(&frame_pid[0],PID_POSITION,pid,frame_left_max_out,frame_left_max_iout);
  pid[0]=frame_right_kp;
	pid[1]=frame_right_ki;
	pid[2]=frame_right_kd;
	PID_init(&frame_pid[1],PID_POSITION,pid,frame_right_max_out,frame_right_max_iout);
}

void frame_reset()
{
	CAN_cmd_portal_frame(0,0,0);
	if(frame_change[0]<5000)	frame_change[0]=0;
	if(frame_change[1]<5000)	frame_change[1]=0;
}

void frame_code_control(uint16_t i)
{
	if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))
		frame_delta[i-4]=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
	else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))
		frame_delta[i-4]=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
	else
		frame_delta[i-4]=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
	frame_change[i-4]+=frame_delta[i-4];	

	PID_calc(&frame_pid[0],frame_change[0],frame_high);
				PID_calc(&frame_pid[1],frame_change[1],frame_high);
				if(fabs((float)(frame_high-frame_change[0]))<20000)
				{
					v1_control=motor_chassis[4].speed_rpm;
					v2_control=motor_chassis[5].speed_rpm;
				}
				else
				{
					v1_control=0;
					v2_control=0;
				}
				frame_pid[0].out+=(frame_pid[1].fdb-frame_pid[0].fdb)*kflag;
			  CAN_cmd_portal_frame(frame_pid[0].out-v1_control*2,frame_pid[1].out-v2_control*2,0);	
}



