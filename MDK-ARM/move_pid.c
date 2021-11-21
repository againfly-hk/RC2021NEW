#include "pid.h"
#include "move_pid.h"
#include "CAN_receive.h"

extern CAR car;
extern motor_measure_t motor_chassis[7];
pid_type_def motor_move_speed_pid[4];

void move_pid_calc(void)
{
		//相对于车体坐标系
		//要乘以一个-1,这里是根据vx,vy,w反解出来的速度，用于速度环的控制,控制rpm  
		car.v1=(-1*(1*car.vx+1*car.vy+28*car.w)*23.7946f);//cm/s->rpm
		car.v2=(-1*(-1*car.vx+1*car.vy+28*car.w)*23.7946f);//cm
		car.v3=(-1*(-1*car.vx-1*car.vy+28*car.w)*23.7946f);
		car.v4=(-1*(1*car.vx+-1*car.vy+28*car.w)*23.7946f);
		PID_calc(&motor_move_speed_pid[0],motor_chassis[0].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[1],motor_chassis[1].speed_rpm,car.v2);
		PID_calc(&motor_move_speed_pid[2],motor_chassis[2].speed_rpm,car.v3);
		PID_calc(&motor_move_speed_pid[3],motor_chassis[3].speed_rpm,car.v4);
		CAN_cmd_chassis(motor_move_speed_pid[0].out,motor_move_speed_pid[1].out,motor_move_speed_pid[2].out,motor_move_speed_pid[3].out);			
}

void codemove_init()
{
	#ifdef speed_control_using
  float codemove_pid[3]={0.002,0.000,0};
	PID_init(&motor_move_displace_pid[0],PID_POSITION,codemove_pid,20,5);
	PID_init(&motor_move_displace_pid[1],PID_POSITION,codemove_pid,20,5);
	PID_init(&motor_move_displace_pid[2],PID_POSITION,codemove_pid,20,5);
	PID_init(&motor_move_displace_pid[3],PID_POSITION,codemove_pid,20,5);
	#endif
	
	float speed_pid[3]={3.8,1,10};
	PID_init(&motor_move_speed_pid[0],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[1],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[2],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[3],PID_POSITION,speed_pid,6000,2000);
	
	#ifdef yaw_control
	float yaw_pid[3]={0.02,,0,0};
	#endif
}


