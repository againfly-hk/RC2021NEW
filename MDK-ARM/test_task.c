#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "test_task.h"
#include "tim.h"
#include "can.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"

#include "door_control.h"
#include "ist8310.h"
#include "bmi088_driver.h"
#include "frame.h"
#include "pid.h"
#include "callback.h"
#include "struct_typedef.h"
#include "kalman.h"
#include "move_pid.h"
uint8_t doortesting=0;
extern pid_type_def imu_temp_pid;
extern uint8_t temp_flag;
extern uint8_t gyro_flag;
extern uint8_t accel_flag;
extern CAR car;
extern float quat[];
extern bmi088_raw_data_t 	imu_raw_data;
extern bmi088_real_data_t 	imu_real_data;
extern float mag[3];
extern float gyro_erro[3];
extern float accel_errodata[3];

static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
extern KalmanInfo accel[3];
extern uint8_t kalman_accel_flag;
extern int frame_high;
void test_task(void const * argument)
{
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,500);
	door_left();
	left_door_off();
	right_door_off();
	front_door_down();
	back_door_down();
	door_reset();
	
	PID_init(&imu_temp_pid,PID_POSITION,imu_temp_PID,TEMPERATURE_PID_MAX_OUT,TEMPERATURE_PID_MAX_IOUT);	
	while(BMI088_init())	{osDelay(5);}
	while(ist8310_init())	{osDelay(5);}
	frame_pid_init();
	move_pid_init();
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	uint8_t cnt=0;
	while(temp_flag==0)	osDelay(10);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1000);
	if(temp_flag==1){
		for(cnt=0;cnt<100;cnt++){
			gyro_erro[0]+=imu_real_data.gyro[0];
			gyro_erro[1]+=imu_real_data.gyro[1];
			gyro_erro[2]+=imu_real_data.gyro[2];
		
			accel_errodata[0]+=imu_real_data.accel[0];
			accel_errodata[1]+=imu_real_data.accel[1];
			accel_errodata[2]+=imu_real_data.accel[2];
			osDelay(20);
		}
		gyro_erro[0]=gyro_erro[0]/100.0f;
		gyro_erro[1]=gyro_erro[1]/100.0f;
		gyro_erro[2]=gyro_erro[2]/100.0f;
		
		accel_errodata[0]=accel_errodata[0]/100.0f;
		accel_errodata[1]=accel_errodata[1]/100.0f;
		accel_errodata[2]=accel_errodata[2]/100.0f;
		
		gyro_flag=1;//gyro_flag change
		accel_flag=1;//accel_flag change
		//代表已经完成误差值记录
		
		car.mag_begin[0]=mag[0];
		car.mag_begin[1]=mag[1];
		car.mag_begin[2]=mag[2];//Record the compass Angle at the beginning
		osDelay(200);//delay 0.5ms
		Init_KalmanInfo(&accel[0],50,0.000560f);
		Init_KalmanInfo(&accel[1],50,0.000560f);
		Init_KalmanInfo(&accel[2],50,0.000560f);
		kalman_accel_flag=1;	
    osDelay(50);		
		AHRS_init(quat,car.raccel,car.mag_begin);	
		osDelay(50);
		gyro_flag=2;
		osDelay(20);
		car.begin_yaw=car.yaw;
		frame_high=150000;
	}

	
	while(1){
		move_pid_calc();
		osDelay(2);
	}
}
