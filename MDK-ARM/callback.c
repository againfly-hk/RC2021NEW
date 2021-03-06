#include "bmi088_driver.h"
#include "bmi088.h"
#include "ist8310.h"
#include "pid.h"
#include "struct_typedef.h"
#include "movement_struct.h"
#include "callback.h"
#include "tim.h"
#include "usart.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "ultrasonic.h"
#include "kalman.h"
#define gravity				9.79484f

CAR car;//记录开始的姿态
bmi088_raw_data_t 	imu_raw_data;
bmi088_real_data_t 	imu_real_data;
pid_type_def imu_temp_pid;

uint8_t accel_flag=0;
uint8_t temp_flag=0;
uint8_t gyro_flag=0;
uint8_t failure_warning=0;
float gyro_erro[3];
float accel_errodata[3];
float mag[3];
float quat[4];

uint8_t kalman_accel_flag=0;
KalmanInfo accel[3];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ 
	if(GPIO_Pin==INT1_ACCEL_Pin){			
		BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);	
		if(accel_flag!=0){
			car.accel[0]=1.0072f*(imu_real_data.accel[0]-accel_errodata[0]);
			car.accel[1]=1.0065f*(imu_real_data.accel[1]-accel_errodata[1]);
			car.accel[2]=1.0073f*(imu_real_data.accel[2]-accel_errodata[2])+gravity;//修正校准误差
		}	
		if(kalman_accel_flag==1)
		{
			car.raccel[0]=KalmanFilter(&accel[0],car.accel[0]);
			car.raccel[1]=KalmanFilter(&accel[1],car.accel[1]);
			car.raccel[2]=KalmanFilter(&accel[2],car.accel[2]);
		}
	}
	if(GPIO_Pin==INT1_GYRO_Pin){
			BMI088_read_gyro(imu_real_data.gyro,&imu_real_data.temp);
			if(!temp_flag){
				if(imu_real_data.temp>=38.5f){
					temp_flag=1;
					imu_temp_pid.Iout=MPU6500_TEMP_PWM_MAX/2;
					__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.Iout);
					return;
				}
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,MPU6500_TEMP_PWM_MAX);
				return;
			}	
			else if(temp_flag!=0){
				PID_calc(&imu_temp_pid,imu_real_data.temp,40.0f);
				if (imu_temp_pid.out < 0.0f)	imu_temp_pid.out = 0.0f;
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.out);	
			}
			if(gyro_flag!=0){
				car.rgyro[0]=imu_real_data.gyro[0]-gyro_erro[0];
				car.rgyro[1]=imu_real_data.gyro[1]-gyro_erro[1];
				car.rgyro[2]=imu_real_data.gyro[2]-gyro_erro[2];
				if(gyro_flag==2){
				AHRS_update(quat,0.001f,car.rgyro,car.raccel,car.mag);			
				car.yaw=get_yaw(quat);
				}
			}
  }
	if(GPIO_Pin==IST8310_EXIT_Pin){
		ist8310_read_mag(mag);
		car.mag[0]=mag[0];
		car.mag[1]=mag[1];
		car.mag[2]=mag[2];//对变量进行处理
    echo_enable_ones();
	}
	if(GPIO_Pin==Failture_Pin){
		failure_warning=10;//当值为10时触发故障保护
	}
}
