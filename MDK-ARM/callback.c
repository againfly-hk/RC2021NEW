#include "bmi088_driver.h"
#include "ist8310.h"

#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"

#include "pid.h"
#include "Callback.h"
#include "frame.h"
#include "door_servo.h"
#include "can.h"
#include "math.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "CAN_receive.h"

#include "user_lib.h"
#include "arm_math.h"
#include "codemove.h"
#include "movement.h"

#define Communication 3
#define gravity				9.79484
#define move_test
#define CAR_TURE failure_warning!=10
bmi088_raw_data_t 	imu_raw_data;
bmi088_real_data_t 	imu_real_data;

uint8_t door_flag=4;
uint8_t temp_flag=0;
fp32 ist8310real[3];

pid_type_def motor_pid[7];	
pid_type_def imu_temp_pid;
uint8_t gyro_flag=0;
uint8_t accel_flag=0;
double cos_tri[3];
float accel_errodata[3];

extern fp32 BMI088_ACCEL_SEN;
extern fp32 BMI088_GYRO_SEN;
extern uint8_t rx_echo_buff[5];
extern uint8_t buf;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern code motor[4];
extern uint8_t rx_line_buff[];

extern uint16_t step_cnt;
extern order_t order[];

extern pid_type_def motor_move_displace_pid[4];
extern pid_type_def motor_move_speed_pid[4];

extern pid_type_def speed_pid;//整体速度环
extern pid_type_def motor_speed_pid[4];//电机速度环(电机环)
extern pid_type_def roll_pid;//roll环控制pid
//怎么解算整体速度环
//具体内容看速度的运动学状态解析（先完成直线平移和旋转的简单命令，并测试互补逻辑）
extern motor_measure_t motor_chassis[7];
uint8_t spi_rxbuff[20];
uint8_t spi_txbuff[20];
uint8_t rx_light[3];
extern uint8_t failure_warning;
float rx_echo;
float mag[3];
float quat[4]={1.0f,0.0f,0.0f,0.0f};
int pwm_set=1300;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
float gyro_erro[3];
float accel_erro[3];
extern int echo_distance;
float angle;
float cosa;
float sina;

int displacement=0;

uint8_t code_record=1;
uint8_t order_step=0;
extern uint8_t spi_tx_buff[8];
extern uint8_t spi_rx_buff[8];
extern int frame_high;
first_order_filter_type_t accel_filter[3];
extern int motor_code_using;
//在中断里面解算姿态
CAR car;//记录开始的姿态


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	if(GPIO_Pin==INT1_ACCEL_Pin)
	{			
		BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);	
		if(accel_flag!=0)//在校准完后开始读取数据
		{
			car.raccel[0]=1.0072*(imu_real_data.accel[0]-accel_errodata[0]);
			car.raccel[1]=1.0065*(imu_real_data.accel[1]-accel_errodata[1]);
			car.raccel[2]=1.0073*(imu_real_data.accel[2]-accel_errodata[2])+gravity;//修正校准误差
		}		
	}
	if(GPIO_Pin==INT1_GYRO_Pin)
	{
		{
			BMI088_read_gyro(imu_real_data.gyro,&imu_real_data.temp);
			if((!temp_flag))
			{
				if(imu_real_data.temp>=38.5f)	
				{
					temp_flag=1;
					imu_temp_pid.Iout=MPU6500_TEMP_PWM_MAX/2;
					__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.Iout);
					return;
				}
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,MPU6500_TEMP_PWM_MAX);
				return;
			}	
			else if(temp_flag!=0)
			{
				PID_calc(&imu_temp_pid,imu_real_data.temp,40.0f);
				if (imu_temp_pid.out < 0.0f)	imu_temp_pid.out = 0.0f;
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.out);	//
			}
		}//BMI088的温度控制
	
		{
			if(gyro_flag!=0)
			{
				car.rgyro[0]=imu_real_data.gyro[0]-gyro_erro[0];
				car.rgyro[1]=imu_real_data.gyro[1]-gyro_erro[1];
				car.rgyro[2]=imu_real_data.gyro[2]-gyro_erro[2];
				if(gyro_flag==2)
				{
				AHRS_update(quat,0.001f,car.rgyro,car.raccel,car.mag);			
				car.yaw=get_yaw(quat);
				}
			}
		}
  }
	if(GPIO_Pin==IST8310_EXIT_Pin)//需要使用磁力计进行方向角的确定
	{
		ist8310_read_mag(mag);
		car.mag[0]=mag[0];
		car.mag[1]=mag[1];
		car.mag[2]=mag[2];//对变量进行处理
		HAL_UART_Transmit_DMA(&huart6,&buf,1);//使能超声波
	}
	
	if(GPIO_Pin==Failture_Pin)
	{
		failure_warning=10;//当值为10时触发故障保护
	}
}
