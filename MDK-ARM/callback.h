#ifndef CALLBACK_H
#define CALLBACK_H

#include "struct_typedef.h"
#include "movement_struct.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2
#define IST8310_RX_BUF_DATA_OFFSET 16

#define TEMPERATURE_PID_KP 4000.0f //�¶ȿ���PID��kp
#define TEMPERATURE_PID_KI 0.050f    //�¶ȿ���PID��ki
#define TEMPERATURE_PID_KD 0.000f    //�¶ȿ���PID��kd

#define TEMPERATURE_PID_MAX_OUT   4000.0f //�¶ȿ���PID��max_out
#define TEMPERATURE_PID_MAX_IOUT 1500.0f  //�¶ȿ���PID��max_iout

#define MPU6500_TEMP_PWM_MAX 4000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1

#define INS_TASK_INIT_TIME 7 //����ʼ���� delay һ��ʱ��

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

#endif
