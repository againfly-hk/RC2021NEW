#ifndef __MOVEMENT_STRUCT_H__
#define __MOVEMENT_STRUCT_H__

#include "struct_typedef.h"

typedef struct
{
	uint8_t mode;
	uint8_t	order_final;
	float rata;
	float v;
	float w;
	float angle;
	double displacement;
}order_t;//移动命令格式

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;//CAN_3508电机数据

typedef struct{
	int change;
	int delta;
	int last_value;
	int cnt;
}code;//3508电机编码器

typedef struct{
	float raccel[3];
	float rgyro[3];
	float mag[3];
	float yaw;
	float begin_yaw;
	float v1,v2,v3,v4,vx,vy,w;
	float integral_gyro[3];	
	float mag_begin[3];//初始位置的mag角度	
}CAR;//车的数据
#endif
