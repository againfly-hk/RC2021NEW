#include "door_control.h"

void left_door_on()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000);
	
}

void left_door_off()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);
}

void right_door_on()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,2070);
}

void right_door_off()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,2570);
}

void door_left()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,2100);
}

void door_middle()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);
}

void door_right()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1100);
}

void door_reset()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,500);
}

void front_door_lift()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,600);
}
void front_door_down()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1300);
}

void rockert_level()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,2500);
}
void rocker_vertical()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1500);
}
void rocket_reserse_side()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,500);
}

void back_door_away()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
}

void back_door_down()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1000);
}
