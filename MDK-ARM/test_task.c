#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "test_task.h"
#include "tim.h"

#include "door_control.h"
#include "ist8310.h"
#include "bmi088_driver.h"

void test_task(void const * argument)
{
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,500);
	door_left();
	left_door_off();
	right_door_off();
	front_door_down();
	back_door_down();
	door_reset();
	
	
	while(1){
		osDelay(100);
	}
}
