#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "user_task.h"

void line_detect_task(void const * argument)
{
	while(1){
		osDelay(100);
	}
}
	

