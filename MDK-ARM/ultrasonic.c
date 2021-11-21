#include "ultrasonic.h"
#include "struct_typedef.h"
#include "usart.h"
int rx_echo;
uint8_t rx_echo_buff[2];
uint8_t buf=0x55;
void echo_enable_ones()
{
	HAL_UART_Transmit_DMA(&huart6,&buf,1);//Ê¹ÄÜ³¬Éù²¨}
}
