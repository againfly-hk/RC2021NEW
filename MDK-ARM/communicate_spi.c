#include "main.h"
#include "spi.h"

uint8_t spicnt=0;
uint8_t spi_tx_buff[3];
uint8_t spi_rx_buff[3];

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi==&hspi2)
	{
			HAL_SPI_Transmit(&hspi2,&spi_tx_buff[spicnt],1,10);
			spicnt+=1;
			spicnt%=3;
			HAL_SPI_Receive_IT(&hspi2,&spi_rx_buff[spicnt],1);
	}
}

