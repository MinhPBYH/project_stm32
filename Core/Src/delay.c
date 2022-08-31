#include "main.h"

void delay_init(TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Start(htim);
}

void delay_us(TIM_HandleTypeDef *htim, uint16_t time)
{
	__HAL_TIM_SET_COUNTER(htim,0);
	while(__HAL_TIM_GET_COUNTER(htim)<time){}
}
void delay_ms(TIM_HandleTypeDef *htim, uint16_t Time)
{
	__HAL_TIM_SET_COUNTER(htim,0);
	while(Time--)
	{
		while(__HAL_TIM_GET_COUNTER(htim)<1000){}
	}
}
