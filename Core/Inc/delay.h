#ifndef _DELAY_H
#define _DELAY_H
#include "stm32f1xx_hal_tim.h"

void delay_init(TIM_HandleTypeDef *htim);
void delay_us(TIM_HandleTypeDef *htim, uint16_t time);
void delay_ms(TIM_HandleTypeDef *htim, uint16_t Time);

#endif

