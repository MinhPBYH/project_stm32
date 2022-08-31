#ifndef _DS18B20_H
#define	_DS18B20_H

#include "stm32f1xx_hal.h"

typedef struct
{	
	TIM_HandleTypeDef* Timer;
	uint16_t Pin;
	GPIO_TypeDef* Port;
	float Temp;
}ds18b20_name;

#define DS18B20_SKIP_ROM					0xCC //gui tin hieu den tat ca Slave
#define DS18B20_CONVERT_T					0x44 // bat dau chuyen doi nhiet do
#define DS18B20_READ_SCRATCH_PAD	0xBE //Doc du lieu tu Slave

void ds18b20_init(ds18b20_name* ds18b20, TIM_HandleTypeDef* Timer, GPIO_TypeDef* DS_PORT, uint16_t DS_Pin);
float temperature(ds18b20_name* ds18b20);

#endif
