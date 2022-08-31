#include "main.h"
#include "ds18b20.h"
#include "delay.h"

static void ds18b20_delay_init(ds18b20_name* ds18b20)
{
	delay_init(ds18b20->Timer);
}

static void ds18b20_delay_us(ds18b20_name* ds18b20, uint16_t time)
{
	delay_us(ds18b20->Timer, time);
}

static void ds18b20_delay_ms(ds18b20_name* ds18b20, uint16_t time)
{
	delay_ms(ds18b20->Timer, time);
}
//set pin output
static void Set_Pin_Output (ds18b20_name* ds18b20)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ds18b20->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ds18b20->Port, &GPIO_InitStruct);
}

//set pin input
static void Set_Pin_Input (ds18b20_name* ds18b20)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ds18b20->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ds18b20->Port, &GPIO_InitStruct);
}
// Return 0: OK ; 1:FAIL
static unsigned char ds18b20_reset(ds18b20_name* ds18b20){
	unsigned char result;
	Set_Pin_Output (ds18b20);
	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 0);

	ds18b20_delay_us(ds18b20, 480);
	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 1);
	Set_Pin_Input (ds18b20);	
	ds18b20_delay_us(ds18b20, 70);

	result = HAL_GPIO_ReadPin(ds18b20->Port, ds18b20->Pin);
	ds18b20_delay_us(ds18b20, 410);

	return result;
}
// ham gui 1 bit vao DS18B20
static void ds18b20_write_bit(ds18b20_name* ds18b20, unsigned char bit){
	if(bit == 1){
	Set_Pin_Output (ds18b20);
	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 0);

	ds18b20_delay_us(ds18b20, 6);

	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 1);
	ds18b20_delay_us(ds18b20, 64);
	}else{

	Set_Pin_Output (ds18b20);
	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 0);


	ds18b20_delay_us(ds18b20, 60);

	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 1);

	ds18b20_delay_us(ds18b20, 10);
	}
}
//ham doc 1 bit nhan ve tu DS18B20
static unsigned char ds18b20_read_bit(ds18b20_name* ds18b20){
	unsigned char result = 0;

	Set_Pin_Output (ds18b20);
	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 0);


	ds18b20_delay_us(ds18b20, 6);

	HAL_GPIO_WritePin(ds18b20->Port, ds18b20->Pin, 1);
	Set_Pin_Input (ds18b20);	
	ds18b20_delay_us(ds18b20, 9);

	result = HAL_GPIO_ReadPin(ds18b20->Port, ds18b20->Pin);

	ds18b20_delay_us(ds18b20, 55);
	return result;
}
//ham ghi 1 byte vao DS18B20 (gui bit thap nhat truoc)
static void ds18b20_write_byte(ds18b20_name* ds18b20, unsigned char byte){
	unsigned char i = 8;
	while(i--){
		ds18b20_write_bit(ds18b20, byte & 0x01);		//gui bit co trong so thap nhat (b=1010 1111 & 0000 0001 -> 0000 0001)
		byte >>= 1;												//byte >>=1 -> 0101 0111
	}
}
//ham doc 1 byte nhan ve tu DS18B20
static unsigned char ds18b20_read_byte(ds18b20_name* ds18b20){
	unsigned char i = 8, result = 0;
	while(i--){
		result >>= 1;
		result |= ds18b20_read_bit(ds18b20)<<7;			//0000 0001 << 7 = 1000 0000
	}
	return result;
}

float temperature(ds18b20_name* ds18b20){
	uint8_t tempL, tempH;
	float temp = 0;
	int t = 1000;
	int check = 1;
	//Ket noi den Slave, bat dau qua trinh chuyen doi nhiet do
	while(t--)
	{
		check = ds18b20_reset(ds18b20);
		if(check == 0)
			break;
	}
	if(check == 1)
	{
		return 0;
	}
	ds18b20_write_byte(ds18b20, DS18B20_SKIP_ROM);
	ds18b20_write_byte(ds18b20, DS18B20_CONVERT_T);
	ds18b20_delay_ms(ds18b20, 94);   //che do 9 bit mat 93.75ms de chuyen doi

	//Ket noi den Slave, bat dau doc du lieu nhiet do
	while(ds18b20_reset(ds18b20));
	ds18b20_write_byte(ds18b20, DS18B20_SKIP_ROM);
	ds18b20_write_byte(ds18b20, DS18B20_READ_SCRATCH_PAD);

	tempL = ds18b20_read_byte(ds18b20);
	tempH = ds18b20_read_byte(ds18b20);

	temp = ((tempH << 8)+ tempL)*1.0f/16; //lay nhiet do tu temp_Register
	return temp;
}

void ds18b20_init(ds18b20_name* ds18b20, TIM_HandleTypeDef* Timer, GPIO_TypeDef* DS_PORT, uint16_t DS_Pin){
	ds18b20->Port = DS_PORT;
	ds18b20->Pin = DS_Pin;
	ds18b20->Timer = Timer;
	ds18b20_delay_init(ds18b20);
}




