#define	LCD_COMMAND	0
#define LCD_DATA 1
#define LCD5110__LED_GPIO_Port GPIOA
#define LCD5110__LED_Pin GPIO_PIN_10
#define LCD5110__CS_GPIO_Port GPIOB
#define LCD5110__CS_Pin GPIO_PIN_0
#define LCD5110__RST_GPIO_Port GPIOB
#define LCD5110__RST_Pin GPIO_PIN_1
#define LCD5110__DC_GPIO_Port GPIOA
#define LCD5110__DC_Pin GPIO_PIN_7
#define LCD5110__DIN_GPIO_Port GPIOA
#define LCD5110__DIN_Pin GPIO_PIN_6
#define LCD5110__led_GPIO_Port GPIOA
#define LCD5110__led_Pin GPIO_PIN_4
#define LCD5110__SCK_GPIO_Port GPIOA
#define LCD5110__SCK_Pin GPIO_PIN_5

#include "stm32f1xx_hal.h"

typedef struct
{
	TIM_HandleTypeDef* Timer;
}LCD5110_Time;
void LCD5110_init(LCD5110_Time* LCD5110,TIM_HandleTypeDef* Timer);

void LCD5110_write_char(unsigned char c);

void LCD5110_write_char_inv(unsigned char c);

void LCD5110_clear(void);

void LCD5110_set_XY(unsigned char X, unsigned char Y);

void LCD5110_write_string(char *s, int color);

void LCD5110_Write_Dec(unsigned int buffer, int amount_num, int color);

void LCD5110_Led(unsigned char c);


