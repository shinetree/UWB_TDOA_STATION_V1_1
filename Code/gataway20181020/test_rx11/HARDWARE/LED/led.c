#include "led.h"



void GPIO_Configuration(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;					 //定义GPIO结构体
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能PC端口时钟
	
	// Enable GPIO used for LEDs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}
 
void led_off (led_t led)
{
	switch (led)
	{
	case LED_PC6:
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		break;
	case LED_PC7:
		GPIO_ResetBits(GPIOB, GPIO_Pin_7);
		break;
	case LED_PC8:
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
		break;
	case LED_PC9:
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
		break;
	case LED_ALL:
		GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

void led_on (led_t led)
{
	switch (led)
	{
	case LED_PC6:
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		break;
	case LED_PC7:
		GPIO_SetBits(GPIOB, GPIO_Pin_7);
		break;
	case LED_PC8:
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
		break;
	case LED_PC9:
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
		break;
	case LED_ALL:
		GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}
