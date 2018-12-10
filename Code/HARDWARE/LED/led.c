#include "led.h"


#define LED_CA 1
void GPIO_Configuration(void)
{
    static uint8_t io_status;
	GPIO_InitTypeDef  GPIO_InitStructure;					 //定义GPIO结构体
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE | RCC_APB2Periph_AFIO);	//使能PC端口时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	// Enable GPIO used for LEDs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//    led_on(LED_ALL);
    GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_3);
    io_status = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3);
}
 
void led_off (led_t led)
{
	switch (led)
	{
	case LED_1:
        if(LED_CA)
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_4); 
        }
        else
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        }
		break;
	case LED_2:
        if(LED_CA)
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_3); 
        }
        else
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        }
		break;
	case LED_ALL:
        if(LED_CA)
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_3);
        }
        else
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_3);
        }
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
	case LED_1:
        if(!LED_CA)
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_4); 
        }
        else
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        }
		break;
	case LED_2:
        if(!LED_CA)
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_3); 
        }
        else
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        }
		break;
	case LED_ALL:
        if(!LED_CA)
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_3);
        }
        else
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_3);
        }
    
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}
