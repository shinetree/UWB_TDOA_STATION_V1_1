#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
typedef enum
{
    LED_PC6,
    LED_PC7,
    LED_PC8,
    LED_PC9,
    LED_ALL,
    LEDn
} led_t;
void GPIO_Configuration(void);
void led_off (led_t led);
void led_on  (led_t led);

		 				    
#endif
