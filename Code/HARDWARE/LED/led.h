#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
typedef enum
{
    LED_1,
    LED_2,
    LED_ALL,
    LEDn
} led_t;
void GPIO_Configuration(void);
void led_off (led_t led);
void led_on  (led_t led);

		 				    
#endif
