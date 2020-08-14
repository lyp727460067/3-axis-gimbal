#ifndef __LED_H
#define __LED_H

#include "stm32f30x.h"

#define LED0    0X01
#define LED1    0X02
void LED_Init(void);//≥ı ºªØ	
void LED_Lignt(u8 ID,u8 Sates);
void Led_Flash(void);
void set_led_state(uint8_t state);
#endif
