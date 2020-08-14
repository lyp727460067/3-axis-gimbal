#include "IObit_remap.h"
#include "led.h" 
#include "time.h"

 PIN_TYPE  LED_pin = {GPIOB,GPIO_Pin_2};
static u8 LEDState ;
static  uint16_t FlashTime;
void LED_Init(void)
{    	 
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_pin.Pin ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    LEDState  = 0;
    FlashTime =0;
	//GPIO_SetBits(GPIOB,GPIO_Pin_1);	
}
extern void Set_gpio(PIN_TYPE gpio,uint8_t states);

void LED_Lignt(u8 ID,u8 Sates)
{
	if(Sates){
		if(ID&0x01){
			Set_gpio(LED_pin,1);
		}
	}else {
		if(ID&0x01){
			Set_gpio(LED_pin,0);
		}
	}
}


void set_led_state(uint8_t state)
{   
    LEDState  =state;
}
void Led_Flash(void)
{
    FlashTime++;
	switch(LEDState)
	{
    case 0:
		if(FlashTime<=500){
			LED_Lignt(0X01,1);
		}else if(FlashTime<=1000){
			LED_Lignt(0X01,0);
		}else {
			FlashTime = 0;
		}			
		break;
    case 1: 
 		if(FlashTime<=100){
			LED_Lignt(0X01,1);
		}else if(FlashTime<=200){
			LED_Lignt(0X01,0);
		}else {
			FlashTime = 0;
		}
        break;
    case 2: 
 		if(FlashTime<=300){
			LED_Lignt(0X01,1);
		}else if(FlashTime<=400){
			LED_Lignt(0X01,0);
		}else {
			FlashTime = 0;
		}
        break;
    case 3: 
 		if(FlashTime<=50){
			LED_Lignt(0X01,1);
		}else if(FlashTime<=100){
			LED_Lignt(0X01,0);
		}else {
			FlashTime = 0;
		}
        break;       
  
    case 4: 
 		if(FlashTime<=500){
			LED_Lignt(0X01,1);
		}else if(FlashTime<=2000){
			LED_Lignt(0X01,0);
		}else {
			FlashTime = 0;
		}
        break;  
        
    case 5: 
        LED_Lignt(0X01,1);
    break;
	}
}


