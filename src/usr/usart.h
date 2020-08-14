#ifndef __USART_H
#define __USART_H
#include "stm32f30x.h"
#include "fifo.h"

#ifdef  DEFIN_USART_EXTERN
	#define  USART_EXTERN
#else 
	#define  USART_EXTERN extern 
#endif



USART_EXTERN  FIFO_Type     USART_TX_FIFO;
USART_EXTERN  FIFO_Type     USART_RX_FIFO;


	
void uart_init(u32 bound);
u8  USART1_send_data(u8* data,u8 lenth);
	
#define 	USART_RX_BUFF_LEN    100
USART_EXTERN  u8  USART_RX_BUFF[USART_RX_BUFF_LEN];
u8 USART1_recive_data(u16* lenth);
#endif


