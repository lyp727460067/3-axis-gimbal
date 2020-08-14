#ifndef _FIFO_H
#define _FIFO_H
#include "stm32f30x.h"

typedef struct
{
	uint16_t lenth;
	uint8_t fullflag; 
	uint8_t DATAlenth;
	u8* data;
}FIFO_DATA_Str;
typedef struct
{
	uint8_t	fullwriterflag;	
	uint8_t direction; 
	
	uint16_t front;
	uint16_t rear;
	
	uint16_t lenth;
	uint16_t Datath;	
	u8 * data;
	u8 IsWrite;	
}FIFO_Type;

typedef struct
{	
	uint16_t 	Datath;
	u8* 	data;
}FIFO_DATA_SIZE_Str;

uint8_t FIFO_POP_size(FIFO_Type* fifo,FIFO_DATA_SIZE_Str* data);
uint8_t FIFO_PUSH_size(FIFO_Type* fifo,FIFO_DATA_SIZE_Str* data);
void fifo_init_size(FIFO_Type* fifo,FIFO_DATA_Str*datastr);
uint8_t FIFO_Get_states_size(FIFO_Type* fifo);

uint8_t FIFO_POP(FIFO_Type* fifo,u8* data);
uint8_t FIFO_PUSH(FIFO_Type* fifo,u8* data);
void fifo_init(FIFO_Type* fifo,FIFO_DATA_Str*datastr);
uint8_t get_fifo_states(FIFO_Type* fifo);

#define   ENABLEIRQ()		__enable_irq()
#define   DISABLEIRQ()		__disable_irq()

#endif



