#ifndef _COMMONVALUE_H
#define _COMMONVALUE_H
#include "stm32f30x.h"
#include "arm_math.h"
#define   sqrt3			1.732050807f	
#define   sqrt3div2		0.866025403f

typedef  struct
{
	u16 T1;
	u16 T2;	
	u16 T3;		
}DUTY_type;
typedef   struct
{
	float x;
	float y;		
}vector;
typedef   struct
{
	q31_t x;
	q31_t y;		
}vector_Q;
typedef   struct
{
	float A;
	float B;
	float C;	
}ThrCurrent_type;

typedef   struct
{
	q31_t A;
	q31_t B;
	q31_t C;	
}ThrCurrent_type_Q;

typedef   struct
{
	float speed;
	float oldthata;	
	float thata;
	float Ithata;	
}Positon_type;
typedef   struct
{
	q31_t speed;
	q31_t oldthata;	
	q31_t thata;
	q31_t Ithata;	
}Positon_type_Q;

typedef struct 
{
	u16 I_PhaseA;
	u16 I_PhaseB;
}CurrentType;



#endif



