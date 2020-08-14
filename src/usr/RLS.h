#ifndef RLS_H
#define  RLS_H

#include "stm32f30x.h"
typedef struct 
{
    float P1[2][2];
    float X1[2];   
}tLRS;


void LRS(tLRS*LRS,float  t,float y,float R);
void RLSInit(tLRS*LRS,float* X,uint8_t f);
#endif