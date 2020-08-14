 #ifndef _FILTER_H
 #define _FILTER_H
 #include "stm32f30x.h"

typedef struct 
{
	float lastout;
	float beta;
}LP_TYPE;
typedef struct 
{
  float  x[3];
  double  y[3];
  double a[3];
  double b[3];
  double Gain;
}tTwoOrder;

typedef struct 
{
  tTwoOrder FirstOlder;
  tTwoOrder SecondOlder;
}tFourOrder;

typedef struct 
{
  int32_t  x[3];
  int32_t  y[3];
  int64_t a[3];
  int16_t b[3];
  int32_t Gain;
}tfpTwoOrder;

typedef struct 
{
  tfpTwoOrder FirstOlder;
  tfpTwoOrder SecondOlder;
}tfpFourOrder;



float IIR4Order(tFourOrder* IIR,float x);
void  IIR4OrderInit(tFourOrder* IIR,float sampleRate,float frequency );
void  IIR4OrderInitfpEmAng(tFourOrder* IIR,int32_t sampleRate,int32_t frequency );
 #endif