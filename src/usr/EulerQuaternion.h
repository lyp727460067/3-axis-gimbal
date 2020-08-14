#ifndef  EULERQUATERNION_H
#define  EULERQUATERNION_H
#include "stm32f30x.h"
typedef struct
{
  int16_t hiAcc[3];//ax ay az
  int16_t hiGry[3];//gx gy gz
  float   fAcc[3];//ax ay az
  float   fGry[3];//gx gy gz  
  float   EulAngle[3];
  float   fQua[4];
}EulerType;
#ifdef  DEFIN_EULER_EXTERN
	#define  EULER_EXTERN
#else 
	#define  EULER_EXTERN extern 
#endif
EULER_EXTERN   EulerType  EulerQuter;

#include 	"MadgwickAHRS.h"
#include 	"MahonyAHRS.h"
uint8_t Init_ulerquaternion(void);
uint8_t GetEuler(void);
void SetEulerFixPare(uint8_t index);
#endif

