#ifndef DATACALIBRATE_H
#define DATACALIBRATE_H
#include "stm32f30x.h"
typedef struct{
	float  ax;
	float  ay;
	float  az;
	float  gx;
	float  gy;
	float  gz;
	float yaw;
	float pitch;
	float roll;
	u8 dataupdate;
}Senseordata_Type;

#ifdef  DEFIN_CALIB_EXTERN
	#define  CALIB_EXTERN
#else 
	#define  CALIB_EXTERN extern 
#endif
#define  MadgwickAHRS 
CALIB_EXTERN Senseordata_Type   Sensordata;
char sensor_update(void);
void initcalibrate(void);
CALIB_EXTERN  float deltat ;

/*****/

void set_gra_clic(uint8_t idex);
#endif

