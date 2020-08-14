#include "CORDIC_math.h"
#include <math.h>
const int angle[] = {11520, 6801, 3593, 1824, 916, 458, 229, 115, 57, 29, 14, 7, 4, 2, 1};


unsigned int division(unsigned int x,unsigned int y)
{
unsigned	int index ;
unsigned	int value=0;
unsigned	int temp= 0;
	for(index = 1;index<32;index++)
		if(x>>(32-index))break;
	index--;
	for(;index<32;index++){
		temp<<=1;
		temp |=(x<<index)>>(31);
		value<<=1;
		if(temp>=y){
			value+=1;
			temp -=y;
		}
	}
	return value;
}
//x<y
unsigned int divisiondec(unsigned int x,unsigned int y)
{
unsigned	int index ;
unsigned	int value=0;
unsigned	int temp= 0;
	x>>=1;
	y>>=1;
	for(index = 0;index<32;index++){
		x<<=1;
		value<<=1;
		if(temp>=y){
			value+=1;
			temp -=y;
		}
	}
	return value;
}
float CORDIC_atan2(float y1, float x1)
{
    int i = 0;
    int x_new, y_new;
    int angleSum = 0;
	char Fourquadrant = 0;
	int  x,y;
	float returnangle;
	//float x2,y2;
	//arm_abs_f32(&x1,&x2,1);
	//arm_abs_f32(&y1,&y2,1);
	if(x1>0.0f && y1>0.f){
		Fourquadrant = 1;
	}else if(x1<0.0f && y1>0.f){
		x1 = -x1;
		Fourquadrant = 2;
	}else  if(x1<0.0f && y1<0.f){
		x1 = -x1;
		y1 = -y1;
		Fourquadrant = 3;
	}else if(x1>0.0f && y1<0.f){
		y1 = -y1;
		Fourquadrant = 4;
	}else if(fabs(x1)<=0.0001f && y1>0.0f){
		return	90;
	}else if(fabs(x1)<=0.0001f && y1<0.0f){ 
		return	-90;
	}else if(fabs(y1)<=0.0001f && x1<0.0f){ 
		return	-180;
	}else if(fabs(y1)<=0.0001f && x1>0.0f){ 
		return	0;
	}
	  x = (int)(x1*10000);
	  y = (int)(y1*10000);
    for(i = 0; i < 15; i++){
        if(y > 0) {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            x = x_new;
            y = y_new;
            angleSum += angle[i];
		}else{
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            x = x_new;
            y = y_new;
            angleSum -= angle[i];
        }
    }
	switch(Fourquadrant){
		case 1:
			//return returnangle;
			break;
		case 2:
			  angleSum = 46080-angleSum;
			  break;
		case 3:
			 angleSum -=46080;
			  break;
		case 4:
			 angleSum =  -angleSum ;
			 break;
	}
	 returnangle = angleSum /256.0f;
	return  returnangle;
}
float globesin = 0.0f;
float globecos = 0.0f;
void CORDIC_trigfunc(float theta)
{
    int i = 0;
    int x_new, y_new;

    int angleSum = 0;
	int  x = 607252;
	int  y = 0;
	angleSum = 0;
	if(theta<=0.0001f &&theta>=-0.0001f){
		globesin = 0.0f;
		globecos = 1.0f;
		return ;
	}else if(theta==90.0f){
		globesin = 1.0f;
		globecos = 0.0f	;
		return ;
	}
    theta *= 256;
    for(i = 0; i < 15; i++){
        if(angleSum > theta) {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            x = x_new;
            y = y_new;
            angleSum -= angle[i];
		}else{
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            x = x_new;
            y = y_new;
            angleSum += angle[i];
        }
    }
	if(y_new>=1000000)y_new = 1000000;
	if(x_new>=1000000)x_new = 1000000;

	if(y_new<=-1000000)y_new = -1000000;
	if(x_new<=-1000000)x_new = -1000000;
	globesin = y_new /1000000.0f; 
	globecos = x_new /1000000.0f; 
}

float  CORDIC_sin(float theta)
{
	CORDIC_trigfunc(theta);
	return globesin;
}
float  CORDIC_cos(float theta)
{
	CORDIC_trigfunc(theta);
	return globecos;
}
float invSqrt(float x);

float CORDIC_asin(float value)
{

    int i = 0;
    int x_new, y_new;
    int angleSum = 0;
	int Fourquadrant = 0;
	
	int  x,y;
	float returnangle;
	if(value>=1.0f){
		return 90.0f;
	}else if(value<=-1.0f){
		return -90.0f;
	}
	if(value<=-0.0f){
		Fourquadrant = 1 ;
		value = -value;
	}
	  x = (int)(sqrtf(1-value*value)*100000);
	  y = (int)(value*100000);
	angleSum = 0;
    for(i = 0; i < 15; i++){
        if(y > 0) {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            x = x_new;
            y = y_new;
            angleSum += angle[i];
		}else{
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            x = x_new;
            y = y_new;
            angleSum -= angle[i];
        }
    }

	
	if(Fourquadrant){
		angleSum = -angleSum;
	} 
	 returnangle = angleSum /256.0f;
	return returnangle;
}

float CORDIC_acos(float value)
{
    int i = 0;
    int x_new, y_new;
    int angleSum = 0;
	int Fourquadrant = 0;
	int  x,y;
	float returnangle;
	if(value>=1.0f){
		return 0.0f;
	}else if(value<=-1.0f){
		return 180.0f;
	}
	if(value<=0.0f){
		Fourquadrant = 1 ;
		value = -value;
	}
	  x =  (int)(value*1000000 );
	  y = (int)(sqrtf(1-value*value)*1000000);
	angleSum = 0;
    for(i = 0; i < 15; i++){
        if(y > 0) {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            x = x_new;
            y = y_new;
            angleSum += angle[i];
		}else{
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            x = x_new;
            y = y_new;
            angleSum -= angle[i];
        }
    }
	if(Fourquadrant){
		angleSum = 46080-angleSum;
	} 
	returnangle = angleSum /256.0f;
	return returnangle;
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

int instability_fix = 0; 
float invSqrt(float x) {
	if (instability_fix == 0)
	{
		/* original code */
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}
	else if (instability_fix == 1)
	{
		/* close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
		unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
		float tmp = *(float*)&i;
		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	}
	else
	{
		/* optimal but expensive method: */
		return 1.0f / sqrtf(x);
	}
}