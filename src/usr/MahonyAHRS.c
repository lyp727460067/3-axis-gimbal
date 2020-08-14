//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>
#include  "datacalibrate.h"
#include "time.h"
#include "CORDIC_math.h"
#define Kp 0.5f                        
#define Ki   0.003f  
#define twoKpDef	(2.0f * Kp)	// 2 * proportional gain
#define twoKiDef	(2.0f * Ki)	// 2 * integral gain
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;										// 2 * integral gain (Ki)
#ifndef MadgwickAHRS
	
//---------------------------------------------------------------------------------------------------
// Definitions



//---------------------------------------------------------------------------------------------------
// Variable definitions


 float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
u16 Flag = 0;
#define   LIMT    200.f
float  limint(float value)
{
    if(value>=LIMT){
       value = LIMT;
    }
    if(value<=  -LIMT){
        value  = -LIMT;
    }
    return value;

}
float halfex, halfey, halfez;
void GetGryErr(float* gryerr)
{
    //gryerr[0] = GryErr0*57.3f;
    gryerr[0] =   -halfex*57.3f;
    gryerr[1] =    halfez*57.3f;
    gryerr[2] =    -halfey*57.3f;
}
void GetGryDynOffset(float* gryerr)
{

    gryerr[0] =  -integralFBx*57.3f;
    gryerr[1] =   integralFBz*57.3f;
    gryerr[2] =   -integralFBy*57.3f;
}

void  SetGryInterZero(uint8_t index,float trust)
{

    if(index == 0)integralFBx = integralFBx*trust;
    if(index == 1)integralFBz = integralFBz*trust;
    if(index == 2)integralFBy = integralFBy*trust;
    
}
float FixGry[3];
void GetFixGyr(float * gyr)
{
    for(int i = 0;i<3;i++){
        gyr[i] = FixGry[i]*57.3f;
    }
  
}
//惯性导航 3.63旋转矩阵为  惯性系到BODY,从body到惯性系 为装置
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(Senseordata_Type*Sensondata) 
{
float gx,  gy,  gz,  ax,  ay,  az ;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	
	float qa, qb, qc;
	ax =Sensondata->ax ;     
	ay =Sensondata->ay ;     
	az =Sensondata->az ;	
	gx =Sensondata->gx ;     
	gy =Sensondata->gy ;     
	gz =Sensondata->gz ;     
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

    
    
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

        
        
		// Compute and apply integral feedback if enabled
		if(!(twoKi == 0.0f )) {
			integralFBx += twoKi * halfex *deltat;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * deltat;
			integralFBz += twoKi * halfez * deltat;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}	

  FixGry[0] = gx;
  FixGry[1] = gy;
  FixGry[2] = gz;    
	// Integrate rate of change of quaternion
	gx *= (0.5f * deltat);		// pre-multiply common factors
	gy *= (0.5f * deltat);
	gz *= (0.5f * deltat);
  

  
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;  
	q2 *= recipNorm;
	q3 *= recipNorm;
}

 void MahonyAHRSupdate(Senseordata_Type*Sensondata) //背东地
{
//float exInt = 0, eyInt = 0, ezInt = 0; 	
float gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz;
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy,hz, bx, bz, by;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;


	ax =Sensondata->ax ;     
	ay =Sensondata->ay ;     
	az =Sensondata->az ;	
	gx =Sensondata->gx ;     
	gy =Sensondata->gy ;     
	gz =Sensondata->gz ;            
	mx =0;//Sensondata->mx;  
	my =0;//Sensondata->my ;  	
	mz =0;//Sensondata->mz ; 

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(Sensondata);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   
		
		hx = (ay * mz - az * my) ;
		hy = (az * mx - ax * mz) ;
		hz = (ax * my - ay * mx) ;
		recipNorm = invSqrt(hx * hx + hy * hy + hz * hz);
		ax = hx*recipNorm;
		ay = hy*recipNorm;
		az = hz*recipNorm;   
//        // Reference direction of Earth's magnetic field
//        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
//        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
//        bx = sqrt(hx * hx + hy * hy);
//        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
//		halfvx = q1q3 - q0q2;
//		halfvy = q0q1 + q2q3;
//		halfvz = q0q0 - 0.5f + q3q3;
		
		halfvx = q1q2 - q0q3;            
		halfvy = 0.5f-q1*q1-q3*q3;
		halfvz = q2q3- q0q1  ;

       // halfwx = by * (q1q2 + q0q3) + bz * (q1q3 - q0q2);
        //halfwy = by * (0.5f - q1q1 - q3q3) + bz * (q0q1 + q2q3);
       // halfwz = by * (q2q3 - q0q1) + bz * (0.5f - q1q1 - q2q2); 
		
//        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
		
		//wx = 2*bx*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
		//wy = 2*bx*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
		//wz = 2*bx*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy);// + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz);// + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) ;//+ (mx * halfwy - my * halfwx);
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
		//if(halfex !=0.0f  &&   halfey !=0.0f && halfez!=0.0f ){
			integralFBx += twoKi * halfex * (deltat);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (deltat);
			integralFBz += twoKi * halfez * (deltat);   
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;

		}
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;			
		// Apply proportional feedback

	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * deltat);		// pre-multiply common factors
	gy *= (0.5f * deltat);
	gz *= (0.5f * deltat);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
    
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


#endif
//====================================================================================================
// END OF CODE
//====================================================================================================
