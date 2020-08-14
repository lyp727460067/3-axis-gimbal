#include  "imu.h"



volatile float twoKp = 0.01;											// 2 * proportional gain (Kp)
volatile float twoKi = 0.001;	


volatile float Mgdgki = 0.005f;
volatile float beta = 0.003;

//
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f,	integralFBw = 0.0f;// integral error terms scaled by Ki
float halfex, halfey, halfez,halfew;
float  g_recipNorm = 0.0f;
//



void MadgwickAHRSupdateIMU(Senseordata_Type*Sensondata) {
	float gx,  gy,  gz,  ax,  ay,  az;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	ax =Sensondata->ax ;     
	ay =Sensondata->ay ;     
	az =Sensondata->az ;	
	gx =Sensondata->gx ;     
	gy =Sensondata->gy ;     
	gz =Sensondata->gz ;     
	
	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
//	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
//	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
//	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm1;
		ay *= recipNorm1;
		az *= recipNorm1;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

        
        
        
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        

     

        g_recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= g_recipNorm;
		s1 *= g_recipNorm;
		s2 *= g_recipNorm;
		s3 *= g_recipNorm;  
        
        halfew = 2* (q0*s0+q1*s1+q2*s2+q3*s3);
        halfex = 2* (q0*s1-q1*s0-q2*s3+q3*s2);
        halfey = 2* (q0*s2+q1*s3-q2*s0-q3*s1);
        halfez = 2* (q0*s3-q1*s2+q2*s1-q3*s0);            
     
        if(Mgdgki>0){
            integralFBw+=(halfew)*Mgdgki*deltat;
            integralFBx+=(halfex)*Mgdgki*deltat;
            integralFBy+=(halfey)*Mgdgki*deltat;
            integralFBz+=(halfez)*Mgdgki*deltat;   
        }else {
            integralFBw=0;//
            integralFBx=0;//
            integralFBy=0;//
            integralFBz=0;//   
        }
        gx -= integralFBx;
        gy -= integralFBy;
        gz -= integralFBz;
            	// Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
         
        qDot1-= 0.5f*q0*integralFBw;
        qDot2-= 0.5f*q1*integralFBx;
        qDot3-= 0.5f*q2*integralFBy;
        qDot4-= 0.5f*q3*integralFBz;
        
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	float recipNorm= invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm2;
	q1 *= recipNorm2;
	q2 *= recipNorm2;
	q3 *= recipNorm2;
}


void MahonyAHRSupdateIMU(Senseordata_Type*Sensondata) 
{
    float gx,  gy,  gz,  ax,  ay,  az ;
	float recipNorm;
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
		if(twoKi > 0.0f ) {
			integralFBx += twoKi * halfex *deltat;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * deltat;
			integralFBz += twoKi * halfez * deltat;
            gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else { 
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
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


void GetGryErr(float* gryerr)
{
    //gryerr[0] = GryErr0*57.3f;
    gryerr[0] =    -halfvx*57.3f;
    gryerr[1] =    -halfvz*57.3f;
    gryerr[2] =     halfvy*57.3f;
}
void GetGryDynOffset(float* gryerr)
{

    gryerr[0] =  -integralFBx*57.3f;
    gryerr[1] =  -integralFBz*57.3f;
    gryerr[2] =   integralFBy*57.3f;
}

void  SetGryInterZero(uint8_t index,float trust)
{
    if(index == 0)integralFBx = integralFBx*trust;
    if(index == 1)integralFBz = integralFBz*trust;
    if(index == 2)integralFBy = integralFBy*trust; 
}