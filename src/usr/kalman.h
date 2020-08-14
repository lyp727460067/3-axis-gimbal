
#ifndef _KALMAN_H
#define	_KALMAN_H
typedef struct
{
	double fQ;
	double fR;
	double fXLast;
	double fPLast;	
}tKalman;



	
void KalmanInit(tKalman* Kalman,float q,float r);
float  KalmanFilter(tKalman* Kalman,float mX,float U);

#endif


