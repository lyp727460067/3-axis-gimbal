#include "svpwm.h"
#include "pwm.h"
#include "arm_math.h"
#include "math.h"
#include "CORDIC_math.h"
#include "IObit_remap.h"


#define		MAX_PWM   		PWM_PERIOD
#define		PWM_NORMAL   		MAX_PWM
float		TS			 =	 (MAX_PWM>>1);

float		Udc 		=1.f;

#define  	 longtime 		((TS-T1-T2)*0.25)// //((2*TS - T1)-T2)*1/3
#define  	 midtime   		((TS+T1-T2)*0.25)// ((2*TS  -T2)*2+T1)/6
#define  	 shaorttime 	((TS+T1+T2)*0.25)///((6*TS - T1)-T2)/6
extern void Cut_OFFpwm();
extern void Cut_Onpwm();
void Breakpwm(void)
{
	Channel1Pulse = PWM_PERIOD-1;
	Channel2Pulse = PWM_PERIOD-1;
	Channel3Pulse = PWM_PERIOD-1;
}

#define LimitT()	{\
					if(u1+u2>TS){\
						u1 = u1/(u1+u2)*(TS-10);\
						u2 = u2/(u1+u2)*(TS-10);}\
					}	
#define sqrt3div4   0.4330127018f
#define sqrt3div8	0.2165063509f				
void SVPWM(vector  Uref)
{
	u8 A,B,C;
	float temp,temp1;
	float u1,u2,u3;

	u1 =    Uref.y*sqrt3div4;//sqrt3/4
	temp =  Uref.x*0.375f;//1.5/4
	temp1 = Uref.y*sqrt3div8;//sqrt3div2/4

	u2 = temp - temp1;
	u3 = -temp - temp1;	
		
	if(u1>0)A =1;
	else A =0,u1= -u1;
	if(u2>0)B =1;
	else B =0,u2= -u2;
	if(u3>0)C =1;
	else C =0,u3= -u3;	
   // LimitT();
	switch(4*C+2*B+A){
		case 3:			
			Channel1Pulse = (uint16_t)((TS-u2-u1));//sector 1;
			Channel2Pulse =	(uint16_t)((TS+u2-u1));		
			Channel3Pulse = (uint16_t)((TS+u2+u1));
			break;
		case 1:
			Channel1Pulse = (uint16_t)(TS+u2-u3);//sector 2;
			Channel2Pulse = (uint16_t)(TS-u2-u3);	
			Channel3Pulse = (uint16_t)(TS+u2+u3);	
			break;
		case 5:
			Channel1Pulse = (uint16_t)(TS+u1+u3); //sector 3;
			Channel2Pulse = (uint16_t)(TS-u1-u3);	
			Channel3Pulse = (uint16_t)(TS+u1-u3);	
			break;
		case 4:
			Channel1Pulse = (uint16_t)(TS+u1+u2); //sector 4;
			Channel2Pulse = (uint16_t)(TS+u1-u2);	
			Channel3Pulse = (uint16_t)(TS-u1-u2); 	
			break;
		case 6:
			Channel1Pulse =	(uint16_t)(TS+u3-u2);//sector 5;
			Channel2Pulse =	(uint16_t)(TS+u3+u2);
			Channel3Pulse = (uint16_t)(TS-u3-u2);		
		break ;
		case 2:
			Channel1Pulse = (uint16_t)(TS-u3-u1);//sector 6;
			Channel2Pulse = (uint16_t)(TS+u3+u1);	
			Channel3Pulse = (uint16_t)(TS+u3-u1);	
		break ;	
		default:break ;	
	}	
	Chang_PWMPulse();
}
//227023363.44966788461722944687386
#define  sqrt3q14	(q31_t)28378
#define  threeq14	(q31_t)24578
#define  shift_q     (u8)19
q31_t		TSq			 =	PWM_PERIOD<<(shift_q-1); //(PWM_PERIOD*524288*sqrt3); //2^18*PWM_PERIOD   524288*

void SVPWMq(vector_Q  Uref)
{
	q31_t u1,u2,u3,product1,product2;

	u8 A,B,C;
	u1  =	Uref.y*sqrt3q14;
	product1 = (Uref.x*threeq14);
	product2 = u1>>1;
	u2 = __QSUB(product1, product2);
	u3 = __QSUB(-product1, product2);
//	u2 = product1-product2;
//	u3 = -product1-product2;
	
	if(u1>0)A =1;
	else A =0,u1= -u1;
	if(u2>0)B =1;
	else B =0,u2= -u2;
	if(u3>0)C =1;
	else C =0,u3= -u3;		
	switch(4*C+2*B+A){
		case 3:
			Channel1Pulse = ((TSq-u2-u1))>>shift_q;//sector 1;
			Channel2Pulse =	((TSq+u2-u1))>>shift_q;		
			Channel3Pulse = ((TSq+u2+u1))>>shift_q;
			break;
		case 1:
			Channel1Pulse = (TSq+u2-u3)>>shift_q;//sector 2;
			Channel2Pulse = (TSq-u2-u3)>>shift_q;	
			Channel3Pulse = (TSq+u2+u3)>>shift_q;	
			break;
		case 5:
			Channel1Pulse = (TSq+u1+u3)>>shift_q; //sector 3;
			Channel2Pulse = (TSq-u1-u3)>>shift_q;	
			Channel3Pulse = (TSq+u1-u3)>>shift_q;	
			break;
		case 4:
			Channel1Pulse = (TSq+u1+u2)>>shift_q; //sector 4;
			Channel2Pulse = (TSq+u1-u2)>>shift_q;	
			Channel3Pulse = (TSq-u1-u2)>>shift_q; 	
			break;
		case 6:
			Channel1Pulse =	(TSq+u3-u2)>>shift_q;//sector 5;
			Channel2Pulse =	(TSq+u3+u2)>>shift_q;
			Channel3Pulse = (TSq-u3-u2)>>shift_q;		
		break ;
		case 2:
			Channel1Pulse = (TSq-u3-u1)>>shift_q;//sector 6;
			Channel2Pulse = (TSq+u3+u1)>>shift_q;	
			Channel3Pulse = (TSq+u3-u1)>>shift_q;	
		break ;	
		default:break ;	
		}
		Chang_PWMPulse();
}


