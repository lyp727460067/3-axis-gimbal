#include "filter.h"
//Butterworth filter


// 一阶电容低通   y = y+a*(x-y)  f = a/(2*pi*t)-->a = f*(2*pi*t)

/*
(1)
https://wenku.baidu.com/view/f36a611c7e21af45b307a8a4.html
low pass
s = 1/C1 *  (1-Z^-1)/(1+Z^-1)
C1 = Rc*tan(Wc/2);
Rc = 1;
Wc = 2*pi*fc/fs;

fs  采样频率
fc  截至频率  -3db
*/
/*
二阶butterworth 
H(s) = 1/(1+1.41421356*s+s^2)

*/
/*
把(1)带入一下公式  求解  
https://en.wikipedia.org/wiki/Butterworth_filter
To four decimal places, they are Factors of Polynomial
 
1	{(s+1)} 
2	{(s^{2}+1.4142s+1)} 
3	{(s+1)(s^{2}+s+1)} 
4	{(s^{2}+0.7654s+1)(s^{2}+1.8478s+1)} 
5	{(s+1)(s^{2}+0.6180s+1)(s^{2}+1.6180s+1)} 
6	{(s^{2}+0.5176s+1)(s^{2}+1.4142s+1)(s^{2}+1.9319s+1)} 
7	{(s+1)(s^{2}+0.4450s+1)(s^{2}+1.2470s+1)(s^{2}+1.8019s+1)} 
8	{(s^{2}+0.3902s+1)(s^{2}+1.1111s+1)(s^{2}+1.6629s+1)(s^{2}+1.9616s+1)} 

*/
float IIR1Order(tTwoOrder* IIR,float d)
{
  float sum= (d+IIR->x[0])*IIR->Gain  - IIR->y[0]*IIR-> a[0];
  IIR->x[0] = d;
  IIR->y[0] = sum;
  return sum;
}


float IIR2Order(tTwoOrder* IIR,float x)
{
    for(int i =0 ;i<2;i++){
        IIR->x[i] = IIR->x[i+1];
    }
    IIR->x[2] = x;
    double xSum = IIR->x[2]*IIR->b[0]+IIR->x[1]*IIR->b[1]+IIR->x[0]*IIR->b[2];
    double ysum = - IIR->y[1]*IIR->a[1]- IIR->y[0]*IIR->a[2];
    IIR->y[2]= IIR->Gain*xSum + ysum;
    IIR->y[2] = IIR->y[2];//IIR->a[0];
    for(int i =0 ;i<2;i++){
        IIR->y[i] = IIR->y[i+1];
    } 
    return IIR->y[2];
}
float IIR4Order(tFourOrder* IIR,float x)
{
   float y = IIR2Order(&IIR->FirstOlder,x);
   return  IIR2Order(&IIR->SecondOlder,y);
}

//2HZ  
void IIR4OrderInit(tFourOrder* IIR,float sampleRate,float frequency )
{
//由matlab 计算得
//SOS Matrix:                                                 
//1  2  1  1  -1.9902712416617285  0.99042839752060186        
//1  2  1  1  -1.97689135428712    0.97704745364291579        
//                                                            
//Scale Values:                                               
//0.000039288964718310744                                     
//0.000039024838948922117           
  IIR->FirstOlder.a[0] = 1;
  IIR->FirstOlder.a[1] = -1.9927241098599966 ;
  IIR->FirstOlder.a[2] =  0.99281261642961327   ;

  IIR->FirstOlder.b[0] = 1;
  IIR->FirstOlder.b[1] = 2;
  IIR->FirstOlder.b[2] = 1;
  
  IIR->FirstOlder.Gain = 0.000022126642404234759     ;
  
  IIR->SecondOlder.a[0] = 1;
  IIR->SecondOlder.a[1] =  -1.9826478027009247  ;
  IIR->SecondOlder.a[2] =  0.98273586173273308;

  IIR->SecondOlder.b[0] = 1;
  IIR->SecondOlder.b[1] = 2;
  IIR->SecondOlder.b[2] = 1;
  
  IIR->SecondOlder.Gain = 0.000022014757952111739     ;
}
//输入不能大于 int16
int32_t IIR2Orderfp(tfpTwoOrder* IIR,int32_t x)
{
    int64_t xSum ;
    int64_t ysum ;
    int64_t y2   ;
    for(int i =0 ;i<2;i++){
        IIR->x[i] = IIR->x[i+1];
    }
    IIR->x[2] = x;
    xSum =  IIR->x[2]*IIR->b[0]+IIR->x[1]*IIR->b[1]+IIR->x[0]*IIR->b[2];
    ysum = - IIR->y[1]*IIR->a[1]- IIR->y[1]*4294967296 - IIR->y[0]*IIR->a[2];
    y2 = IIR->Gain*xSum + ysum;
    //y2 = y2/IIR->a[0];
    IIR->y[2] = (int32_t)(y2>>32);
    for(int i =0 ;i<2;i++){
        IIR->y[i] = IIR->y[i+1];
    } 
    return IIR->y[2];
}
int32_t IIR4Orderfp(tfpFourOrder* IIR,int32_t x)
{
   int32_t y = IIR2Orderfp(&IIR->FirstOlder,x);
   return  IIR2Orderfp(&IIR->SecondOlder,y);
}
void IIR4OrderInitfp(tfpFourOrder* IIR,int32_t sampleRate,int32_t frequency )
{

}


 void IIR4OrderInitfpEmAng(tFourOrder* IIR,int32_t sampleRate,int32_t frequency )
{

  IIR->FirstOlder.a[0] = 1;
  IIR->FirstOlder.a[1] = -1.9989682316022659 ;
  IIR->FirstOlder.a[2] =  0.99897004345292961   ;

  IIR->FirstOlder.b[0] = 1;
  IIR->FirstOlder.b[1] = 2;
  IIR->FirstOlder.b[2] = 1;
  
  IIR->FirstOlder.Gain = 0.00000045296266587551775     ;
  
  IIR->SecondOlder.a[0] = 1;
  IIR->SecondOlder.a[1] =  -1.997513464002203  ;
  IIR->SecondOlder.a[2] =  0.99751527453427558;

  IIR->SecondOlder.b[0] = 1;
  IIR->SecondOlder.b[1] = 2;
  IIR->SecondOlder.b[2] = 1;
  
  IIR->SecondOlder.Gain = 0.00000045263301811026753     ;
}

