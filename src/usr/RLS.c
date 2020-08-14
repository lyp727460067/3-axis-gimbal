#include  "RLS.h"



// model   y = ax +b
void RLSInit(tLRS*LRS,float* X,uint8_t f)
{
    if(f){
        LRS->P1[0][0] = 1.f;
        LRS->P1[0][1] = 0.1f;
        LRS->P1[1][0] = 0.1f;
        LRS->P1[1][1] = 0.1f;
    }
    LRS->X1[0] =X[0];
    LRS->X1[1] =X[1];
}


void LRS(tLRS*LRS,float  t,float y,float R)
{
    
    float X1[2] = {LRS->X1[0],LRS->X1[1]};
    float P1[2][2] = {LRS->P1[0][0],LRS->P1[0][1],LRS->P1[1][0],LRS->P1[1][1]};
    // K2 = P1*H2'/(H2*P1*H2'+R2);
    float K2[2];
    float K = P1[0][0]+P1[1][0]*t +P1[0][1]*t+P1[1][1]*t*t +R;
    K2[0] = P1[0][0]+P1[0][1]*t;
    K2[0] /=K;
    
    K2[1] = P1[1][0]+P1[1][1]*t;
    K2[1] /=K;
    
    //X2 = X1 +K2*(Y2-H2*X1); 
    float ytemp = y-X1[0]-X1[1]*t;
    float X2[2];
    X2[0] = X1[0]+K2[0]*ytemp ;
    X2[1] = X1[1]+K2[1]*ytemp ;
    //P2 = (I-K2*H2)*P1*(I-K2*H2)'+K2*R2*K2';
    float I_K2H1[2][2] = {1.f-K2[0],-K2[0]*t,-K2[1],1.f-K2[1]*t}; 
    float I_K2H1_[2][2] = {I_K2H1[0][0],I_K2H1[1][0],I_K2H1[0][1],I_K2H1[1][1]};
    
    float I_K2H1xP1[2][2] = {   I_K2H1[0][0]*P1[0][0]+I_K2H1[0][1]*P1[1][0],
                                I_K2H1[0][0]*P1[0][1]+I_K2H1[0][1]*P1[1][1],
                                I_K2H1[1][0]*P1[0][0]+I_K2H1[1][1]*P1[1][0],
                                I_K2H1[1][0]*P1[0][1]+I_K2H1[1][1]*P1[1][1] };
    
    float I_K2H1xP1xI_K2H1_[2][2] = { I_K2H1xP1[0][0]*I_K2H1_[0][0]+I_K2H1xP1[0][1]*I_K2H1_[1][0],
                                      I_K2H1xP1[0][0]*I_K2H1_[0][1]+I_K2H1xP1[0][1]*I_K2H1_[1][1],
                                      I_K2H1xP1[1][0]*I_K2H1_[0][0]+I_K2H1xP1[1][1]*I_K2H1_[1][0],
                                      I_K2H1xP1[1][0]*I_K2H1_[0][1]+I_K2H1xP1[1][1]*I_K2H1_[1][1]};  
     
     
    float K200  = K2[0]*K2[0]*R;
    float K210  = K2[1]*K2[0]*R;
    float K211  = K2[1]*K2[1]*R;
    
    float K2R2K2[2][2] = {K200,K210,K210,K211};
    for(int i = 0;i<2;i++){
        for(int j = 0;j<2;j++){
            LRS->P1[i][j] = I_K2H1xP1xI_K2H1_[i][j]+K2R2K2[i][j];
        }
    }
    LRS->X1[0] = X2[0];
    LRS->X1[1] = X2[1];  
}
//model  Xn = Xn-1+ Dy
void KalManEest()
{
    

}

