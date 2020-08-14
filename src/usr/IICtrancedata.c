 #define  DEFIN_IICTRANCEDATA_EXTERN
 #include "IICtrancedata.h"
#include  "datacalibrate.h"
#include "usart.h"
#include <string.h>
#include "AngSensor.h"
#include "usart_data.h"
#include "flash.h"
#include "Control.h"
#include "device.h"


#define DEVICEARR0 0x68
#define DEVICEARR1 0x67
#define DEVICEARR2 0x66


/*
	SendSlavebuffer[0] = PID
	SendSlavebuffer[1] = Reserve		

*/
u8	Slveadrr[DeviceNuber*2] = { DEVICEARR0<<1,(DEVICEARR0<<1)|1,DEVICEARR1<<1,(DEVICEARR1<<1)|1};
void iic_tran_device_init_master(void)
{
	u8 i = 0;
 	for(;i<2*2;i++){
		MIIC_DATA[i].number = i;
		MIIC_DATA[i].adrr =	Slveadrr[i];
		MIIC_DATA[i].lenth =6;
		MIIC_DATA[i].data = (u8*)SendSlavebuffer[i];	
 	}	
   MIIC_DATA[0].lenth =6; 
   MIIC_DATA[1].lenth =4; 
   MIIC_DATA[2].lenth =6;
   MIIC_DATA[3].lenth =7;
}
extern void iic_slave_data_init(uint8_t *sed,uint8_t *rev);
void iic_tran_device_init_slave(void)
{
    
    
 	for(u8 i = 0;i<2;i++){
		MIIC_DATA[i].number = i;
		#ifdef IS_MAIN_CONTROLLOR
			MIIC_DATA[i].adrr =	DEVICEARR1; 
            MIIC_DATA[i].lenth =7;
		#else 
			MIIC_DATA[i].adrr =	DEVICEARR0;
            MIIC_DATA[i].lenth =4;
		#endif
		
		MIIC_DATA[i].data = (u8*)SendSlavebuffer[i];	
 	}
	iic_slave_data_init((u8*)SendSlavebuffer[0],(u8*)SendSlavebuffer[1]);
}

void  iic_tran_init(void)
{
	switch(g_ucDeviceNum){
		case 0:
			iic_tran_device_init_master();	
		break;
		case 1:
		case 2:
			iic_tran_device_init_slave();
		break;
	}	
	//g_ucWaitFinish = 1;
	iic_init();
}


static  uint8_t FactoryClibStates = 0;
void IICSendToCalibreteStates(uint8_t states)
{   
    FactoryClibStates = states;
}
/*
 *@master
 *cabck直接在中断里面读取，数据比较短
*/
#define  DATASTART    1
extern uint16_t Capture1  ;
extern uint16_t Capture2 ;

void iic_rev_cabck(void)
{
	static  uint16_t OldCapture1 = 1500;
	static  uint16_t OldCapture2 = 1500;
	
	
	uint8_t Xor = SendSlavebuffer[1][0];
	uint8_t Xor1 = SendSlavebuffer[3][0];
	for(int i = 1;i<3;i++){
		Xor =  Xor^SendSlavebuffer[1][i];	
	}	
	if(Xor==SendSlavebuffer[1][3]){
		memcpy((void*)&g_hiRecEMReangle[1],(void*)&SendSlavebuffer[1][DATASTART],2);//0号 
	}
	for(int i = 1;i<6;i++){
		Xor1 = Xor1^SendSlavebuffer[3][i];	
	}
	if(Xor1==SendSlavebuffer[3][6]){
		memcpy((void*)&g_hiRecEMReangle[2],(void*)&SendSlavebuffer[3][DATASTART],2);//2号	
		
		uint16_t temp = ((SendSlavebuffer[3][DATASTART+2])<<4)|(SendSlavebuffer[3][DATASTART+4]&0x0f); 
		OldCapture1  += (temp>>2) - (OldCapture1>>2);
		g_hiReceptionExp[1] = (1500-(OldCapture1&0xfff));
		
		temp = ((SendSlavebuffer[3][DATASTART+3])<<4)|((SendSlavebuffer[3][DATASTART+4]&0xf0)>>4); 
		
		OldCapture2  += (temp>>2) - (OldCapture2>>2);
		g_hiReceptionExp[0] = ((OldCapture2&0xfff)-1500);  
	}
	
	

	
    
      
    
}
void iic_master_trance(void)
{	
	if(g_ucWaitFinish)return ;
	memcpy((void*)&SendSlavebuffer[0][DATASTART],(void*)&g_fSpeedPiderr[1],4);//0号
	memcpy((void*)&SendSlavebuffer[2][DATASTART],(void*)&g_fSpeedPiderr[2],4);//2号	
	SendSlavebuffer[0][0] = FactoryClibStates<<3 |0x02;
	SendSlavebuffer[2][0] = FactoryClibStates<<3 |0x02; 
	
	uint8_t Xor = SendSlavebuffer[0][0];
	uint8_t Xor1 = SendSlavebuffer[2][0];
	for(int i = 1;i<5;i++){
		Xor = Xor^SendSlavebuffer[0][i];
		Xor1 = Xor1^SendSlavebuffer[2][i];
	}
	SendSlavebuffer[0][5] = Xor;
	SendSlavebuffer[2][5] = Xor1;
	
	iic_write_data(&MIIC_DATA[0]);
	iic_write_data(&MIIC_DATA[2]);	 
	iic_write_data(&MIIC_DATA[1]);	  
	iic_write_data(&MIIC_DATA[3]);
	g_ucWaitFinish = 1;
}


/*
 *@salve
 *cabck直接在中断里面读取，数据比较短
*/
void iic_slave_trance(void);
void iic_slave_send_cabck(void)
{  
   iic_slave_trance(); 
//	if(MIIC_DATA[0].dataupdataflag){
//		MIIC_DATA[0].dataupdataflag = 0;
//		memcpy((void*)(void*)&SendSlavebuffer[2],(void*)&SendSlavebuffer[0],MIIC_DATA[0].lenth);	
//	}

	
}
extern void FactorySubMainSetCalibreStates(uint8_t states);
void iic_slave_rev_cabck(void)
{
    float fSpeedPiderr = 0;
	uint8_t Xor = SendSlavebuffer[1][0];
	for(int i = 1;i<5;i++){
		Xor = Xor^SendSlavebuffer[1][i];
	}
	if(Xor==SendSlavebuffer[1][5]){
		memcpy((void*)&fSpeedPiderr,(void*)&SendSlavebuffer[1][DATASTART],4);
		g_fSpeedPiderr[g_ucDeviceNum] = fSpeedPiderr;
		uint8_t FactoryClibre =SendSlavebuffer[1][0];
		FactorySubMainSetCalibreStates(FactoryClibre>>3);
	}	


}



void iic_slave_trance(void)
{	
    SendSlavebuffer[0][0] = 0x02;
    memcpy((void*)&SendSlavebuffer[0][DATASTART],(void*)&g_hiEMReAngle,2);	
	uint8_t Xor = SendSlavebuffer[0][0];
#ifdef IS_MAIN_CONTROLLOR
    SendSlavebuffer[0][DATASTART+2]  = (Capture1&0xff0)>>4;
    SendSlavebuffer[0][DATASTART+3]  = (Capture2&0xff0)>>4;
    SendSlavebuffer[0][DATASTART+4]  = ((Capture2&0x00f)<<4)| (Capture1&0x00f);
	for(int i = 1;i<6;i++){
		Xor = Xor^SendSlavebuffer[0][i];
	}
	SendSlavebuffer[0][DATASTART+5]  = Xor;
#else
	for(int i = 1;i<3;i++){
		Xor = Xor^SendSlavebuffer[0][i];
	}
	SendSlavebuffer[0][3]  = Xor;
#endif;
   // MIIC_DATA[0].dataupdataflag =1;		
	
}
void iic_tran_data(void)
{

	if(g_ucDeviceNum==0){
        iic_master_trance();
	}else {
		//iic_slave_trance();
	}
}
void iic_tran_write(void)
{

	if(g_ucDeviceNum==0){
        iic_master_trance();
	}else {
		//iic_slave_trance();
	}
}
void iic_tran_read(void)
{

	if(g_ucDeviceNum==0){
        iic_write_data(&MIIC_DATA[1]);	  
        iic_write_data(&MIIC_DATA[3]);
        IIC_Sche();
	}
}









