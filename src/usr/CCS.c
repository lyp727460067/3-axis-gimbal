#include "CCS.h"
#define   CSQ() 

//void Delay_13nu(u32  time)
//{	u32 i = time>>2;
//	for( i=0;i<time;i++);
//			//__NOP;	
//}
#define Delay_13nu(n) 
//²Î¿´Ê±ÐòÍ¼
void CCS_Start()
{
	CCSDATA_OUT();
	CCS_DATA0();
	CCS_SCK0();	
	CCS_CSQ0();Delay_13nu(8);	//>105nu
}
void CCS_Stop()
{
	CCSDATA_OUT();	
	CCS_DATA0();
	CCS_SCK0();	
	CCS_CSQ1();Delay_13nu(8);	//>105nu
}
void Write_16Byte(uint16_t data)
{
	u8 i = 0;
	CCS_SCK1();	
	Delay_13nu(3);//>40
	for(i=0;i<16;i++){
		if(data&0x8000)CCS_DATA1();
		//CCS_DATA((data&0x8000)?1:0);
		data<<=1;
		Delay_13nu(2);//>30
		CCS_SCK0();//>30
		Delay_13nu(3);//>40
  		CCS_DATA0();
		CCS_SCK1();//>40
	}
}
u16 Read_16Byte(void)
{
	u8 i = 0;
	u16 temp = 0;
	CCS_SCK1();
	CCSDATA_IN();	
	Delay_13nu(3);//>40
	CCS_SCK0();//>30
	for(i=0;i<16;i++){
		CCS_SCK0();
		Delay_13nu(2);//>30
		temp<<=1;
		if(READ_DATA())temp++;
		CCS_SCK1();//>40
		Delay_13nu(3);//>40
	}
	return temp;
}
u8 CCSWrite_Data(u8 adrr,u16* data,u8 len)
{
	u16 temp = 0;u8 i =0;
	CCS_Start();
	temp = (adrr>=0x04&&adrr<=0x11)?0X5000:0;
	temp |=	adrr<<4;
	temp |=len;
	Write_16Byte(temp);//COMMAD
	if(len>=1){
		for(i=0;i<len;i++){
			Write_16Byte(data[i]);
		}
	}
	CCSDATA_IN();
	Delay_13nu(10);//>130
	temp = Read_16Byte();
	if(temp&0x6000) temp =1;
	else temp = 0;
	CCS_Stop();
	return 	temp;
}
u8 CCSRead_Data(u8 adrr,u16* data,u8 len)
{
	u16 temp = 0;u8 i =0;
	CCS_Start();
	temp = (adrr>=0x04 && adrr<=0x11)?0XD000:0x8000;
	temp |=	adrr<<4;
	temp |=len;
	Write_16Byte(temp);//COMMAD
	Delay_13nu(10);//>130
	if(len>=1){
		for(i=0;i<len;i++){
			data[i] = Read_16Byte();
		}
	}
//	if(Read_16Byte()&0x6000) temp =1;
//	else temp = 0;
	CCS_Stop();
	return 	1;
}
/*******************************

*******************************/




