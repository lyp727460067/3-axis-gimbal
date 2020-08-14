#include "FIFO.h"
#include <string.h>
//#include <iostream>
//using namespace std;
void fifo_init(FIFO_Type* fifo,FIFO_DATA_Str*datastr)
{
	if(datastr->data==NULL)return ;
	fifo->front = 0;
	fifo->rear = 0;	
	fifo->Datath = datastr->DATAlenth;	
	fifo->lenth = datastr->lenth*fifo->Datath;
	fifo->data = datastr->data;
	fifo->fullwriterflag = datastr->fullflag;
}
uint8_t FIFO_PUSH(FIFO_Type* fifo,u8* data)
{
	//if(fifo==NULL)return 0;
	
	if(((fifo->rear+fifo->Datath)%fifo->lenth)==fifo->front){//man ;write new data to buffer
        return 0;
	}
	memcpy((void*)&fifo->data[fifo->rear],(void*)data,fifo->Datath);	
	fifo->rear = (fifo->rear+fifo->Datath)%fifo->lenth; 
	return 1;
}
uint8_t FIFO_POP(FIFO_Type* fifo,u8* data)
{
	//if(fifo==NULL)return 0;
	if(fifo->front==fifo->rear){//KONG;
		return 0;
	}
	memcpy((void*)data,(void*)&fifo->data[fifo->front],fifo->Datath);	
 	fifo->front = (fifo->front+fifo->Datath)%fifo->lenth;	
	return 1;
}
uint8_t get_fifo_states(FIFO_Type* fifo)
{
    if(fifo->front==fifo->rear){//KONG;
		 return 0;
	}else if(((fifo->rear+fifo->Datath)%fifo->lenth)==fifo->front){
        return 1;//full
    }else  return 2;
}

void fifo_init_size(FIFO_Type* fifo,FIFO_DATA_Str*datastr)
{
	if(datastr->data==NULL)return ;
	fifo->front = 0;
	fifo->rear = 0;	
	fifo->Datath = datastr->DATAlenth;	
	fifo->lenth = datastr->lenth;
	fifo->data = datastr->data;
	fifo->fullwriterflag = datastr->fullflag;
	fifo->direction = 1; 
}
void FIFO_getdirecton(FIFO_Type* fifo)
{	
	if(fifo->rear >=fifo->front){
		fifo->direction = 1;
	}else {
		fifo->direction = 0;
	}

}
 

uint8_t FIFO_PUSH_size(FIFO_Type* fifo,FIFO_DATA_SIZE_Str* Data)
{
	u16 i = Data->Datath+2;
	u8  overflag ;
	u16 reartemp ;
	u16 fronttemp ;
	u16 DataWitdth ;
	u16 DataWitdth1;

	
	if(fifo->data==NULL)return   0;
	DISABLEIRQ();	
	reartemp = (fifo->rear+i)%fifo->lenth;
	overflag = (fifo->rear+i)/fifo->lenth;
	fronttemp = fifo->front;
	
	if(fifo->direction&&overflag){
		if(reartemp>=fronttemp){
			if(fifo->fullwriterflag==0){  
					ENABLEIRQ();
				return 0;
			}else {
				DataWitdth1 = fifo->lenth-fifo->rear+fronttemp;
				fifo->direction = 0;
				do{
					DataWitdth = fifo->data[fifo->front]|fifo->data[(fifo->front+1)%fifo->lenth]<<8;
					DataWitdth1+=DataWitdth;
					fifo->front = (fifo->front+DataWitdth)%fifo->lenth;	
				}while(DataWitdth1<=i);
				if(fifo->front<fronttemp){
					fifo->direction = 1;
				}
			}		
		}else {
			fifo->direction = 0;	
		}
	}else if(fifo->direction==0){
		if(fifo->rear+i>=fronttemp){
			if(fifo->fullwriterflag==0){
				//cout<<"full"<<endl;
					ENABLEIRQ();
				return 0;
			}else {
				DataWitdth1 = fronttemp-fifo->rear;
				do{
					DataWitdth = fifo->data[fifo->front]|fifo->data[(fifo->front+1)%fifo->lenth]<<8;
					DataWitdth1+=DataWitdth;
					fifo->front = (fifo->front+DataWitdth)%fifo->lenth;	
				}while(DataWitdth1<=i);
				if(fifo->front<fronttemp){
					if(overflag ==0)
						fifo->direction = 1;
				}
			}		
		}	
	
	}
	if(overflag){
		switch(fifo->lenth-fifo->rear){
			case 1:
				fifo->data[fifo->rear]= i;
				fifo->data[0]= i>>8;
				memcpy((void*)&fifo->data[1],(void*)Data->data,Data->Datath);
				break;
			case 2:
				
				memcpy((void*)&fifo->data[fifo->rear],(void*)&i,2);
				memcpy((void*)&fifo->data[0],(void*)Data->data,Data->Datath);	
				break;
			default:
				memcpy((void*)&fifo->data[fifo->rear],(void*)&i,2);
				memcpy((void*)&fifo->data[fifo->rear+2],(void*)Data->data,fifo->lenth-fifo->rear-2);
				memcpy((void*)&fifo->data[0],(void*)&Data->data[fifo->lenth-fifo->rear-2],reartemp);
				break;
		}			
	}else {
		memcpy((void*)&fifo->data[fifo->rear],(void*)&i,2);
		memcpy((void*)&fifo->data[fifo->rear+2],(void*)Data->data,Data->Datath);
	}
	fifo->rear = reartemp;

	ENABLEIRQ();
	return 1;
}
uint8_t FIFO_POP_size(FIFO_Type* fifo,FIFO_DATA_SIZE_Str* Data)
{
	u8  overflag ;
	u16 Datalenth ;

	
	if(fifo==NULL)return 0;
	DISABLEIRQ();
	if(fifo->front==fifo->rear){//KONG;
		fifo->direction = 1;
		ENABLEIRQ();
		//cout<<"enmpty"<<endl;
		return 0;
	}
	Datalenth = fifo->data[fifo->front]|fifo->data[(fifo->front+1)%fifo->lenth]<<8;
	overflag = (fifo->front+Datalenth)/fifo->lenth;	
	if(overflag){
		fifo->direction = 1;
		switch(fifo->lenth -fifo->front){
			case 1:
				memcpy((void*)(Data->data),(void*)&fifo->data[1],Datalenth-2);
				break;
			case 2:
				memcpy((void*)(Data->data),(void*)&fifo->data[0],Datalenth-2);
				break;	
			default:
				memcpy((void*)Data->data,(void*)&fifo->data[fifo->front+2],fifo->lenth-fifo->front-2);
				memcpy((void*)(Data->data+fifo->lenth-fifo->front-2),(void*)&fifo->data[0],(fifo->front+Datalenth)%fifo->lenth);
			break;
		}
	}else {
		memcpy((void*)Data->data,(void*)&fifo->data[fifo->front+2],Datalenth-2);	
	}
	Data->Datath  = Datalenth-2;
	fifo->front = (fifo->front+Datalenth)%fifo->lenth;
	ENABLEIRQ();
	return 1;
}

uint8_t FIFO_Get_states_size(FIFO_Type* fifo)
{
	return 0;

}



















