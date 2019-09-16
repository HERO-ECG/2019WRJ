#include "tripod.h"
#include "usart.h"
#include "tx2_Protocol.h"
#include "m_moto.h"

TX2_Data_t tx2_ReadData;
short TX2_DataProcess(uint8_t *pData, TX2_Data_t *tx2_ReadData)
{
	uint8_t i;
	uint8_t checksum = 0; 
	uint8_t* data_ptr;
	 
	for(i=0;i<18;i++)
	{
		if((pData[i]==0x55)&&(pData[i+1]==0x50)&&(pData[i+8]==0xA5))
		{ 
			data_ptr = (uint8_t*)(pData+i+2);	
			tx2_ReadData->yaw_angle =  (short)(data_ptr[0] | data_ptr[1]<<8)/1000.0f;	
			tx2_ReadData->pitch_angle =  (short)(data_ptr[2] | data_ptr[3]<<8)/1000.0f;	
			tx2_ReadData->control_mode = (float)(data_ptr[4])/10.0f;
			for(i = 0; i < 5; i++){
				checksum += data_ptr[i];
			} 
			if(tx2_ReadData->control_mode!=0){
				tx2_ReadData->autoaim_sta = 1;
				Tripod.Yaw.visualAngle = Tripod.Yaw.angle + tx2_ReadData->yaw_angle;//Ïò×óÎªÕý
				Tripod.Pitch.visualAngle = Tripod.Pitch.angle + tx2_ReadData->pitch_angle;//
				return 1;
			}
		}
	}		
	tx2_ReadData->autoaim_sta = 0;	
	return 0;
}	

void TX2DataParaInit(TX2_Data_t *tx2_ReadData)
{
	tx2_ReadData->TX2DataParaInit = &TX2DataParaInit;
	tx2_ReadData->yaw_angle = 0;
	tx2_ReadData->pitch_angle = 0;
	tx2_ReadData->control_mode = 0 ;
	tx2_ReadData->autoaim_sta = 0;
}



