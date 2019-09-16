#ifndef __M_TX2_PROTOCOL_H
#define __M_TX2_PROTOCOL_H
#include "mytype.h"
#include "robodata.h"
typedef struct TX2_Ctl_t
{
	void (*TX2DataParaInit)(struct TX2_Ctl_t *tx2_ReadData);
	//gimbal
	short yaw_angle;
  short pitch_angle;
	uint8_t control_mode;
	uint8_t autoaim_sta;
}TX2_Data_t;  //สýพÝึก

extern TX2_Data_t tx2_ReadData;
extern uint8_t TX2_Data_Send[7];
extern short TX2_DataProcess(uint8_t *pData, TX2_Data_t *tx2_ReadData);
extern void TX2DataParaInit(TX2_Data_t *tx2_ReadData);


#endif
