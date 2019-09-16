/*******************************************************************************************
  * @Include @Headfile
 *******************************************************************************************/
#include "moto_encoder.h"
#include "main.h"
#include "can.h"
#include "m_moto.h"
#include "m_remote.h"
#include "tripod.h"
#include "robodata.h"
#include "t_monitor.h"
#include "dji_Protocol.h"
#include "tx2_Protocol.h"
#include "strike.h"

/*******************************************************************************************
  * @Parameter @Statement
 *******************************************************************************************/

/*******************************************************************************************
  * @Func			HAL_CAN_RxCpltCallback
  * @Brief    CAN接收回调函数
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2018.10.10
  *	@Author		SZC		LZK添加CAN入口判断
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1){	
		switch(hcan->pRxMsg->StdId){
			/*双电机主从模式，左电机为主控制电机，右电机为从动电机*/
			case YAW_6623_ID:
 				Yaw_MOTO.getpara.GetEncoderMeasure(&Yaw_MOTO.getpara, hcan);
					if(Yaw_MOTO.getpara.init_sta == 1){
						Tripod.Yaw.angle_source.encoder.angle_now = (Yaw_MOTO.getpara.total_angle - YAWMIDANGLE)/8192.0f*360.0f;
//						Tripod.Yaw.data_source.encoder.angle_relativeToChassis = (Yaw_MOTO.getpara.total_angle - YAWMIDANGLE)/8192.0f*360.0f;
//						Tripod.Yaw.ready_flag = 1;
					}	
			break;
			case AMMO_3508_ID:
				Ammunition_MOTO.getpara.GetEncoderMeasure(&Ammunition_MOTO.getpara, hcan);
				shoot_monitor=0;
			break;
			case PITCH_6020_ID:
				Pitch_MOTO.getpara.GetEncoderMeasure(&Pitch_MOTO.getpara, hcan);
				if(Pitch_MOTO.getpara.init_sta == 1){				
					Tripod.Pitch.angle_source.encoder.angle_now = (Pitch_MOTO.getpara.total_angle - PITCHHORANGLE)/8192.0f*360.0f;
					if(Tripod.Pitch.angle_source.encoder.angle_now<-200)
						Tripod.Pitch.angle_source.encoder.angle_now+=360;
					//					Tripod.Pitch.angle_source.encoder.ready_flag = 1;
					}
			break;
//			case PITCHR_6020_ID:
//				Pitch_MOTO[1].getpara.GetEncoderMeasure(&Pitch_MOTO[1].getpara, hcan);
	//		break;
			case MOTO1_3508_ID:
//			Friction_Wheel_MOTO[0].getpara.GetEncoderMeasure(&Friction_Wheel_MOTO[0].getpara, hcan);
			break;
			case MOTO2_3508_ID:
//			Friction_Wheel_MOTO[1].getpara.GetEncoderMeasure(&Friction_Wheel_MOTO[1].getpara, hcan);
			break;
			case MOTO3_3508_ID:
//			LittleAmmo_MOTO.getpara.GetEncoderMeasure(&LittleAmmo_MOTO.getpara, hcan);
			case MOTO4_3508_2006_ID:
//			Ctrl_2006_MOTO.getpara.GetEncoderMeasure(&Ctrl_2006_MOTO.getpara, hcan);
			break;
		}
	}
	if(hcan == &hcan2){	
		switch(hcan->pRxMsg->StdId){
			case MOTO1_3508_ID:
			case MOTO2_3508_ID:
			case MOTO3_3508_ID:
			case MOTO4_3508_2006_ID:{
//				uint32_t can_id = 0;
//				can_id = hcan->pRxMsg->StdId - MOTO1_3508_ID;
//				Chassis_MOTO[can_id].getpara.GetEncoderMeasure(&Chassis_MOTO[can_id].getpara, hcan);
			}break;
		}
	}
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);//重新使能can中断标志
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}
