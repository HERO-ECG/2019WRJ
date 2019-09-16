#include "main.h"
#include "control.h"
#include "robodata.h"
#include "t_moto.h"
#include "tripod.h"
#include "strike.h"
#include "t_monitor.h"
#include "m_remote.h"
#include "strike.h"

uint32_t time_piece=0x0000;
extern float mocalun_dengdaishijian;
extern float mocalun_dengdaishijian_flag;
/*Robodata�еľɲ����������õ�*/
float friction_set_speed_debug = 7000.0f;
float ctrl_spd_test = 0.0f;
float ammo_spd_test = 0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3){         //100hz
		time_piece |= time_piece_0100hz;
//		Tripod_Fun();
//		if(mocalun_dengdaishijian_flag==1)
//		{	mocalun_dengdaishijian++;}
//		else
//		{mocalun_dengdaishijian=0;}
		
		shoot_monitor++;//�������������
		if(shoot_monitor>500)
			shoot_monitor=500;
		
		if(RoboData.robo_ctrlmode.ctrl_source==FROM_PC)
		{
		if(shoot_flag==1)//���˻�Ħ��������
			shoot_pending++;
		else
			shoot_pending=0;

		if(Bodan_Help_Flag==1)//���Ʋ�������ʱ���ʱ
			Bodan_Help++;
		else 
			Bodan_Help=0;
		if(Bodan_Help>680)
			Bodan_Help=640;
		
		if(yanshi_flag==1)//��Ħ���ֳ�ʼ����ʱ��
			yanshi++;
		else if(yanshi_flag==3)
			yanshi=0;
		if(yanshi>150&&yanshi_flag==1)
			yanshi_flag=2;
	}
		else
			qiangxingqidong=0;
//		if(mocalun_qidong_pending_flag==1)
//		mocalun_qidong_pending++;
//		else 
//		mocalun_qidong_pending=0;//Ħ���ֽ�������
//		if(mocalun_qidong_pending>120)
//			mocalun_qidong_pending=101;
		
		RoboData.GetRemoteControlData(&RoboData);
		/*������ƺ�����������ͺ�������*/
	}
	else if(htim == &htim5){		//1000hz
		time_piece |= time_piece_1000hz;
		IMU_Get_Data();//��ð�������������
			if(imu_data.init_sta){//�ȴ������ǳ�ʼ�����
				Strike_fun();
				Tripod_Fun();
				
			}
		
		HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//����źŽ���
		Strike_fun();
//		SetMotoCurrent(&hcan1, STDID_HIGH, Yaw_MOTO.send_current, Pitch_MOTO.send_current, 0, Ammunition_MOTO.send_current );
		SetMotoCurrent(&hcan1, STDID_HIGH, Yaw_MOTO.send_current,Pitch_MOTO.send_current, 0, Ammunition_MOTO.send_current );

	}
}
