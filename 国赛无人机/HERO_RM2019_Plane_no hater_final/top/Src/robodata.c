#include "robodata.h"
#include "m_remote.h"
#include "dji_Protocol.h"
#include "tx2_Protocol.h"
#include "tripod.h"
#include "strike.h"
#include "ammo.h"
#include "t_moto.h"
#include "t_monitor.h"
#include "gpio.h"

extern float ammo_spd_test;
extern float ctrl_spd_test;
extern float yaw_angle_test;
extern float pitchL_angle_test;
extern float pitchR_angle_test;

RoboData_t RoboData={&InitRoboData};//�������û�������

/*function�����ÿ���ģʽ*******************************
 *parameter��Remote_t�࣬RoboControlMode_t��
 */
void SetRoboControlMode(Remote_t rc, RoboControlMode_t *rcmode){
	//��
	switch(rc.s1){
		case 1:
			rcmode->left_mode = LEFT_UP;break;
		case 2:
			rcmode->left_mode = LEFT_DOWN;break;
		case 3:
			rcmode->left_mode = LEFT_MID;break;
		default: 
			rcmode->left_mode = LEFT_MID;break;
	}
	//��
	switch(rc.s2){
		case 1:
			rcmode->ctrl_source = FROM_PC;break;
		case 2:
			rcmode->ctrl_source = RIGHT_RES;break;
		case 3:
			rcmode->ctrl_source = FROM_REMOTE;break;
		default: 
			rcmode->ctrl_source = FROM_REMOTE;break;
	}
}

/*������̨�˶�����*/
void GenerallySetTripodAngle(RC_Ctl_t rc_data, RoboControlMode_t robo_ctrlmode, TripodControl_t *tripod_ctrl){
	if(rc_data.rc.ch1 != 0) { tripod_ctrl->pitch_angle = (rc_data.rc.ch1-1024.0)/360.0f; }
		else { tripod_ctrl->pitch_angle = 0; }	
	if(rc_data.rc.ch0 != 0) { tripod_ctrl->yaw_angle = (rc_data.rc.ch0-1024.0)/360.0f; }
		else { tripod_ctrl->yaw_angle = 0; }	
}


/*function��ң�������ݴ�����*******************************
 *parameter��RoboData_t��
 */
void GetRemoteControlData(RoboData_t *RoboData){
	RCReadKey(&RC_CtrlData);
	SetRoboControlMode(RC_CtrlData.rc, &RoboData->robo_ctrlmode);//���ÿ���ģʽ
	GenerallySetTripodAngle(RC_CtrlData, RoboData->robo_ctrlmode, &RoboData->tripod_ctrl);//������̨�˶�����
}


/*function����ʼ��������������RoboData�ڵ�����*******************************
 *parameter��RoboData_t��
 */
void InitRoboData(RoboData_t *RoboData)
{
	
	Pitch_MOTO.MotoParaInit = &MotoParaInit;
	Pitch_MOTO.MotoParaInit(&Pitch_MOTO);

	Yaw_MOTO.MotoParaInit = &MotoParaInit;
	Yaw_MOTO.MotoParaInit(&Yaw_MOTO);
	
	Ammunition_MOTO.MotoParaInit = &MotoParaInit;
	Ammunition_MOTO.MotoParaInit(&Ammunition_MOTO);



	RC_CtrlData.RCDataParaInit = &RCDataParaInit;
	RC_CtrlData.RCDataParaInit(&RC_CtrlData);
	tx2_ReadData.TX2DataParaInit = &TX2DataParaInit;
	tx2_ReadData.TX2DataParaInit(&tx2_ReadData);
	
	monitor_remote.MonitorParaInit = &MonitorParaInit;
	monitor_remote.MonitorParaInit(&monitor_remote);
	monitor_tx2.MonitorParaInit = &MonitorParaInit;
	monitor_tx2.MonitorParaInit(&monitor_tx2);
	monitor_can_power.MonitorParaInit = &MonitorParaInit;
	monitor_can_power.MonitorParaInit(&monitor_can_power);
	

	RoboData->GetRemoteControlData = &GetRemoteControlData;


	RoboData->robo_ctrlmode.left_mode = LEFT_MID;//�󲦸�ģʽ
	RoboData->robo_ctrlmode.ctrl_source = FROM_REMOTE;//������ԴģʽĬ��Ϊ����ң����
	RoboData->AmmunitionControl.shoot17_order = Order_stop;//���跢������Ĭ��Ϊ������
	RoboData->AmmunitionControl.shoot42_order = Order_stop;//���跢������Ĭ��Ϊ������
	RoboData->AmmunitionControl.shoot17_working_status = WorkingStatus_ready;
	RoboData->AmmunitionControl.shoot42_working_status = WorkingStatus_ready;

}

