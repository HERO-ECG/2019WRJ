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

RoboData_t RoboData={&InitRoboData};//机器人用户数据类

/*function：设置控制模式*******************************
 *parameter：Remote_t类，RoboControlMode_t类
 */
void SetRoboControlMode(Remote_t rc, RoboControlMode_t *rcmode){
	//左
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
	//右
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

/*设置云台运动控制*/
void GenerallySetTripodAngle(RC_Ctl_t rc_data, RoboControlMode_t robo_ctrlmode, TripodControl_t *tripod_ctrl){
	if(rc_data.rc.ch1 != 0) { tripod_ctrl->pitch_angle = (rc_data.rc.ch1-1024.0)/360.0f; }
		else { tripod_ctrl->pitch_angle = 0; }	
	if(rc_data.rc.ch0 != 0) { tripod_ctrl->yaw_angle = (rc_data.rc.ch0-1024.0)/360.0f; }
		else { tripod_ctrl->yaw_angle = 0; }	
}


/*function：遥控器数据处理函数*******************************
 *parameter：RoboData_t类
 */
void GetRemoteControlData(RoboData_t *RoboData){
	RCReadKey(&RC_CtrlData);
	SetRoboControlMode(RC_CtrlData.rc, &RoboData->robo_ctrlmode);//设置控制模式
	GenerallySetTripodAngle(RC_CtrlData, RoboData->robo_ctrlmode, &RoboData->tripod_ctrl);//设置云台运动控制
}


/*function：初始化机器人数据类RoboData内的数据*******************************
 *parameter：RoboData_t类
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


	RoboData->robo_ctrlmode.left_mode = LEFT_MID;//左拨杆模式
	RoboData->robo_ctrlmode.ctrl_source = FROM_REMOTE;//控制来源模式默认为来自遥控器
	RoboData->AmmunitionControl.shoot17_order = Order_stop;//弹丸发射设置默认为不发射
	RoboData->AmmunitionControl.shoot42_order = Order_stop;//弹丸发射设置默认为不发射
	RoboData->AmmunitionControl.shoot17_working_status = WorkingStatus_ready;
	RoboData->AmmunitionControl.shoot42_working_status = WorkingStatus_ready;

}

