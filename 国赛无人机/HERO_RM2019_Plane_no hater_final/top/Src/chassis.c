#include "chassis.h"
#include "tripod.h"
#include "m_moto.h"
#include "math.h"
#include "t_monitor.h"

Chassis_t Chassis;

#define ABS(X) ((X>0)? (X): (-X))

void Chassis_SetModeFromControl(void);
float ChassissFollow(float angle_now,float angle_set,float deadband);
void ChassisSetSpeed(float theta);

/*底盘运动参数初始化*/
void Chassis_Init(float speed_base, float angle_deadband, float mode)
{
	Chassis.speed.base = speed_base;
//	Chassis.speed.spd_spin_perunit = speed_spin;
	Chassis.follow.deadband = angle_deadband;
	Chassis.follow.mode.now = mode;
	Chassis.follow.mode.last = mode;
}

/*底盘运动总功能函数*/
void Chassis_Fun(void)//100Hz
{
//	static uint16_t swing_cnt=0;
//	float swing_angle;
//	Chassis_SetModeFromControl();
	RoboData.GetRemoteControlData(&RoboData);//遥控器数据处理
//	Chassis.follow.angle = ((Yaw_MOTO.getpara.angle)/8192.0f*360.0f - YAWMIDANGLE);
//	Chassis.follow.total_angle = ((Yaw_MOTO.getpara.total_angle)/8192.0f*360.0f - YAWMIDANGLE);
	if(Chassis.follow.mode.now == NOT_FOLLOW)
	{
//		Chassis.speed.spd_yaw = 0;
		ChassisSetSpeed(0);
	}
//	else if(Chassis.follow.mode.now == FRONT_FOLLOW)//前部跟随模式
//	{
//		Chassis.speed.spd_yaw = ChassissFollow(Chassis.follow.total_angle, 0, Chassis.follow.deadband);
//		ChassisSetSpeed(-Chassis.follow.total_angle);
//	}
//	else if(Chassis.follow.mode.now == SWAY)//摇摆模式
//	{
//		uint16_t time_period = 200;
//		if(Chassis.follow.mode.last != SWAY)
//			swing_cnt = 0;
//		swing_cnt++;
//		if(swing_cnt == time_period)//周期
//			swing_cnt = 0;
//		swing_angle = 25.0f*sin(2*3.1416*(double)swing_cnt/(double)time_period);
//		Chassis.speed.spd_yaw = ChassissFollow(Chassis.follow.total_angle, swing_angle, 1);//Chassis.speed.spd_spin_perunit;
//		ChassisSetSpeed(-Chassis.follow.total_angle);
//	}
	Chassis.follow.mode.last=Chassis.follow.mode.now;
}

/*底盘运动控制*/
void Chassis_SetModeFromControl(void)
{
	if(RoboData.robo_ctrlmode.ctrl_source == FROM_REMOTE)
	{
		if(RoboData.robo_ctrlmode.left_mode == LEFT_DOWN)
			Chassis.follow.mode.now=FRONT_FOLLOW;
		
		else if(RoboData.robo_ctrlmode.left_mode == LEFT_UP)
			Chassis.follow.mode.now=SWAY;
		
		if(RoboData.robo_ctrlmode.left_mode == LEFT_MID)
		Chassis.follow.mode.now=NOT_FOLLOW;
	}
	else if(RoboData.robo_ctrlmode.ctrl_source == FROM_PC)
	{
		if(RC_CtrlData.key.key_data.F==1&&
				RC_CtrlData.key.key_data.ctrl==0&&
				RC_CtrlData.key.key_data.shift==0)
			Chassis.follow.mode.now=FRONT_FOLLOW;
		else if(RC_CtrlData.key.key_data.F==1&&
				RC_CtrlData.key.key_data.ctrl==0&&
				RC_CtrlData.key.key_data.shift==1)
			Chassis.follow.mode.now=NOT_FOLLOW;
		else if(RC_CtrlData.key.key_data.C==1)
			Chassis.follow.mode.now=SWAY;
		
		if(RC_CtrlData.key.key_data.W==1||
				RC_CtrlData.key.key_data.S==1||
				RC_CtrlData.key.key_data.A==1||
				RC_CtrlData.key.key_data.D==1)
		{
			if(RC_CtrlData.key.key_data.ctrl==0&&
				RC_CtrlData.key.key_data.shift==0)
				Chassis.speed.base = 4000;
			else if(RC_CtrlData.key.key_data.ctrl==0&&
				RC_CtrlData.key.key_data.shift==1)
				Chassis.speed.base = 6000;
			else if(RC_CtrlData.key.key_data.ctrl==1&&
				RC_CtrlData.key.key_data.shift==0)
				Chassis.speed.base = 2000;
		}
	}
}

/*底盘跟随云台运动//yaw_speed解算*/
float ChassissFollow(float angle_now, float angle_set, float deadband)
{
	float angle_err, speed_yaw;
	angle_err = angle_now - angle_set;
	if(ABS(angle_err)<deadband)
		angle_err = 0;
	else if(angle_err>deadband)
		angle_err -= deadband;
	else if(angle_err<-deadband)
		angle_err += deadband;
	/*速度转换*/
	speed_yaw = angle_err;
	speed_yaw/=(Chassis.speed.base*0.015f);
	if(speed_yaw>1)
		speed_yaw = 1;
	if(speed_yaw<-1)
		speed_yaw = -1;
	return speed_yaw;
}

/*底盘电机速度分配&&电流发送*/
void ChassisSetSpeed(float theta) 
{
	//theta>0为右转
	if(monitor_remote.status == monitor_regular)
	{
		float ksin=0,kcos=0,W_sin=0,W_cos=0,D_sin=0,D_cos=0,cm_set_spd[5]={0,0,0,0,0};
		ksin = (float)sin(theta*3.1416f/180.0f);
		kcos = (float)cos(theta*3.1416f/180.0f);
		W_sin = Chassis.speed.spd_forward*ksin;
		W_cos = Chassis.speed.spd_forward*kcos;
		D_sin = Chassis.speed.spd_right*ksin;
		D_cos = Chassis.speed.spd_right*kcos;
		//速度分配矩阵
		cm_set_spd[0] = ( (W_cos-D_sin) + (W_sin+D_cos) + Chassis.speed.spd_yaw);
		cm_set_spd[1] = (-(W_cos-D_sin) + (W_sin+D_cos) + Chassis.speed.spd_yaw);
		cm_set_spd[2] = ( (W_cos-D_sin) - (W_sin+D_cos) + Chassis.speed.spd_yaw);
		cm_set_spd[3] = (-(W_cos-D_sin) - (W_sin+D_cos) + Chassis.speed.spd_yaw);
		//底盘各电机期望速度设定
		for(int i=0;i<4;i++)	cm_set_spd[4] += ABS(cm_set_spd[i]);
		cm_set_spd[4] = cm_set_spd[4]>1 ? cm_set_spd[4] : 1;
		for(int i=0;i<4;i++)	Chassis_MOTO[i].set_speed = Chassis.speed.base * cm_set_spd[i] / cm_set_spd[4] * 4;
	}
	else
	{
		Chassis_MOTO[0].set_speed=0;
		Chassis_MOTO[1].set_speed=0;
		Chassis_MOTO[2].set_speed=0;
		Chassis_MOTO[3].set_speed=0;
	}		

	int i = 0;
//	float power = 0;
//	power = 0.01f + Chassis_MOTO[0].getpara.get_power + Chassis_MOTO[1].getpara.get_power + Chassis_MOTO[2].getpara.get_power + Chassis_MOTO[3].getpara.get_power;
//	for(i=0; i<4; i++)
//	{
//		int current_max;
//		/*各轮功率计算*/
//		 Chassis_MOTO[i].getpara.set_power = Chassis_MOTO[i].getpara.get_power / power * DJI_ReadData.extPowerHeatData.chassisPower;
//		/*功率环_&_各轮最大输出电流限制*/
//		current_max = 5000+(ABS(Chassis_MOTO[i].set_speed)/Chassis.speed.base*80 + DJI_ReadData.extPowerHeatData.chassisPowerBuffer - Chassis_MOTO[i].getpara.set_power)*1000;
//		current_max = current_max>CHASSIS_MAX_CURRENT ? CHASSIS_MAX_CURRENT : current_max;
//		current_max = current_max<1500 ? 1500 : current_max;
//		Chassis_MOTO[i].pid_speed.max_output = current_max;
//		/*速度环_&_电机输出信号赋值*/
//		Chassis_MOTO[i].send_current = Chassis_MOTO[i].pid_speed.PidCalc(&Chassis_MOTO[i].pid_speed, Chassis_MOTO[i].getpara.speed_rpm, Chassis_MOTO[i].set_speed);	  
//	}	
	
//	float RealPower_Total = 0;
//	static float SpeedReduceFactor = 1;
//	
//	RealPower_Total = 0.9*RealPower_Total + 0.1*(Chassis_MOTO[0].getpara.get_power + Chassis_MOTO[1].getpara.get_power + Chassis_MOTO[2].getpara.get_power + Chassis_MOTO[3].getpara.get_power + 12);//3为静止时消耗功率
//	if(RealPower_Total > POWER_LIMIT)
//	{		
//		SpeedReduceFactor -= 0.1;
//		if(SpeedReduceFactor < 0.2) 
//		{
//			SpeedReduceFactor = 0.2;
//		}
//	}
//	else
//	{
//		SpeedReduceFactor += 0.1;
//		if(SpeedReduceFactor > 1) 
//		{
//			SpeedReduceFactor = 1;
//		}
//	}
	/*速度环_&_电机输出信号赋值*/
	for(i=0; i<4; i++)
	{
//		Chassis_MOTO[i].set_speed *= SpeedReduceFactor;		
		Chassis_MOTO[i].send_current = Chassis_MOTO[i].pid_speed.PidCalc(&Chassis_MOTO[i].pid_speed, Chassis_MOTO[i].getpara.speed_rpm, Chassis_MOTO[i].set_speed);	  
	}	
}
