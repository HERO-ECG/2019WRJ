#include "robodata.h"
#include "tripod.h"
#include "strike.h"
#include "hwt901b.h"
#include "m_moto.h"
#include "math.h"
#include "m_imu.h"

Tripod_t Tripod = {&Tripod_Init,&Tripod_Fun};

//void Tripod_SetModeFromControl(void)
//{
//	static uint8_t G_key_last=0;
//	
//	if(G_key_last!=RC_CtrlData.key.key_data.G&&RC_CtrlData.key.key_data.G==1)
//		Tripod.Yaw.angle+=180;
//}


float GetCoupledYawSpeedFromPitch(float pitch, float yaw_speed_raw)
{
	float coupled_yaw_speed;
	coupled_yaw_speed=cos(Tripod.Pitch.angle/180.0f*3.1415926f)*yaw_speed_raw;
	return coupled_yaw_speed;
}

float GetIntergralYaw(float yaw_speed, uint16_t frequency)
{
	static float yaw_integral;
	yaw_integral+=(yaw_speed)/(float)frequency;
	return yaw_integral;
}
//获取Pitch速度的积分，获得角度
float GetIntergralPitch(float Pitch_speed,uint16_t frequency)
{
	static float Pitch_integral;
	Pitch_integral+=(Pitch_speed)/(float)frequency;
	return Pitch_integral;
}

float pitch_text_angle;
void Pitch_SetAngle(float pitch)
{
	Tripod.Pitch.angle_set=pitch;
	/*angle_set相对水平参考系，角度限幅相对于编码器*///Tripod.Pitch.angle是一个量度基准，在限幅角度上面加上这个量度基准
	
	if((Tripod.Pitch.angle_set-Tripod.Pitch.angle+Tripod.Pitch.angle_source.encoder.angle_now)>(Tripod.Pitch.angle_max))//+Pitch_MOTO.getpara.round_cnt*360))
		Tripod.Pitch.angle_set=(Tripod.Pitch.angle_max)+Tripod.Pitch.angle-Tripod.Pitch.angle_source.encoder.angle_now;
	if((Tripod.Pitch.angle_set-Tripod.Pitch.angle+Tripod.Pitch.angle_source.encoder.angle_now)<(Tripod.Pitch.angle_min))//+Pitch_MOTO.getpara.round_cnt*360))
		Tripod.Pitch.angle_set=(Tripod.Pitch.angle_min)+Tripod.Pitch.angle-Tripod.Pitch.angle_source.encoder.angle_now;
//	if(Tripod.Pitch.angle_set-Tripod.Pitch.angle+Tripod.Pitch.angle_source.encoder.angle_now>(Tripod.Pitch.angle_max-Pitch_MOTO[0].getpara.round_cnt*360))
//		Tripod.Pitch.angle_set=(Tripod.Pitch.angle_max-Pitch_MOTO[0].getpara.round_cnt*360)+Tripod.Pitch.angle-Tripod.Pitch.angle_source.encoder.angle_now;
//	if(Tripod.Pitch.angle_set-Tripod.Pitch.angle+Tripod.Pitch.angle_source.encoder.angle_now<(Tripod.Pitch.angle_min-Pitch_MOTO[0].getpara.round_cnt*360))
//		Tripod.Pitch.angle_set=(Tripod.Pitch.angle_min-Pitch_MOTO[0].getpara.round_cnt*360)+Tripod.Pitch.angle-Tripod.Pitch.angle_source.encoder.angle_now;
	
	Tripod.Pitch.speed_set=PidCalc(&Pitch_MOTO.pid_angle,Tripod.Pitch.angle,Tripod.Pitch.angle_set);
	
	Pitch_MOTO.send_current=PidCalc(&Pitch_MOTO.pid_speed,Tripod.Pitch.speed,Tripod.Pitch.speed_set);
}



void Yaw_SetAngle(float angle_set)
{
	
	Tripod.Yaw.angle_set=angle_set;		
//	if(Tripod.Yaw.angle_set > Tripod.Yaw.angle_max)
//		Tripod.Yaw.angle_set = Tripod.Yaw.angle_max;
//	if(Tripod.Yaw.angle_set < Tripod.Yaw.angle_min)
//		Tripod.Yaw.angle_set = Tripod.Yaw.angle_min;
	if((Tripod.Yaw.angle_set-Tripod.Yaw.angle+Tripod.Yaw.angle_source.encoder.angle_now)>(Tripod.Yaw.angle_max+Yaw_MOTO.getpara.round_cnt*360))
		Tripod.Yaw.angle_set=(Tripod.Yaw.angle_max+Yaw_MOTO.getpara.round_cnt*360)+Tripod.Yaw.angle-Tripod.Yaw.angle_source.encoder.angle_now;
	if((Tripod.Yaw.angle_set-Tripod.Yaw.angle+Tripod.Yaw.angle_source.encoder.angle_now)<(Tripod.Yaw.angle_min+Yaw_MOTO.getpara.round_cnt*360))
		Tripod.Yaw.angle_set=(Tripod.Yaw.angle_min+Yaw_MOTO.getpara.round_cnt*360)+Tripod.Yaw.angle-Tripod.Yaw.angle_source.encoder.angle_now;
	
	Tripod.Yaw.speed_set=PidCalc(&Yaw_MOTO.pid_angle,Tripod.Yaw.angle,Tripod.Yaw.angle_set);
	Yaw_MOTO.send_current=PidCalc(&Yaw_MOTO.pid_speed,Tripod.Yaw.speed,Tripod.Yaw.speed_set);
}

//void Yaw_SetSpeed(float speed)
//{
//	ABSLimit(&Tripod.Yaw.speed_set,Tripod.Yaw.PID_angle.max_output);
//	Tripod.Yaw.current_set=PidCalc(&Tripod.Yaw.PID_speed,Tripod.Yaw.speed,Tripod.Yaw.speed_set);
//}

//void Pitch_SetSpeed(float speed)
//{
//	ABSLimit(&Tripod.Pitch.speed_set,Tripod.Pitch.PID_angle.max_output);
//	Tripod.Pitch.current_set=PidCalc(&Tripod.Pitch.PID_speed,Tripod.Pitch.speed,Tripod.Pitch.speed_set);
//}

void GetYawAngle100Hz(void)//位置来源通过键位在编码器和陀螺仪间切换
{
	
	
	Tripod.Yaw.speed = imu_data.gz/16.384f;//左正
	Tripod.Yaw.angle_source.gyro.angle_now=GetIntergralYaw(Tripod.Yaw.speed,1000);	//左正
//	Tripod.Pitch.speed = imu_data.gy/16.384f;//下负
//	Tripod.Pitch.angle_source.gyro.angle_now = imu_data.angley*0.05f + 0.95f*Tripod.Pitch.angle_source.gyro.angle_now;//下负
	
//	Tripod.Pitch.angle = (Tripod.Pitch.angle_source.encoder.angle_now);//下正
	
	if(RoboData.robo_ctrlmode.ctrl_source==FROM_PC)
	{
		if(RC_CtrlData.key.key_data.F==1)
		{
			if(RC_CtrlData.key.key_data.ctrl!=1)
			Tripod.Yaw.angle_source.source_now=From_Gyro;
			else
			Tripod.Yaw.angle_source.source_now=From_Encoder;
		}
	}
	else
	{
		if(RoboData.robo_ctrlmode.ctrl_source==RIGHT_RES)
			Tripod.Yaw.angle_source.source_now=From_Gyro;
		else
			Tripod.Yaw.angle_source.source_now=From_Encoder;
	}
	

	if(Tripod.Yaw.angle_source.source_now==From_Encoder) //编码器
	{
		Tripod.Yaw.angle_source.source_now=From_Encoder;
		if(Tripod.Yaw.angle_source.source_last==From_Gyro)
		{
			Tripod.Yaw.angle_source.encoder.angle_offset=(Tripod.Yaw.angle_source.encoder.angle_now-Tripod.Yaw.angle);
		}
		Tripod.Yaw.angle=Tripod.Yaw.angle_source.encoder.angle_now-Tripod.Yaw.angle_source.encoder.angle_offset;
	}
	else //陀螺仪
	{
		{
			Tripod.Yaw.angle_source.source_now=From_Gyro;
			if(Tripod.Yaw.angle_source.source_last==From_Encoder)
			{
				Tripod.Yaw.angle_source.gyro.angle_offset=(Tripod.Yaw.angle_source.gyro.angle_now-Tripod.Yaw.angle);
			}
			Tripod.Yaw.angle=Tripod.Yaw.angle_source.gyro.angle_now-Tripod.Yaw.angle_source.gyro.angle_offset;
		}
	}
	Tripod.Yaw.angle=(Tripod.Yaw.angle);
	Tripod.Yaw.angle_source.source_last=Tripod.Yaw.angle_source.source_now;
}


void GetPitchAngle100Hz(void)//位置来源通过键位在编码器和陀螺仪间切换
{

	
	Tripod.Pitch.speed = imu_data.gy/16.384f;//下负
	Tripod.Pitch.angle_source.gyro.angle_now = GetIntergralPitch(Tripod.Pitch.speed,1000);
	
//	Tripod.Pitch.angle = (Tripod.Pitch.angle_source.encoder.angle_now);//下正
	
	if(RoboData.robo_ctrlmode.ctrl_source==FROM_PC)
	{
		if(RC_CtrlData.key.key_data.F==1)
		{
			if(RC_CtrlData.key.key_data.ctrl!=1)
			{
//			{Tripod.Yaw.angle_source.source_now=From_Gyro;
			Tripod.Pitch.angle_source.source_now=From_Gyro;}
			else
//			{{Tripod.Yaw.angle_source.source_now=From_Encoder;
			{Tripod.Pitch.angle_source.source_now=From_Encoder;}
		}
	}
	else
	{
		if(RoboData.robo_ctrlmode.ctrl_source==RIGHT_RES)
			Tripod.Pitch.angle_source.source_now=From_Gyro;
		else
			Tripod.Pitch.angle_source.source_now=From_Encoder;
	}
	

	if(Tripod.Pitch.angle_source.source_now==From_Encoder) //编码器
	{
		Tripod.Pitch.angle_source.source_now=From_Encoder;
		if(Tripod.Pitch.angle_source.source_last==From_Gyro)
		{
			Tripod.Pitch.angle_source.encoder.angle_offset=(Tripod.Pitch.angle_source.encoder.angle_now-Tripod.Pitch.angle);
		}
		Tripod.Pitch.angle=Tripod.Pitch.angle_source.encoder.angle_now-Tripod.Pitch.angle_source.encoder.angle_offset;
	}
	else //陀螺仪
	{
		{
			Tripod.Pitch.angle_source.source_now=From_Gyro;
			if(Tripod.Pitch.angle_source.source_last==From_Encoder)
			{
				Tripod.Pitch.angle_source.gyro.angle_offset=(Tripod.Pitch.angle_source.gyro.angle_now-Tripod.Pitch.angle);
			}
			Tripod.Pitch.angle=Tripod.Pitch.angle_source.gyro.angle_now-Tripod.Pitch.angle_source.gyro.angle_offset;
		}
	}
	Tripod.Pitch.angle=(Tripod.Pitch.angle);
	Tripod.Pitch.angle_source.source_last=Tripod.Pitch.angle_source.source_now;
}



void Tripod_Fun(void)
{

	if(Yaw_MOTO.getpara.init_sta==1)
		Tripod.Yaw.ready_flag=1;
	if(Pitch_MOTO.getpara.init_sta==1)
		Tripod.Pitch.ready_flag=1;
	GetYawAngle100Hz();
	GetPitchAngle100Hz();
	
	if(RoboData.robo_ctrlmode.ctrl_source==FROM_REMOTE){
		Tripod.control_mode.now = Position;
		Tripod.Yaw.angle_set+=RoboData.tripod_ctrl.yaw_angle/10;
		Tripod.Pitch.angle_set+=RoboData.tripod_ctrl.pitch_angle/10.0f;
	}
	else if(RoboData.robo_ctrlmode.ctrl_source==FROM_PC){
		if(KEY_AUTOAIM==1){//自瞄
			Tripod.control_mode.now = autoSight;
			Tripod.Yaw.angle_set = Tripod.Yaw.visualAngle;
			Tripod.Pitch.angle_set = Tripod.Pitch.visualAngle;
		}else{//非自瞄
			Tripod.control_mode.now = Position;
			if(RC_CtrlData.key.key_data.V==1)//按v键降低云台灵敏度 辅助瞄准
				{
						Tripod.Yaw.angle_set += RC_CtrlData.mouse.x_speed*0.002f;
						Tripod.Pitch.angle_set += -RC_CtrlData.mouse.y_speed*0.002f;	
				}
			else
				{
				Tripod.Yaw.angle_set += RC_CtrlData.mouse.x_speed*0.005f;
				Tripod.Pitch.angle_set += -RC_CtrlData.mouse.y_speed*0.005f;
				}
		}
	}
	Tripod.control_mode.last=Tripod.control_mode.now;
	
	switch((uint16_t)Tripod.control_mode.now){
		case Position:{
			if(Tripod.Pitch.ready_flag==1)
				Pitch_SetAngle(Tripod.Pitch.angle_set);
			if(Tripod.Yaw.ready_flag==1)	
				Yaw_SetAngle(Tripod.Yaw.angle_set);
		}break;
		case autoSight:{
			/*自瞄*/
			if(Tripod.Yaw.ready_flag==1){
				Yaw_SetAngle(Tripod.Yaw.angle_set);
			}
			if(Tripod.Pitch.ready_flag==1){
				Pitch_SetAngle(Tripod.Pitch.angle_set);
			}
		}break;
		default:{
		}break;
	}	
}

void Yaw_Init(void)
{
	Tripod.Yaw.angle_max = 190;
	Tripod.Yaw.angle_min = -65;	
	
	Tripod.Yaw.visualAngle = 0;
	Tripod.Yaw.angle_source.encoder.angle_offset = 0;
	Tripod.Yaw.angle_source.gyro.angle_offset = 0;
//	Yaw_MOTO.getpara.total_angle = YAWMIDANGLE/360.0f*8192;
}

void Pitch_Init(void)
{	
	Tripod.Pitch.angle_max = 92;
	Tripod.Pitch.angle_min = -145;

	Tripod.Pitch.visualAngle = 0;
	Tripod.Pitch.angle_source.encoder.angle_offset = 0;
	Tripod.Pitch.angle_source.gyro.angle_offset = 0;
//	Pitch_MOTO.getpara.total_angle = PITCH6020_OFFSET/360.0f*8192;
}

void Tripod_Init(void)
{
	Tripod.control_mode.now = Position;
	Tripod.Yaw.angle_source.source_now = From_Encoder;
	Tripod.Yaw.angle_source.source_last = From_Encoder;
	Pitch_Init();
	Yaw_Init();
}
