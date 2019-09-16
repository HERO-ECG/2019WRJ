#include "strike.h"
#include "tripod.h"
#include "robodata.h"
#include "m_moto.h"
#include "math.h"
#include "tx2_Protocol.h"
#include "tim.h"

Strike_t Strike;
uint16_t speedst = 2000;
uint32_t FrictionMotorServo(uint32_t setspeed)
{
	static uint32_t speed_now=400;
	if(speed_now<setspeed)
		speed_now++;
	if(speed_now>setspeed)
		speed_now--;
	if(speed_now<1000)
		return 1000;
	return speed_now;
}

uint32_t FrictionMotorServo2(uint32_t setspeed)
{
	static uint32_t speed_now2=400;
	if(speed_now2<setspeed)
		speed_now2++;
	if(speed_now2>setspeed)
		speed_now2--;
	if(speed_now2<1000)
		return 1000;
	return speed_now2;
}

extern uint16_t bottom_cnt;
float mocalun_dengdaishijian=0;
float mocalun_dengdaishijian_flag=0;
float friction_state_last=0;
float mocalun_qidong_pending;
uint8_t mocalun_qidong_pending_flag;
float last_timccr;
uint8_t Bodan_Help_Flag;
uint8_t qiangxingqidong;
uint8_t yanshi_flag;
float yanshi;
float Bodan_Help;
float set_TGspeed = 0;

float aaatest=1000;
	uint8_t little_order = 0;
uint8_t  friction_state = 0;
float TGspeed;
float shoot_monitor;
char shoot_flag;
float shoot_pending;
/*发射小弹丸,速度环连发,预供弹*/
void little_strike_fun(uint16_t switch_count, uint8_t shoot_order, float TGspeed, uint8_t friction_state, uint16_t frictionMotoSpeed)
{

//	if(switch_count == 0 || (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0) == GPIO_PIN_RESET))
//	{
//		set_TGspeed = TGspeed;
//	}
//	else
//	{
		if((shoot_order == 1 && friction_state == 1)||qiangxingqidong==1){
			set_TGspeed = TGspeed;
		}else{
			set_TGspeed = 0;
		}	
//	}
	/*小摩擦轮速度*/  
	if(friction_state == 1)
	{
		if(frictionMotoSpeed>=1000 || frictionMotoSpeed<=2000)
		{
////		HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);
////		HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_2);
			//这里是先启动一个摩擦轮，另一个延时启动，使用效果不好
//		TIM2->CCR1 = 1000;

//		TIM2->CCR2 = 1000;
//			if(friction_state_last==0||friction_state_last==2)
//			{mocalun_dengdaishijian_flag=1;}
//		if(mocalun_dengdaishijian >400	)
//			{

//		mocalun_dengdaishijian_flag=2;
//				
//			}
//			if(mocalun_dengdaishijian_flag==2)
//			{
//								mocalun_dengdaishijian=0;
//			mocalun_qidong_pending_flag=1;
					TIM2->CCR1 = frictionMotoSpeed;//FrictionMotorServo(frictionMotoSpeed);
//			if(mocalun_qidong_pending>100)
					TIM2->CCR2 = frictionMotoSpeed;//frictionMotoSpeed;//FrictionMotorServo(frictionMotoSpeed);
			if(last_timccr==1000)
				{
				
					yanshi_flag=1;
				
				}
				if(yanshi_flag==2)
				{
				qiangxingqidong=1;
				Bodan_Help_Flag=1;
					yanshi_flag=3;
				}
				if(Bodan_Help>210)
				{
					
					Bodan_Help_Flag=2;
					qiangxingqidong=0;
				}
//			}
		}
	}
	else if(friction_state == 2)
	{
//		mocalun_dengdaishijian_flag=0;
		Bodan_Help_Flag=0;
		TIM2->CCR1 = 1000;
		TIM2->CCR2 = 1000;	
//		HAL_TIM_OC_Stop(&htim2,TIM_CHANNEL_1);
//		HAL_TIM_OC_Stop(&htim2,TIM_CHANNEL_2);
	
	}
		else if(friction_state == 0)
	{
		Bodan_Help_Flag=0;
		TIM2->CCR1 = 1000;
		TIM2->CCR2 = 1000;	
	
	}
	
	last_timccr=TIM2->CCR1;
		/*小拨弹速度环*/
	Ammunition_MOTO.send_current = Ammunition_MOTO.pid_speed.PidCalc(&Ammunition_MOTO.pid_speed, Ammunition_MOTO.getpara.speed_rpm, set_TGspeed);
	friction_state_last=friction_state;
}




void Strike_fun(void)
{
	static uint8_t key_e_now = 0, key_e_last = 0;

	if(RoboData.robo_ctrlmode.ctrl_source==FROM_PC)
	{
		if(RC_CtrlData.mouse.press_1==1)
		{
			little_order = 1;
		}
		else little_order = 0;
		if(RC_CtrlData.key.key_data.E == 1&&RC_CtrlData.key.key_data.ctrl!=1 )	
			friction_state = 1;
		else if(RC_CtrlData.key.key_data.E == 1&&RC_CtrlData.key.key_data.ctrl==1 )
			friction_state = 2;
//		if(RC_CtrlData.mouse.press_2==1)
//		{

//		}	
  }
	else if(RoboData.robo_ctrlmode.ctrl_source==FROM_REMOTE)
	{
		if(RoboData.robo_ctrlmode.left_mode == LEFT_UP)
		{
      little_order = 1;
		}
		else 
		{
			little_order =0;
		}
		if(RoboData.robo_ctrlmode.left_mode == LEFT_DOWN)
		{
			key_e_now = 1;
			if(key_e_now && (!key_e_last))
			{
				if(friction_state==2)
				{
					friction_state = 1;
				}
				else if(friction_state==0)
				friction_state = 1;
				else
					friction_state=2;
			}
		}
		else key_e_now = 0;
	}
	key_e_last = key_e_now;
		if(RC_CtrlData.key.key_data.S==1)
			TGspeed=4320;
		else 
			TGspeed=7200;
		if(shoot_monitor>200)
		{
			friction_state=2;
			shoot_flag=0;
		}
		
	if(RoboData.robo_ctrlmode.ctrl_source==FROM_PC)//PC模式 发射机构上电后自动开启摩擦轮 2.5秒摩擦轮初始化过后 拨弹助推发射
	{
	 if(shoot_monitor<10)
		{
			shoot_flag=1;
		}
		if(shoot_pending>250)//&&(friction_state==0||friction_state==2))
		{	
			friction_state=1;
			shoot_pending=0;
		}
	}
	little_strike_fun(bottom_cnt, little_order, TGspeed, friction_state, 1335);//有预供弹,littleTG_start==1时发弹
}
