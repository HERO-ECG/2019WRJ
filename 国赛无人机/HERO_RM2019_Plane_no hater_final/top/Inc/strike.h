#ifndef __STRIKE_H
#define __STRIKE_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "m_moto.h"
#include "robodata.h"

#define	KEY_FIREMODE	RC_CtrlData.key.key_data.Q
#define KEY_SEMIMODE  RC_CtrlData.key.key_data.E
#define	KEY_ADDSTATE	RC_CtrlData.key.key_data.R
#define	KEY_AUTOAIM		RC_CtrlData.mouse.press_2
#define	KEY_FIRE			RC_CtrlData.mouse.press_1
#define friction_of_windcar 1170
#define friction_of_normal 1070

typedef enum
{
	ManualAim_ManualStrike,
	ManualAim_AutoStrike,
	AutoAim_ManualStrike,
	AutoAim_AutoStrike,
	WindMill_ManualStrike,
	windmill_AutoStrike,
}StrikeMode_e;//运行状态

typedef enum
{
	OnTarget,
	OffTarget
}AimState_e;//运行状态

typedef	struct
{
	float	delta_pitch;
	float	delta_yaw;
}DeltaAngle_t;

typedef struct
{
	DeltaAngle_t deltaangle_now;	
	DeltaAngle_t deltaangle_last;
	float OnTargetAngleMax;
	uint8_t		autoaim_flag;
	AimState_e aimstate;
}Vision_t;

typedef enum
{
	Semi_Mode,
	Burst_Mode,
	Auto_Mode
}FireMode_e;

typedef struct
{
	FireMode_e now;
	FireMode_e last;
}FireMode_t;

typedef struct
{
	uint8_t now;
	uint8_t last;
}Fire_t;

typedef struct
{
	uint16_t all;
	uint16_t burst;
}Counter_t;

typedef struct
{
	Fire_t fire;
	Counter_t count;
	Vision_t	vision;
	uint16_t	shootspd;
	FireMode_t	firemode;
	StrikeMode_e	strikemode;
}Strike_t;

extern Strike_t Strike;
extern float shoot_monitor;
extern char shoot_flag;
extern float shoot_pending;
extern float mocalun_qidong_pending;
extern uint8_t mocalun_qidong_pending_flag;
extern uint8_t Bodan_Help_Flag;
extern float Bodan_Help;
extern uint8_t yanshi_flag;
extern float yanshi;
extern uint8_t qiangxingqidong;
extern void Strike_fun(void);

#endif
