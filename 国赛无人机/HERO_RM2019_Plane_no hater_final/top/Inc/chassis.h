#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "robodata.h"

#define POWER_LIMIT 100.0F
#define CHASSIS_MAX_CURRENT 6000.0F

typedef enum
{
	FRONT_FOLLOW=1,//正面跟随
	NOT_FOLLOW,//不跟随
	SWAY//摇摆
}ChassisFollowMode_e;

typedef struct
{
	ChassisFollowMode_e now;
	ChassisFollowMode_e last;
}ChassisFollowMode_t;

typedef struct
{
	ChassisFollowMode_t mode;
	float angle;//云台相对于底盘角度，逆时针
	float deadband;//跟随死区
	float total_angle;//防止摇摆疯狂
}ChassisFollow_t;

typedef struct
{
	float now[4];		
	float set[4];		
	float spd_forward;
	float spd_right;	
	float spd_yaw;		
	float spd_yaw_gyro;
	float spd_spin_perunit;
	float base;
}ChassisSpeed_t;

typedef struct
{
	ChassisSpeed_t speed;
	ChassisFollow_t follow;	
}Chassis_t;

extern Chassis_t Chassis;

void Chassis_Init(float speed_base, float angle_deadband, float mode);
void Chassis_Fun(void);

#endif
