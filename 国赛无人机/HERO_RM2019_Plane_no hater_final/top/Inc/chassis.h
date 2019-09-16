#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "robodata.h"

#define POWER_LIMIT 100.0F
#define CHASSIS_MAX_CURRENT 6000.0F

typedef enum
{
	FRONT_FOLLOW=1,//�������
	NOT_FOLLOW,//������
	SWAY//ҡ��
}ChassisFollowMode_e;

typedef struct
{
	ChassisFollowMode_e now;
	ChassisFollowMode_e last;
}ChassisFollowMode_t;

typedef struct
{
	ChassisFollowMode_t mode;
	float angle;//��̨����ڵ��̽Ƕȣ���ʱ��
	float deadband;//��������
	float total_angle;//��ֹҡ�ڷ��
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
