#ifndef __TRIPOD_H
#define __TRIPOD_H

#include "stm32f4xx_hal.h"

#define PITCHHORANGLE	7900.0F	//炮口方向为前，以左侧电机为主动电机
#define YAWMIDANGLE 	3445.0F

typedef struct
{
	float angle_now;
	float angle_offset;
}AngleReference_t;

typedef enum
{
	From_Encoder,
	From_Gyro
}AngleSource_e;//运行状态

typedef enum
{
	Position=1,
	Speed=2,
	TripodFollowChassis=3,
	autoSight=4,
}ControlMode_e;//运行状态

typedef struct
{
	ControlMode_e now;
	ControlMode_e last;
}ControlMode_t;//运行状态


typedef struct
{
	AngleSource_e source_last;
	AngleSource_e source_now;
	AngleReference_t encoder;
	AngleReference_t gyro;
}AngleSource_t;//运行状态


typedef struct
{
	void (*Init)(void);
	void (*Fun)(float);
	//void (*Init)(void);
	float angle;
	float speed;
	float speed_set;
	float angle_set;
	float current_set;
	float angle_offset;
	float angle_max;
	float angle_min;
	uint8_t ready_flag;
	AngleSource_t angle_source;
	
	float visualAngle;
}Axis_t;

typedef struct
{
	void *Init;
	void *Fun;
	Axis_t Pitch;
	Axis_t Yaw;
	ControlMode_t control_mode;
}Tripod_t;//运行状态

extern Tripod_t Tripod;
extern float coupled_yaw_speed;

void Pitch_Init(void);
void Pitch_SetAngle(float pitch);

void Yaw_Init(void);
void Yaw_SetAngle(float pitch);
void GetYawAngle100Hz(void);
void Tripod_SetAngle(float yaw,float pitch);
void Tripod_SetSpeed(float speed_yaw,float speed_pitch);
void Yaw_SetSpeed(float speed);
void Pitch_SetSpeed(float speed);
void Tripod_Init(void);
float GetIntergralYaw(float yaw_speed,uint16_t frequency);
float GetCoupledYawSpeedFromPitch(float pitch, float yaw_speed_raw);
void Tripod_Fun(void);

//typedef struct
//{
//	float angle_now;
//	float angle_offset;
//	float angle_relativeToChassis;//相对于底盘的角度 正前方大装甲板中央为零点，逆时针为正
//	float speed_now;
//	uint8_t ready_flag;
//}DataReference_t;

//typedef enum
//{
//	From_Encoder,
//	From_Gyro
//}DataSource_e;//运行状态

//typedef enum
//{
//	Position = 1,
//	Speed
//}ControlMode_e;//运行状态

//typedef struct
//{
//	ControlMode_e now;
//	ControlMode_e last;
//}ControlMode_t;//运行状态

//typedef struct
//{
//	DataSource_e source_last;
//	DataSource_e source_now;
//	DataReference_t encoder;
//	DataReference_t gyro;
//}DataSource_t;//运行状态

//typedef struct
//{
////	float angle;
////	float speed;
//	float angle_set;
//	float speed_set;
////	float current_set;
//	float angle_offset;
//	float angle_max;
//	float angle_min;
////	uint8_t ready_flag;
//	DataSource_t data_source;
////	void (*Init)(void);
////	void (*Fun)(float);
//}Axis_t;

//typedef struct
//{
//	void *Init;
//	void *Fun;
//	Axis_t Pitch;
//	Axis_t Yaw;
////	ControlMode_t control_mode;
//}Tripod_t;//运行状态

//extern Tripod_t Tripod;

//void Tripod_Fun(void);
//void Tripod_Init(float source, float pitch_max, float pitch_min, float yaw_limit);

#endif
