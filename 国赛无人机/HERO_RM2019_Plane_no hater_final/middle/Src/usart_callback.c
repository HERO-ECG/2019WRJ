#include "main.h"
#include "usart.h"
#include "tripod.h"
#include "dji_Protocol.h"
#include "tx2_Protocol.h"
#include "t_monitor.h"
#include "hwt901b.h"
#include "m_remote.h"
#include "m_moto.h"
#include "t_moto.h"

extern float	pitchL_angle_test;
extern float	pitchR_angle_test;
extern float yaw_angle_test;
float debug_pitch_up = 0.04;
float debug_pitch_down = 0.04;
float debug_yaw_left = 0.05;
float debug_yaw_down = 0.05;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){		
		monitor_remote.circle_number = 0;
		RemoteDataProcess(uart1_rx_buff, &RC_CtrlData);	
		HAL_UART_Receive_DMA(&huart1,uart1_rx_buff,len_uart1_rx_buff); 
	}
	else if(huart == &huart2){

		HAL_UART_Receive_DMA(&huart2,uart2_rx_buff,len_uart2_rx_buff); 
	}
	else if(huart == &huart3){
		monitor_tx2.circle_number = 0;
		TX2_DataProcess(uart3_rx_buff, &tx2_ReadData);
		HAL_UART_Receive_DMA(&huart3,uart3_rx_buff,len_uart3_rx_buff);
	}
	else if(huart == &huart6){
		HWT901DataProcess(uart6_rx_buff,len_uart6_rx_buff);
//		Tripod.Pitch.speed = -ANGLE.Angular_Velocity[0];
//		if(ANGLE.Angle[0]<0)
//		Tripod.Pitch.angle_source.gyro.angle_now = -180-ANGLE.Angle[0];
//		else
//		Tripod.Pitch.angle_source.gyro.angle_now = 180-ANGLE.Angle[0];
		
//		Tripod.Pitch.angle = Tripod.Pitch.angle_source.gyro.angle_now;
//		Tripod.Pitch.ready_flag = 1;
		HAL_UART_Receive_DMA(&huart6,uart6_rx_buff,len_uart6_rx_buff);
	}
	else if(huart == &huart7){
		TX2_DataProcess(uart7_rx_buff, &tx2_ReadData);
		HAL_UART_Receive_DMA(&huart7,uart7_rx_buff,len_uart7_rx_buff);
	}
	else if(huart == &huart8){
		dji_DataProcess(uart8_rx_buff, &DJI_ReadData, &RoboData);
		HAL_UART_Receive_DMA(&huart8,uart8_rx_buff,len_uart8_rx_buff);
	}
}

