#include "main.h"
#include "gpio.h"
#include "can.h"
#include "m_moto.h"
#include "control.h"
#include "robodata.h"

uint16_t bottom_cnt = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){//L2
		bottom_cnt++;
	}
	else if(GPIO_Pin == GPIO_PIN_2)	{//L1

	}
}
