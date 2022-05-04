#include "logic.h"
#include "swdriver.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "tmc4671/TMC4671.h"
#include "as5047U.h"
#include "as5147.h"
#include "tim.h"
#include <stdbool.h>
#include <math.h>


void logic_init(void)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	HAL_Delay(100);

	tmc6200_highLevel_init(1);
	HAL_Delay(10);

	swdriver_setEnable(1, true);
	HAL_Delay(10);

	TMC4671_highLevel_init(1);
	HAL_Delay(10);

	TMC4671_highLevel_initEncoder(1);

	HAL_Delay(100);

	TMC4671_highLevel_referenceEndStop(1); // also actiavtes position mode

	HAL_Delay(100);
}

void logic_loop(void)
{
	HAL_Delay(2500);
	TMC4671_highLevel_setPosition(1, -65535 * 3);
	HAL_Delay(2500);
	TMC4671_highLevel_setPosition(1, 0);

//	//if position close to target, turn off motor
//	static uint16_t angle_correct_counter_1 = 0;
//	if((abs((int32_t)TMC4671_highLevel_getPositionActual(1) - (int32_t)TMC4671_highLevel_getPositionTarget(1)) < 65535) && angle_correct_counter_1 < 65535)
//		angle_correct_counter_1++;
//	else
//		angle_correct_counter_1 = 0;
//
//	if(angle_correct_counter_1 > 500)
//		swdriver_setEnable(1, false);
//	else
//		swdriver_setEnable(1, true);
//
//	HAL_Delay(1);
}


void HAL_SYSTICK_Callback(void)
{
	// error led
	if(swdriver_getStatus(0) || swdriver_getStatus(1) || swdriver_getStatus(2) || swdriver_getStatus(3) ||
	   swdriver_getFault(0)  || swdriver_getFault(1)  || swdriver_getFault(2)  || swdriver_getFault(3)  )
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
}
