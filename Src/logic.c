#include "logic.h"
#include "swdriver.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"


void logic_init(void)
{
	HAL_Delay(100);

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	uint8_t i;
	for(i=0; i<4; i++) tmc6200_highLevel_init(i);

	for(i=0; i<4; i++) swdriver_setEnable(i, true);

	TMC4671_highLevel_openLoopTest(3);
}


void logic_loop(void)
{

}


void HAL_SYSTICK_Callback(void)
{
	if(swdriver_getStatus(0) || swdriver_getStatus(1) || swdriver_getStatus(2) || swdriver_getStatus(3) ||
	   swdriver_getFault(0) || swdriver_getFault(1) || swdriver_getFault(2) || swdriver_getFault(3))
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
}
