#include "logic.h"
#include "swdriver.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "as5147.h"
#include "usart.h"

void logic_init(void)
{
	HAL_Delay(100);

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	uint8_t i;
	for(i=0; i<4; i++) tmc6200_highLevel_init(i);

	for(i=0; i<4; i++) swdriver_setEnable(i, true);

	TMC4671_highLevel_init(3);
	//TMC4671_highLevel_openLoopTest(3);
	//TMC4671_highLevel_printOffsetAngle(3);
	//TMC4671_highLevel_torqueTest(3);

	TMC4671_highLevel_pwmOff(3);

	while(1)
	{
		char string[64];
		uint16_t angle = as5147_getAngle(3);
		uint16_t len = snprintf(string, 64, "driver %d encoder angle: %d (11bit) %d (16bit)\n", 3, angle, (int16_t)(angle << 5));
		HAL_UART_Transmit(&huart3, (uint8_t*)string, len, 100000000);
		HAL_Delay(100);
	}
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
