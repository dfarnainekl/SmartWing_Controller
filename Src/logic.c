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


volatile uint16_t pwm_in[4] = {1500, 1500, 1500, 1500};
uint32_t positions[4] = {0, 0, 0, 0};

void update_positions(bool noFilter)
{
	for(uint8_t i=0; i<4; i++)
	{
		uint32_t position = pwm_in[i];
		if(position < 1000) position = 1000;
		if(position > 2000) position = 2000;
		position -= 1000;
		if(noFilter) positions[i] = (uint32_t)((65535.0 * 150.0) * (position / 1000.0) + 0.5);
		else positions[i] = (uint32_t)(0.1 * ((65535.0 * 150.0) * (position / 1000.0) + 0.5) + 0.9 * (float)positions[i]);
	}
}

void logic_init(void)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	HAL_Delay(100);

	tmc6200_highLevel_init(1);
	tmc6200_highLevel_init(3);
	HAL_Delay(10);

	swdriver_setEnable(1, true);
	swdriver_setEnable(3, true);
	HAL_Delay(10);

	TMC4671_highLevel_init(1);
	TMC4671_highLevel_init(3);
	HAL_Delay(10);

	TMC4671_highLevel_initEncoder(1);
	TMC4671_highLevel_initEncoder(3);

	HAL_Delay(100);

	tmc4671_writeInt(1, TMC4671_PID_POSITION_ACTUAL, (uint32_t)((float)as5147_getAngleOtherEnc(1) * 32 * 231.1222));
	tmc4671_writeInt(3, TMC4671_PID_POSITION_ACTUAL, (uint32_t)((float)as5147_getAngleOtherEnc(3) * 32 * 231.1222));

	update_positions(true);
	TMC4671_highLevel_setPosition(1, positions[0]);
	TMC4671_highLevel_setPosition(3, positions[1]);

	TMC4671_highLevel_positionMode2(1);
	TMC4671_highLevel_positionMode2(3);

	HAL_Delay(100);
}

void logic_loop(void)
{
	update_positions(false);
	TMC4671_highLevel_setPosition(1, positions[0]);
	TMC4671_highLevel_setPosition(3, positions[1]);

	//if position close to target, turn off motor
	static uint16_t angle_correct_counter_1 = 0;
	if((abs((int32_t)TMC4671_highLevel_getPositionActual(1) - (int32_t)TMC4671_highLevel_getPositionTarget(1)) < 65535) && angle_correct_counter_1 < 65535)
		angle_correct_counter_1++;
	else
		angle_correct_counter_1 = 0;

	if(angle_correct_counter_1 > 500)
		swdriver_setEnable(1, false);
	else
		swdriver_setEnable(1, true);

	static uint16_t angle_correct_counter_3 = 0;
	if((abs((int32_t)TMC4671_highLevel_getPositionActual(3) - (int32_t)TMC4671_highLevel_getPositionTarget(3)) < 65535) && angle_correct_counter_3 < 65535)
		angle_correct_counter_3++;
	else
		angle_correct_counter_3 = 0;

	if(angle_correct_counter_3 > 500)
		swdriver_setEnable(3, false);
	else
		swdriver_setEnable(3, true);

	HAL_Delay(1);
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


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2) // pwm in
	{
		static uint32_t timestamp_risingEdge[4] = {0, 0, 0, 0};

		switch(htim->Channel)
		{
			case HAL_TIM_ACTIVE_CHANNEL_1:
				if(HAL_GPIO_ReadPin(PWM_IN_0_GPIO_Port, PWM_IN_0_Pin))
				{
					timestamp_risingEdge[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //rising edge
				}
				else
				{
					pwm_in[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - timestamp_risingEdge[0]; //falling edge
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				if(HAL_GPIO_ReadPin(PWM_IN_1_GPIO_Port, PWM_IN_1_Pin))
				{
					timestamp_risingEdge[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //rising edge
				}
				else
				{
					pwm_in[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) - timestamp_risingEdge[1]; //falling edge
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				if(HAL_GPIO_ReadPin(PWM_IN_2_GPIO_Port, PWM_IN_2_Pin))
				{
					timestamp_risingEdge[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); //rising edge
				}
				else
				{
					pwm_in[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - timestamp_risingEdge[2]; //falling edge
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				if(HAL_GPIO_ReadPin(PWM_IN_3_GPIO_Port, PWM_IN_3_Pin))
				{
					timestamp_risingEdge[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); //rising edge
				}
				else
				{
					pwm_in[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - timestamp_risingEdge[3]; //falling edge
				}
				break;
			default:
				break;
		}
	}
}
