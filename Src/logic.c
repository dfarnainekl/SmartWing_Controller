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
#include "channels/servo_channel.h"
#include "boards/blmb.h"
#include "hardware/adc.h"
#include "channels/channels.h"
#include "hardware/can.h"
#include "channels/generic_channel.h"



#define NODE_ID 3


static AdcData_t *torque_ptr = NULL;

Node_t node = { .node_id = 0, .firmware_version = 0xdeadbeef,
		        .generic_channel = { NULL, NULL, NULL, NULL, DEFAULT_REFRESH_DIVIDER, DEFAULT_REFRESH_RATE },
				.channels =
				{
					{ 0, CHANNEL_TYPE_SERVO, {{0}} },
					{ 1, CHANNEL_TYPE_ADC16, {{0}} },
					{ 2, CHANNEL_TYPE_ADC16, {{0}} }
				}
			  };


void BLMB_LoadSettings(void)
{
	Servo_Channel_t *servo = &node.channels[BLMB_SERVO_CHANNEL].channel.servo;
	servo->startpoint = 0;
	servo->endpoint = 65535;
	servo->max_accel = 0;
	servo->max_speed = 0;
	servo->max_torque = 0;
	servo->p_param = 0;
	servo->i_param = 0;
	servo->d_param = 0;
	servo->pwm_in_enabled = 0;
	servo->refresh_divider = 0; // TODO ask about default setting
	servo->pressure_control_enabled = 0;
	servo->pos_p_param = 0;
	servo->pos_i_param = 0;
	servo->vel_p_param = 0;
	servo->vel_i_param = 0;
	servo->torq_p_param = 0;
	servo->torq_i_param = 0;

//	Serial_PutString("\r\nLoadSettings\r\nStartpoint: ");
//	Serial_PutInt(servo->startpoint);
//	Serial_PutString("\r\nEndpoint: ");
//	Serial_PrintInt(servo->endpoint);
}


uint32_t BLMB_CalcMotorPos(uint32_t var)
{
	return (uint32_t) (BLMB_REDUCTION * var);
}


Servo_Channel_t* servo;


void logic_init(void)
{
	node.node_id = NODE_ID;
	Can_Init(node.node_id);

	Servo_InitChannel(&node.channels[BLMB_SERVO_CHANNEL].channel.servo);
	node.channels[2].channel.adc16.analog_in = torque_ptr;
	BLMB_LoadSettings();
	servo = &node.channels[BLMB_SERVO_CHANNEL].channel.servo;

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

volatile static bool systick_tick = false;

void logic_loop(void)
{
//	HAL_Delay(2000);
//	TMC4671_highLevel_setPosition(1, 65535 * 4);
//	HAL_Delay(2000);
//	TMC4671_highLevel_setPosition(1, 0);


	Can_checkFifo(BLMB_MAIN_CAN_BUS);

	servo->position = TMC4671_highLevel_getPositionActual(1);
	Servo_GetRawData(BLMB_SERVO_CHANNEL, NULL); // sets servo->position_percentage

	TMC4671_highLevel_setPosition(1, BLMB_CalcMotorPos(servo->target_position));

	*torque_ptr = TMC4671_highLevel_getTorqueActual(1);


	//if position close to target, turn off motor FIXME
	if(systick_tick)
	{
		systick_tick = false;

		static uint16_t angle_correct_counter_1 = 0;

		if(abs(TMC4671_highLevel_getPositionActual(1) - TMC4671_highLevel_getPositionTarget(1)) < 2000)
		{
			if(angle_correct_counter_1 < 65535) angle_correct_counter_1++;
		}
		else
		{
			angle_correct_counter_1 = 0;
		}

		if(angle_correct_counter_1 > 1000)
			swdriver_setEnable(1, false);
		else
			swdriver_setEnable(1, true);
	}
}


void HAL_SYSTICK_Callback(void)
{
	systick_tick = true;

	// error led
	if(swdriver_getStatus(1) || swdriver_getFault(1))
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}

	static int32_t refresh_counter = 0;
	if(node.generic_channel.refresh_divider)
	{
		if(++refresh_counter >= (node.generic_channel.refresh_divider * 1000 / DEFAULT_REFRESH_RATE))
		{
			Generic_Data();
			refresh_counter = 0;
		}
	}
}
