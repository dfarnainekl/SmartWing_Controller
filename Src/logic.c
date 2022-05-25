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

#define CLOSE_OFFSET (65535 * 5)


volatile static bool systick_tick = false;


static AdcData_t torque = 0;
static AdcData_t pressure = 0;

Servo_Channel_t* servo;

Node_t node = { .node_id = NODE_ID, .firmware_version = 0xdeadbeef,
		        .generic_channel = { NULL, NULL, NULL, NULL, DEFAULT_REFRESH_DIVIDER, DEFAULT_REFRESH_RATE },
				.channels =
				{
					{ 0, CHANNEL_TYPE_SERVO, {{0}} },
					{ 1, CHANNEL_TYPE_ADC16, {{0}} },
					{ 2, CHANNEL_TYPE_ADC16, {{0}} }
				}
			  };


void logic_init(void)
{
	Can_Init(node.node_id);

	servo = &node.channels[BLMB_SERVO_CHANNEL].channel.servo;

	servo->startpoint = 0;
	servo->endpoint = 65535;
	servo->max_accel = 0;
	servo->max_speed = 0;
	servo->max_torque = 0;
	servo->p_param = 0;
	servo->i_param = 0;
	servo->d_param = 0;
	servo->pwm_in_enabled = 0;
	servo->refresh_divider = 0;
	servo->pressure_control_enabled = 0; // TODO: use this as well as target_pressure, deactivate or modify existing pressure control
	servo->pos_p_param = 0;
	servo->pos_i_param = 0;
	servo->vel_p_param = 0;
	servo->vel_i_param = 0;
	servo->torq_p_param = 0;
	servo->torq_i_param = 0;

	node.channels[1].channel.adc16.analog_in = &pressure;
	node.channels[2].channel.adc16.analog_in = &torque;

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

	TMC4671_highLevel_referenceEndStop(1); // also activates position mode

	HAL_Delay(500);

//	tmc4671_writeInt(1, TMC4671_PID_VELOCITY_LIMIT, 1000);
}


void logic_loop(void)
{
	Can_checkFifo(BLMB_MAIN_CAN_BUS);

	servo->position = (CLOSE_OFFSET - TMC4671_highLevel_getPositionActual(1)) / BLMB_REDUCTION / 4;

	Servo_GetRawData(BLMB_SERVO_CHANNEL, NULL); // sets servo->position_percentage

	TMC4671_highLevel_setPosition(1, CLOSE_OFFSET - servo->target_position * BLMB_REDUCTION);

	torque = (uint16_t)((int32_t)TMC4671_highLevel_getTorqueActual(1) - INT16_MIN);

	//if position close to target, turn off motor
	if(systick_tick)
	{
		systick_tick = false;

		static uint16_t angle_correct_counter_1 = 0;

		if(abs(TMC4671_highLevel_getPositionActual(1) - TMC4671_highLevel_getPositionTarget(1)) < 1000)
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
		if(++refresh_counter >= (node.generic_channel.refresh_divider * 1000 / node.generic_channel.refresh_rate))
		{
			Generic_Data();
			refresh_counter = 0;
		}
	}
}
