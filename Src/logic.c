#include "logic.h"
#include "swdriver.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "as5147.h"
#include "usart.h"
#include "tim.h"
#include <stdbool.h>


volatile uint16_t systick_counter = 0;
volatile uint16_t systick_counter_2 = 0;

volatile uint8_t rx_byte_new = 0;
uint8_t rx_byte;

volatile uint16_t pwm_in[4] = {1500, 1500, 1500, 1500};
volatile bool pwm_updated = false;
volatile float angleOut[4] = {0, 0, 0, 0};

const char clear_string[7] = {27, '[', '2','J', 27, '[', 'H'};

void logic_init(void)
{
	uint8_t i;
	uint16_t angle[4];
	static char string[50];

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	for(i=0; i<4; i++) tmc6200_highLevel_init(i);
	HAL_Delay(10);

	for(i=0; i<4; i++) swdriver_setEnable(i, true);
	HAL_Delay(10);

	for(i=0; i<4; i++) TMC4671_highLevel_init(i);
	HAL_Delay(10);

	//TMC4671_highLevel_openLoopTest2(0);
	//TMC4671_highLevel_positionMode2(0);

	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	rx_byte_new = 0;
	// TODO: wait for button press to set zero positions
	while(1)
	{
			if(rx_byte_new &&rx_byte == 's')
			{
				rx_byte_new=0;
				break;
			}

			angle[0] = as5147_getAngle(0);
			angle[1] = as5147_getAngle(1);
			uint16_t len = snprintf(string, 50, "enc0: %d\tenc1: %d\n\r", (angle[0] << 5), (angle[1] << 5));
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, len);
			HAL_Delay(200);
	} //TODO: do-while()
	rx_byte_new = 0;

	TMC4671_highLevel_initEncoder_new(0);
	TMC4671_highLevel_positionMode_fluxTorqueRamp(0);
	TMC4671_highLevel_positionMode_rampToZero(0);
	TMC4671_highLevel_stoppedMode(0);
	HAL_Delay(100);
	TMC4671_highLevel_initEncoder_new(1);
	TMC4671_highLevel_positionMode_fluxTorqueRamp(1);
	TMC4671_highLevel_positionMode_rampToZero(1);

	TMC4671_highLevel_positionMode_fluxTorqueRamp(0);
	TMC4671_highLevel_positionMode_rampToZero(0);


	//HAL_Delay(1000);
	// for(i=0; i<2; i++)
	// {
	// 	TMC4671_highLevel_initEncoder_new(i);
	// 	TMC4671_highLevel_positionMode_fluxTorqueRamp(i);
	// 	TMC4671_highLevel_positionMode_rampToZero(i);
	// 	HAL_Delay(1000);
	// }


	// pwm inputs //FIXME: timer interrupt priority lower than spi?
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	// HAL_Delay(5000);
	// TMC4671_highLevel_pwmOff(3);
	//for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
	// for(i=0; i<4; i++)  TMC4671_highLevel_positionMode2(i);
}


void logic_loop(void)
{
	static int32_t positionTarget[4] = {0, 0, 0, 0};
	static float angleIn[4] = {0, 0, 0, 0};
	static uint8_t i;

	if(rx_byte_new)
	{
		rx_byte_new = 0;

		if(rx_byte == ' ')	//TODO: switch case
		{
				for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
				for(i=0; i<4; i++)	TMC4671_highLevel_pwmOff(i);
		}
		else if(rx_byte == 'f')	// toggle filters for target position
		{
				for(i=0; i<4; i++)	TMC4671_highLevel_togglePositionFilter(i);
		}
		else if(rx_byte == 'i')
		{
			for(i=0; i<4; i++)	TMC4671_highLevel_setCurrentLimit(i, 10000);
		}
	}


	if(pwm_updated)
	{
		pwm_updated = false;

		for(i=0; i<4; i++)
		{
			//positionTarget[i] = (int32_t)pwm_in[i] - 1500;
			angleIn[i] =  ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree
		}

		for(i=0; i<4; i++)	positionTarget[i] = clacAngle(i, angleIn);
		for(i=0; i<4; i++) 	TMC4671_highLevel_setPosition_nonBlocking(i, positionTarget[i]);
	}

	if(systick_counter >= 20) //50Hz
	{
		systick_counter = 0;
	}

	if(systick_counter_2 >= 200) //5Hz
	{
		systick_counter_2 = 0;
		// -------------------------------------------------------------------------
		static char string[128];
		// -------------------------------------------------------------------------
		// uint16_t angle0 = as5147_getAngle(0);
		// uint16_t angle1 = as5147_getAngle(1);
		// uint16_t len = snprintf(string, 128, "dr %d: enc11= %d enc16=%d\tdr %d: enc11= %d enc16=%d\n\r", 0, angle0, (angle0 << 5), 1, angle1, (angle1 << 5));
		// -------------------------------------------------------------------------
		// uint16_t angle0 = as5147_getAngle(0);
		//uint16_t len = snprintf(string, 128, "driver %d encoder angle: %d (11bit) %d (16bit)\n\r", 0, angle0, ((uint16_t)angle0 << 5));
		// -------------------------------------------------------------------------
		uint16_t len = snprintf(string, 128,
			"%s"
			"pwm_in:     %d %d %d %d\r\n"
			"angleIn:    % 2.1f % 2.1f % 2.1f % 2.1f\n\r"
			"angleOut:   % 2.1f % 2.1f % 2.1f % 2.1f\n\r\n\r", clear_string,
			pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3], angleIn[0], angleIn[1], angleIn[2], angleIn[3],
			angleOut[0], angleOut[1], angleOut[2], angleOut[3]);
		// -------------------------------------------------------------------------
		// uint16_t len = snprintf(string, 128, "(uint16_t)%d\t(int16_t)%d\t%d\t%d\t%d\n\r", ((uint16_t)angle << 5),(int16_t)(angle << 5), DRV0_OFFSET_ENC_PHIM, DRV0_OFFSET_ENC_PHIE, DRV0_OFFSET_PHIM_PHIE);
		// -------------------------------------------------------------------------
		// uint16_t len = snprintf(string, 128, "%d\n\r", tmc6200_readInt(1, 0x01));
		// -------------------------------------------------------------------------
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, len);
	}
}


void HAL_SYSTICK_Callback(void)
{
	systick_counter++;
	systick_counter_2++;

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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    rx_byte_new = 1;
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}


int32_t clacAngle(uint8_t drv, float *angleIn)
{
	if(drv == 0 || drv == 2)
	{
		angleOut[drv] = angleIn[drv] + angleIn[drv+1];
	}
	else if(drv == 1)
	{
		angleOut[drv]= 2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv];
	}

	return (int32_t)(angleOut[drv] * 2.0 / 360.0 * 65536.0);
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
					pwm_updated = true;
				}
				else
				{
					uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - timestamp_risingEdge[0]; //falling edge
					if(pwm < 1000) pwm = 1000;
					else if(pwm > 2000) pwm = 2000;
					pwm_in[0] = pwm;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				if(HAL_GPIO_ReadPin(PWM_IN_1_GPIO_Port, PWM_IN_1_Pin))
				{
					timestamp_risingEdge[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //rising edge
					pwm_updated = true;
				}
				else
				{
					uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) - timestamp_risingEdge[1]; //falling edge
					if(pwm < 1000) pwm = 1000;
					else if(pwm > 2000) pwm = 2000;
					pwm_in[1] = pwm;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				if(HAL_GPIO_ReadPin(PWM_IN_2_GPIO_Port, PWM_IN_2_Pin))
				{
					timestamp_risingEdge[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); //rising edge
					pwm_updated = true;
				}
				else
				{
					uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - timestamp_risingEdge[2]; //falling edge
					if(pwm < 1000) pwm = 1000;
					else if(pwm > 2000) pwm = 2000;
					pwm_in[2] = pwm;
				}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				if(HAL_GPIO_ReadPin(PWM_IN_3_GPIO_Port, PWM_IN_3_Pin))
				{
					timestamp_risingEdge[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); //rising edge
					pwm_updated = true;
				}
				else
				{
					uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - timestamp_risingEdge[3]; //falling edge
					if(pwm < 1000) pwm = 1000;
					else if(pwm > 2000) pwm = 2000;
					pwm_in[3] = pwm;
				}
				break;
			default:
				break;
		}
	}
}
