#include "logic.h"
#include "swdriver.h"
#include "tmc6200/TMC6200.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "tmc4671/TMC4671.h"
#include "as5047U.h"
#include "usart.h"
#include "tim.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define TA				0.001
#define DATA_N		1024

#define MATLAB 0

#define MODE_STOP								0
#define MODE_TORQUE							1
#define MODE_VELOCITY						2
#define MODE_POSITION						3
#define MODE_IDENTIFY_VELOCITY 	4
#define MODE_CHIRP_TORQUE				5
#define MODE_VELOCITY_STEP			6
#define MODE_POSITION_STEP			7

uint8_t mode = MODE_STOP;


sweep_t sweep = {	.Ta = TA,
									.N = DATA_N,
									.U = 1,
									.omegaStart = 2*M_PI*50,
									.omegaEnd = 2*M_PI*500,
									.k = 0,
									.mode = AIL
								};

__attribute__((section(".ram_d1"))) data1_t data1[DATA_N] = {0};
__attribute__((section(".ram_d2"))) data2_t data2[DATA_N] = {0};

static variables_t motor_data[4] = {0};
static control_t   motor_control[4] = {0};

volatile uint16_t systick_counter = 0;
volatile uint16_t systick_counter_2 = 0;
volatile uint8_t 	systick_counter_3 = 0;

volatile uint8_t rx_byte_new = 0;
uint8_t rx_byte;

volatile uint16_t pwm_in[4] = {1500, 1500, 1500, 1500};
volatile bool pwm_updated = false;
volatile float angleOut[4] = {0, 0, 0, 0};
uint16_t angle[4];





bool stats = true;
uint16_t integral_pos = 0;
uint16_t integral_vel = 500;
uint16_t proportional_pos = 600;
uint16_t proportional_vel = 8000;

bool current = false;

#if MATLAB
	char clear_string[1] = {'\0'};
#else
	char clear_string[8] = {27, '[', '2','J', 27, '[', 'H', '\0'};
#endif

void logic_init(void)
{
	uint8_t i;
	static char string[500];
	static bool button_init=true;

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	for(i=0; i<4; i++) tmc6200_highLevel_init(i);
	HAL_Delay(10);

	for(i=0; i<4; i++) swdriver_setEnable(i, true);
	HAL_Delay(10);

	for(i=0; i<4; i++) TMC4671_highLevel_init(i);
	HAL_Delay(10);

	for(i=0; i<4; i++) as5047U_setABIResolution14Bit(i); //FIXME
	for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
	// for(i=0; i<4; i++) 	TMC4671_highLevel_pwmOff(i);

	//TMC4671_highLevel_openLoopTest2(2, 120);

	 HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	 rx_byte_new = 0;
	 while( !(rx_byte_new && rx_byte == 's') && button_init == true)
	 {
	 	button_init = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		#if MATLAB == 0
		static uint16_t adcRaw0[4];
		static uint16_t adcRaw1[4];
		for(i=2; i<4; i++)	angle[i] = as5047U_getAngle(i); //FIXME
		//angle[1] = as5047U_getAngle(1);
		// angle[3] = as5047U_getAngle(3);
	 	for(i=0; i<4; i++)	adcRaw0[i] = TMC4671_getAdcRaw0(i);
	 	for(i=0; i<4; i++)	adcRaw1[i] = TMC4671_getAdcRaw1(i);
		snprintf(string, 500, 	"%s", clear_string);
		snprintf(string+strlen(string), 500, "enc[0]: %5d\tenc[1]: %5d\tenc[2]: %5d\tenc[3]: %5d \n", angle[0], angle[1], angle[2], angle[3] );
		snprintf(string+strlen(string), 500, "adcRaw0[0]: %5d\tadcRaw1[0]: %5d\n", adcRaw0[0], adcRaw1[0] );
		snprintf(string+strlen(string), 500, "adcRaw0[1]: %5d\tadcRaw1[1]: %5d\n", adcRaw0[1], adcRaw1[1] );
		snprintf(string+strlen(string), 500, "adcRaw0[2]: %5d\tadcRaw1[2]: %5d\n", adcRaw0[2], adcRaw1[2] );
		snprintf(string+strlen(string), 500, "adcRaw0[3]: %5d\tadcRaw1[3]: %5d\n", adcRaw0[3], adcRaw1[3] );
		snprintf(string+strlen(string), 500, "%d\t%d\t%d\t%d\n", tmc6200_readInt(0, 0x01), tmc6200_readInt(1, 0x01), tmc6200_readInt(2, 0x01), tmc6200_readInt(3, 0x01));
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
		#endif
	 }
	 rx_byte_new = 0;
	HAL_Delay(100);

	 // for(i=0; i<4; i++) 	TMC4671_highLevel_pwmOff(i);
	 // while(1);

	motor_control[2].velocityP = 10;
	motor_control[2].velocityI = 10;

	motor_control[2].positionP = 0.5;
	motor_control[2].positionI = 0;

	// initialize right wing
	TMC4671_highLevel_initEncoder_new(2);
	TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	TMC4671_highLevel_positionMode_rampToZero(2);

	TMC4671_highLevel_pwmOff(0);
	TMC4671_highLevel_pwmOff(1);
	TMC4671_highLevel_pwmOff(3);



	// pwm inputs //FIXME: timer interrupt priority lower than spi?
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	for(i=0; i<4; i++)	TMC4671_highLevel_setPositionFilter(i, false);
	//for(i=0; i<4; i++)	TMC4671_highLevel_setCurrentLimit(i, 15000);



	#if MATLAB
		snprintf(string, 500, 	"fininit\n");
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
	#endif
}


void logic_loop(void)
{
	static char string[1500];
	static uint32_t i = 0;



/* -------------------------------------------------------------------------  */
	static bool button_stop=true;
	button_stop = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	if(button_stop == false)
	{
		for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
		for(i=0; i<4; i++)	TMC4671_highLevel_pwmOff(i);
	}
/* -------------------------------------------------------------------------  */
	if(pwm_updated)
	{
		pwm_updated = false;

		// for(i=0; i<4; i++) 	angleIn[i] =  ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree
		// for(i=0; i<4; i++)	positionTarget[i] = clacAngle(i, angleIn);
		// for(i=0; i<4; i++) 	TMC4671_highLevel_setPosition_nonBlocking(i, positionTarget[i]);
	}
/* -------------------------------------------------------------------------  */
	if(rx_byte_new)
	{
		rx_byte_new = 0;

		switch(rx_byte)
		{
			case ' ':
				for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
				for(i=0; i<4; i++)	TMC4671_highLevel_pwmOff(i);
				break;

			case 'a':
				stats = false;
				break;
			case 'y':
				stats = true;
				break;
			case 'f':
				for(i=0; i<4; i++)	TMC4671_highLevel_setPositionFilter(i, true);
				break;

			case 'c':
				if(current)
				{
					for(i=0; i<4; i++)	TMC4671_highLevel_setCurrentLimit(i, 5000);
					current = false;
				}
				else
				{
					for(i=0; i<4; i++)	TMC4671_highLevel_setCurrentLimit(i, 15000);
					current = true;
				}
				break;

			case '0':
				mode = MODE_STOP;
				stats = true;
				tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_STOPPED);
				break;

			case '1':
				mode = MODE_TORQUE;
				motor_data[2].torqueTarget = 2000;
				tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
				break;

			case '2':
				mode = MODE_VELOCITY;
				stats = false;
				motor_data[2].velocityTarget = 1000;
				motor_data[2].torqueTarget = 0;
				motor_control[2].velocityIntegratorValue = 0;
				motor_control[2].velocityIntegratorLimit = 5000;
				tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
				break;

			case '3':
				mode = MODE_POSITION;
				stats = false;
				motor_data[2].positionTarget = 0;
				motor_data[2].torqueTarget = 0;
				motor_data[2].velocityTarget = 0;
				motor_control[2].velocityIntegratorValue = 0;
				motor_control[2].velocityIntegratorLimit = 5000;
				motor_control[2].positionIntegratorValue = 0;
				motor_control[2].positionIntegratorLimit = 10000;
				tmc4671_writeInt(2, TMC4671_PID_POSITION_ACTUAL, 0 );
				tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
				break;

			case '4':
					mode = MODE_IDENTIFY_VELOCITY;
					sweep.k = 0;
					sweep.U = 5000;
					stats = false;
					motor_data[2].velocityTarget = 0;
					tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
					break;

			case '5':
				mode = MODE_CHIRP_TORQUE;
				sweep.k = 0;
				sweep.U = 5000;
				stats = false;
				tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
				break;

			case '6':
				mode = MODE_VELOCITY_STEP;
				stats = false;
				sweep.k = 0;
				motor_data[2].velocityTarget = 0;
				motor_data[2].torqueTarget = 0;
				sweep.U = 2000;
				motor_control[2].velocityIntegratorValue = 0;
				motor_control[2].velocityIntegratorLimit = 5000;
				tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
				break;

			case '+':
				//motor_data[2].torqueTarget += 500;
				//motor_data[2].velocityTarget += 200;
				motor_data[2].positionTarget += 20000;
				break;

			case '-':
				//motor_data[2].torqueTarget -= 500;
				//motor_data[2].velocityTarget -= 200;
				motor_data[2].positionTarget -= 20000;
				break;

			case 'g':
				if(motor_control[2].velocityP>=1)
					motor_control[2].velocityP -= 1;
				else
					motor_control[2].velocityP = 0;
				break;

			case 't':
				motor_control[2].velocityP += 1;
				break;

			case 'j':
				if(motor_control[2].velocityI>=5)
					motor_control[2].velocityI-= 5;
				else
					motor_control[2].velocityI = 0;
				break;

			case 'u':
				motor_control[2].velocityI += 5;
				break;

			case 'h':
				if(motor_control[2].positionP>=0.1)
					motor_control[2].positionP-= 0.1;
				else
					motor_control[2].positionP = 0;
				break;

			case 'z':
				motor_control[2].positionP += 0.1;
				break;

			case 'k':
				if(motor_control[2].positionI>=1)
					motor_control[2].positionI-= 1;
				else
					motor_control[2].positionI = 0;
				break;

			case 'i':
				motor_control[2].positionI += 1;
				break;

			case 'l': // print data
				snprintf(string, 1500, "%s",   clear_string);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				HAL_Delay(2);

				for(i=0; i<DATA_N; i++)
				{
					snprintf(string, 1500, "%ld;",  i);
					snprintf(string+strlen(string), 1500-strlen(string), "%d;",  data1[i].torqueTarget[2]);
					snprintf(string+strlen(string), 1500-strlen(string), "%d;",  data1[i].torqueActual[2]);
					snprintf(string+strlen(string), 1500-strlen(string), "%ld;", data1[i].velocityActual[2]);
					snprintf(string+strlen(string), 1500-strlen(string), "%ld;", data1[i].velocityTarget[2]);
					snprintf(string+strlen(string), 1500-strlen(string), "%ld\n", data1[i].velocityIntegratorValue[2]);
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
					HAL_Delay(2);
				}
				#if MATLAB
					snprintf(string, 200, 	"finplot\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				#endif
				break;

			case 'r':
				for(i=0; i<4; i++)	angle[i] = as5047U_getAngle(i);

				snprintf(string, 1500, 	"%s", clear_string);
				//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(0));
				//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(1));
				snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(2));
				snprintf(string+strlen(string), 1500, "Velocity P %.1f\n",  motor_control[2].velocityP);
				snprintf(string+strlen(string), 1500, "Velocity I %.1f\n",  motor_control[2].velocityI);
				snprintf(string+strlen(string), 1500, "Position P %.1f\n",  motor_control[2].positionP);
				snprintf(string+strlen(string), 1500, "Positino I %.1f\n",  motor_control[2].positionI);
				//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(3));
				// snprintf(string+strlen(string), 1500, "pwm_in:     %d %d %d %d\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
				// snprintf(string+strlen(string), 1500, "angleIn:    % 2.1f % 2.1f % 2.1f % 2.1f\n", angleIn[0], angleIn[1], angleIn[2], angleIn[3]);
				// snprintf(string+strlen(string), 1500, "angleOut:   % 2.1f % 2.1f % 2.1f % 2.1f\n", angleOut[0], angleOut[1], angleOut[2], angleOut[3]);
				snprintf(string+strlen(string), 1500, "---------------------------\n");
				snprintf(string+strlen(string), 1500, "enc0: %5d\tenc1: %5d\tenc2: %5d\tenc3: %5d\n", angle[0], angle[1], angle[2], angle[3]);
				snprintf(string+strlen(string), 1500, "---------------------------\n");
				snprintf(string+strlen(string), 1500, "[o] ... stopped mode\n[p] ... position mode\n[SPACE] ... STOP\n");
				snprintf(string+strlen(string), 1500, "fin\n");
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				break;


			default:
				break;
		} // end of: switch(rx_byte)
	} // end of: if(rx_byte_new)
/* -------------------------------------------------------------------------  */



if(systick_counter) //1ms
{
	systick_counter = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

 	//motor_data[2].SensorPos = as5047U_getAngle_fast(2);
	// motor_data[2].SensorPos = as5047U_getAngle(2);
	// motor_data[2].SensorVel = as5047U_getVelocity(2);

	motor_data[2].torqueActual = tmc4671_getActualTorque_raw(2);
	motor_data[2].positionActual = tmc4671_getActualPosition(2);
	motor_data[2].velocityActual = tmc4671_getActualVelocity(2);


	if(mode == MODE_STOP)
	{
		tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_STOPPED);
	}
	else if(mode == MODE_TORQUE)
	{
		tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
	}
	else if(mode == MODE_VELOCITY || mode == MODE_VELOCITY_STEP)
	{
		if(mode == MODE_VELOCITY_STEP)
		{
			if(sweep.k == 0)
			{
				motor_data[2].velocityTarget = 0;
			}
			else if(sweep.k == 100)
			{
				motor_data[2].velocityTarget = sweep.U;
			}
			else if(sweep.k == 3000)
			{
				motor_data[2].velocityTarget = 0;
			}
			else if(sweep.k >= (DATA_N-1) ) // stop
			{
				motor_data[2].velocityTarget = 0;
				stats = true;
				mode = MODE_STOP;
				#if MATLAB
					snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				#endif
			}
		}


		velocityPI();

		tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);

		if(mode == MODE_VELOCITY_STEP)
		{
			data1[sweep.k].torqueTarget[2] 	= motor_data[2].torqueTarget;
			data1[sweep.k].torqueActual[2] 	= motor_data[2].torqueActual;
			data1[sweep.k].velocityActual[2] = motor_data[2].velocityActual;
			data1[sweep.k].velocityTarget[2] = motor_data[2].velocityTarget;
			data1[sweep.k].velocityIntegratorValue[2] = (int32_t)motor_control[2].velocityIntegratorValue;
			sweep.k++;
		}
	}
	else if(mode == MODE_POSITION || mode == MODE_POSITION_STEP)
	{
		if(mode == MODE_POSITION_STEP)
		{
			if(sweep.k == 0)
			{
				motor_data[2].positionTarget = 0;
			}
			else if(sweep.k == 100)
			{
				motor_data[2].positionTarget = sweep.U;
			}
			else if(sweep.k == 3000)
			{
				motor_data[2].positionTarget = 0;
			}
			else if(sweep.k >= (DATA_N-1) ) // stop
			{
				motor_data[2].positionTarget = 0;
				stats = true;
				mode = MODE_STOP;
				#if MATLAB
					snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				#endif
			}
		}

		positionPI();
		velocityPI();

		tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);

		if(mode == MODE_POSITION_STEP)
		{
			data1[sweep.k].torqueTarget[2] 	= motor_data[2].torqueTarget;
			data1[sweep.k].torqueActual[2] 	= motor_data[2].torqueActual;
			data1[sweep.k].velocityActual[2] = motor_data[2].velocityActual;
			data1[sweep.k].velocityTarget[2] = motor_data[2].velocityTarget;
			data1[sweep.k].velocityIntegratorValue[2] = (int32_t)motor_control[2].velocityIntegratorValue;

			data2[sweep.k].positionActual = motor_data[2].positionActual;
			data2[sweep.k].positionTarget = motor_data[2].positionTarget;
			data2[sweep.k].positionIntegratorValue = (int32_t)motor_control[2].positionIntegratorValue;
			sweep.k++;
		}
	}
	else if(mode == MODE_IDENTIFY_VELOCITY)
	{

		if(sweep.k == 0)
		{
				motor_data[2].torqueTarget = 0;
		}
		else if(sweep.k == 100)
		{
			motor_data[2].torqueTarget =	sweep.U;
		}
		else if(sweep.k == 500)
		{
			motor_data[2].torqueTarget = 0;
		}
		else if(sweep.k >= (DATA_N-1) ) // stop
		{
			motor_data[2].torqueTarget = 0;
			mode = MODE_STOP;
			stats = true;

			#if MATLAB
				snprintf(string, 200, 	"fintest\n");
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
			#endif

		}
		tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);


		data1[sweep.k].torqueTarget[2] 	= motor_data[2].torqueTarget;
		data1[sweep.k].torqueActual[2] 	= motor_data[2].torqueActual;
		data1[sweep.k].velocityActual[2] = motor_data[2].velocityActual;
		data1[sweep.k].velocityTarget[2] = motor_data[2].velocityTarget;
		sweep.k++;
	}
	else if(mode == MODE_CHIRP_TORQUE)
	{

		sweep.t = sweep.k * sweep.Ta;
		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
		motor_data[2].torqueTarget = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t	+ (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

		if(sweep.k >= (DATA_N-1) ) // stop
		{
			mode = MODE_STOP;
			stats = true;
			sweep.k = 0;

			motor_data[2].torqueTarget  = 0;
			tmc4671_setTargetTorque_raw(2, 0);

			#if MATLAB
				snprintf(string, 200, 	"fintest\n");
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
			#endif

		}
		tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);

		data1[sweep.k].torqueTarget[2] 	= motor_data[2].torqueTarget;
		data1[sweep.k].torqueActual[2] 	= motor_data[2].torqueActual;
		data1[sweep.k].velocityActual[2] = motor_data[2].velocityActual;
		data1[sweep.k].velocityTarget[2] = motor_data[2].velocityTarget;
		data1[sweep.k].velocityIntegratorValue[2] = (int32_t)motor_control[2].velocityIntegratorValue;

		data2[sweep.k].positionActual = motor_data[2].positionActual;
		data2[sweep.k].positionTarget = motor_data[2].positionTarget;
		data2[sweep.k].positionIntegratorValue = (int32_t)motor_control[2].positionIntegratorValue;

		sweep.k++;

	}



	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
}
/* -------------------------------------------------------------------------  */


	if(stats == true && systick_counter_3 >= 200 ) //5Hz
	{
		systick_counter_3 = 0;


		#if MATLAB == 0
		for(i=0; i<4; i++)	angle[i] = as5047U_getAngle(i);

		snprintf(string, 1500, 	"%s", clear_string);
		//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(0));
		//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(1));
		snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(2));
		// snprintf(string+strlen(string), 1500, "Vel Int Val  = %f\n", motor_control[2].velocityIntegratorValue);
		// snprintf(string+strlen(string), 1500, "TorqueTarget = %ld\n", motor_data[2].torqueTarget);
		// snprintf(string+strlen(string), 1500, "V Target  = %ld\n", motor_data[2].velocityTarget);
		// snprintf(string+strlen(string), 1500, "V Actual  = %ld\n", motor_data[2].velocityActual);
		snprintf(string+strlen(string), 1500, "Velocity P %.1f\n",  motor_control[2].velocityP);
		snprintf(string+strlen(string), 1500, "Velocity I %.1f\n",  motor_control[2].velocityI);
		snprintf(string+strlen(string), 1500, "Position P %.1f\n",  motor_control[2].positionP);
		snprintf(string+strlen(string), 1500, "Positino I %.1f\n",  motor_control[2].positionI);
		// snprintf(string+strlen(string), 1500, "V ActualS = %d\n", 	motor_data[2].SensorVel);
		//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(3));
		//snprintf(string+strlen(string), 1500, "pwm_in:     %d %d %d %d\r\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
		// snprintf(string+strlen(string), 1500, "angleIn:    % 2.1f % 2.1f % 2.1f % 2.1f\n", angleIn[0], angleIn[1], angleIn[2], angleIn[3]);
		// snprintf(string+strlen(string), 1500, "angleOut:   % 2.1f % 2.1f % 2.1f % 2.1f\n", angleOut[0], angleOut[1], angleOut[2], angleOut[3]);
		snprintf(string+strlen(string), 1500, "---------------------------\n");
		snprintf(string+strlen(string), 1500, "enc0: %5d\tenc1: %5d\tenc2: %5d\tenc3: %5d\n", angle[0], angle[1], angle[2], angle[3]);
		snprintf(string+strlen(string), 1500, "---------------------------\n");
		snprintf(string+strlen(string), 1500, "[o] ... stopped mode\n[p] ... position mode\n[SPACE] ... STOP\n\n");

		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		#endif
	} // end of: if(systick_counter_3 >= 200) //5Hz

} // end of: void logic_loop(void)


void HAL_SYSTICK_Callback(void)
{
	systick_counter++;
	systick_counter_2++;
	systick_counter_3++;


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


int32_t clacAngle(uint8_t drv, float *angleIn)
{
	if(drv == 0)
	{
		angleOut[drv] = -(angleIn[drv] + angleIn[drv+1]);
	}
	else if(drv == 2)
	{
		angleOut[drv] = angleIn[drv] + angleIn[drv+1];
	}
	else if(drv == 1)
	{
		angleOut[drv]= -(2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv]);
	}
	else if(drv == 3)
	{
		angleOut[drv]= (2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv]);
	}

	return (int32_t)(angleOut[drv] * 2.0 / 360.0 * 65536.0);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    rx_byte_new = 1;
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
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

float sat(float x)
{
	if(x >= 1)
		return 1;
	else if (x <= -1)
		return -1;
	else
		return x;
}



void velocityPI()
{
	motor_control[2].velocityError = motor_data[2].velocityTarget-motor_data[2].velocityActual;
	motor_data[2].torqueTarget = 	(int32_t)(motor_control[2].velocityI*motor_control[2].velocityIntegratorValue + motor_control[2].velocityP*motor_control[2].velocityError);

	if(motor_data[2].torqueTarget > motor_control[2].velocityIntegratorLimit)
	{
		if(motor_control[2].velocityI>0)
			motor_control[2].velocityIntegratorValue = (float)(((float)motor_control[2].velocityIntegratorLimit - motor_control[2].velocityP*(float)motor_control[2].velocityError)/motor_control[2].velocityI);
		else
			motor_control[2].velocityIntegratorValue = 0;

		motor_data[2].torqueTarget = motor_control[2].velocityIntegratorLimit;
	}
	else if(motor_data[2].torqueTarget < (-motor_control[2].velocityIntegratorLimit))
	{
		if(motor_control[2].velocityI>0)
			motor_control[2].velocityIntegratorValue = (float)((-motor_control[2].velocityIntegratorLimit - motor_control[2].velocityP*(float)motor_control[2].velocityError)/motor_control[2].velocityI);
		else
			motor_control[2].velocityIntegratorValue = 0;

		motor_data[2].torqueTarget = -motor_control[2].velocityIntegratorLimit;
	}
	else
	{
		motor_control[2].velocityIntegratorValue = motor_control[2].velocityIntegratorValue + TA*(float)motor_control[2].velocityError;
	}

}

void positionPI()
{
	motor_control[2].positionError = motor_data[2].positionTarget-motor_data[2].positionActual;
	motor_data[2].velocityTarget = 	(int32_t)(motor_control[2].positionI*motor_control[2].positionIntegratorValue + motor_control[2].positionP*motor_control[2].positionError);

	if(motor_data[2].velocityTarget > motor_control[2].positionIntegratorLimit)
	{
		if(motor_control[2].positionI>0)
			motor_control[2].positionIntegratorValue = (float)(((float)motor_control[2].positionIntegratorLimit - motor_control[2].positionP*(float)motor_control[2].positionError)/motor_control[2].positionI);
		else
			motor_control[2].positionIntegratorValue = 0;

		motor_data[2].velocityTarget = motor_control[2].positionIntegratorLimit;
	}
	else if(motor_data[2].velocityTarget < (-motor_control[2].positionIntegratorLimit))
	{
		if(motor_control[2].positionI>0)
			motor_control[2].positionIntegratorValue = (float)((-motor_control[2].positionIntegratorLimit - motor_control[2].positionP*(float)motor_control[2].positionError)/motor_control[2].positionI);
		else
			motor_control[2].positionIntegratorValue = 0;

		motor_data[2].velocityTarget = -motor_control[2].positionIntegratorLimit;
	}
	else
	{
		motor_control[2].positionIntegratorValue = motor_control[2].positionIntegratorValue + TA*(float)motor_control[2].positionError;
	}

}
