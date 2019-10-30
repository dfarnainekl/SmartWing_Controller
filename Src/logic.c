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
// #define DATA_N		4096
//#define DATA_N		8192

#define MATLAB 0

#define MODE_STOP				0
#define MODE_TORQUE				1
#define MODE_VELOCITY			2
#define MODE_POSITION			3
#define MODE_IDENTIFY_VELOCITY 	4
#define MODE_CHIRP_TORQUE		5
#define MODE_VELOCITY_STEP		6
#define MODE_POSITION_STEP		7
#define MODE_RCCONTROL			8

uint8_t mode = MODE_STOP;

sweep_t sweep = { .Ta = TA,
		.N = DATA_N,
		.U = 1,
		.omegaStart = 2 * M_PI * 50,
		.omegaEnd = 2 * M_PI * 500,
		.k = 0,
		.mode = AIL
};

__attribute__((section(".ram_d1")))  data1_t data1[DATA_N] = { 0 };
__attribute__((section(".ram_d2")))  data2_t data2[DATA_N] = { 0 };

static motor_t motor_data[4] = { 0 };
static control_t motor_control[4] = { 0 };

uint8_t as5074uErrorCounter[4] = {0};

volatile uint16_t systick_counter = 0;
volatile uint16_t systick_counter_2 = 0;
volatile uint16_t systick_counter_3 = 0;

volatile uint8_t rx_byte_new = 0;
uint8_t rx_byte;

volatile uint16_t pwm_in[4] = { 1500, 1500, 1500, 1500 };
volatile bool pwm_updated = false;
//volatile float angleOut[4] = { 0, 0, 0, 0 };
uint16_t angle[4];

bool stats = true;
uint16_t integral_pos = 0;
uint16_t integral_vel = 500;
uint16_t proportional_pos = 600;
uint16_t proportional_vel = 8000;

bool current = false;

#if MATLAB
char clear_string[1] = { '\0' };
#else
char clear_string[8] = { 27, '[', '2','J', 27, '[', 'H', '\0'};
#endif

void logic_init(void)
{
	uint8_t i;
	static char string[500];
	static bool button_init = true;

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	for (i = 0; i < 4; i++) tmc6200_highLevel_init(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) swdriver_setEnable(i, true);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) tmc6200_highLeve_resetErrorFlags(i);

	for (i = 0; i < 4; i++) TMC4671_highLevel_init(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) as5047U_setABIResolution14Bit(i); //FIXME
	for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
	// for(i=0; i<4; i++) 	TMC4671_highLevel_pwmOff(i);

	//TMC4671_highLevel_openLoopTest2(2, 120);

	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	rx_byte_new = 0;
	while (!(rx_byte_new && rx_byte == 's') && button_init == true)
	{
		button_init = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
#if MATLAB == 0
		for(i=2; i<4; i++) angle[i] = as5047U_getAngle(i);
		snprintf(string, 500, "%s", clear_string);
		snprintf(string+strlen(string), 500-strlen(string), "as5047U Position:  %5d\t%5d\t%5d\t%5d\n", angle[0], angle[1], angle[2], angle[3] );
		snprintf(string+strlen(string), 500-strlen(string), "TMC6200 GSTAT:     %5d\t%5d\t%5d\t%5d\n", tmc6200_readInt(0, TMC6200_GSTAT), tmc6200_readInt(1, TMC6200_GSTAT), tmc6200_readInt(2, TMC6200_GSTAT), tmc6200_readInt(3, TMC6200_GSTAT));
		snprintf(string+strlen(string), 500-strlen(string), "TMC6200 GCONF:     %5d\t%5d\t%5d\t%5d\n", tmc6200_readInt(0, TMC6200_GCONF), tmc6200_readInt(1, TMC6200_GCONF), tmc6200_readInt(2, TMC6200_GCONF), tmc6200_readInt(3, TMC6200_GCONF));
		snprintf(string+strlen(string), 500-strlen(string), "TMC6200 DRV_CONF:  %5d\t%5d\t%5d\t%5d\n", tmc6200_readInt(0, TMC6200_DRV_CONF), tmc6200_readInt(1, TMC6200_DRV_CONF), tmc6200_readInt(2, TMC6200_DRV_CONF), tmc6200_readInt(3, TMC6200_DRV_CONF));
		snprintf(string+strlen(string), 500-strlen(string), "as5074u Errors:    %5d\t%5d\t%5d\t%5d\n", as5074uErrorCounter[0], as5074uErrorCounter[1], as5074uErrorCounter[2], as5074uErrorCounter[3]);
		/* Should read:
		TMC6200 GSTAT = 0
		TMC6200 GCONF = 48
		TMC6200 DRV_CONF = 0
		as5074u Errors = 0
		*/
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
#endif
	}
	rx_byte_new = 0;
	HAL_Delay(100);

	// for(i=0; i<4; i++) 	TMC4671_highLevel_pwmOff(i);
	// while(1);

	//outer
	motor_control[0].velocityP = 10;
	motor_control[0].velocityI = 10;
	motor_control[0].positionP = 0.5;
	motor_control[0].positionI = 0;

	motor_control[2].velocityP = 10;
	motor_control[2].velocityI = 10;
	motor_control[2].positionP = 0.5;
	motor_control[2].positionI = 0;

	//inner
	motor_control[1].velocityP = 10;
	motor_control[1].velocityI = 10;
	motor_control[1].positionP = 0.5;
	motor_control[1].positionI = 0;

	motor_control[3].velocityP = 10;
	motor_control[3].velocityI = 10;
	motor_control[3].positionP = 0.5;
	motor_control[3].positionI = 0;



	// initialize right wing
	 TMC4671_highLevel_initEncoder_new(2);
	 TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	 TMC4671_highLevel_positionMode_rampToZero(2);

	 TMC4671_highLevel_initEncoder_new(3);
	 TMC4671_highLevel_positionMode_fluxTorqueRamp(3);
	 TMC4671_highLevel_positionMode_rampToZero(3);

	TMC4671_highLevel_pwmOff(0);
	TMC4671_highLevel_pwmOff(1);
//	TMC4671_highLevel_pwmOff(2);
//	TMC4671_highLevel_pwmOff(3);

	//for(i=2; i<4; i++) tmc4671_writeInt(i, TMC4671_PID_POSITION_ACTUAL, 0);

	// pwm inputs //FIXME: timer interrupt priority lower than spi?
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	//for (i = 0; i < 4; i++) TMC4671_highLevel_setPositionFilter(i, false);
	//for(i=0; i<4; i++)	TMC4671_highLevel_setCurrentLimit(i, 15000);

#if MATLAB
	snprintf(string, 500, "fininit\n");
	HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
#endif
}

void logic_loop(void)
{
	static char string[1500];
	static uint32_t i = 0;



	/* -------------------------------------------------------------------------  */
	static bool button_stop = true;
	button_stop = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	if (button_stop == false)
	{
		for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
		for (i = 0; i < 4; i++) TMC4671_highLevel_pwmOff(i);
	}
	/* -------------------------------------------------------------------------  */
	if (pwm_updated)
	{
		pwm_updated = false;
		for(i=0; i<4; i++) motor_control[i].angleIn =   ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree

		if(mode == MODE_RCCONTROL)
		{

			for(i=0; i<4; i++) motor_control[i].positionTarget = clacAngle(i);
		}

	}
	/* -------------------------------------------------------------------------  */

	if (systick_counter) //1ms
	{
		systick_counter = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);


		for(i=0; i<4; i++) motor_data[i].positionActual = tmc4671_getActualPosition(i);
		for(i=0; i<4; i++) motor_data[i].velocityActual = tmc4671_getActualVelocity(i);

		for(i=0; i<4; i++) motor_control[i].positionActual = (float)motor_data[i].positionActual;
		for(i=0; i<4; i++) motor_control[i].velocityActual = (float)motor_data[i].velocityActual;



		if (mode == MODE_STOP)
		{
			for(i=0; i<4; i++) motor_control[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
		}
		else if (mode == MODE_TORQUE)
		{

			//state transform
			for(i=0; i<4; i++) motor_data[i].torqueTarget = (int32_t)(motor_control[i].torqueTarget);
			for(i=0; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);
		}
		else if (mode == MODE_POSITION || mode == MODE_RCCONTROL)
		{


			for (i = 0; i < 4; i++) positionPI(i);
			for (i = 0; i < 4; i++) velocityPI(i);


			//for (i = 0; i < 4; i++) motor_data[i].torqueBeta = calcTorque(i, motor_data[i].angleAlpha, motor_data[i].torqueAlpha);

			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;

			for(i=0; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);

		}


		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
	}


	/* -------------------------------------------------------------------------  */

	if (stats == true && systick_counter_3 >= 200) //5Hz
	{
		systick_counter_3 = 0;

#if MATLAB == 0

		snprintf(string, 2000, "%s", clear_string);
		snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(2));
		snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(3));
		snprintf(string+strlen(string), 2000-strlen(string), "Outer\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Velocity P      %.1f\n", motor_control[2].velocityP);
		snprintf(string+strlen(string), 2000-strlen(string), "Velocity I      %.1f\n", motor_control[2].velocityI);
		snprintf(string+strlen(string), 2000-strlen(string), "Position P      %.1f\n", motor_control[2].positionP);
		snprintf(string+strlen(string), 2000-strlen(string), "Position I      %.1f\n", motor_control[2].positionI);
		snprintf(string+strlen(string), 2000-strlen(string), "Position Error  %.1f\n", motor_control[2].positionError);
		snprintf(string+strlen(string), 2000-strlen(string), "Velocity Error  %.1f\n", motor_control[2].velocityError);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Inner\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Velocity P      %.1f\n", motor_control[3].velocityP);
		snprintf(string+strlen(string), 2000-strlen(string), "Velocity I      %.1f\n", motor_control[3].velocityI);
		snprintf(string+strlen(string), 2000-strlen(string), "Position P      %.1f\n", motor_control[3].positionP);
		snprintf(string+strlen(string), 2000-strlen(string), "Position I      %.1f\n", motor_control[3].positionI);
		snprintf(string+strlen(string), 2000-strlen(string), "Position Error  %.1f\n", motor_control[3].positionError);
		snprintf(string+strlen(string), 2000-strlen(string), "Velocity Error  %.1f\n", motor_control[3].velocityError);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "pwm_in:      %d  %d  %d  %d\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
		snprintf(string+strlen(string), 2000-strlen(string), "angleIn:     % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].angleIn, motor_control[1].angleIn, motor_control[2].angleIn, motor_control[3].angleIn);
		snprintf(string+strlen(string), 2000-strlen(string), "angleOut:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].angleOut, motor_control[1].angleOut, motor_control[2].angleOut, motor_control[3].angleOut);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Target:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionTarget, motor_control[1].positionTarget, motor_control[2].positionTarget, motor_control[3].positionTarget);
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionActual, motor_control[1].positionActual, motor_control[2].positionActual, motor_control[3].positionActual);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		// snprintf(string+strlen(string), 2000-strlen(string), "TorqueTarget C:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].torqueTarget, motor_control[1].torqueTarget, motor_control[2].torqueTarget, motor_control[3].torqueTarget);
		// snprintf(string+strlen(string), 2000-strlen(string), "TorqueTarget M:    %ld  %ld  %ld  %ld\n", motor_data[0].torqueTarget, motor_data[1].torqueTarget, motor_data[2].torqueTarget, motor_data[3].torqueTarget);
		// snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
#endif
	} // end of: if(systick_counter_3 >= 200) //5Hz

	/* -------------------------------------------------------------------------  */
	if (rx_byte_new)
	{
		rx_byte_new = 0;

		switch (rx_byte)
		{
		case ' ':
			for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
			for (i = 0; i < 4; i++) TMC4671_highLevel_pwmOff(i);
			break;

		case 'a':
			stats = false;
			break;
		case 'q':
			stats = true;
			break;

		case 'c':
			if (current)
			{
				for (i = 0; i < 4; i++) TMC4671_highLevel_setCurrentLimit(i, 5000);
				current = false;
			}
			else
			{
				for (i = 0; i < 4; i++) TMC4671_highLevel_setCurrentLimit(i, 15000);
				current = true;
			}
			break;

		case '0':
			mode = MODE_STOP;
			stats = true;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case '1':
			mode = MODE_TORQUE;
			motor_control[0].torqueTarget = 0;
			motor_control[1].torqueTarget = 0;
			motor_control[2].torqueTarget = 0;
			motor_control[3].torqueTarget = 2000;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '2':
			mode = MODE_POSITION;
			//for(i=2; i<4; i++) tmc4671_writeInt(i, TMC4671_PID_POSITION_ACTUAL, 0);
			// stats = false;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 5000;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
			motor_control[0].positionTarget = 0;
			motor_control[1].positionTarget = 0;
			motor_control[2].positionTarget = 15;
			motor_control[3].positionTarget = 0;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			// for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case '3':
			mode = MODE_POSITION;
			//for(i=2; i<4; i++) tmc4671_writeInt(i, TMC4671_PID_POSITION_ACTUAL, 0);
			// stats = false;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 5000;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
			motor_control[0].positionTarget = 0;
			motor_control[1].positionTarget = 0;
			motor_control[2].positionTarget = 0;
			motor_control[3].positionTarget = 15;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			// for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case '4':
			mode = MODE_POSITION;
			//for(i=2; i<4; i++) tmc4671_writeInt(i, TMC4671_PID_POSITION_ACTUAL, 0);
			// stats = false;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 5000;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
			for (i = 0; i < 4; i++) motor_control[i].positionTarget = 0;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			// for (i = 0; i < 4; i++) tmc4671_switchToMotion40 Mode(i, TMC4671_MOTION_MODE_STOPPED);
			break;


		case '9':
			mode = MODE_RCCONTROL;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 5000;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
			for (i = 0; i < 4; i++) motor_control[i].positionTarget = 0;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;



		case '+':
			break;

		case '-':
			break;

		case 'w':
			motor_control[0].velocityP += 1.0;
			motor_control[2].velocityP += 1.0;
			break;

		case 's':
			if (motor_control[0].velocityP >= 1.0)
			{
				motor_control[0].velocityP -= 1.0;
				motor_control[2].velocityP -= 1.0;
			}
			else
			{
				motor_control[0].velocityP = 0.0;
				motor_control[2].velocityP = 0.0;
			}
			break;

		case 'e':
			motor_control[0].velocityI += 1.0;
			motor_control[2].velocityI += 1.0;
			break;

		case 'd':
			if (motor_control[0].velocityI >= 1.0)
			{
				motor_control[0].velocityI -= 1.0;
				motor_control[2].velocityI -= 1.0;
			}

			else
			{
				motor_control[0].velocityI = 0.0;
				motor_control[2].velocityI = 0.0;
			}
			break;


		case 'r':
			motor_control[0].positionP += 0.1;
			motor_control[2].positionP += 0.1;
			break;

		case 'f':
			if (motor_control[0].positionP >= 0.1)
			{
				motor_control[0].positionP -= 0.1;
				motor_control[2].positionP -= 0.1;
			}
			else
			{
				motor_control[0].positionP = 0.0;
				motor_control[2].positionP = 0.0;
			}
			break;

		case 't':
			motor_control[0].positionI += 1.0;
			motor_control[2].positionI += 1.0;
			break;

		case 'g':
			if (motor_control[0].positionI >= 1.0)
			{
				motor_control[0].positionI -= 1.0;
				motor_control[2].positionI -= 1.0;
			}
			else
			{
				motor_control[0].positionI = 0.0;
				motor_control[2].positionI = 0.0;
			}
			break;


		case 'z':
			motor_control[1].velocityP += 1.0;
			motor_control[3].velocityP += 1.0;
			break;

		case 'h':
			if (motor_control[1].velocityP >= 1.0)
			{
				motor_control[1].velocityP -= 1.0;
				motor_control[3].velocityP -= 1.0;
			}
			else
			{
				motor_control[1].velocityP = 0.0;
				motor_control[3].velocityP = 0.0;
			}
			break;

		case 'u':
			motor_control[1].velocityI += 1.0;
			motor_control[3].velocityI += 1.0;
			break;

		case 'j':
			if (motor_control[1].velocityI >= 1.0)
			{
				motor_control[1].velocityI -= 1.0;
				motor_control[3].velocityI -= 1.0;
			}

			else
			{
				motor_control[1].velocityI = 0.0;
				motor_control[3].velocityI = 0.0;
			}
			break;


		case 'i':
			motor_control[1].positionP += 0.1;
			motor_control[3].positionP += 0.1;
			break;

		case 'k':
			if (motor_control[1].positionP >= 0.1)
			{
				motor_control[1].positionP -= 0.1;
				motor_control[3].positionP -= 0.1;
			}
			else
			{
				motor_control[1].positionP = 0.0;
				motor_control[3].positionP = 0.0;
			}
			break;

		case 'o':
			motor_control[1].positionI += 1.0;
			motor_control[3].positionI += 1.0;
			break;

		case 'l':
			if (motor_control[1].positionI >= 1.0)
			{
				motor_control[1].positionI -= 1.0;
				motor_control[3].positionI -= 1.0;
			}
			else
			{
				motor_control[1].positionI = 0.0;
				motor_control[3].positionI = 0.0;
			}
			break;


		case 'x': // print data
			snprintf(string, 1500, "%s", clear_string);
			HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
			HAL_Delay(2);

			for (i = 0; i < DATA_N; i++)
			{
				snprintf(string, 1500, "%ld;", i);
				snprintf(string + strlen(string), 1500 - strlen(string), "%d;", data1[i].torqueTarget[2]);
				snprintf(string + strlen(string), 1500 - strlen(string), "%d;", data1[i].torqueActual[2]);
				snprintf(string + strlen(string), 1500 - strlen(string), "%ld;", data1[i].velocityActual[2]);
				snprintf(string + strlen(string), 1500 - strlen(string), "%ld;", data1[i].velocityTarget[2]);
				snprintf(string + strlen(string), 1500 - strlen(string), "%ld\n", data1[i].velocityIntegratorValue[2]);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
				HAL_Delay(2);
			}
#if MATLAB
			snprintf(string, 200, "finplot\n");
			HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
#endif
			break;

		case 'y':
//			for (i = 0; i < 4; i++) angle[i] = as5047U_getAngle(i);
			snprintf(string, 1500, "%s", clear_string);
			//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(0));
			//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(1));
			snprintf(string + strlen(string), 1500, "%s", TMC4671_highLevel_getStatus(2));
			snprintf(string + strlen(string), 1500, "%s", TMC4671_highLevel_getStatus(3));
			snprintf(string+strlen(string), 1500-strlen(string), "Twist\n");
			snprintf(string+strlen(string), 1500-strlen(string), "Velocity P      %.1f\n", motor_control[2].velocityP);
			snprintf(string+strlen(string), 1500-strlen(string), "Velocity I      %.1f\n", motor_control[2].velocityI);
			snprintf(string+strlen(string), 1500-strlen(string), "Position P      %.1f\n", motor_control[2].positionP);
			snprintf(string+strlen(string), 1500-strlen(string), "Position I      %.1f\n", motor_control[2].positionI);
			snprintf(string+strlen(string), 1500-strlen(string), "Position Error  %.1f\n", motor_control[2].positionError);
			snprintf(string+strlen(string), 1500-strlen(string), "Velocity Error  %.1f\n", motor_control[2].velocityError);
			snprintf(string+strlen(string), 1500-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 1500-strlen(string), "Rotate\n");
			snprintf(string+strlen(string), 1500-strlen(string), "Velocity P      %.1f\n", motor_control[3].velocityP);
			snprintf(string+strlen(string), 1500-strlen(string), "Velocity I      %.1f\n", motor_control[3].velocityI);
			snprintf(string+strlen(string), 1500-strlen(string), "Position P      %.1f\n", motor_control[3].positionP);
			snprintf(string+strlen(string), 1500-strlen(string), "Position I      %.1f\n", motor_control[3].positionI);
			snprintf(string+strlen(string), 1500-strlen(string), "Position Error  %.1f\n", motor_control[3].positionError);
			snprintf(string+strlen(string), 1500-strlen(string), "Velocity Error  %.1f\n", motor_control[3].velocityError);
			snprintf(string+strlen(string), 1500-strlen(string), "---------------------------\n");
//			snprintf(string + strlen(string), 1500, "enc0: %5d\tenc1: %5d\tenc2: %5d\tenc3: %5d\n", angle[0], angle[1], angle[2], angle[3]);
			// snprintf(string + strlen(string), 1500, "[o] ... stopped mode\n[p] ... position mode\n[SPACE] ... STOP\n");
			snprintf(string + strlen(string), 1500, "finstats\n");
			HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
			break;

		default:
			break;
		} // end of: switch(rx_byte)
	} // end of: if(rx_byte_new)
	/* -------------------------------------------------------------------------  */


} // end of: void logic_loop(void)

void HAL_SYSTICK_Callback(void)
{
	systick_counter++;
	systick_counter_2++;
	systick_counter_3++;

	// error led
	if (swdriver_getStatus(0) || swdriver_getStatus(1) || swdriver_getStatus(2) || swdriver_getStatus(3) ||
			swdriver_getFault(0) || swdriver_getFault(1) || swdriver_getFault(2) || swdriver_getFault(3))
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
}


int32_t clacAngle(uint8_t drv)
{
	if(drv == 0)
	{
		motor_control[drv].angleOut = -(motor_control[drv].angleIn + motor_control[drv+1].angleIn);
		//angleOut[drv] = -(angleIn[drv] + angleIn[drv+1]);
	}
	else if(drv == 2)
	{
		motor_control[drv].angleOut = (motor_control[drv].angleIn + motor_control[drv+1].angleIn);
		//angleOut[drv] = angleIn[drv] + angleIn[drv+1];
	}
	else if(drv == 1)
	{
		motor_control[drv].angleOut = -(  2.490378*motor_control[drv].angleIn
										+ 0.001711*motor_control[drv].angleIn*motor_control[drv].angleIn
										+ 0.000138*motor_control[drv].angleIn*motor_control[drv].angleIn*motor_control[drv].angleIn );
		//angleOut[drv]= -(2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv]);
	}
	else if(drv == 3)
	{
		motor_control[drv].angleOut =  (  2.490378*motor_control[drv].angleIn
										+ 0.001711*motor_control[drv].angleIn*motor_control[drv].angleIn
										+ 0.000138*motor_control[drv].angleIn*motor_control[drv].angleIn*motor_control[drv].angleIn );
		//angleOut[drv]= (2.490378*angleIn[drv] + 0.001711*angleIn[drv]*angleIn[drv] + 0.000138*angleIn[drv]*angleIn[drv]*angleIn[drv]);
	}
	motor_control[drv].angleOut = motor_control[drv].angleOut * 2.0;

	return (int32_t)(motor_control[drv].angleOut / 360.0 * 65536.0);
}



float calcTorque(uint8_t drv, float angleAlpha, float torqueAlpha)
{
	static float torqueBeta = 0;
	static float angleAlphaSat = 0;

	/*
	torqueBeta(torqueAlpha, angleAlpha)
	is convex and well defined for angleAlpha =-100..100
	*/

	if(angleAlpha > 50.0)
		angleAlphaSat = 50.0;
	else if(angleAlpha < -50.0)
		angleAlphaSat = -50.0;
	else
		angleAlphaSat = angleAlpha;


	if(drv == 0)
	{
		torqueBeta = torqueAlpha/2;
	}
	else if (drv == 2)
	{
		torqueBeta = torqueAlpha/2;
	}
	if(drv == 1)
	{
		angleAlphaSat = -angleAlphaSat;
		torqueBeta = torqueAlpha*
			(0.194852609015931322e0
			+ angleAlphaSat * (-0.217758117033939151e-3)
			+ angleAlphaSat * angleAlphaSat * 0.225298149350079040e-3
			+ pow(angleAlphaSat, 3) * 0.571754382667929612e-6
			+ pow(angleAlphaSat, 4) * (-0.281179358438909426e-6)
			+ pow(angleAlphaSat, 4) * (-0.210702833388430834e-8)
			+ pow(angleAlphaSat, 6) * 0.291589620920940693e-9
			+ pow(angleAlphaSat, 7) * 0.210820574656015764e-11);
	}
	if(drv == 3)
	{
		torqueBeta = torqueAlpha*
			(0.194852609015931322e0
			+ angleAlphaSat * (-0.217758117033939151e-3)
			+ angleAlphaSat * angleAlphaSat * 0.225298149350079040e-3
			+ pow(angleAlphaSat, 3) * 0.571754382667929612e-6
			+ pow(angleAlphaSat, 4) * (-0.281179358438909426e-6)
			+ pow(angleAlphaSat, 4) * (-0.210702833388430834e-8)
			+ pow(angleAlphaSat, 6) * 0.291589620920940693e-9
			+ pow(angleAlphaSat, 7) * 0.210820574656015764e-11);
	}
	return torqueBeta;


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
	if (htim == &htim2) // pwm in
	{
		static uint32_t timestamp_risingEdge[4] = { 0, 0, 0, 0 };

		switch (htim->Channel)
		{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			if (HAL_GPIO_ReadPin(PWM_IN_0_GPIO_Port, PWM_IN_0_Pin))
			{
				timestamp_risingEdge[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //rising edge
				pwm_updated = true;
			}
			else
			{
				uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - timestamp_risingEdge[0]; //falling edge
				if (pwm < 1000)
					pwm = 1000;
				else if (pwm > 2000)
					pwm = 2000;
				pwm_in[0] = pwm;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			if (HAL_GPIO_ReadPin(PWM_IN_1_GPIO_Port, PWM_IN_1_Pin))
			{
				timestamp_risingEdge[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //rising edge
				pwm_updated = true;
			}
			else
			{
				uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) - timestamp_risingEdge[1]; //falling edge
				if (pwm < 1000)
					pwm = 1000;
				else if (pwm > 2000)
					pwm = 2000;
				pwm_in[1] = pwm;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if (HAL_GPIO_ReadPin(PWM_IN_2_GPIO_Port, PWM_IN_2_Pin))
			{
				timestamp_risingEdge[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); //rising edge
				pwm_updated = true;
			}
			else
			{
				uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - timestamp_risingEdge[2]; //falling edge
				if (pwm < 1000)
					pwm = 1000;
				else if (pwm > 2000)
					pwm = 2000;
				pwm_in[2] = pwm;
			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			if (HAL_GPIO_ReadPin(PWM_IN_3_GPIO_Port, PWM_IN_3_Pin))
			{
				timestamp_risingEdge[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); //rising edge
				pwm_updated = true;
			}
			else
			{
				uint16_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - timestamp_risingEdge[3]; //falling edge
				if (pwm < 1000)
					pwm = 1000;
				else if (pwm > 2000)
					pwm = 2000;
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
	if (x >= 1)
		return 1;
	else if (x <= -1)
		return -1;
	else
		return x;
}

void velocityPI(uint8_t drv)
{
	motor_control[drv].velocityError = motor_control[drv].velocityTarget - motor_control[drv].velocityActual;
	motor_control[drv].torqueTarget = motor_control[drv].velocityI * motor_control[drv].velocityIntegratorValue + motor_control[drv].velocityP * motor_control[drv].velocityError;

	if (motor_control[drv].torqueTarget > motor_control[drv].velocityIntegratorLimit)
	{
		if (motor_control[drv].velocityI > 0)
			motor_control[drv].velocityIntegratorValue = (float) (((float) motor_control[drv].velocityIntegratorLimit - motor_control[drv].velocityP * (float) motor_control[drv].velocityError) / motor_control[drv].velocityI);
		else
			motor_control[drv].velocityIntegratorValue = 0;

		motor_control[drv].torqueTarget = motor_control[drv].velocityIntegratorLimit;
	}
	else if (motor_control[drv].torqueTarget < (-motor_control[drv].velocityIntegratorLimit))
	{
		if (motor_control[drv].velocityI > 0)
			motor_control[drv].velocityIntegratorValue = (float) ((-motor_control[drv].velocityIntegratorLimit - motor_control[drv].velocityP * (float) motor_control[drv].velocityError) / motor_control[drv].velocityI);
		else
			motor_control[drv].velocityIntegratorValue = 0;

		motor_control[drv].torqueTarget = -motor_control[drv].velocityIntegratorLimit;
	}
	else
	{
		motor_control[drv].velocityIntegratorValue = motor_control[drv].velocityIntegratorValue + TA * (float) motor_control[drv].velocityError;
	}

}

void positionPI(uint8_t drv)
{
	motor_control[drv].positionError = motor_control[drv].positionTarget - motor_control[drv].positionActual;
	motor_control[drv].velocityTarget = motor_control[drv].positionI * motor_control[drv].positionIntegratorValue + motor_control[drv].positionP * motor_control[drv].positionError;

	if (motor_control[drv].velocityTarget > motor_control[drv].positionIntegratorLimit)
	{
		if (motor_control[drv].positionI > 0)
			motor_control[drv].positionIntegratorValue = (float) (((float) motor_control[drv].positionIntegratorLimit - motor_control[drv].positionP * (float) motor_control[drv].positionError) / motor_control[drv].positionI);
		else
			motor_control[drv].positionIntegratorValue = 0;

		motor_control[drv].velocityTarget = motor_control[drv].positionIntegratorLimit;
	}
	else if (motor_control[drv].velocityTarget < (-motor_control[drv].positionIntegratorLimit))
	{
		if (motor_control[drv].positionI > 0)
			motor_control[drv].positionIntegratorValue = (float) ((-motor_control[drv].positionIntegratorLimit - motor_control[drv].positionP * (float) motor_control[drv].positionError) / motor_control[drv].positionI);
		else
			motor_control[drv].positionIntegratorValue = 0;

		motor_control[drv].velocityTarget = -motor_control[drv].positionIntegratorLimit;
	}
	else
	{
		motor_control[drv].positionIntegratorValue = motor_control[drv].positionIntegratorValue + TA * (float) motor_control[drv].positionError;
	}

}
