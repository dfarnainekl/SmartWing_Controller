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

#define TA			0.0005
//#define DATA_N		1024
// #define DATA_N		2048
#define DATA_N		8192

#define MATLAB 0

#define MODE_STOP				0
#define MODE_TORQUE				1
#define MODE_VELOCITY			2
#define MODE_POSITION			3
#define MODE_TORQUE_SWEEP		4
#define MODE_VELOCITY_SWEEP		5
#define MODE_POSITION_SWEEP		6
#define MODE_POSITION_STEP		7
#define MODE_RCCONTROL			8
#define MODE_IDLE				9
#define MODE_POSITION_SWEEP2	10
#define MODE_CONTROL_TEST		11
#define MODE_POSITION_NEW		12
#define MODE_RCCONTROL_NEW		14


uint8_t mode = MODE_STOP;

sweep_t sweep = { .Ta = TA,
		.N = DATA_N,
		.U = 1,
		.omegaStart = 2 * M_PI * 0.5,
		.omegaEnd = 2 * M_PI * 500,
		.k = 0,
		.mode = AIL
};

__attribute__((section(".ram_d1")))  data1_t data1[DATA_N] = { 0 };
__attribute__((section(".ram_d2")))  data2_t data2[DATA_N] = { 0 };

static motor_t motor_data[4] = { 0 };
static control_t motor_control[4] = { 0 };

volatile uint16_t systick_counter = 0;
volatile uint16_t systick_counter_2 = 0;
volatile uint16_t systick_counter_3 = 0;

// SPI Interrupt
uint8_t txBuffer[4][5] = {0};
uint8_t rxBuffer[4][5] = {0};
volatile uint8_t rxSPICplt[4] = {0};
volatile int32_t rxSPIData [4] = {0};

// UART Interrupt
volatile uint8_t rx_byte_new = 0;
uint8_t rx_byte;

// SPI Interrupt
volatile uint16_t pwm_in[4] = { 1500, 1500, 1500, 1500 };
volatile bool pwm_updated = false;

//AS5047U
uint16_t angle[4];
uint8_t as5074uErrorCounter[4] = {0};


bool stats = true;
bool current = false;

#if MATLAB
char clear_string[1] = { '\0' };
#else
char clear_string[8] = { 27, '[', '2','J', 27, '[', 'H', '\0'};
#endif



void logic_init(void)
{
	int32_t i;
	static char string[500];
	static bool button_init = true;

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	for (i = 0; i < 4; i++) spiSpeedSlow_set(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) tmc6200_highLevel_init(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) swdriver_setEnable(i, true);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) tmc6200_highLeve_resetErrorFlags(i);

	for (i = 0; i < 4; i++) TMC4671_highLevel_init(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) as5047U_setABIResolution14Bit(i); //FIXME
	for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);

	HAL_Delay(10);
	for (i = 0; i < 4; i++) spiSpeedSlow_reset(i);
	HAL_Delay(10);

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

	//outer
	motor_control[0].velocityP = 10;
	motor_control[0].velocityI = 1;
	motor_control[0].positionP = 0.1;
	motor_control[0].positionI = 0.1;

	motor_control[2].velocityP = 10;
	motor_control[2].velocityI = 1;
	motor_control[2].positionP = 0.1;
	motor_control[2].positionI = 0.1;

	//inner
	motor_control[1].velocityP = 10;
	motor_control[1].velocityI = 1;
	motor_control[1].positionP = 0.1;
	motor_control[1].positionI = 0.1;

	motor_control[3].velocityP = 10;
	motor_control[3].velocityI = 1;
	motor_control[3].positionP = 0.1;
	motor_control[3].positionI = 0.1;


    TMC4671_highLevel_initEncoder_new(2);
	TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	TMC4671_highLevel_positionMode_rampToZero(2);
	TMC4671_highLevel_stoppedMode(2);
	HAL_Delay(100);
	TMC4671_highLevel_initEncoder_new(3);
	TMC4671_highLevel_positionMode_fluxTorqueRamp(3);
	TMC4671_highLevel_positionMode_rampToZero(3);
	TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	TMC4671_highLevel_positionMode_rampToZero(2);


	TMC4671_highLevel_pwmOff(0);
	TMC4671_highLevel_pwmOff(1);
	// TMC4671_highLevel_pwmOff(2);
	// TMC4671_highLevel_pwmOff(3);

	//for(i=2; i<4; i++) tmc4671_writeInt(i, TMC4671_PID_POSITION_ACTUAL, 0);

	// pwm inputs //FIXME: timer interrupt priority lower than spi?
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	for (i = 0; i < 4; i++) TMC4671_highLevel_setCurrentLimit(i, 15000);

	//for (i = 0; i < 4; i++) TMC4671_highLevel_setPositionFilter(i, false);

#if MATLAB
	snprintf(string, 500, "fininit\n");
	HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
#endif
}

void logic_loop(void)
{
	static char string[1500];
    static uint8_t data[500] = {0};
	static uint32_t i = 0;
	static uint32_t j = 0;


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

		if(mode == MODE_RCCONTROL || mode == MODE_RCCONTROL_NEW)
		{
			//for(i=0; i<4; i++) motor_control[i].angleIn =   ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree

			 motor_control[2].angleIn =   ( (float)(pwm_in[0] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree
			 motor_control[3].angleIn =   ( (float)(pwm_in[1] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree

			for(i=0; i<4; i++) motor_control[i].positionTarget = calcAngleTarget(i);
			if(mode == MODE_RCCONTROL_NEW)
			{
				//for(i=0; i<4; i++) motor_control[i].positionTarget = calcAngleBetaAlpha(i, motor_control[i].positionTarget/65536.0*360.0)*65536.0/360.0;
			}
		}
	}
	/* -------------------------------------------------------------------------  */

	if (stats == true && systick_counter_3 >= 500) //4Hz
	{
		systick_counter_3 = 0;

#if MATLAB == 0

		snprintf(string, 2000, "%s", clear_string);
		snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(2));
		snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(3));
		snprintf(string+strlen(string), 2000-strlen(string), "pwm_in:      %d  %d  %d  %d\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
		snprintf(string+strlen(string), 2000-strlen(string), "angleIn:     % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].angleIn, motor_control[1].angleIn, motor_control[2].angleIn, motor_control[3].angleIn);
		snprintf(string+strlen(string), 2000-strlen(string), "angleOut:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].angleOut, motor_control[1].angleOut, motor_control[2].angleOut, motor_control[3].angleOut);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Target:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionTarget, motor_control[1].positionTarget, motor_control[2].positionTarget, motor_control[3].positionTarget);
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionActual, motor_control[1].positionActual, motor_control[2].positionActual, motor_control[3].positionActual);
        snprintf(string+strlen(string), 2000-strlen(string), "Error:       % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionError, motor_control[1].positionError, motor_control[2].positionError, motor_control[3].positionError);
        snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "rxSPICplt:   %d  %d  %d  %d\n", rxSPICplt[0], rxSPICplt[1], rxSPICplt[2], rxSPICplt[3]);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
#endif
	} // end of: if(systick_counter_3 >= 200) //5Hz




	/* -------------------------------------------------------------------------  */

	if (systick_counter) //1ms
	{
		systick_counter = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

		for(i=2; i<4; i++) motor_data[i].torqueActual   = tmc4671_getActualTorque_raw(i);
		for(i=2; i<4; i++) motor_data[i].positionActual = tmc4671_getActualPosition(i);
		for(i=2; i<4; i++) motor_data[i].velocityActual = tmc4671_getActualVelocity(i);

		// for(i=0; i<4; i++) rxSPICplt[i] = 0;
 		// for(i=0; i<4; i++) tmc4671_readInt_nonBlocking(i, TMC4671_PID_POSITION_ACTUAL);
		// while(rxSPICplt[0]==0 || rxSPICplt[1]==0 || rxSPICplt[2]==0 || rxSPICplt[3]==0);
		// // for(i=0; i<4; i++) motor_data[i].positionActual = rxSPIData[i];
		// for(i=0; i<4; i++) motor_data[i].positionActual = (int32_t)((rxBuffer[i][1] << 24) | (rxBuffer[i][2] << 16) | (rxBuffer[i][3] << 8) | (rxBuffer[i][4] << 0));
		//
		// for(i=0; i<4; i++) rxSPICplt[i] = 0;
		// for(i=0; i<4; i++) tmc4671_readInt_nonBlocking(i, TMC4671_PID_VELOCITY_ACTUAL);
		// while(rxSPICplt[0]==0 || rxSPICplt[1]==0 || rxSPICplt[2]==0 || rxSPICplt[3]==0);
		// for(i=0; i<4; i++) motor_data[i].velocityActual = (int32_t)((rxBuffer[i][1] << 24) | (rxBuffer[i][2] << 16) | (rxBuffer[i][3] << 8) | (rxBuffer[i][4] << 0));






		for(i=0; i<4; i++) motor_control[i].positionActual = (float)motor_data[i].positionActual;
		for(i=0; i<4; i++) motor_control[i].velocityActual = (float)motor_data[i].velocityActual;



		if (mode == MODE_STOP)
		{
			for(i=0; i<4; i++) motor_control[i].torqueTarget = 0;
			for (i = 2; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
		}
		else if(mode == MODE_IDLE)
		{
		}
		else if (mode == MODE_TORQUE)
		{
			// for(i=0; i<4; i++) motor_data[i].torqueTarget = (int32_t)(motor_control[i].torqueTarget);
			// for(i=0; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);
		}
		else if (mode == MODE_POSITION || mode == MODE_RCCONTROL)
		{
			for (i = 2; i < 4; i++) positionPI(i);
			for (i = 2; i < 4; i++) velocityPI(i);

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;
			for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);
		}
		else if (mode == MODE_POSITION_NEW || mode == MODE_RCCONTROL_NEW)
		{
			// beta -> alpha
			for (i = 2; i < 4; i++) motor_control[i].velocityActual = clacAngleVelocityBetaAlpha(i, motor_control[i].positionActual/65536.0*360.0, motor_control[i].velocityActual);
			for (i = 2; i < 4; i++) motor_control[i].positionActual = calcAngleBetaAlpha(i, motor_control[i].positionActual/65536.0*360.0)*65536.0/360.0;

			// control at alpha
			for (i = 2; i < 4; i++) positionPICompensation(i);
			for (i = 2; i < 4; i++) velocityPICompensation(i);

			// alpha -> beta
			for (i = 2; i < 4; i++) motor_control[i].torqueTarget = calcTorqueAlphaBeta(i, motor_control[i].positionActual/65536.0*360.0, motor_control[i].torqueTarget);

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;
			for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);
		}
		else if(mode == MODE_POSITION_STEP)
		{
			if(sweep.k == 0)
			{
				for(i=0; i<4; i++) motor_control[i].angleIn = 0;
			}
			else if(sweep.k == 500)
			{
				if(sweep.mode == JMP_FLP)
				{
					motor_control[0].angleIn = 0;
					motor_control[1].angleIn = 0;
					motor_control[2].angleIn = 0;
					motor_control[3].angleIn = sweep.U;
				}
				else if(sweep.mode == JMP_AIL)
				{
					motor_control[0].angleIn = 0;
					motor_control[1].angleIn = 0;
					motor_control[2].angleIn = sweep.U;
					motor_control[3].angleIn = 0;
				}
			}
			else if(sweep.k == 1500)
			{
				for(i=0; i<4; i++) motor_control[i].angleIn = 0;
			}
			else if(sweep.k >= (DATA_N-1) ) // stop
			{
				for(i=0; i<4; i++) motor_control[i].angleIn = 0;
				stats = true;
				mode = MODE_POSITION;
				#if MATLAB
					snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				#endif
			}
			for(i=0; i<4; i++) motor_control[i].positionTarget = calcAngleTarget(i);

			for (i = 2; i < 4; i++) positionPI(i);
			for (i = 2; i < 4; i++) velocityPI(i);
			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;
			for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);

			for (i = 2; i < 4; i++) data1[sweep.k].torqueTarget[i-2] = (int32_t)motor_data[i].torqueTarget;
			for (i = 2; i < 4; i++) data1[sweep.k].torqueActual[i-2] = (int32_t)motor_data[i].torqueActual;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = (int32_t)motor_control[i].velocityActual;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = (int32_t)motor_control[i].velocityTarget;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityIntegratorValue[i-2] = (int32_t)motor_control[i].velocityIntegratorValue;

			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = (int32_t)motor_control[i].positionActual;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = (int32_t)motor_control[i].positionTarget;
			for (i = 2; i < 4; i++) data1[sweep.k].positionIntegratorValue[i-2] = (int32_t)motor_control[i].positionIntegratorValue;
			sweep.k++;

		}
        else if(mode == MODE_POSITION_SWEEP || mode == MODE_POSITION_SWEEP2)
		{
            sweep.t = sweep.k * sweep.Ta;
    		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
    		sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t	+ (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

            if(mode == MODE_POSITION_SWEEP && sweep.mode == JMP_FLP)
            {
                motor_control[0].angleIn = 0;
                motor_control[1].angleIn = 0;
                motor_control[2].angleIn = 0;
                motor_control[3].angleIn = sweep.out;
            }
            else if(mode == MODE_POSITION_SWEEP && sweep.mode == JMP_AIL)
            {
                motor_control[0].angleIn = 0;
                motor_control[1].angleIn = 0;
                motor_control[2].angleIn = sweep.out;
                motor_control[3].angleIn = 0;
            }
			else if(mode == MODE_POSITION_SWEEP2 && sweep.mode == JMP_AIL)
			{
				motor_control[0].angleIn = 0;
				motor_control[1].angleIn = 0;
				motor_control[2].angleIn = 0;
				motor_control[3].angleIn = 0;
			}

            for(i=0; i<4; i++) motor_control[i].positionTarget = calcAngleTarget(i);



			for (i = 0; i < 4; i++) positionPI(i);
			for (i = 0; i < 4; i++) velocityPI(i);
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;


			if(mode == MODE_POSITION_SWEEP2 && sweep.mode == JMP_AIL)
			{
				motor_data[2].torqueTarget = motor_data[2].torqueTarget + (int32_t)sweep.out;
				motor_data[2].torqueTarget = (int32_t)(sweep.out);
				motor_data[2].torqueTarget = (int32_t)(-sweep.out);
				//motor_data[3].torqueTarget = (int32_t)0;
			}


			for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);

			for (i = 2; i < 4; i++) data1[sweep.k].torqueTarget[i-2] = (int32_t)motor_data[i].torqueTarget;
			for (i = 2; i < 4; i++) data1[sweep.k].torqueActual[i-2] = (int32_t)motor_data[i].torqueActual;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = (int32_t)motor_control[i].velocityActual;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = (int32_t)motor_control[i].velocityTarget;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityIntegratorValue[i-2] = (int32_t)motor_control[i].velocityIntegratorValue;

			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = (int32_t)motor_control[i].positionActual;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = (int32_t)motor_control[i].positionTarget;
			for (i = 2; i < 4; i++) data1[sweep.k].positionIntegratorValue[i-2] = (int32_t)motor_control[i].positionIntegratorValue;


			if(sweep.k >= (DATA_N-1) ) // stop
            {
                for(i=0; i<4; i++) motor_control[i].angleIn = 0;
                mode = MODE_STOP;
                stats = true;

                #if MATLAB
                    snprintf(string, 200, 	"fintest\n");
                    HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
                #endif

            }


			sweep.k++;

		}
		else if(mode == MODE_CONTROL_TEST)
		{
            sweep.t = sweep.k * sweep.Ta;
    		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
    		sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t + (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

			 for (i = 0; i < 4; i++) motor_control[i].velocityTarget = sweep.out;
			 for (i = 0; i < 4; i++) motor_control[i].velocityActual = 0;


			velocityPICompensation(3);


			//for (i = 0; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;
			data2[sweep.k].torqueTarget   = motor_control[3].torqueTarget;
			data2[sweep.k].velocityActual = motor_control[3].velocityActual;
			data2[sweep.k].velocityTarget = motor_control[3].velocityTarget;
			data2[sweep.k].positionActual = motor_control[3].positionActual;
			data2[sweep.k].positionTarget = motor_control[3].positionTarget;

			if(sweep.k >= (DATA_N-1) ) // stop
            {
                mode = MODE_STOP;
                stats = true;

                #if MATLAB
                    snprintf(string, 200, 	"fintest\n");
                    HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
                #endif

            }


			sweep.k++;

		}


		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
	}


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
		case 'm':
			mode = MODE_IDLE;
			stats = true;
			//for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;


		case '1':
			mode = MODE_POSITION;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 15000;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
			motor_control[0].positionTarget = 0;
			motor_control[1].positionTarget = 0;
			motor_control[2].positionTarget = 0;
			motor_control[3].positionTarget = 0;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;


		case '2':
			mode = MODE_POSITION_NEW;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionTarget = 0;
			for (i = 0; i < 4; i++){
				for (j = 0; j < 2; j++) {
					motor_control[i].bq_pos_delay1[j] = 0;
					motor_control[i].bq_vel_delay1[j] = 0;
					motor_control[i].bq_vel_delay2[j] = 0;
					motor_control[i].bq_vel_delay3[j] = 0;
					motor_control[i].bq_vel_delay4[j] = 0;
				}
			}
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;


		case '3':
			sweep.k = 0;
			sweep.U = 7;
			sweep.mode = JMP_AIL;
			stats = false;
            if(mode==MODE_STOP)
            {
    			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 15000;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
            }
            mode = MODE_POSITION_STEP;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '4':

			sweep.k = 0;
			sweep.U = 5;
			sweep.mode = JMP_FLP;
			stats = false;
            if(mode==MODE_STOP)
            {
    			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 15000;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
            }
            mode = MODE_POSITION_STEP;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '5':
			sweep.k = 0;
			sweep.U = 10;
            sweep.mode = JMP_AIL;
			stats = false;
            if(mode==MODE_STOP)
            {
    			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 15000;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
            }
            mode = MODE_POSITION_SWEEP;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

        case '6':
			sweep.k = 0;
			sweep.U = 3;
            sweep.mode = JMP_FLP;
			stats = false;
            if(mode==MODE_STOP)
            {
    			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 15000;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
    			for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
            }
            mode = MODE_POSITION_SWEEP;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '7':
			sweep.k = 0;
			sweep.omegaStart = 2 * M_PI * 30,
			sweep.omegaEnd = 2 * M_PI * 300,
			sweep.U = 5000;
			sweep.mode = JMP_AIL;
			stats = false;
			if(mode==MODE_STOP)
			{
				for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
				for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
				for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
				for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorLimit = 15000;
				for (i = 0; i < 4; i++) motor_control[i].positionIntegratorValue = 0;
				for (i = 0; i < 4; i++) motor_control[i].positionIntegratorLimit = 5000;
			}
			mode = MODE_POSITION_SWEEP2;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '8':
			sweep.k = 0;
			sweep.omegaStart = 2 * M_PI * 8,
			sweep.omegaEnd = 2 * M_PI *80,
			sweep.U = 1;

			for (i = 0; i < 4; i++){
				for (j = 0; j < 2; j++) {
					motor_control[i].bq_pos_delay1[j] = 0;
					motor_control[i].bq_vel_delay1[j] = 0;
					motor_control[i].bq_vel_delay2[j] = 0;
					motor_control[i].bq_vel_delay3[j] = 0;
					motor_control[i].bq_vel_delay4[j] = 0;
				}
			}

			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			mode = MODE_CONTROL_TEST;
			break;




		case '9':
			mode = MODE_RCCONTROL_NEW;
			for (i = 0; i < 4; i++) motor_data[i].torqueTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTarget = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValue = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionTarget = 0;

			for (i = 0; i < 4; i++){
				for (j = 0; j < 2; j++) {
					motor_control[i].bq_pos_delay1[j] = 0;
					motor_control[i].bq_vel_delay1[j] = 0;
					motor_control[i].bq_vel_delay2[j] = 0;
					motor_control[i].bq_vel_delay3[j] = 0;
					motor_control[i].bq_vel_delay4[j] = 0;
				}
			}
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;


		case 'x': // print data
			for (i = 0; i < DATA_N; i++)
			{
                //snprintf(string, 1500, "%ld\n", i);
				print_data((int32_t)i,   							(uint8_t*)(data));
				print_data(data1[i].torqueTarget[0], 				(uint8_t*)(data)+4*1);
				print_data(data1[i].torqueActual[0],  				(uint8_t*)(data)+4*2);
				print_data(data1[i].velocityTarget[0],   			(uint8_t*)(data)+4*3);
				print_data(data1[i].velocityActual[0],  			(uint8_t*)(data)+4*4);
				print_data(data1[i].velocityIntegratorValue[0],   	(uint8_t*)(data)+4*5);
				print_data(data1[i].positionTarget[0],  			(uint8_t*)(data)+4*6);
				print_data(data1[i].positionActual[0],   			(uint8_t*)(data)+4*7);
				print_data(data1[i].positionIntegratorValue[0],  	(uint8_t*)(data)+4*8);
				print_data(data1[i].torqueTarget[1], 				(uint8_t*)(data)+4*9);
				print_data(data1[i].torqueActual[1],  				(uint8_t*)(data)+4*10);
				print_data(data1[i].velocityTarget[1],   			(uint8_t*)(data)+4*11);
				print_data(data1[i].velocityActual[1],  			(uint8_t*)(data)+4*12);
				print_data(data1[i].velocityIntegratorValue[1],   	(uint8_t*)(data)+4*13);
				print_data(data1[i].positionTarget[1],  			(uint8_t*)(data)+4*14);
				print_data(data1[i].positionActual[1],   			(uint8_t*)(data)+4*15);
				print_data(data1[i].positionIntegratorValue[1],  	(uint8_t*)(data)+4*16);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)data, 4*17);
				HAL_Delay(2);
			}
            HAL_Delay(1);
			break;

		case 'v': // print data
			for (i = 0; i < DATA_N; i++)
			{
				//snprintf(string, 1500, "%ld\n", i);
				print_data2((float)i,   						(uint8_t*)(data));
				print_data2(data2[i].torqueTarget, 				(uint8_t*)(data)+4*1);
				print_data2(data2[i].velocityTarget,   			(uint8_t*)(data)+4*2);
				print_data2(data2[i].velocityActual,  			(uint8_t*)(data)+4*3);
				print_data2(data2[i].positionTarget,   			(uint8_t*)(data)+4*4);
				print_data2(data2[i].positionActual,  			(uint8_t*)(data)+4*5);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)data, 4*6);
				HAL_Delay(1);
			}
			HAL_Delay(1);
			break;

		case 'y':
//			for (i = 0; i < 4; i++) angle[i] = as5047U_getAngle(i);
			snprintf(string, 1500, "%s", clear_string);
			//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(0));
			//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(1));
			snprintf(string + strlen(string), 1500, "%s", TMC4671_highLevel_getStatus(2));
			snprintf(string + strlen(string), 1500, "%s", TMC4671_highLevel_getStatus(3));
			// snprintf(string+strlen(string), 1500-strlen(string), "Outer\n");
			// // snprintf(string+strlen(string), 1500-strlen(string), "Position Error  %.1f\n", motor_control[2].positionError);
			// // snprintf(string+strlen(string), 1500-strlen(string), "Velocity Error  %.1f\n", motor_control[2].velocityError);
			// snprintf(string+strlen(string), 1500-strlen(string), "---------------------------\n");
			// snprintf(string+strlen(string), 1500-strlen(string), "Inner\n");
			// snprintf(string+strlen(string), 1500-strlen(string), "Position Error  %.1f\n", motor_control[3].positionError);
			// snprintf(string+strlen(string), 1500-strlen(string), "Velocity Error  %.1f\n", motor_control[3].velocityError);
			snprintf(string+strlen(string), 1500-strlen(string), "---------------------------\n");
            snprintf(string+strlen(string), 1500-strlen(string), "Pos Target:  % 2.1f  % 2.1f\n", motor_control[2].positionTarget, motor_control[3].positionTarget);
            snprintf(string+strlen(string), 1500-strlen(string), "Pos Actual:  % 2.1f  % 2.1f\n", motor_control[2].positionActual, motor_control[3].positionActual);
            snprintf(string+strlen(string), 1500-strlen(string), "Error:       % 2.1f  % 2.1f\n", motor_control[2].positionError,  motor_control[3].positionError );
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



void velocityPICompensation(uint8_t drv)
{
	float A_PI = 1;
	float B_PI = 16;
	float C_PI = 15.6764221;
	float D_PI = 1.2241776e+03;


	static float velocityIntegratorLimit = 50000;


	motor_control[drv].velocityError = motor_control[drv].velocityTarget - motor_control[drv].velocityActual;

	motor_control[drv].torqueTarget_Limited = C_PI * motor_control[drv].velocityIntegratorValue + D_PI * motor_control[drv].velocityError;

	if (motor_control[drv].torqueTarget_Limited > velocityIntegratorLimit)
	{
		motor_control[drv].velocityIntegratorValue = ((velocityIntegratorLimit - D_PI*motor_control[drv].velocityError) / C_PI);
		motor_control[drv].torqueTarget_Limited = velocityIntegratorLimit;
	}
	else if (motor_control[drv].torqueTarget_Limited < (-velocityIntegratorLimit))
	{
		motor_control[drv].velocityIntegratorValue = ((-velocityIntegratorLimit - D_PI*motor_control[drv].velocityError) / C_PI);
		motor_control[drv].torqueTarget_Limited = -velocityIntegratorLimit;
	}
	else
	{
		motor_control[drv].velocityIntegratorValue =  A_PI*motor_control[drv].velocityIntegratorValue + B_PI*motor_control[drv].velocityError;
	}


	//matched
	float coeffs1[6] = {1.0000000,	-1.7683651,	0.7704546,	1.0000000,	-1.7843640,	0.7892329};
	float coeffs2[6] = {1.0000000,	-1.9798883,	0.9853169,	1.0000000,	-0.9999964,	0.0000000};
	float coeffs3[6] = {1.0000000,	-1.9301999,	0.9322069,	1.0000000,	-1.9671578,	0.9685792};
	float coeffs4[6] = {1.0000000,	-1.9824102,	0.9838421,	1.0000000,	-1.9835303,	0.9855943};
	float gain = 0.2195863;

	// motor_control[drv].torqueTarget_Limited = motor_control[drv].velocityError;

	motor_control[drv].bq_intermediate1 = biquad(motor_control[drv].torqueTarget_Limited, coeffs1, 1.0,  motor_control[drv].bq_vel_delay1);
	motor_control[drv].bq_intermediate2 = biquad(motor_control[drv].bq_intermediate1,     coeffs2, 1.0,  motor_control[drv].bq_vel_delay2);
	motor_control[drv].bq_intermediate3 = biquad(motor_control[drv].bq_intermediate2,     coeffs3, 1.0,  motor_control[drv].bq_vel_delay3);
	motor_control[drv].torqueTarget     = biquad(motor_control[drv].bq_intermediate3,     coeffs4, gain, motor_control[drv].bq_vel_delay4);

}



float biquad(float in, float* coeffs, float gain, float* w_)
{
	float out;
	float w;

	w = in - coeffs[4]*w_[0] - coeffs[5]*w_[1];
	out = coeffs[0]*w + coeffs[1]*w_[0] + coeffs[2]*w_[1];
	out *= gain;

	w_[1] = w_[0];
	w_[0] = w;

	return out;
}

void positionPICompensation(uint8_t drv)
{
	float coeffs[6] = {1.0000000,	-1.2987443,	0.2987663,	1.0000000,	-0.7779691,	-0.2220309};
	float gain = 0.5108376;

	motor_control[drv].positionError = motor_control[drv].positionTarget - motor_control[drv].positionActual;

 	motor_control[drv].velocityTarget = biquad(motor_control[drv].positionError, coeffs, gain, motor_control[drv].bq_pos_delay1);
}





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


int32_t calcAngleTarget(uint8_t drv)
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

float clacAngleVelocityBetaAlpha(uint8_t drv, float angleBeta, float velocityBeta)
{
	float veloctyAlpha = 0;
	float dalphadbeta = 0;
	float angleBetaSat = 0;

	if(angleBeta > 50.0)
		angleBetaSat = 50.0;
	else if(angleBeta < -50.0)
		angleBetaSat = -50.0;
	else
		angleBetaSat = angleBeta;

	if (drv == 0)
	{
		veloctyAlpha = velocityBeta/2.0;
	}
	else if (drv == 2)
	{
		veloctyAlpha = velocityBeta/2.0;
	}
	else if (drv == 1)
	{
		angleBetaSat = -angleBetaSat;
		dalphadbeta =     0.400935427396172939e0
						+(-0.151316755738938497e-3) * angleBetaSat
						+(-0.903150716409656768e-5) * angleBetaSat * angleBetaSat
						+(-0.138545436985387085e-7) * pow(angleBetaSat, 3)
						+  0.177013373152695001e-9  * pow(angleBetaSat, 4)
						+  0.441290283747026576e-12 * pow(angleBetaSat, 5)
						+(-0.543487101064459958e-13)* pow(angleBetaSat, 6) ;

		veloctyAlpha = dalphadbeta * (velocityBeta/2.0) ;
	}
	else if (drv == 3)
	{
		dalphadbeta =     0.400935427396172939e0
						+(-0.151316755738938497e-3) * angleBetaSat
						+(-0.903150716409656768e-5) * angleBetaSat * angleBetaSat
						+(-0.138545436985387085e-7) * pow(angleBetaSat, 3)
						+  0.177013373152695001e-9  * pow(angleBetaSat, 4)
						+  0.441290283747026576e-12 * pow(angleBetaSat, 5)
						+(-0.543487101064459958e-13)* pow(angleBetaSat, 6) ;
		dalphadbeta = 1;
		veloctyAlpha = dalphadbeta * (velocityBeta/2.0) ;
	}
	return veloctyAlpha;
}

float calcAngleBetaAlpha(uint8_t drv, float angleBeta) // in degree
{
	float angleAlpha = 0;
	float angleBetaSat = 0;

	angleBetaSat = angleBeta;


	if (drv == 0)
	{
		angleAlpha = angleBetaSat/2.0;
	}
	else if (drv == 2)
	{
		angleAlpha = angleBetaSat/2.0;
	}
	else if (drv == 1)
	{
		// angleBetaSat = angleBetaSat;
		angleAlpha = (  0.400958225615243*angleBetaSat
					  - 0.926978370360763e-4*angleBetaSat*angleBetaSat
					  - 3.02679825358732e-6*angleBetaSat*angleBetaSat*angleBetaSat )/2.0;
		angleAlpha = angleBetaSat/2.0;
	}
	else if (drv == 3)
	{
		angleAlpha = (  0.400958225615243*angleBetaSat
					  - 0.926978370360763e-4*angleBetaSat*angleBetaSat
					  - 3.02679825358732e-6*angleBetaSat*angleBetaSat*angleBetaSat )/2.0;
		angleAlpha = angleBetaSat/2.0;
	}

	return angleAlpha;
}





float calcTorqueAlphaBeta(uint8_t drv, float angleAlpha, float torqueAlpha)
{
	float torqueBeta = 0;
	float angleAlphaSat = 0;

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
		torqueBeta = torqueAlpha/2.0;
	}
	else if (drv == 2)
	{
		torqueBeta = torqueAlpha/2.0;
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

float sat(float x)
{
	if (x >= 1)
		return 1;
	else if (x <= -1)
		return -1;
	else
		return x;
}


void print_data(int32_t data , uint8_t* string)
{
	string[0] = (data & 0x000000ff)        ;
	string[1] = ((data & 0x0000ff00) >> 8 );
	string[2] = ((data & 0x00ff0000) >> 16);
	string[3] = ((data & 0xff000000) >> 24);
}

void print_data2(float data , uint8_t* string)
{
	union {
	    float data_float;
	    uint8_t bytes[4];
	  } thing;

	thing.data_float = data;

	memcpy(string, thing.bytes, 4);

	// string[0] = ( thing.data_int32_t & 0x000000ff)        ;
	// string[1] = ((thing.data_int32_t & 0x0000ff00) >> 8 );
	// string[2] = ((thing.data_int32_t & 0x00ff0000) >> 16);
	// string[3] = ((thing.data_int32_t & 0xff000000) >> 24);
}

// void print_data2(float data , uint8_t* string)
// {
// 	static union {
// 	    float data_float;
// 	    uint32_t data_int32_t;
// 	  } thing;
//
// 	thing.data_float = data;
//
// 	string[0] = ( thing.data_int32_t & 0x000000ff)        ;
// 	string[1] = ((thing.data_int32_t & 0x0000ff00) >> 8 );
// 	string[2] = ((thing.data_int32_t & 0x00ff0000) >> 16);
// 	string[3] = ((thing.data_int32_t & 0xff000000) >> 24);
// }

// //outer
// motor_control[0].velocityP = 10;
// motor_control[0].velocityI = 10;
// motor_control[0].positionP = 0.2;
// motor_control[0].positionI = 0.5;
//
// motor_control[2].velocityP = 10;
// motor_control[2].velocityI = 10;
// motor_control[2].positionP = 0.2;
// motor_control[2].positionI = 0.5;
//
// //inner
// motor_control[1].velocityP = 10;
// motor_control[1].velocityI = 10;
// motor_control[1].positionP = 0.2;
// motor_control[1].positionI = 0.5;
//
// motor_control[3].velocityP = 10;
// motor_control[3].velocityI = 10;
// motor_control[3].positionP = 0.2;
// motor_control[3].positionI = 0.5;


//
// case '+':
// 	break;
//
// case '-':
// 	break;
//
// case 'w':
// 	motor_control[0].velocityP += 1.0;
// 	motor_control[2].velocityP += 1.0;
// 	break;
//
// case 's':
// 	if (motor_control[0].velocityP >= 1.0)
// 	{
// 		motor_control[0].velocityP -= 1.0;
// 		motor_control[2].velocityP -= 1.0;
// 	}
// 	else
// 	{
// 		motor_control[0].velocityP = 0.0;
// 		motor_control[2].velocityP = 0.0;
// 	}
// 	break;
//
// case 'e':
// 	motor_control[0].velocityI += 1.0;
// 	motor_control[2].velocityI += 1.0;
// 	break;
//
// case 'd':
// 	if (motor_control[0].velocityI >= 1.0)
// 	{
// 		motor_control[0].velocityI -= 1.0;
// 		motor_control[2].velocityI -= 1.0;
// 	}
//
// 	else
// 	{
// 		motor_control[0].velocityI = 0.0;
// 		motor_control[2].velocityI = 0.0;
// 	}
// 	break;
//
//
// case 'r':
// 	motor_control[0].positionP += 0.01;
// 	motor_control[2].positionP += 0.01;
// 	break;
//
// case 'f':
// 	if (motor_control[0].positionP >= 0.01)
// 	{
// 		motor_control[0].positionP -= 0.01;
// 		motor_control[2].positionP -= 0.01;
// 	}
// 	else
// 	{
// 		motor_control[0].positionP = 0.0;
// 		motor_control[2].positionP = 0.0;
// 	}
// 	break;
//
// case 't':
// 	motor_control[0].positionI += 0.1;
// 	motor_control[2].positionI += 0.1;
// 	break;
//
// case 'g':
// 	if (motor_control[0].positionI >= 0.1)
// 	{
// 		motor_control[0].positionI -= 0.1;
// 		motor_control[2].positionI -= 0.1;
// 	}
// 	else
// 	{
// 		motor_control[0].positionI = 0.0;
// 		motor_control[2].positionI = 0.0;
// 	}
// 	break;
//
//
// case 'z':
// 	motor_control[1].velocityP += 1.0;
// 	motor_control[3].velocityP += 1.0;
// 	break;
//
// case 'h':
// 	if (motor_control[1].velocityP >= 1.0)
// 	{
// 		motor_control[1].velocityP -= 1.0;
// 		motor_control[3].velocityP -= 1.0;
// 	}
// 	else
// 	{
// 		motor_control[1].velocityP = 0.0;
// 		motor_control[3].velocityP = 0.0;
// 	}
// 	break;
//
// case 'u':
// 	motor_control[1].velocityI += 1.0;
// 	motor_control[3].velocityI += 1.0;
// 	break;
//
// case 'j':
// 	if (motor_control[1].velocityI >= 1.0)
// 	{
// 		motor_control[1].velocityI -= 1.0;
// 		motor_control[3].velocityI -= 1.0;
// 	}
//
// 	else
// 	{
// 		motor_control[1].velocityI = 0.0;
// 		motor_control[3].velocityI = 0.0;
// 	}
// 	break;
//
//
// case 'i':
// 	motor_control[1].positionP += 0.01;
// 	motor_control[3].positionP += 0.01;
// 	break;
//
// case 'k':
// 	if (motor_control[1].positionP >= 0.01)
// 	{
// 		motor_control[1].positionP -= 0.01;
// 		motor_control[3].positionP -= 0.01;
// 	}
// 	else
// 	{
// 		motor_control[1].positionP = 0.0;
// 		motor_control[3].positionP = 0.0;
// 	}
// 	break;
//
// case 'o':
// 	motor_control[1].positionI += 0.1;
// 	motor_control[3].positionI += 0.1;
// 	break;
//
// case 'l':
// 	if (motor_control[1].positionI >= 0.1)
// 	{
// 		motor_control[1].positionI -= 0.1;
// 		motor_control[3].positionI -= 0.1;
// 	}
// 	else
// 	{
// 		motor_control[1].positionI = 0.0;
// 		motor_control[3].positionI = 0.0;
// 	}
// 	break;
//
// void positionPICompensation(uint8_t drv)
// {
// 	uint8_t i = 0;
// 	uint8_t j = 0;
//
// 	static float A_filt[ORDER_POS_FILT][ORDER_POS_FILT] =
// 	{
// 		{ 1.72848950367273368478, -0.72848950367273368478 },
// 		{ 1.00000000000000000000, 0.00000000000000000000 }
// 	};
//
// 	static float B_filt[ORDER_POS_FILT] =
// 	{
// 		0.25000000000000000000,
// 		0.00000000000000000000
// 	};
//
// 	static float C_filt[ORDER_POS_FILT] =
// 	{
// 		-0.06638595940176683641, 	0.06648163083098512782
// 	};
//
// 	static float D_filt = 0.34197119797002761832;
//
//
//
// 	motor_control[drv].positionError = motor_control[drv].positionTarget - motor_control[drv].positionActual;
//
//
// 	motor_control[drv].velocityTarget = 0;
// 	for (i = 0; i < ORDER_POS_FILT; i++)
// 		motor_control[drv].velocityTarget += C_filt[i] * motor_control[drv].x_pos_filt[i];
// 	motor_control[drv].velocityTarget += D_filt * motor_control[drv].positionError;
//
// 	for (i = 0; i < ORDER_POS_FILT; i++)
// 	{
// 		motor_control[drv].x_vel_filt_[i] = 0;
// 		for (j = 0; j < ORDER_POS_FILT; j++)
// 			motor_control[drv].x_pos_filt_[i] += A_filt[i][j] * motor_control[drv].x_pos_filt[j];
// 		motor_control[drv].x_pos_filt_[i] += B_filt[i]* motor_control[drv].positionError;
// 	}
//
// 	for (i = 0; i < ORDER_POS_FILT; i++)
// 		motor_control[drv].x_pos_filt[i] = motor_control[drv].x_pos_filt_[i];
// }


// void velocityPICompensation(uint8_t drv)
// {
// 	uint8_t i = 0;
// 	uint8_t j = 0;
//
// 	static float A_filt[ORDER_VEL_FILT][ORDER_VEL_FILT] =
// 	{
// 		{ 6.73504656701971526900, -4.85743255179843558267, 3.89021969280977586436, -1.86816884082702761027, 1.07582260048542144304, -0.68785967574535655800, 0.18833583728109382083, -0.00000000000000000000 },
// 		{ 4.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000 },
// 		{ 0.00000000000000000000, 2.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000 },
// 		{ 0.00000000000000000000, 0.00000000000000000000, 2.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000 },
// 		{ 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000 },
// 		{ 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.50000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000 },
// 		{ 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.50000000000000000000, 0.00000000000000000000, 0.00000000000000000000 },
// 		{ 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, 0.50000000000000000000, 0.00000000000000000000 }
// 	};
//
// 	static float B_filt[ORDER_VEL_FILT] =
// 	{
// 		2.00000000000000000000,
// 		0.00000000000000000000,
// 		0.00000000000000000000,
// 		0.00000000000000000000,
// 		0.00000000000000000000,
// 		0.00000000000000000000,
// 		0.00000000000000000000,
// 		0.00000000000000000000
// 	};
//
// 	static float C_filt[ORDER_VEL_FILT] =
// 	{
// 		-0.10162500713287991871, 0.17110492862099258016, -0.24678206859814716401, 0.19760940300879589748, -0.18974744035444143786, 0.21847115984221629481, -0.13963092676207458642, 	0.03821243363925096170
// 	};
//
// 	static float D_filt = 0.21961809407685534135;
//
// 	static float Kp = 1098.76622274372357423999;
// 	static float Ki = 456.55345486531837195798;
//
//
// 	static float velocityIntegratorLimit = 10000;
//
//
// 	motor_control[drv].velocityError = motor_control[drv].velocityTarget - motor_control[drv].velocityActual;
//
// 	motor_control[drv].torqueTarget_Limited = Ki * motor_control[drv].velocityIntegratorValue + Kp * motor_control[drv].velocityError;
//
// 	if (motor_control[drv].torqueTarget_Limited > velocityIntegratorLimit)
// 	{
// 		if (motor_control[drv].velocityI > 0)
// 			motor_control[drv].velocityIntegratorValue = (float) (((float) velocityIntegratorLimit - Kp* (float) motor_control[drv].velocityError) / Ki);
// 		else
// 			motor_control[drv].velocityIntegratorValue = 0;
//
// 		motor_control[drv].torqueTarget_Limited = velocityIntegratorLimit;
// 	}
// 	else if (motor_control[drv].torqueTarget_Limited < (-velocityIntegratorLimit))
// 	{
// 		if (motor_control[drv].velocityI > 0)
// 			motor_control[drv].velocityIntegratorValue = (float) ((-velocityIntegratorLimit - Kp * (float) motor_control[drv].velocityError) / Ki);
// 		else
// 			motor_control[drv].velocityIntegratorValue = 0;
//
// 		motor_control[drv].torqueTarget_Limited = -velocityIntegratorLimit;
// 	}
// 	else
// 	{
// 		motor_control[drv].velocityIntegratorValue = motor_control[drv].velocityIntegratorValue + TA * (float) motor_control[drv].velocityError;
// 	}
//
//
//
//
//
// 	motor_control[drv].torqueTarget = 0;
// 	for (i = 0; i < ORDER_VEL_FILT; i++)
// 		motor_control[drv].torqueTarget += C_filt[i] * motor_control[drv].x_vel_filt[i];
// 	motor_control[drv].torqueTarget += D_filt * motor_control[drv].torqueTarget_Limited;
//
// 	for (i = 0; i < ORDER_VEL_FILT; i++)
// 	{
// 		motor_control[drv].x_vel_filt_[i] = 0;
// 		for (j = 0; j < ORDER_VEL_FILT; j++)
// 			motor_control[drv].x_vel_filt_[i] += A_filt[i][j] * motor_control[drv].x_vel_filt[j];
// 		motor_control[drv].x_vel_filt_[i] += B_filt[i]* motor_control[drv].torqueTarget_Limited;
// 	}
//
// 	for (i = 0; i < ORDER_VEL_FILT; i++)
// 		motor_control[drv].x_vel_filt[i] = motor_control[drv].x_vel_filt_[i];
// }
