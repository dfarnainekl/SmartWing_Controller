#include "logic.h"



#define MODE_STOP				0
#define MODE_TORQUE				1
#define MODE_VELOCITY			2
#define MODE_POSITION			3
#define MODE_RCCONTROL			4
#define MODE_TORQUE_SWEEP		5
#define MODE_IDLE				6
#define MODE_CONTROL_TEST		7
#define MODE_POSITION_STEP		8


uint8_t mode = MODE_STOP;
sweep_t sweep = {.Ta = TA, .N = DATA_N};

__attribute__((section(".ram_d1")))  data1_t data1[DATA_N] = { 0 };
__attribute__((section(".ram_d2")))  data2_t data2[DATA_N] = { 0 };

motor_t motor_data[4] = { 0 };
control_t motor_control[4] = { 0 };

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
		// Should read: TMC6200 GSTAT = 0, TMC6200 GCONF = 48, TMC6200 DRV_CONF = 0, as5074u Errors = 0
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
#endif
	}
	rx_byte_new = 0;
	HAL_Delay(100);


	initPIControl();


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
	static uint32_t n = 0;


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

		if(mode == MODE_RCCONTROL)
		{
			//for(i=0; i<4; i++) motor_control[i].angleIn =   ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree

			 motor_control[2].angleIn =   ( (float)(pwm_in[0] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree
			 motor_control[3].angleIn =   ( (float)(pwm_in[1] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree


			for(i=0; i<4; i++) motor_control[i].positionTargetGamma = motor_control[i].angleIn;
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
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Gamma:\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Target:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionTargetGamma, motor_control[1].positionTargetGamma, motor_control[2].positionTargetGamma, motor_control[3].positionTargetGamma);
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionActualGamma, motor_control[1].positionActualGamma, motor_control[2].positionActualGamma, motor_control[3].positionActualGamma);
        snprintf(string+strlen(string), 2000-strlen(string), "Error:       % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionErrorGamma, motor_control[1].positionErrorGamma, motor_control[2].positionErrorGamma, motor_control[3].positionErrorGamma);
    	snprintf(string+strlen(string), 2000-strlen(string), "I Target:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].torqueTargetGamma, motor_control[1].torqueTargetGamma, motor_control[2].torqueTargetGamma, motor_control[3].torqueTargetGamma);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Alpha:\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionActualAlpha, motor_control[1].positionActualAlpha, motor_control[2].positionActualAlpha, motor_control[3].positionActualAlpha);
		snprintf(string+strlen(string), 2000-strlen(string), "I Target:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].torqueTargetAlpha, motor_control[1].torqueTargetAlpha, motor_control[2].torqueTargetAlpha, motor_control[3].torqueTargetAlpha);
        snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Beta:\n");
		snprintf(string+strlen(string), 2000-strlen(string), "Pos Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionActualBeta, motor_control[1].positionActualBeta, motor_control[2].positionActualBeta, motor_control[3].positionActualBeta);
		snprintf(string+strlen(string), 2000-strlen(string), "I Target:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].torqueTargetBeta, motor_control[1].torqueTargetBeta, motor_control[2].torqueTargetBeta, motor_control[3].torqueTargetBeta);
		snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");

		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
#endif
	} // end of: if(systick_counter_3 >= 200) //5Hz




	/* -------------------------------------------------------------------------  */

	if (systick_counter) //Ta = 500us
	{
		systick_counter = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

		//for(i=2; i<4; i++) motor_data[i].torqueActual   = tmc4671_getActualTorque_raw(i);
		for(i=2; i<4; i++) motor_data[i].positionActual = tmc4671_getActualPosition(i);
		for(i=2; i<4; i++) motor_data[i].velocityActual = tmc4671_getActualVelocity(i);



		for(i=0; i<4; i++) motor_control[i].positionActualBeta = (float)motor_data[i].positionActual/65536.0*360.0; //now in degree
		for(i=0; i<4; i++) motor_control[i].velocityActualBeta = (float)motor_data[i].velocityActual; 				//in RPM

		for(i=0; i<4; i++) motor_control[i].positionActualAlpha = calcAngleBetaAlpha(i, motor_control[i].positionActualBeta);
		for(i=0; i<4; i++) motor_control[i].velocityActualAlpha = clacAngleVelocityBetaAlpha(i, motor_control[i].positionActualBeta, motor_control[i].velocityActualBeta);

		//state transform
		motor_control[0].positionActualGamma = (motor_control[1].positionActualAlpha-motor_control[0].positionActualAlpha)/2.0;
		motor_control[1].positionActualGamma = (motor_control[1].positionActualAlpha+motor_control[0].positionActualAlpha)/2.0;
		motor_control[2].positionActualGamma = (motor_control[3].positionActualAlpha-motor_control[2].positionActualAlpha)/2.0;
		motor_control[3].positionActualGamma = (motor_control[3].positionActualAlpha+motor_control[2].positionActualAlpha)/2.0;

		motor_control[0].velocityActualGamma = (motor_control[1].velocityActualAlpha-motor_control[0].velocityActualAlpha)/2.0;
		motor_control[1].velocityActualGamma = (motor_control[1].velocityActualAlpha+motor_control[0].velocityActualAlpha)/2.0;
		motor_control[2].velocityActualGamma = (motor_control[3].velocityActualAlpha-motor_control[2].velocityActualAlpha)/2.0;
		motor_control[3].velocityActualGamma = (motor_control[3].velocityActualAlpha+motor_control[2].velocityActualAlpha)/2.0;



		if (mode == MODE_STOP)
		{
			for (i = 2; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
		}
		else if(mode == MODE_IDLE)
		{
		}
		else if (mode == MODE_TORQUE)
		{
			motor_control[2].torqueTargetGamma = 7000; 	// Gegentakt
			motor_control[3].torqueTargetGamma = 0;				// Gleichtakt


			motor_control[0].torqueTargetAlpha = (motor_control[1].torqueTargetGamma - motor_control[0].torqueTargetGamma);
			motor_control[1].torqueTargetAlpha = (motor_control[1].torqueTargetGamma + motor_control[0].torqueTargetGamma);
			motor_control[2].torqueTargetAlpha = (motor_control[3].torqueTargetGamma - motor_control[2].torqueTargetGamma);
			motor_control[3].torqueTargetAlpha = (motor_control[3].torqueTargetGamma + motor_control[2].torqueTargetGamma);

			for (i = 2; i < 4; i++) motor_control[i].torqueTargetBeta = calcTorqueAlphaBeta(i, motor_control[i].positionActualAlpha, motor_control[i].torqueTargetAlpha);
			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTargetBeta;

			for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);

		}
		else if (mode == MODE_POSITION || mode == MODE_RCCONTROL)
		{
			// control at alpha
			for (i = 2; i < 4; i++) positionPICompensation(i);
			for (i = 2; i < 4; i++) velocityPICompensation(i);

			// state transform
			motor_control[0].torqueTargetAlpha = (motor_control[1].torqueTargetGamma - motor_control[0].torqueTargetGamma);
			motor_control[1].torqueTargetAlpha = (motor_control[1].torqueTargetGamma + motor_control[0].torqueTargetGamma);
			motor_control[2].torqueTargetAlpha = (motor_control[3].torqueTargetGamma - motor_control[2].torqueTargetGamma);
			motor_control[3].torqueTargetAlpha = (motor_control[3].torqueTargetGamma + motor_control[2].torqueTargetGamma);





			for (i = 2; i < 4; i++) motor_control[i].torqueTargetBeta = calcTorqueAlphaBeta(i, motor_control[i].positionActualAlpha, motor_control[i].torqueTargetAlpha);



			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTargetBeta;
			//for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);
		}
        else if(mode == MODE_TORQUE_SWEEP)
		{
            sweep.t = sweep.k * sweep.Ta;
    		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
    		sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t	+ (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

			if(sweep.mode == SINE)
			{


			}


			motor_control[2].torqueTargetGamma = 0; 	// Gegentakt
			motor_control[3].torqueTargetGamma = sweep.out;				// Gleichtakt


			motor_control[0].torqueTargetAlpha = (motor_control[1].torqueTargetGamma - motor_control[0].torqueTargetGamma);
			motor_control[1].torqueTargetAlpha = (motor_control[1].torqueTargetGamma + motor_control[0].torqueTargetGamma);
			motor_control[2].torqueTargetAlpha = (motor_control[3].torqueTargetGamma - motor_control[2].torqueTargetGamma);
			motor_control[3].torqueTargetAlpha = (motor_control[3].torqueTargetGamma + motor_control[2].torqueTargetGamma);

			motor_control[0].torqueTargetAlpha *= (1.0+0.0262*1);
			motor_control[1].torqueTargetAlpha *= (1.0+0.1638*1);
			motor_control[2].torqueTargetAlpha *= (1.0+0.0262*1);
			motor_control[3].torqueTargetAlpha *= (1.0+0.1638*1);


			for (i = 2; i < 4; i++) motor_control[i].torqueTargetBeta = calcTorqueAlphaBeta(i, motor_control[i].positionActualAlpha, motor_control[i].torqueTargetAlpha);

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTargetBeta;
			for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);


			for (i = 2; i < 4; i++) data1[sweep.k].torqueTarget[i-2]   = motor_control[i].torqueTargetGamma;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = motor_control[i].velocityActualGamma;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = motor_control[i].velocityTargetGamma;
			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = motor_control[i].positionActualGamma;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = motor_control[i].positionTargetGamma;



			if(sweep.k >= (DATA_N-1) ) // stop
            {
                mode = MODE_STOP;
                stats = true;

                #if MATLAB
                    snprintf(string, 200, 	"fintest\n");
                    HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
                #endif
            }

			n++;
			//sweep.k++;

			if(sweep.mode == HIGH)
			{
				sweep.k++;
			}
			else if(sweep.mode == MID)
			{
				if (n%2 == 0)
	   				sweep.k++;
			}
			else if(sweep.mode == LOW)
			{
				if (n%5 == 0)
					sweep.k++;
			}
			else if(sweep.mode == SINE)
			{
				sweep.k++;
			}

		}
		else if(mode == MODE_CONTROL_TEST)
		{
            sweep.t = sweep.k * sweep.Ta;
    		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
    		sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t + (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

			 for (i = 0; i < 4; i++) motor_control[i].velocityTargetGamma = sweep.out;
			 for (i = 0; i < 4; i++) motor_control[i].velocityActualGamma = 0;


			velocityPICompensation(3);


			// //for (i = 0; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;
			// data2[sweep.k].torqueTarget   = motor_control[3].torqueTargetGamma;
			// data2[sweep.k].velocityActual = motor_control[3].velocityActualGamma;
			// data2[sweep.k].velocityTarget = motor_control[3].velocityTargetGamma;
			// data2[sweep.k].positionActual = motor_control[3].positionActualGamma;
			// data2[sweep.k].positionTarget = motor_control[3].positionTargetGamma;

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
		// else if(mode == MODE_POSITION_STEP)
		// {
		// 	if(sweep.k == 0)
		// 	{
		// 		for(i=0; i<4; i++) motor_control[i].angleIn = 0;
		// 	}
		// 	else if(sweep.k == 500)
		// 	{
		// 		if(sweep.mode == JMP_FLP)
		// 		{
		// 			motor_control[0].angleIn = 0;
		// 			motor_control[1].angleIn = 0;
		// 			motor_control[2].angleIn = 0;
		// 			motor_control[3].angleIn = sweep.U;
		// 		}
		// 		else if(sweep.mode == JMP_AIL)
		// 		{
		// 			motor_control[0].angleIn = 0;
		// 			motor_control[1].angleIn = 0;
		// 			motor_control[2].angleIn = sweep.U;
		// 			motor_control[3].angleIn = 0;
		// 		}
		// 	}
		// 	else if(sweep.k == 1500)
		// 	{
		// 		for(i=0; i<4; i++) motor_control[i].angleIn = 0;
		// 	}
		// 	else if(sweep.k >= (DATA_N-1) ) // stop
		// 	{
		// 		for(i=0; i<4; i++) motor_control[i].angleIn = 0;
		// 		stats = true;
		// 		mode = MODE_POSITION;
		// 		#if MATLAB
		// 			snprintf(string, 200, 	"fintest\n");
		// 			HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		// 		#endif
		// 	}
		// 	for(i=0; i<4; i++) motor_control[i].positionTarget = calcAngleTarget(i);
		//
		// 	//for (i = 2; i < 4; i++) positionPI(i);
		// 	//for (i = 2; i < 4; i++) velocityPI(i);
		// 	for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)motor_control[i].torqueTarget;
		// 	for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);
		//
		// 	for (i = 2; i < 4; i++) data1[sweep.k].torqueTarget[i-2] = (int32_t)motor_data[i].torqueTarget;
		// 	for (i = 2; i < 4; i++) data1[sweep.k].torqueActual[i-2] = (int32_t)motor_data[i].torqueActual;
		// 	for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = (int32_t)motor_control[i].velocityActual;
		// 	for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = (int32_t)motor_control[i].velocityTarget;
		// 	for (i = 2; i < 4; i++) data1[sweep.k].velocityIntegratorValue[i-2] = (int32_t)motor_control[i].velocityIntegratorValue;
		//
		// 	for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = (int32_t)motor_control[i].positionActual;
		// 	for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = (int32_t)motor_control[i].positionTarget;
		// 	for (i = 2; i < 4; i++) data1[sweep.k].positionIntegratorValue[i-2] = (int32_t)motor_control[i].positionIntegratorValue;
		// 	sweep.k++;
		//
		// }


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
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 10000;
			sweep.omegaStart = 2 * M_PI * 1,
			sweep.omegaEnd = 2 * M_PI * 20,
			sweep.mode = LOW;
			n = 0;
			stats = true;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '2':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 5000;
			sweep.omegaStart = 2 * M_PI * 6,
			sweep.omegaEnd = 2 * M_PI * 60,
			sweep.mode = MID;
			n = 0;
			stats = true;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '3':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 10000;
			sweep.omegaStart = 2 * M_PI * 30,
			sweep.omegaEnd = 2 * M_PI * 300,
			sweep.mode = HIGH;
			n = 0;
			stats = true;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;





		case '4':
			mode = MODE_TORQUE; // test torque calcluations
			stats = true;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case '5':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 5000;
			sweep.omegaStart = 2 * M_PI * 5,
			sweep.omegaEnd = 2 * M_PI * 300,
			sweep.mode = SINE;
			n = 0;
			stats = true;
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
			mode = MODE_RCCONTROL;
			for (i = 0; i < 4; i++) motor_control[i].torqueTargetGamma = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityTargetGamma = 0;
			for (i = 0; i < 4; i++) motor_control[i].velocityIntegratorValueGamma = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionTargetGamma = 0;

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
				print_data2((float)i,   								(uint8_t*)(data));
				print_data2(data1[i].torqueTarget[0], 				(uint8_t*)(data)+4*1);
				print_data2(data1[i].torqueActual[0],  				(uint8_t*)(data)+4*2);
				print_data2(data1[i].velocityTarget[0],   			(uint8_t*)(data)+4*3);
				print_data2(data1[i].velocityActual[0],  			(uint8_t*)(data)+4*4);
				print_data2(data1[i].positionTarget[0],  			(uint8_t*)(data)+4*5);
				print_data2(data1[i].positionActual[0],   			(uint8_t*)(data)+4*6);
				print_data2(data1[i].torqueTarget[1], 				(uint8_t*)(data)+4*7);
				print_data2(data1[i].torqueActual[1],  				(uint8_t*)(data)+4*8);
				print_data2(data1[i].velocityTarget[1],   			(uint8_t*)(data)+4*9);
				print_data2(data1[i].velocityActual[1],  			(uint8_t*)(data)+4*10);
				print_data2(data1[i].positionTarget[1],  			(uint8_t*)(data)+4*11);
				print_data2(data1[i].positionActual[1],   			(uint8_t*)(data)+4*12);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)data, 4*13);
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
            snprintf(string+strlen(string), 1500-strlen(string), "Pos Target:  % 2.1f  % 2.1f\n", motor_control[2].positionTargetGamma, motor_control[3].positionTargetGamma);
            snprintf(string+strlen(string), 1500-strlen(string), "Pos Actual:  % 2.1f  % 2.1f\n", motor_control[2].positionActualGamma, motor_control[3].positionActualGamma);
            snprintf(string+strlen(string), 1500-strlen(string), "Error:       % 2.1f  % 2.1f\n", motor_control[2].positionErrorGamma,  motor_control[3].positionErrorGamma );
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


	motor_control[drv].velocityErrorGamma = motor_control[drv].velocityTargetGamma - motor_control[drv].velocityActualGamma;

	motor_control[drv].torqueTargetLimitedGamma = C_PI * motor_control[drv].velocityIntegratorValueGamma + D_PI * motor_control[drv].velocityErrorGamma;

	if (motor_control[drv].torqueTargetLimitedGamma > velocityIntegratorLimit)
	{
		motor_control[drv].velocityIntegratorValueGamma = ((velocityIntegratorLimit - D_PI*motor_control[drv].velocityErrorGamma) / C_PI);
		motor_control[drv].torqueTargetLimitedGamma = velocityIntegratorLimit;
	}
	else if (motor_control[drv].torqueTargetLimitedGamma < (-velocityIntegratorLimit))
	{
		motor_control[drv].velocityIntegratorValueGamma = ((-velocityIntegratorLimit - D_PI*motor_control[drv].velocityErrorGamma) / C_PI);
		motor_control[drv].torqueTargetLimitedGamma = -velocityIntegratorLimit;
	}
	else
	{
		motor_control[drv].velocityIntegratorValueGamma =  A_PI*motor_control[drv].velocityIntegratorValueGamma + B_PI*motor_control[drv].velocityErrorGamma;
	}


	//matched
	float coeffs1[6] = {1.0000000,	-1.7683651,	0.7704546,	1.0000000,	-1.7843640,	0.7892329};
	float coeffs2[6] = {1.0000000,	-1.9798883,	0.9853169,	1.0000000,	-0.9999964,	0.0000000};
	float coeffs3[6] = {1.0000000,	-1.9301999,	0.9322069,	1.0000000,	-1.9671578,	0.9685792};
	float coeffs4[6] = {1.0000000,	-1.9824102,	0.9838421,	1.0000000,	-1.9835303,	0.9855943};
	float gain = 0.2195863;

	// motor_control[drv].torqueTarget_Limited = motor_control[drv].velocityError;

	motor_control[drv].bq_intermediate1  = biquad(motor_control[drv].torqueTargetLimitedGamma, coeffs1, 1.0,  motor_control[drv].bq_vel_delay1);
	motor_control[drv].bq_intermediate2  = biquad(motor_control[drv].bq_intermediate1,     coeffs2, 1.0,  motor_control[drv].bq_vel_delay2);
	motor_control[drv].bq_intermediate3  = biquad(motor_control[drv].bq_intermediate2,     coeffs3, 1.0,  motor_control[drv].bq_vel_delay3);
	motor_control[drv].torqueTargetGamma = biquad(motor_control[drv].bq_intermediate3,     coeffs4, gain, motor_control[drv].bq_vel_delay4);

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

	motor_control[drv].positionErrorGamma = motor_control[drv].positionTargetGamma - motor_control[drv].positionActualGamma;

 	motor_control[drv].velocityTargetGamma = biquad(motor_control[drv].positionErrorGamma, coeffs, gain, motor_control[drv].bq_pos_delay1);
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
