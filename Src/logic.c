#include "logic.h"


uint8_t mode = MODE_STOP;
sweep_t sweep = {.Ta = TA, .N = DATA_N};

__attribute__((section(".ram_d1")))  data1_t data1[DATA_N] = { 0 };
__attribute__((section(".ram_d2")))  data2_t data2[DATA_N] = { 0 };

motor_t 	motor_data[4] = { 0 };
control_t 	motor_control[4] = { 0 };

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
volatile uint16_t 	pwm_in[4] = { 1500, 1500, 1500, 1500 };
volatile bool 		pwm_updated = false;


//AS5047U
uint16_t angleInRaw[4];
uint8_t as5074uErrorCounter[4] = {0};


bool stats = true;
bool current = false;
bool mode_matlab = 0;


void logic_init(void)
{
	int32_t i;
	static char string[500];
	static bool button_init = true;
	char clear_string[8];

	if(mode_matlab)
	{
		clear_string[0] = '\0';
	}
	else
	{
		 clear_string[0] = 27;
		 clear_string[1] = '[';
		 clear_string[2] = '2';
		 clear_string[3] = 'J';
		 clear_string[4] = 27;
		 clear_string[5] = '[';
		 clear_string[6] = 'H';
		 clear_string[7] = '\0';
	}

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

		if(mode_matlab == 0)
		{
			for(i=2; i<4; i++) angleInRaw[i] = as5047U_getAngle(i);
			snprintf(string, 500, "%s", clear_string);
			snprintf(string+strlen(string), 500-strlen(string), "as5047U Position:  %5d\t%5d\t%5d\t%5d\n", angleInRaw[0], angleInRaw[1], angleInRaw[2], angleInRaw[3] );
			snprintf(string+strlen(string), 500-strlen(string), "TMC6200 GSTAT:     %5d\t%5d\t%5d\t%5d\n", tmc6200_readInt(0, TMC6200_GSTAT), tmc6200_readInt(1, TMC6200_GSTAT), tmc6200_readInt(2, TMC6200_GSTAT), tmc6200_readInt(3, TMC6200_GSTAT));
			snprintf(string+strlen(string), 500-strlen(string), "TMC6200 GCONF:     %5d\t%5d\t%5d\t%5d\n", tmc6200_readInt(0, TMC6200_GCONF), tmc6200_readInt(1, TMC6200_GCONF), tmc6200_readInt(2, TMC6200_GCONF), tmc6200_readInt(3, TMC6200_GCONF));
			snprintf(string+strlen(string), 500-strlen(string), "TMC6200 DRV_CONF:  %5d\t%5d\t%5d\t%5d\n", tmc6200_readInt(0, TMC6200_DRV_CONF), tmc6200_readInt(1, TMC6200_DRV_CONF), tmc6200_readInt(2, TMC6200_DRV_CONF), tmc6200_readInt(3, TMC6200_DRV_CONF));
			snprintf(string+strlen(string), 500-strlen(string), "as5074u Errors:    %5d\t%5d\t%5d\t%5d\n", as5074uErrorCounter[0], as5074uErrorCounter[1], as5074uErrorCounter[2], as5074uErrorCounter[3]);
			snprintf(string+strlen(string), 500-strlen(string), "PHI_M:             %5d\t%5d\t%5d\t%5d\n", tmc4671_readRegister16BitValue(0, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15), tmc4671_readRegister16BitValue(1, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15), tmc4671_readRegister16BitValue(2, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15), tmc4671_readRegister16BitValue(3, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15));
			snprintf(string+strlen(string), 500-strlen(string), "PHI_E:             %5d\t%5d\t%5d\t%5d\n", tmc4671_readRegister16BitValue(0, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31), tmc4671_readRegister16BitValue(1, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31), tmc4671_readRegister16BitValue(2, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31), tmc4671_readRegister16BitValue(3, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31));
			// Should read: TMC6200 GSTAT = 0, TMC6200 GCONF = 48, TMC6200 DRV_CONF = 0, as5074u Errors = 0
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
		}
	}
	rx_byte_new = 0;
	HAL_Delay(100);

	TMC4671_highLevel_initEncoder_new(2);
	HAL_Delay(100);
	// TMC4671_highLevel_initEncoder_new(3);
	// HAL_Delay(100);

	// TMC4671_highLevel_initEncoder_new(2);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	// TMC4671_highLevel_positionMode_rampToZero(2);
	// HAL_Delay(100);
	// TMC4671_highLevel_initEncoder_new(3);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(3);
	// TMC4671_highLevel_positionMode_rampToZero(3);

    // TMC4671_highLevel_initEncoder_new(2);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	// TMC4671_highLevel_positionMode_rampToZero(2);
	// TMC4671_highLevel_stoppedMode(2);
	// HAL_Delay(100);
	// TMC4671_highLevel_initEncoder_new(3);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(3);
	// TMC4671_highLevel_positionMode_rampToZero(3);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(2);
	// TMC4671_highLevel_positionMode_rampToZero(2);


	TMC4671_highLevel_pwmOff(0);
	TMC4671_highLevel_pwmOff(1);
	// TMC4671_highLevel_pwmOff(2);
	TMC4671_highLevel_pwmOff(3);

	//for(i=2; i<4; i++) tmc4671_writeInt(i, TMC4671_PID_POSITION_ACTUAL, 0);

	// pwm inputs //FIXME: timer interrupt priority lower than spi?
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	for (i = 0; i < 4; i++) TMC4671_highLevel_setCurrentLimit(i, 15000);

	if(mode_matlab == 1)
	{
		snprintf(string, 500, "fininit\n");
		HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
	}
}

void logic_loop(void)
{
	static float angleIn[4] = {0};
	static char string[1500];
	static char clear_string[8];
	static uint32_t i = 0;
	// static uint8_t rate = 1;
	// static uint8_t rate_old = 1;

	// static pi_controller_t piVelocity[4];
	// static pi_controller_t piPosition[4];

	if(mode_matlab)
	{
		clear_string[0] = '\0';
	}
	else
	{
		 clear_string[0] = 27; clear_string[1] = '['; clear_string[2] = '2'; clear_string[3] = 'J';
		 clear_string[4] = 27; clear_string[5] = '['; clear_string[6] = 'H'; clear_string[7] = '\0';
	}


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
			//for(i=0; i<4; i++) angleIn[i] =   ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree

			angleIn[2] =   ( (float)(pwm_in[0] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree
			angleIn[3] =   ( (float)(pwm_in[1] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree

			for(i=0; i<4; i++) motor_control[i].phiIn = calcAngleTarget(i, angleIn)*M_PI/180;
		}
	}
	/* -------------------------------------------------------------------------  */

	if (stats == true && systick_counter_3 >= 500) //4Hz
	{
		systick_counter_3 = 0;

		if(mode_matlab == 0)
		{
			snprintf(string, 2000, "%s", clear_string);
			snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(2));
			snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(3));
			snprintf(string+strlen(string), 2000-strlen(string), "pwm_in:      %d  %d  %d  %d\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
			snprintf(string+strlen(string), 2000-strlen(string), "angleIn:     % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", angleIn[0], angleIn[1], angleIn[2], angleIn[3]);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
		}
	} // end of: if(systick_counter_3 >= 200) //5Hz




	/* -------------------------------------------------------------------------  */

	// if (systick_counter >= rate) //Ta = rate * 500us
	if (systick_counter >= 1) //Ta = rate * 500us
	{
		systick_counter = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

		// for(i=2; i<4; i++) motor_data[i].torqueActual   = tmc4671_getActualTorque_raw(i);
		for(i=2; i<4; i++) motor_data[i].positionActual = tmc4671_getActualPosition(i);
		// for(i=2; i<4; i++) motor_data[i].velocityActual = tmc4671_getActualVelocity(i);

		// discard wrong values!!!!!


		//for(i=2; i<4; i++) motor_control[i].currentActualBeta = (float)motor_data[i].torqueActual/500.0; 				// Ampere
		// for(i=2; i<4; i++) motor_control[i].positionActualBeta = (float)motor_data[i].positionActual/65536.0*2*M_PI; 	// rad
		// for(i=2; i<4; i++) motor_control[i].velocityActualBeta = (float)motor_data[i].velocityActual/60*2*M_PI;			// rad per s

		for(i=2; i<4; i++) motor_control[i].phi = (float)motor_data[i].positionActual/65536.0*2*M_PI; 	// rad


		if (mode == MODE_STOP)
		{
			for (i = 2; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);

		}
		else if(mode == MODE_IDLE)
		{

		}
		else if(mode == MODE_TORQUE)
		{
			motor_data[2].torqueTarget = (int32_t)(500.0*motor_control[2].iq);
			tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);


		}
		else if(mode == MODE_TRAJTEST)
		{
			//

			if(sweep.k < 20)
				motor_control[2].phiIn = 0;
			else if(sweep.k == 20)
				motor_control[2].phiIn = 10*M_PI/180;
			else if(sweep.k > 5000)
				motor_control[2].phiIn = 0;

			//Trajectory
			motor_control[2].phiInLimited = rateLimiter(&motor_control[2].limTrajPhi, motor_control[2].phiIn);
			motor_control[2].phiDes   = biquad(&motor_control[2].bqTrajPhi, motor_control[2].phiInLimited);
			motor_control[2].omegaDes = biquad(&motor_control[2].bqTrajOmega, motor_control[2].phiDes);
			motor_control[2].alphaDes = biquad(&motor_control[2].bqTrajAlpha, motor_control[2].omegaDes);

			motor_control[2].phiDes   = biquad(&motor_control[2].bqTrajPhi1, motor_control[2].phiDes);
			motor_control[2].phiDes   = biquad(&motor_control[2].bqTrajPhi2, motor_control[2].phiDes);
			motor_control[2].omegaDes = biquad(&motor_control[2].bqTrajOmega1, motor_control[2].omegaDes);

			for (i = 2; i < 4; i++) data1[sweep.k].phiIn[i-2]  	 		= motor_control[i].phiIn;
			for (i = 2; i < 4; i++) data2[sweep.k].phiInLimited[i-2] 	= motor_control[i].phiInLimited;
			for (i = 2; i < 4; i++) data2[sweep.k].phiDes[i-2]   		= motor_control[i].phiDes;
			for (i = 2; i < 4; i++) data2[sweep.k].omegaDes[i-2] 		= motor_control[i].omegaDes;
			for (i = 2; i < 4; i++) data2[sweep.k].alphaDes[i-2] 		= motor_control[i].alphaDes;



			if(sweep.k >= (DATA_N-1)) // stop
		    {
		        mode = MODE_STOP;
		        stats = true;
				// rate = 1;

		        if(mode_matlab)
				{
		            snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		    	}
		    }
			sweep.k++;
		}
		else if(mode == MODE_POSITION)
		{


			if(sweep.k < 20)
				motor_control[2].phiIn = 0;
			else if(sweep.k == 20)
				motor_control[2].phiIn = 40*M_PI/180;
			else if(sweep.k > 5000)
				motor_control[2].phiIn = 0;

			//Input filtering
			// motor_control[2].phi = biquad(&motor_control[2].bqPhi, motor_control[2].phi);

			//Trajectory
			motor_control[2].phiInLimited = rateLimiter(&motor_control[2].limTrajPhi, motor_control[2].phiIn);
			motor_control[2].phiDes   = biquad(&motor_control[2].bqTrajPhi, motor_control[2].phiInLimited);
			motor_control[2].omegaDes = biquad(&motor_control[2].bqTrajOmega, motor_control[2].phiDes);
			motor_control[2].alphaDes = biquad(&motor_control[2].bqTrajAlpha, motor_control[2].omegaDes);
			motor_control[2].phiDes   = biquad(&motor_control[2].bqTrajPhi1, motor_control[2].phiDes);
			motor_control[2].phiDes   = biquad(&motor_control[2].bqTrajPhi2, motor_control[2].phiDes);
			motor_control[2].omegaDes = biquad(&motor_control[2].bqTrajOmega1, motor_control[2].omegaDes);


			disturbanceObserver(&motor_control[2]);


			motor_data[2].torqueTarget = (int32_t)(500.0*motor_control[2].iq);
			tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);




			for (i = 2; i < 4; i++) data1[sweep.k].phiIn[i-2]  	 		= motor_control[i].phiIn;
			for (i = 2; i < 4; i++) data2[sweep.k].phiInLimited[i-2] 	= motor_control[i].phiInLimited;
			for (i = 2; i < 4; i++) data2[sweep.k].phiDes[i-2]   		= motor_control[i].phiDes;
			for (i = 2; i < 4; i++) data2[sweep.k].omegaDes[i-2] 		= motor_control[i].omegaDes;
			for (i = 2; i < 4; i++) data2[sweep.k].alphaDes[i-2] 		= motor_control[i].alphaDes;
			for (i = 2; i < 4; i++) data1[sweep.k].phi[i-2]  	 		= motor_control[i].phi;
			for (i = 2; i < 4; i++) data1[sweep.k].alphaM[i-2]  	 	= motor_control[i].alphaM;
			for (i = 2; i < 4; i++) data1[sweep.k].iq[i-2]       		= motor_control[i].iq;
			for (i = 2; i < 4; i++) data1[sweep.k].phiEst[i-2]  	 	= motor_control[i].phiEst;
			for (i = 2; i < 4; i++) data1[sweep.k].omegaEst[i-2]  	 	= motor_control[i].omegaEst;
			for (i = 2; i < 4; i++) data1[sweep.k].alphaEst[i-2]       	= motor_control[i].alphaEst;
			for (i = 2; i < 4; i++) data1[sweep.k].alphaFrict[i-2]      = motor_control[i].alphaFrict;


			if(sweep.k >= (DATA_N-1)) // stop
		    {
		        mode = MODE_STOP;
		        stats = true;
				// rate = 1;

		        if(mode_matlab)
				{
		            snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		    	}
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

		case 'w':
			mode_matlab = !mode_matlab;
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
			// rate = 1;
			for (i = 0; i < 4; i++) tmc4671_setTargetFlux_raw(2, 0);
			for (i = 0; i < 4; i++) tmc4671_setTargetTorque_raw(2, 0);
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case 'i':
			mode = MODE_IDLE;
			stats = true;
			break;

		case '1':
			mode = MODE_TRAJTEST;
			sweep.k = 0;
			sweep.Ta = TA;
			stats = false;

			for (i = 0; i < 4; i++) rateLimiterInit(&motor_control[i].limTrajPhi, 100, 0);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0098135136, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.6037472486, 0.6430013180);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajOmega,  2444.0617675781, 1.0000000000, -1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajAlpha,  2444.0617675781, 1.0000000000, -1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi1,   0.6110154986, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi2,   0.6110154986, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajOmega1, 0.6110154986, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqPhi, 0.2390572280, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, -0.5218855739, 0.0000000000);
			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_STOPPED);
			break;

		// case '2':
		// 	mode = MODE_TORQUE;
		// 	for (i = 0; i < 4; i++) motor_control[i].iq = 10;
		// 	tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
		// 	tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
		// 	tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
		// 	tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_STOPPED);
		// 	break;


		case '2':
			mode = MODE_POSITION;
			sweep.k = 0;
			sweep.Ta = TA;
			stats = false;

			for (i = 0; i < 4; i++) motor_control[i].iq = 0;
			tmc4671_writeInt(2, TMC4671_PID_POSITION_ACTUAL, 0);

			for (i = 0; i < 4; i++) rateLimiterInit(&motor_control[i].limTrajPhi, 100, 0);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0098135136, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.6037472486, 0.6430013180);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajOmega,  2444.0617675781, 1.0000000000, -1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajAlpha,  2444.0617675781, 1.0000000000, -1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi1,   0.6110154986, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi2,   0.6110154986, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajOmega1, 0.6110154986, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309377, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqPhi, 0.4399008453, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, -0.1201983094, 0.0000000000);
			disturbanceObserverInit(&motor_control[2], 2450.4422698000, 2249282.8430082649, 916607551.2230231762, 139918418361.2410888672, 3553.0575843922, 119.3805208364, 790.0960095481);

			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_STOPPED);
			break;


		// case '+':
		// 	for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta += 10;
		// break;
		//
		// case '-':
		// 	for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta -= 10;
		// break;




		case 'x': // print data
			for (i = 0; i < DATA_N; i++)
			{
				print_data_float((float)i,   					(uint8_t*)(string));
				print_data_float(data1[i].phiIn[0], 			(uint8_t*)(string)+4*1);
				print_data_float(data2[i].phiInLimited[0],  	(uint8_t*)(string)+4*2);
				print_data_float(data2[i].phiDes[0],  			(uint8_t*)(string)+4*3);
				print_data_float(data2[i].omegaDes[0],   		(uint8_t*)(string)+4*4);
				print_data_float(data2[i].alphaDes[0],  		(uint8_t*)(string)+4*5);
				print_data_float(data1[i].phi[0], 				(uint8_t*)(string)+4*6);
				print_data_float(data1[i].alphaM[0],  			(uint8_t*)(string)+4*7);
				print_data_float(data1[i].phiEst[0],   			(uint8_t*)(string)+4*8);
				print_data_float(data1[i].omegaEst[0],  		(uint8_t*)(string)+4*9);
				print_data_float(data1[i].alphaEst[0], 			(uint8_t*)(string)+4*10);
				print_data_float(data1[i].alphaFrict[0],  		(uint8_t*)(string)+4*11);
				print_data_float(data1[i].iq[0],   				(uint8_t*)(string)+4*12);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, 4*13);
				HAL_Delay(1);
			}
            HAL_Delay(1);
			break;


		case 'y':
			snprintf(string, 1500, "%s", clear_string);
			snprintf(string + strlen(string), 1500, "%s", TMC4671_highLevel_getStatus(2));
			snprintf(string + strlen(string), 1500, "%s", TMC4671_highLevel_getStatus(3));
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
