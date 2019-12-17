#include "logic.h"


uint8_t mode = MODE_STOP;
sweep_t sweep = {.Ta = TA, .N = DATA_N};

//log data
__attribute__((section(".ram_d1")))  data1_t data1[DATA_N] = { 0 };
__attribute__((section(".ram_d2")))  data2_t data2[DATA_N] = { 0 };


motor_t 	motor_data[4] = { 0 };
control_t 	motor_control[4] = { 0 };



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
bool mode_matlab = MATLAB;
bool init_done = false;
bool limit = false;


volatile uint16_t Ta_counter = 0;
volatile uint16_t systick_counter = 0;



void logic_init(void)
{
	int32_t i;
	static char string[500];
	static bool button_init = true;
	char clear_string[8];
	static bool led = 1;

	if(mode_matlab)
	{
		clear_string[0] = '\0';
	}
	else
	{
		 clear_string[0] = 27; clear_string[1] = '['; clear_string[2] = '2'; clear_string[3] = 'J'; clear_string[4] = 27; clear_string[5] = '['; clear_string[6] = 'H'; clear_string[7] = '\0';
	}

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	for (i = 0; i < 4; i++) spiSpeedSlow_set(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) tmc6200_highLevel_init(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) swdriver_setEnable(i, true);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) tmc6200_highLevel_resetErrorFlags(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) TMC4671_highLevel_init(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) as5047U_setABIResolution14Bit(i); //FIXME
	for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
	HAL_Delay(10);

	for (i = 0; i < 4; i++) spiSpeedSlow_reset(i);
	HAL_Delay(10);


	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	rx_byte_new = 0;


	while ( (!(rx_byte_new && rx_byte == 's') && button_init == true ) || !(TMC4671_highLevel_getPhiM(2)==0 && TMC4671_highLevel_getPhiM(3)==0 && TMC4671_highLevel_getPhiE(2)==0 && TMC4671_highLevel_getPhiE(3)==0 ) )
	{
		button_init = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

		if(!(TMC4671_highLevel_getPhiM(2)==0 && TMC4671_highLevel_getPhiM(3)==0 && TMC4671_highLevel_getPhiE(2)==0 && TMC4671_highLevel_getPhiE(3)==0 ))
		{
			if(led)
			{
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
				led = 0;
			}
			else
			{
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
				led = 1;
			}
		}

		if(mode_matlab == 0)
		{
			for(i=2; i<4; i++) angleInRaw[i] = as5047U_getAngle(i);
			snprintf(string, 500, "%s", clear_string);
			snprintf(string+strlen(string), 500-strlen(string), "as5047U Position:  %5d\t%5d\t%5d\t%5d\n", angleInRaw[0], angleInRaw[1], angleInRaw[2], angleInRaw[3] );
			snprintf(string+strlen(string), 500-strlen(string), "TMC6200 GSTAT:     %5d\t%5d\t%5d\t%5d\n", tmc6200_highLevel_getRegisterGSTAT(0), tmc6200_highLevel_getRegisterGSTAT(1), tmc6200_highLevel_getRegisterGSTAT(2), tmc6200_highLevel_getRegisterGSTAT(3));
			snprintf(string+strlen(string), 500-strlen(string), "TMC6200 GCONF:     %5d\t%5d\t%5d\t%5d\n", tmc6200_highLevel_getRegisterGCONF(0), tmc6200_highLevel_getRegisterGCONF(1), tmc6200_highLevel_getRegisterGCONF(2), tmc6200_highLevel_getRegisterGCONF(3));
			snprintf(string+strlen(string), 500-strlen(string), "TMC6200 DRV_CONF:  %5d\t%5d\t%5d\t%5d\n", tmc6200_highLevel_getRegisterDRV_CONF(0), tmc6200_highLevel_getRegisterDRV_CONF(1), tmc6200_highLevel_getRegisterDRV_CONF(2), tmc6200_highLevel_getRegisterDRV_CONF(3));
			snprintf(string+strlen(string), 500-strlen(string), "as5074u Errors:    %5d\t%5d\t%5d\t%5d\n", as5074uErrorCounter[0], as5074uErrorCounter[1], as5074uErrorCounter[2], as5074uErrorCounter[3]);
			snprintf(string+strlen(string), 500-strlen(string), "PHI_M:             %5d\t%5d\t%5d\t%5d\n", TMC4671_highLevel_getPhiM(0), TMC4671_highLevel_getPhiM(1), TMC4671_highLevel_getPhiM(2), TMC4671_highLevel_getPhiM(3));
			snprintf(string+strlen(string), 500-strlen(string), "PHI_E:             %5d\t%5d\t%5d\t%5d\n", TMC4671_highLevel_getPhiE(0), TMC4671_highLevel_getPhiE(1), TMC4671_highLevel_getPhiE(2), TMC4671_highLevel_getPhiE(3));
			// Should read: TMC6200 GSTAT = 0, TMC6200 GCONF = 48, TMC6200 DRV_CONF = 0, as5074u Errors = 0
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		}
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
	}
	rx_byte_new = 0;
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);

	TMC4671_highLevel_initEncoder(2);
	HAL_Delay(100);

	TMC4671_highLevel_initEncoder(3);
	HAL_Delay(100);

	disturbanceObserverInit(&motor_control[2], 100, 3, 773.0006664650);
	disturbanceObserverInit(&motor_control[3], 100, 3, 493.3781002056);

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

	HAL_TIM_Base_Start_IT(&htim5);

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
			// for(i=0; i<4; i++) motor_control[i].phiIn = 2*angleIn[i]*M_PI/180;
		}
	}
	/* -------------------------------------------------------------------------  */

	if (stats == true && systick_counter >= 200) //5Hz
	{
		systick_counter = 0;

		if(mode_matlab == 0)
		{
			snprintf(string, 2000, "%s", clear_string);
			snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(2));
			snprintf(string+strlen(string), 2000, "%s", TMC4671_highLevel_getStatus(3));
			snprintf(string+strlen(string), 2000-strlen(string), "pwm_in:      %d  %d  %d  %d\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
			snprintf(string+strlen(string), 2000-strlen(string), "angleIn:     % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", angleIn[0], angleIn[1], angleIn[2], angleIn[3]);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "cM2        [u,j]  %3.1f\n", motor_control[2].CmEst);
			snprintf(string+strlen(string), 2000-strlen(string), "cM3        [i,k]  %3.1f\n", motor_control[3].CmEst);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "f_OBS[2]   [t,g]  %3.1f\n", motor_control[2].fOBS);
			snprintf(string+strlen(string), 2000-strlen(string), "f_OBS[3]   [z,h]  %3.1f\n", motor_control[3].fOBS);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "f_FB[2]    [e,d]  %3.1f\n", motor_control[2].fFB);
			snprintf(string+strlen(string), 2000-strlen(string), "f_FB[3]    [r,f]  %3.1f\n", motor_control[3].fFB);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "iq [2]            %3.1f\n", motor_control[2].iq);
			snprintf(string+strlen(string), 2000-strlen(string), "iq [3]            %3.1f\n", motor_control[3].iq);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "%s\n", limit?"***  LIMIT  ***":" - ");
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "Mode: %s\n", mode==MODE_STOP?"MODE_STOP":mode==MODE_POSITION?"MODE_POSITION":mode==MODE_POSITION_INIT_M3?"MODE_POSITION_INIT_M3":mode==MODE_POSITION_INIT_M2?"MODE_POSITION_INIT_M2":mode==MODE_LIMIT?"MODE_LIMIT":mode==MODE_POSITION_STEP?"MODE_POSITION_STEP":mode==MODE_RCCONTROL?"MODE_RCCONTROL":mode==MODE_SWEEP?"MODE_SWEEP":mode==MODE_IDLE?"MODE_IDLE":" *** UNKNOWN MODE *** ERROR ***");
			HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
		}
	} // end of: if(systick_counter_3 >= 200) //5Hz




	/* -------------------------------------------------------------------------  */

	// if (systick_counter >= rate) //Ta = rate * 500us
	if (Ta_counter >= 1) //Ta = rate * 500us
	{
		Ta_counter = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

		// for(i=2; i<4; i++) motor_data[i].torqueActual   = tmc4671_getActualTorque_raw(i);
		// for(i=2; i<4; i++) motor_data[i].positionActual = TMC4671_highLevel_getPositionActual(i);
		// for(i=2; i<4; i++) motor_data[i].velocityActual = tmc4671_getActualVelocity(i);

		//for(i=2; i<4; i++) motor_control[i].currentActualBeta = (float)motor_data[i].torqueActual/500.0; 				// Ampere
		// for(i=2; i<4; i++) motor_control[i].positionActualBeta = (float)motor_data[i].positionActual/65536.0*2*M_PI; 	// rad
		// for(i=2; i<4; i++) motor_control[i].velocityActualBeta = (float)motor_data[i].velocityActual/60*2*M_PI;			// rad per s


		for(i=2; i<4; i++)
		{
			float phi = TMC4671_highLevel_getPositionActualRad(i);

			if(phi < M_PI && phi > -M_PI)
				motor_control[i].phi = phi;
		}


		float phiAlpha[4];
		for(i=2; i<4; i++) phiAlpha[i] = calcAngleBetaAlpha(i, motor_control[i].phi*180.0/M_PI); // in degree

		//TODO: Umrechnen und richtige begrenzung
		if(motor_control[3].phi > 60.0/180.0*M_PI || motor_control[3].phi < -60.0/180.0*M_PI )
		{
			mode = MODE_LIMIT;
			limit = true;
		}
		else if(phiAlpha[2]-phiAlpha[3] > 25.0 || phiAlpha[2]-phiAlpha[3] < -25.0)
		{
			mode = MODE_LIMIT;
			limit = true;
		}
		else
		{
				limit = false;
		}



		if (mode == MODE_STOP)
		{
			for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
		}
		if (mode == MODE_LIMIT)
		{
			for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
		}
		else if(mode == MODE_IDLE)
		{

		}
		else if(mode == MODE_TORQUE)
		{
			motor_data[2].torqueTarget = (int32_t)(500.0*motor_control[2].iq);
			TMC4671_highLevel_setTorqueTarget(2, motor_data[2].torqueTarget);


		}
		else if(mode == MODE_TRAJTEST)
		{

		}
		else if(mode == MODE_POSITION_INIT_M3)
		{
			i = 3;


			if(sweep.k == 0)
			{
				motor_control[i].iq = 0;
				motor_control[i].phi0 = TMC4671_highLevel_getPositionActualRad(i);

				rateLimiterInit(&motor_control[i].limTrajPhi, 100, motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajPhi,    motor_control[i].phi0);
			 	biquadReset(&motor_control[i].bqTrajOmega,  motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajAlpha,  0);
				biquadReset(&motor_control[i].bqTrajPhi1,   motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajPhi2,   motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajOmega1, 0);
				biquadReset(&motor_control[i].bqPhi,        motor_control[i].phi0);
				biquadReset(&motor_control[i].bqNotch,    0);
				biquadReset(&motor_control[i].bqNotch1,    0);
				biquadReset(&motor_control[i].bqNotch2,    0);

				disturbanceObserverResetPhi0(&motor_control[i], motor_control[i].phi0);
			}
			else if(sweep.k < 500)
			{
				motor_control[i].phiIn = motor_control[i].phi0;
			}
			else
			{
				motor_control[i].phiIn = 0;
			}

			//Input filtering
			motor_control[i].phi = biquad(&motor_control[i].bqPhi, motor_control[i].phi);

			//Trajectory
			motor_control[i].phiInLimited = rateLimiter(&motor_control[i].limTrajPhi, motor_control[i].phiIn);
			motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi, motor_control[i].phiInLimited);
			motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega, motor_control[i].phiDes);
			motor_control[i].alphaDes = biquad(&motor_control[i].bqTrajAlpha, motor_control[i].omegaDes);
			motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi1, motor_control[i].phiDes);
			motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi2, motor_control[i].phiDes);
			motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega1, motor_control[i].omegaDes);

			//Disturbance Observer
			disturbanceObserver(&motor_control[i]);

			TMC4671_highLevel_setTorqueTargetA(i, motor_control[i].iq);



			sweep.k++;

			if(sweep.k >= 2000 &&  motor_control[3].phiDes < 0.000001 && motor_control[3].phiDes > -0.000001)  // stop
		    {
		        mode = MODE_POSITION_INIT_M2;
		        stats = true;
				sweep.k = 0;
		    }
		}
		else if(mode == MODE_POSITION_INIT_M2)
		{
			i = 2;
			if(sweep.k == 0)
			{
				motor_control[i].iq = 0;
				motor_control[i].phi0 = TMC4671_highLevel_getPositionActualRad(i);

				rateLimiterInit(&motor_control[i].limTrajPhi, 100, motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajPhi,    motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajOmega,  motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajAlpha,  0);
				biquadReset(&motor_control[i].bqTrajPhi1,   motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajPhi2,   motor_control[i].phi0);
				biquadReset(&motor_control[i].bqTrajOmega1, 0);
				biquadReset(&motor_control[i].bqPhi,        motor_control[i].phi0);
				biquadReset(&motor_control[i].bqNotch,    0);
				biquadReset(&motor_control[i].bqNotch1,    0);
				biquadReset(&motor_control[i].bqNotch2,    0);


				disturbanceObserverResetPhi0(&motor_control[i], motor_control[i].phi0);
			}
			else if(sweep.k < 500)
			{
				motor_control[i].phiIn = motor_control[i].phi0;
			}
			else
			{
				motor_control[i].phiIn = 0;
			}

			//Input filtering
			for(i=2; i<4; i++) motor_control[i].phi = biquad(&motor_control[i].bqPhi, motor_control[i].phi);

			//Trajectory
			for(i=2; i<4; i++)motor_control[i].phiInLimited = rateLimiter(&motor_control[i].limTrajPhi, motor_control[i].phiIn);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi, motor_control[i].phiInLimited);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].alphaDes = biquad(&motor_control[i].bqTrajAlpha, motor_control[i].omegaDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi1, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi2, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega1, motor_control[i].omegaDes);

			//Disturbance Observer
			for(i=2; i<4; i++) disturbanceObserver(&motor_control[i]);

			for(i=2; i<4; i++) TMC4671_highLevel_setTorqueTargetA(i, motor_control[i].iq);



			sweep.k++;

			if(sweep.k >= 2000 &&  motor_control[2].phiDes < 0.000001 && motor_control[2].phiDes > -0.000001)  // stop
			{
				mode = MODE_POSITION;
				init_done = true;
				stats = true;
				sweep.k = 0;

				if(mode_matlab)
				{
			        snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				}
			}
		}
		else if(mode == MODE_POSITION || mode == MODE_RCCONTROL)
		{
			if(mode == MODE_POSITION)
			{
				for(i=2; i<4; i++) motor_control[i].phiIn = 0;
			}

			if(sweep.k == 0 && mode == MODE_POSITION)
			{
				motor_control[i].phi0 = TMC4671_highLevel_getPositionActualRad(i);
				//switch from 2Hz prefilter to ...
				/*70Hz*/ // for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0098135132, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.6037472896, 0.6430013423);
				/*50Hz*/ // for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0053028263, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.7087179717, 0.7299292767);
				/*30Hz*/ // for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0020252849, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.8199873376, 0.8280884773);
				/*20Hz*/  for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0009277524, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.8781638882, 0.8818748977);
				/*10Hz*/ // for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0002391674, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.9381398440, 0.9390965137);

				biquadReset(&motor_control[i].bqTrajPhi,    motor_control[i].phi0);
				sweep.k++;
			}


			//Input filtering
			for(i=2; i<4; i++) motor_control[i].phi = biquad(&motor_control[i].bqPhi, motor_control[i].phi);

			//Trajectory
			for(i=2; i<4; i++)motor_control[i].phiInLimited = rateLimiter(&motor_control[i].limTrajPhi, motor_control[i].phiIn);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi, motor_control[i].phiInLimited);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].alphaDes = biquad(&motor_control[i].bqTrajAlpha, motor_control[i].omegaDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi1, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi2, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega1, motor_control[i].omegaDes);

			//Disturbance Observer
			for(i=2; i<4; i++) disturbanceObserver(&motor_control[i]);

			for(i=2; i<4; i++)
			{
				if( motor_control[i].iq > I_LIMIT )
					 motor_control[i].iq = I_LIMIT;
				else if ( motor_control[i].iq < -I_LIMIT )
					 motor_control[i].iq = -I_LIMIT;
			 }

			for(i=2; i<4; i++) TMC4671_highLevel_setTorqueTargetA(i, motor_control[i].iq);



		}
		else if(mode == MODE_SWEEP)
		{

			for(i=2; i<4; i++) motor_control[i].phiIn = 0;

			//Input filtering
			for(i=2; i<4; i++) motor_control[i].phi = biquad(&motor_control[i].bqPhi, motor_control[i].phi);

			//Trajectory
			for(i=2; i<4; i++)motor_control[i].phiInLimited = rateLimiter(&motor_control[i].limTrajPhi, motor_control[i].phiIn);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi, motor_control[i].phiInLimited);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].alphaDes = biquad(&motor_control[i].bqTrajAlpha, motor_control[i].omegaDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi1, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi2, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega1, motor_control[i].omegaDes);



			//Disturbance Observer
			for(i=2; i<4; i++) disturbanceObserver(&motor_control[i]);

			if(sweep.rate_counter >= sweep.rate-1)
			{
				sweep.t = sweep.k * sweep.Ta;
				sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
				sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t	+ (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

			}

			if(sweep.mode == M2)
				motor_control[2].alphaM = sweep.out;
			else if(sweep.mode == M3)
				motor_control[3].alphaM = sweep.out;

			for(i=2; i<4; i++)	motor_control[i].iq = motor_control[i].alphaM / motor_control[i].CmEst;

			for(i=2; i<4; i++)
			{
				if( motor_control[i].iq > I_LIMIT )
					 motor_control[i].iq = I_LIMIT;
				else if ( motor_control[i].iq < -I_LIMIT )
					 motor_control[i].iq = -I_LIMIT;
			 }

			for(i=2; i<4; i++) TMC4671_highLevel_setTorqueTargetA(i, motor_control[i].iq);


			if(sweep.rate_counter >= sweep.rate-1)
			{

				for (i = 2; i < 4; i++) data1[sweep.k].phiIn[i-2]  	 		= motor_control[i].phiIn;
				for (i = 2; i < 4; i++) data2[sweep.k].phiInLimited[i-2] 	= motor_control[i].phiInLimited;
				for (i = 2; i < 4; i++) data2[sweep.k].phiDes[i-2]   		= motor_control[i].phiDes;
				for (i = 2; i < 4; i++) data2[sweep.k].omegaDes[i-2] 		= motor_control[i].omegaDes;
				for (i = 2; i < 4; i++) data2[sweep.k].alphaDes[i-2] 		= motor_control[i].alphaDes;
				for (i = 2; i < 4; i++) data1[sweep.k].phi[i-2]  	 		= motor_control[i].phi;
				for (i = 2; i < 4; i++) data1[sweep.k].alphaM[i-2] 			= motor_control[i].alphaM;
				for (i = 2; i < 4; i++) data1[sweep.k].iq[i-2]       		= motor_control[i].iq;
				for (i = 2; i < 4; i++) data1[sweep.k].phiEst[i-2]			= motor_control[i].phiEst;
				for (i = 2; i < 4; i++) data1[sweep.k].omegaEst[i-2]  	 	= motor_control[i].omegaEst;
				for (i = 2; i < 4; i++) data1[sweep.k].alphaEst[i-2]       	= motor_control[i].alphaEst;
				for (i = 2; i < 4; i++) data1[sweep.k].alphaFrict[i-2]		= motor_control[i].alphaFrict;

				if(sweep.k >= (DATA_N-1)) // stop
				{
				    mode = MODE_POSITION;
				    stats = true;

				    if(mode_matlab)
					{
				        snprintf(string, 200, 	"fintest\n");
						HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
					}
				}
				sweep.k++;
				sweep.rate_counter = 0;
			}
			else
			{
				sweep.rate_counter++;
			}
		}
		else if(mode == MODE_POSITION_STEP)
		{


				// motor_control[3].phiIn = 0;
				//
				// if(sweep.k < 20)
				// 	motor_control[2].phiIn = 0;
				// else if(sweep.k == 20)
				// 	motor_control[2].phiIn = 7*M_PI/180;
				// else if(sweep.k > 2000)
				// motor_control[2].phiIn = 0;


			for(i=2; i<4; i++) motor_control[i].phiIn = 0;

			//Input filtering
			for(i=2; i<4; i++) motor_control[i].phi = biquad(&motor_control[i].bqPhi, motor_control[i].phi);

			//Trajectory
			for(i=2; i<4; i++)motor_control[i].phiInLimited = rateLimiter(&motor_control[i].limTrajPhi, motor_control[i].phiIn);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi, motor_control[i].phiInLimited);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].alphaDes = biquad(&motor_control[i].bqTrajAlpha, motor_control[i].omegaDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi1, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].phiDes   = biquad(&motor_control[i].bqTrajPhi2, motor_control[i].phiDes);
			for(i=2; i<4; i++)motor_control[i].omegaDes = biquad(&motor_control[i].bqTrajOmega1, motor_control[i].omegaDes);



			//Disturbance Observer
			for(i=2; i<4; i++) disturbanceObserver(&motor_control[i]);

			for(i=2; i<4; i++)
			{
				if( motor_control[i].iq > I_LIMIT )
					 motor_control[i].iq = I_LIMIT;
				else if ( motor_control[i].iq < -I_LIMIT )
					 motor_control[i].iq = -I_LIMIT;
			 }

			for(i=2; i<4; i++) TMC4671_highLevel_setTorqueTargetA(i, motor_control[i].iq);




			for (i = 2; i < 4; i++) data1[sweep.k].phiIn[i-2]  	 		= motor_control[i].phiIn;
			for (i = 2; i < 4; i++) data2[sweep.k].phiInLimited[i-2] 	= motor_control[i].phiInLimited;
			for (i = 2; i < 4; i++) data2[sweep.k].phiDes[i-2]   		= motor_control[i].phiDes;
			for (i = 2; i < 4; i++) data2[sweep.k].omegaDes[i-2] 		= motor_control[i].omegaDes;
			for (i = 2; i < 4; i++) data2[sweep.k].alphaDes[i-2] 		= motor_control[i].alphaDes;
			for (i = 2; i < 4; i++) data1[sweep.k].phi[i-2]  	 		= motor_control[i].phi;
			for (i = 2; i < 4; i++) data1[sweep.k].alphaM[i-2] 			= motor_control[i].alphaM;
			for (i = 2; i < 4; i++) data1[sweep.k].iq[i-2]       		= motor_control[i].iq;
			for (i = 2; i < 4; i++) data1[sweep.k].phiEst[i-2]			= motor_control[i].phiEst;
			for (i = 2; i < 4; i++) data1[sweep.k].omegaEst[i-2]  	 	= motor_control[i].omegaEst;
			for (i = 2; i < 4; i++) data1[sweep.k].alphaEst[i-2]       	= motor_control[i].alphaEst;
			for (i = 2; i < 4; i++) data1[sweep.k].alphaFrict[i-2]		= motor_control[i].alphaFrict;

			if(sweep.k >= (DATA_N-1)) // stop
			{
				mode = MODE_POSITION;
				stats = true;

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
			init_done = false;
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

		// case 'c':
		// 	if (current)
		// 	{
		// 		for (i = 0; i < 4; i++) TMC4671_highLevel_setCurrentLimit(i, 5000);
		// 		current = false;
		// 	}
		// 	else
		// 	{
		// 		for (i = 0; i < 4; i++) TMC4671_highLevel_setCurrentLimit(i, 15000);
		// 		current = true;
		// 	}
		// 	break;

		case '0':
			mode = MODE_STOP;
			stats = true;
			init_done = false;
			// rate = 1;
			for (i = 0; i < 4; i++) TMC4671_highLevel_setTorqueTarget(2, 0);
			for (i = 0; i < 4; i++) TMC4671_highLevel_setFluxTarget(2, 0);
			for (i = 0; i < 4; i++) TMC4671_highLevel_stoppedMode(i);
			break;

		case '9':
			mode = MODE_IDLE;
			init_done = false;
			stats = true;
			break;


		case '1':
			mode = MODE_POSITION_INIT_M3;
			sweep.k = 0;
			sweep.Ta = TA;
			stats = true;

			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi,    0.0000098079, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, -1.9874729842, 0.9875122157);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajOmega,  2444.0618814066, 1.0000000000, -1.0000000000, 0.0000000000, 1.0000000000, 0.2220309407, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajAlpha,  2444.0618814066, 1.0000000000, -1.0000000000, 0.0000000000, 1.0000000000, 0.2220309407, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi1,   0.6110154704, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309407, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajPhi2,   0.6110154704, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309407, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqTrajOmega1, 0.6110154704, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, 0.2220309407, 0.0000000000);
			for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqPhi,        0.4399008465, 1.0000000000, 1.0000000000, 0.0000000000, 1.0000000000, -0.1201983070, 0.0000000000);

			// // biquadInit(&motor_control[2].bqNotch,      1.0000000000, 1.0000000000, 0.0000000000, 0.0000000000, 1.0000000000, 0.0000000000, 0.0000000000);
			// // biquadInit(&motor_control[2].bqNotch1,      1.0000000000, 1.0000000000, 0.0000000000, 0.0000000000, 1.0000000000, 0.0000000000, 0.0000000000);
			// // biquadInit(&motor_control[2].bqNotch2,      1.0000000000, 1.0000000000, 0.0000000000, 0.0000000000, 1.0000000000, 0.0000000000, 0.0000000000);
			//
			//
			// // /*8.7Hz*/biquadInit(&motor_control[2].bqNotch1,      0.8767447472, 1.0000000000, -1.8928095102, 0.9648543000, 1.0000000000, -1.6595108509, 0.7226756811);
			// // /*8.7Hz*/biquadInit(&motor_control[2].bqNotch2,      0.8767447472, 1.0000000000, -1.8928095102, 0.9648543000, 1.0000000000, -1.6595108509, 0.7226756811);
			//
			// /* 11.5Hz */ biquadInit(&motor_control[2].bqNotch1,      0.9811463952, 1.0000000000, -1.9938943386, 0.9951960444, 1.0000000000, -1.9563022852, 0.9575794339);
			// /* 11.5Hz */ biquadInit(&motor_control[2].bqNotch1,      0.9811463952, 1.0000000000, -1.9938943386, 0.9951960444, 1.0000000000, -1.9563022852, 0.9575794339);
			// /*8.7Hz*/biquadInit(&motor_control[3].bqNotch2,      0.8767447472, 1.0000000000, -1.8928095102, 0.9648543000, 1.0000000000, -1.6595108509, 0.7226756811);
			// /*8.7Hz*/biquadInit(&motor_control[3].bqNotch2,      0.8767447472, 1.0000000000, -1.8928095102, 0.9648543000, 1.0000000000, -1.6595108509, 0.7226756811);
			//
			// /*10Hz*/for (i = 0; i < 4; i++) biquadInit(&motor_control[i].bqNotch,      0.9445903301, 1.0000000000, -1.9911957979, 0.9921786785, 1.0000000000, -1.8808642626, 0.8817926645);



			for (i = 0; i < 4; i++) TMC4671_highLevel_setFluxTarget(2, 0);
			for (i = 0; i < 4; i++) TMC4671_highLevel_setTorqueTarget(2, 0);
			TMC4671_highLevel_stoppedMode(0);
			TMC4671_highLevel_stoppedMode(1);
			TMC4671_highLevel_torqueMode(2);
			// TMC4671_highLevel_stoppedMode(2);
			// TMC4671_highLevel_stoppedMode(3);
			TMC4671_highLevel_torqueMode(3);
			break;

		case '2':
			if(init_done == true)
			{
				mode = MODE_POSITION;
				sweep.k = 0;
				sweep.Ta = TA;
			}
			break;

		case '3':
			if(init_done == true)
			{
				mode = MODE_POSITION_STEP;
				sweep.k = 0;
				sweep.Ta = TA;
			}
			break;

		case '4':
			if(init_done == true)
			{
				mode = MODE_RCCONTROL;
				sweep.k = 0;
				sweep.Ta = TA;
			}
			break;



		case '5':
			if(init_done == true)
			{
				mode = MODE_SWEEP;

				sweep.rate = 3;
				sweep.rate_counter = 3;
				sweep.Ta = sweep.rate*TA;

				sweep.k = 0;
				sweep.mode = M3;
				sweep.U =2*motor_control[2].CmEst;
				sweep.omegaStart = 2 * M_PI * 20;
				sweep.omegaEnd = 2 * M_PI * 3;
				stats = false;
			}
			break;

		case '6':
			if(init_done == true)
			{
				mode = MODE_SWEEP;

				sweep.rate = 1;
				sweep.rate_counter = 1;
				sweep.Ta = sweep.rate*TA;

				sweep.k = 0;
				sweep.mode = M3;
				sweep.U = 3*motor_control[2].CmEst;
				sweep.omegaStart = 2 * M_PI * 5;
				sweep.omegaEnd = 2 * M_PI * 50;
				stats = false;
			}
			break;

		case '7':
			if(init_done == true)
			{
				mode = MODE_SWEEP;

				sweep.rate = 1;
				sweep.rate_counter = 1;
				sweep.Ta = sweep.rate*TA;

				sweep.k = 0;
				sweep.mode = M3;
				sweep.U = 5*motor_control[2].CmEst;
				sweep.omegaStart = 2 * M_PI * 10;
				sweep.omegaEnd = 2 * M_PI * 100;
				stats = false;
			}
			break;


		// case'4':
		// 	disturbanceObserverInit(&motor_control[2], 2*M_PI*100, 2*M_PI*10, 773.0006664650);
		// 	disturbanceObserverInit(&motor_control[3], 2*M_PI*100, 2*M_PI*10, 493.3781002056);
		// 	mode = MODE_STOP;
		// 	stats = false;
		// 	break;

		case 'u':
			motor_control[2].CmEst += 10;
			break;

		case 'j':
			if(motor_control[2].CmEst > 20)
				motor_control[2].CmEst -= 10;
			break;


		case 'i':
			motor_control[3].CmEst += 10;
			break;

		case 'k':
			if(motor_control[3].CmEst > 20)
				motor_control[3].CmEst -= 10;
			break;


		case 'e':
			disturbanceObserverInit(&motor_control[2], motor_control[2].fOBS, motor_control[2].fFB+0.5, motor_control[2].CmEst);
			break;

		case 'd':
			if(motor_control[2].fFB >= 1.0)
				disturbanceObserverInit(&motor_control[2], motor_control[2].fOBS, motor_control[2].fFB-0.5, motor_control[2].CmEst);
			break;

		case 'r':
			disturbanceObserverInit(&motor_control[3], motor_control[3].fOBS, motor_control[3].fFB+0.5, motor_control[3].CmEst);
			break;

		case 'f':
			if(motor_control[3].fFB>=1.0)
				disturbanceObserverInit(&motor_control[3], motor_control[3].fOBS, motor_control[3].fFB-0.5, motor_control[3].CmEst);
			break;


		case 't':
			disturbanceObserverInit(&motor_control[2], motor_control[2].fOBS+1.0, motor_control[2].fFB, motor_control[2].CmEst);
			break;

		case 'g':
			if(motor_control[2].fOBS >= 1.0)
				disturbanceObserverInit(&motor_control[2], motor_control[2].fOBS-1.0, motor_control[2].fFB, motor_control[2].CmEst);
			break;

		case 'z':
			disturbanceObserverInit(&motor_control[3], motor_control[3].fOBS+1.0, motor_control[3].fFB, motor_control[3].CmEst);
			break;

		case 'h':
			if(motor_control[3].fOBS>=1.0)
				disturbanceObserverInit(&motor_control[3], motor_control[3].fOBS-1.0, motor_control[3].fFB, motor_control[3].CmEst);
			break;


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

				print_data_float(data1[i].phiIn[1], 			(uint8_t*)(string)+4*13);
				print_data_float(data2[i].phiInLimited[1],  	(uint8_t*)(string)+4*14);
				print_data_float(data2[i].phiDes[1],  			(uint8_t*)(string)+4*15);
				print_data_float(data2[i].omegaDes[1],   		(uint8_t*)(string)+4*16);
				print_data_float(data2[i].alphaDes[1],  		(uint8_t*)(string)+4*17);
				print_data_float(data1[i].phi[1], 				(uint8_t*)(string)+4*18);
				print_data_float(data1[i].alphaM[1],  			(uint8_t*)(string)+4*19);
				print_data_float(data1[i].phiEst[1],   			(uint8_t*)(string)+4*20);
				print_data_float(data1[i].omegaEst[1],  		(uint8_t*)(string)+4*21);
				print_data_float(data1[i].alphaEst[1], 			(uint8_t*)(string)+4*22);
				print_data_float(data1[i].alphaFrict[1],  		(uint8_t*)(string)+4*23);
				print_data_float(data1[i].iq[1],   				(uint8_t*)(string)+4*24);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, 4*25);
				HAL_Delay(1);
			}
            HAL_Delay(1);
			break;


		case 'y':
			snprintf(string, 1500, "%s", clear_string);
			snprintf(string + strlen(string), 1500-strlen(string), "%s", TMC4671_highLevel_getStatus(2));
			snprintf(string + strlen(string), 1500-strlen(string), "%s", TMC4671_highLevel_getStatus(3));
			snprintf(string + strlen(string), 11500-strlen(string), "---------------------------\n");
			snprintf(string + strlen(string), 1500-strlen(string), "TMC6200 GSTAT:     %5d\t%5d\t%5d\t%5d\n", tmc6200_highLevel_getRegisterGSTAT(0), tmc6200_highLevel_getRegisterGSTAT(1), tmc6200_highLevel_getRegisterGSTAT(2), tmc6200_highLevel_getRegisterGSTAT(3));
			snprintf(string + strlen(string), 1500-strlen(string), "TMC6200 GCONF:     %5d\t%5d\t%5d\t%5d\n", tmc6200_highLevel_getRegisterGCONF(0), tmc6200_highLevel_getRegisterGCONF(1), tmc6200_highLevel_getRegisterGCONF(2), tmc6200_highLevel_getRegisterGCONF(3));
			snprintf(string + strlen(string), 1500-strlen(string), "TMC6200 DRV_CONF:  %5d\t%5d\t%5d\t%5d\n", tmc6200_highLevel_getRegisterDRV_CONF(0), tmc6200_highLevel_getRegisterDRV_CONF(1), tmc6200_highLevel_getRegisterDRV_CONF(2), tmc6200_highLevel_getRegisterDRV_CONF(3));
			snprintf(string + strlen(string), 1500-strlen(string), "as5074u Errors:    %5d\t%5d\t%5d\t%5d\n", as5074uErrorCounter[0], as5074uErrorCounter[1], as5074uErrorCounter[2], as5074uErrorCounter[3]);
			snprintf(string + strlen(string), 1500-strlen(string), "---------------------------\n");
			snprintf(string + strlen(string), 1500-strlen(string), "finstats\n");
			HAL_UART_Transmit_IT(&huart3, (uint8_t*) string, strlen(string));
			break;

		default:
			break;
		} // end of: switch(rx_byte)
	} // end of: if(rx_byte_new)
	/* -------------------------------------------------------------------------  */


} // end of: void logic_loop(void)





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		Ta_counter++;
	}

}


void HAL_SYSTICK_Callback(void)
{
	systick_counter++;

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
