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
bool mode_matlab = 1;


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
	static uint8_t rate = 1;
	static uint8_t rate_old = 1;

	static pi_controller_t piVelocity[4];
	static pi_controller_t piPosition[4];

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

			for(i=0; i<4; i++) motor_control[i].positionTargetBeta = calcAngleTarget(i, angleIn);
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
			snprintf(string+strlen(string), 2000-strlen(string), "Rate = %d => Ta = %1.1f\n", rate, (float)rate*TA*1000);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "Beta:\n");
			snprintf(string+strlen(string), 2000-strlen(string), "Pos Target:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionTargetBeta, motor_control[1].positionTargetBeta, motor_control[2].positionTargetBeta, motor_control[3].positionTargetBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "Pos Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].positionActualBeta, motor_control[1].positionActualBeta, motor_control[2].positionActualBeta, motor_control[3].positionActualBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "Vel Target:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].velocityTargetBeta, motor_control[1].velocityTargetBeta, motor_control[2].velocityTargetBeta, motor_control[3].velocityTargetBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "Vel Actual:  % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].velocityActualBeta, motor_control[1].velocityActualBeta, motor_control[2].velocityActualBeta, motor_control[3].velocityActualBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "I Target:    % 2.1f  % 2.1f  % 2.1f  % 2.1f\n", motor_control[0].currentTargetBeta, motor_control[1].currentTargetBeta, motor_control[2].currentTargetBeta, motor_control[3].currentTargetBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "PI: Velocity 2:\n");
			snprintf(string+strlen(string), 2000-strlen(string), "error: % 2.1f\n", motor_control[2].velocityErrorBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "x: %f\n", piVelocity[2].x);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "PI: Velocity 3:\n");
			snprintf(string+strlen(string), 2000-strlen(string), "error: % 2.1f\n", motor_control[3].velocityErrorBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "x: %f\n", piVelocity[3].x);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "PI: Position 2:\n");
			snprintf(string+strlen(string), 2000-strlen(string), "error: % 2.1f\n", motor_control[2].positionErrorBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "x: %f\n", piPosition[2].x);
			snprintf(string+strlen(string), 2000-strlen(string), "---------------------------\n");
			snprintf(string+strlen(string), 2000-strlen(string), "PI: Position 3:\n");
			snprintf(string+strlen(string), 2000-strlen(string), "error: % 2.1f\n", motor_control[3].positionErrorBeta);
			snprintf(string+strlen(string), 2000-strlen(string), "x: %f\n", piPosition[3].x);
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		}
	} // end of: if(systick_counter_3 >= 200) //5Hz




	/* -------------------------------------------------------------------------  */

	if (systick_counter >= rate) //Ta = rate * 500us
	{
		systick_counter = 0;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

		for(i=2; i<4; i++) motor_data[i].torqueActual   = tmc4671_getActualTorque_raw(i);
		for(i=2; i<4; i++) motor_data[i].positionActual = tmc4671_getActualPosition(i);
		for(i=2; i<4; i++) motor_data[i].velocityActual = tmc4671_getActualVelocity(i);

		// discard wrong values!!!!!


		for(i=2; i<4; i++) motor_control[i].currentActualBeta = (float)motor_data[i].torqueActual/500.0; 				// Ampere
		for(i=2; i<4; i++) motor_control[i].positionActualBeta = (float)motor_data[i].positionActual/65536.0*2*M_PI; 	// rad
		for(i=2; i<4; i++) motor_control[i].velocityActualBeta = (float)motor_data[i].velocityActual/60*2*M_PI;			// rad per s




		if (mode == MODE_STOP)
		{
			for (i = 2; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);

		}
		else if(mode == MODE_IDLE)
		{

		}
		else if (mode == MODE_TORQUE_SWEEP)
		{
			sweep.t = sweep.k * sweep.Ta;
			sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
			sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t	+ (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

			if(sweep.mode == STEP)
				for (i = 2; i < 4; i++) motor_control[i].currentTargetBeta = sweep.U;
			else
				for (i = 2; i < 4; i++) motor_control[i].currentTargetBeta = sweep.out;




			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)(500.0*motor_control[i].currentTargetBeta);
			// for(i=2; i<4; i++) tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);


			if(sweep.mode == STEP)
			{
				tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
				tmc4671_setTargetTorque_raw(3, motor_data[3].torqueTarget);
			}
			else
			{
				// tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
				// tmc4671_setTargetTorque_raw(3, 0);
				tmc4671_setTargetTorque_raw(3, motor_data[3].torqueTarget);
				tmc4671_setTargetTorque_raw(2, 0);
			}

			for (i = 2; i < 4; i++) data1[sweep.k].currentActual[i-2]  = motor_control[i].currentActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].currentTarget[i-2]  = motor_control[i].currentTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = motor_control[i].velocityTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = motor_control[i].positionActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = motor_control[i].positionTargetBeta;

			if(sweep.k >= (DATA_N-1) ) // stop
            {
                mode = MODE_STOP;
                stats = true;
				rate = 1;

                if(mode_matlab)
				{
                    snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
            	}
            }
			sweep.k++;
		}
		else if(mode == MODE_CONTROL_TEST)
		{
            sweep.t = sweep.k * sweep.Ta;
    		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
    		sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t + (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );





			motor_control[2].velocityErrorBeta = sweep.out;
 			motor_control[2].currentTargetBeta = PIControl(&piVelocity[2], motor_control[2].velocityErrorBeta);



			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)(500.0*motor_control[i].currentTargetBeta);

			//for (i = 2; i < 4; i++) = tmc4671_setTargetTorque_raw(i, motor_data[i].torqueTarget);


			for (i = 2; i < 4; i++) data1[sweep.k].currentActual[i-2]  = motor_control[i].currentActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].currentTarget[i-2]  = motor_control[i].currentTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = motor_control[i].velocityTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = motor_control[i].positionActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = motor_control[i].positionTargetBeta;

			if(sweep.k >= (DATA_N-1) ) // stop
            {
                mode = MODE_STOP;
                stats = true;
				rate = 1;

                if(mode_matlab)
				{
                    snprintf(string, 200, 	"fintest\n");
                    HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
                }
            }

			sweep.k++;
		}
		else if (mode == MODE_VELOCITY)
		{
			sweep.t = sweep.k * sweep.Ta;
    		sweep.r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
    		sweep.out = sweep.U*sweep.r*sin(sweep.omegaStart*sweep.t + (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(sweep.t*sweep.t) );

			if(sweep.mode == M2M3)
			{

			}
			else if (sweep.mode == M2)
			{
				motor_control[2].velocityTargetBeta = sweep.out;
				motor_control[3].velocityTargetBeta = 0;
			}
			else if (sweep.mode == M3)
			{
				motor_control[2].velocityTargetBeta = 0;
				motor_control[3].velocityTargetBeta = sweep.out;
			}

			for (i = 2; i < 4; i++) motor_control[i].velocityErrorBeta = motor_control[i].velocityTargetBeta-motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++)  motor_control[i].currentTargetBeta = PIControl(&piVelocity[i], motor_control[i].velocityErrorBeta);

			for (i = 2; i < 4; i++)
			{
				if(motor_control[i].currentTargetBeta >= 30)
					motor_control[i].currentTargetBeta = 30;
				if(motor_control[i].currentTargetBeta <= -30)
					motor_control[i].currentTargetBeta = -30;
			}

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)(500.0*motor_control[i].currentTargetBeta);
			tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
			tmc4671_setTargetTorque_raw(3, motor_data[3].torqueTarget);

			for (i = 2; i < 4; i++) data1[sweep.k].currentActual[i-2]  = motor_control[i].currentActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].currentTarget[i-2]  = motor_control[i].currentTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = motor_control[i].velocityTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = motor_control[i].positionActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = motor_control[i].positionTargetBeta;


			if(sweep.k >= (DATA_N-1)) // stop
            {
                mode = MODE_STOP;
                stats = true;
				rate = 1;

                if(mode_matlab)
				{
                    snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
            	}
            }

			if (sweep.mode == M2 || sweep.mode == M3)
				sweep.k++;
		}
		else if (mode == MODE_VELOCITY_STEP)
		{
			if(sweep.k < 100)
			{
				motor_control[2].velocityTargetBeta = 0;
				motor_control[3].velocityTargetBeta = 0;
			}
			else if (sweep.k == 100)
			{
				if (sweep.mode == M2)
				{
					motor_control[2].velocityTargetBeta = sweep.U;
					motor_control[3].velocityTargetBeta = 0;
				}
				else if (sweep.mode == M3)
				{
					motor_control[2].velocityTargetBeta = 0;
					motor_control[3].velocityTargetBeta = sweep.U;
				}
			}
			else if(sweep.k >= 900)
			{
				motor_control[2].velocityTargetBeta = 0;
				motor_control[3].velocityTargetBeta = 0;
			}


			for (i = 2; i < 4; i++) motor_control[i].velocityErrorBeta = motor_control[i].velocityTargetBeta-motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++)  motor_control[i].currentTargetBeta = PIControl(&piVelocity[i], motor_control[i].velocityErrorBeta);

			for (i = 2; i < 4; i++)
			{
				if(motor_control[i].currentTargetBeta >= 30)
					motor_control[i].currentTargetBeta = 30;
				if(motor_control[i].currentTargetBeta <= -30)
					motor_control[i].currentTargetBeta = -30;
			}

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)(500.0*motor_control[i].currentTargetBeta);
			tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
			tmc4671_setTargetTorque_raw(3, motor_data[3].torqueTarget);

			for (i = 2; i < 4; i++) data1[sweep.k].currentActual[i-2]  = motor_control[i].currentActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].currentTarget[i-2]  = motor_control[i].currentTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = motor_control[i].velocityTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = motor_control[i].positionActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = motor_control[i].positionTargetBeta;


			if(sweep.k >= (DATA_N-1)) // stop
            {
                mode = MODE_STOP;
                stats = true;
				rate = 1;

                if(mode_matlab)
				{
                    snprintf(string, 200, 	"fintest\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
            	}
            }

			if (sweep.mode == M2 || sweep.mode == M3)
				sweep.k++;
		}
		else if(mode == MODE_POSITION)
		{
			for (i = 2; i < 4; i++) motor_control[i].positionErrorBeta = motor_control[i].positionTargetBeta-motor_control[i].positionActualBeta;
			for (i = 2; i < 4; i++) motor_control[i].velocityTargetBeta = PIControl(&piPosition[i], motor_control[i].positionErrorBeta);

			for (i = 2; i < 4; i++) motor_control[i].velocityErrorBeta = motor_control[i].velocityTargetBeta-motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++)  motor_control[i].currentTargetBeta = PIControl(&piVelocity[i], motor_control[i].velocityErrorBeta);

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)(500.0*motor_control[i].currentTargetBeta);
			tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
			tmc4671_setTargetTorque_raw(3, motor_data[3].torqueTarget);

		}
		else if(mode == MODE_TORQUE_X)
		{
			if(sweep.k < 1000)
				motor_control[2].currentTargetBeta = 0;
			else if (sweep.k == 1000)
				motor_control[2].currentTargetBeta = 5;
			else if(sweep.k == 2000)
				motor_control[2].currentTargetBeta = 10;
			else if(sweep.k == 3000)
				motor_control[2].currentTargetBeta = 15;
			else if(sweep.k == 4000)
				motor_control[2].currentTargetBeta = 20;
			else if(sweep.k == 5000)
				motor_control[2].currentTargetBeta = 25;
			else if(sweep.k == 6000)
				motor_control[2].currentTargetBeta = 0;
			else if(sweep.k >=7000)
				motor_control[2].currentTargetBeta = 0;

			for (i = 2; i < 4; i++)
			{
				if(motor_control[i].currentTargetBeta > 30)
					motor_control[i].currentTargetBeta = 30;
				if(motor_control[i].currentTargetBeta < -30)
					motor_control[i].currentTargetBeta = -30;
			}

			for (i = 2; i < 4; i++) motor_data[i].torqueTarget = (int32_t)(500.0*motor_control[i].currentTargetBeta);
			tmc4671_setTargetTorque_raw(2, motor_data[2].torqueTarget);
			tmc4671_setTargetFlux_raw(2,0);
			// tmc4671_setTargetTorque_raw(3, motor_data[3].torqueTarget);

			for (i = 2; i < 4; i++) data1[sweep.k].currentActual[i-2]  = motor_control[i].currentActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].currentTarget[i-2]  = motor_control[i].currentTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityActual[i-2] = motor_control[i].velocityActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].velocityTarget[i-2] = motor_control[i].velocityTargetBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionActual[i-2] = motor_control[i].positionActualBeta;
			for (i = 2; i < 4; i++) data1[sweep.k].positionTarget[i-2] = motor_control[i].positionTargetBeta;


			if(sweep.k >= (DATA_N-1)) // stop
            {
                mode = MODE_STOP;
                stats = true;
				rate = 1;

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
			rate = 1;
			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta = 0;
			tmc4671_setTargetFlux_raw(2, 0);
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case 'i':
			mode = MODE_IDLE;
			stats = true;
			break;


		case '1':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 5;
			sweep.omegaStart = 2 * M_PI * 0.5;
			sweep.omegaEnd = 2 * M_PI * 5;
			sweep.mode = LOW;
			rate = 3;
			rate_old = 3;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '2':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 15;
			sweep.omegaStart = 2 * M_PI * 2;
			sweep.omegaEnd = 2 * M_PI * 20;
			sweep.mode = MID;
			rate = 2;
			rate_old = 2;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '3':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 30;
			sweep.omegaStart = 2 * M_PI * 10;
			sweep.omegaEnd = 2 * M_PI * 100;
			sweep.mode = HIGH;
			rate = 1;
			rate_old = 1;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '4':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 30;
			sweep.omegaStart = 2 * M_PI * 40;
			sweep.omegaEnd = 2 * M_PI * 400;
			sweep.mode = HIGH;
			rate = 1;
			rate_old = 1;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '5':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 5;
			sweep.mode = STEP;
			rate = 1;
			rate_old = 1;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '6':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 10;
			sweep.mode = STEP;
			rate = 1;
			rate_old = 1;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case '7':
			mode = MODE_TORQUE_SWEEP;
			sweep.k = 0;
			sweep.U = 20;
			sweep.mode = STEP;
			rate = 1;
			rate_old = 1;
			sweep.Ta = rate*TA;
			stats = false;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;


		case '8':
			mode = MODE_CONTROL_TEST;
			sweep.k = 0;
			sweep.omegaStart = 2 * M_PI * 20;
			sweep.omegaEnd = 2 * M_PI *200;
			sweep.U = 1;
			rate = 1;
			rate_old = 1;
			sweep.Ta = rate*TA;
			stats = false;



			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_STOPPED);
			break;

		case '9':
			mode = MODE_RCCONTROL;
			for (i = 0; i < 4; i++) tmc4671_switchToMotionMode(i, TMC4671_MOTION_MODE_TORQUE);
			break;

		case 'g':
			mode = MODE_VELOCITY;
			rate = 1; rate_old = 1;
			sweep.mode = M2M3;
			sweep.k = 0;
			PIControlSetup(&piVelocity[2], 1.00000000, 0.12500000, 0.11125419, 0.52416212, 50.0, 0.0);
			PIControlSetup(&piVelocity[3], 1.00000000, 0.12500000, 0.20195143, 0.80663086, 50.0, 0.0);
			PIControlSetup(&piPosition[2], 1.00000000, 2.00000000, 1.62602237, 98.36947803, 20.0, 0.0);
			PIControlSetup(&piPosition[3], 1.00000000, 2.00000000, 1.61337772, 97.75207179, 20.0, 0.0);



			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_TORQUE);
			break;

		case 'z':
			mode = MODE_VELOCITY;
			rate = 1; rate_old = 1;

			PIControlSetup(&piVelocity[2], 1.00000000, 0.12500000, 0.11125419, 0.52416212, 50.0, 0.0);
			PIControlSetup(&piVelocity[3], 1.00000000, 0.12500000, 0.20195143, 0.80663086, 50.0, 0.0);
			PIControlSetup(&piPosition[2], 1.00000000, 2.00000000, 1.62602237, 98.36947803, 20.0, 0.0);
			PIControlSetup(&piPosition[3], 1.00000000, 2.00000000, 1.61337772, 97.75207179, 20.0, 0.0);
			sweep.k = 0;
			sweep.omegaStart = 2 * M_PI * 20;
			sweep.omegaEnd = 2 * M_PI *200;
			sweep.U = 3;
			sweep.Ta = rate*TA;
			stats = false;
			sweep.mode = M2;

			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta = 0;
			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_STOPPED);
			break;

		case 'h':
			mode = MODE_VELOCITY;
			rate = 1; rate_old = 1;

			PIControlSetup(&piVelocity[2], 1.00000000, 0.12500000, 0.11125419, 0.52416212, 50.0, 0.0);
			PIControlSetup(&piVelocity[3], 1.00000000, 0.12500000, 0.20195143, 0.80663086, 50.0, 0.0);
			PIControlSetup(&piPosition[2], 1.00000000, 2.00000000, 1.62602237, 98.36947803, 20.0, 0.0);
			PIControlSetup(&piPosition[3], 1.00000000, 2.00000000, 1.61337772, 97.75207179, 20.0, 0.0);
			sweep.k = 0;
			sweep.omegaStart = 2 * M_PI *20;
			sweep.omegaEnd = 2 * M_PI *200;
			sweep.U = 3;
			sweep.Ta = rate*TA;
			stats = false;
			sweep.mode = M3;

			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta = 0;
			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_TORQUE);
			break;


		case 'u':
			mode = MODE_VELOCITY_STEP;
			rate = 1; rate_old = 1;

			PIControlSetup(&piVelocity[2], 1.00000000, 0.12500000, 0.11125419, 0.52416212, 50.0, 0.0);
			PIControlSetup(&piVelocity[3], 1.00000000, 0.12500000, 0.20195143, 0.80663086, 50.0, 0.0);
			PIControlSetup(&piPosition[2], 1.00000000, 2.00000000, 1.62602237, 98.36947803, 20.0, 0.0);
			PIControlSetup(&piPosition[3], 1.00000000, 2.00000000, 1.61337772, 97.75207179, 20.0, 0.0);

			sweep.k = 0;
			sweep.U = 100*2*M_PI/60;
			sweep.Ta = rate*TA;
			stats = false;
			sweep.mode = M2;

			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta = 0;
			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_STOPPED);
			break;


		case 'j':
			mode = MODE_VELOCITY_STEP;
			rate = 1; rate_old = 1;

			PIControlSetup(&piVelocity[2], 1.00000000, 0.12500000, 0.11125419, 0.52416212, 50.0, 0.0);
			PIControlSetup(&piVelocity[3], 1.00000000, 0.12500000, 0.20195143, 0.80663086, 50.0, 0.0);
			PIControlSetup(&piPosition[2], 1.00000000, 2.00000000, 1.62602237, 98.36947803, 20.0, 0.0);
			PIControlSetup(&piPosition[3], 1.00000000, 2.00000000, 1.61337772, 97.75207179, 20.0, 0.0);


			sweep.k = 0;
			sweep.U = 100*2*M_PI/60;
			sweep.Ta = rate*TA;
			stats = false;
			sweep.mode = M3;

			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta = 0;
			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_TORQUE);
			break;

		case 'o':
			mode = MODE_POSITION;
			rate = 1; rate_old = 1;
			sweep.k = 0;
			PIControlSetup(&piVelocity[2], 1.00000000, 0.25000000, 0.29541424, 0.80627060, 50.0, 0.0);
			PIControlSetup(&piVelocity[3], 1.00000000, 0.25000000, 0.28623800, 1.22981941, 50.0, 0.0);
			PIControlSetup(&piPosition[2], 1.00000000, 2.00000000, 3.09568916, 144.01849946, 50.0, 0.0);
			PIControlSetup(&piPosition[3], 1.00000000, 2.00000000, 3.05086811, 149.19523456, 50.0, 0.0);

			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta = 0;
			for (i = 0; i < 4; i++) motor_control[i].positionTargetBeta = 0;

			tmc4671_switchToMotionMode(0, TMC4671_MOTION_MODE_STOPPED);
			tmc4671_switchToMotionMode(1, TMC4671_MOTION_MODE_STOPPED);
			//tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_STOPPED);

			tmc4671_switchToMotionMode(3, TMC4671_MOTION_MODE_TORQUE);
			break;

		case 'p':

			mode = MODE_IDLE;
			stats = true;
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			tmc4671_setTargetFlux_raw(2, 5000);
			tmc4671_setTargetTorque_raw(2, 0);
			// tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);
			// tmc4671_setTargetTorque_raw(2, 10000);
		break;


		case 'v':

			mode = MODE_TORQUE_X;
			stats = true;
			tmc4671_switchToMotionMode(2, TMC4671_MOTION_MODE_TORQUE);

			rate = 1; rate_old = 1;
			sweep.k = 0;
			sweep.U = 3000;
			sweep.Ta = rate*TA;
			stats = false;
			sweep.mode = M2;

		break;

		case '+':
			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta += 10;
		break;

		case '-':
			for (i = 0; i < 4; i++) motor_control[i].velocityTargetBeta -= 10;
		break;




		case 'x': // print data
			for (i = 0; i < DATA_N; i++)
			{
				print_data_float((float)i,   					(uint8_t*)(string));
				print_data_float((float)rate_old,   			(uint8_t*)(string)+4*1);
				print_data_float(data1[i].currentTarget[0], 	(uint8_t*)(string)+4*2);
				print_data_float(data1[i].currentActual[0],  	(uint8_t*)(string)+4*3);
				print_data_float(data1[i].velocityTarget[0],   	(uint8_t*)(string)+4*4);
				print_data_float(data1[i].velocityActual[0],  	(uint8_t*)(string)+4*5);
				print_data_float(data1[i].positionTarget[0],  	(uint8_t*)(string)+4*6);
				print_data_float(data1[i].positionActual[0],   	(uint8_t*)(string)+4*7);
				print_data_float(data1[i].currentTarget[1], 	(uint8_t*)(string)+4*8);
				print_data_float(data1[i].currentActual[1],  	(uint8_t*)(string)+4*9);
				print_data_float(data1[i].velocityTarget[1],   	(uint8_t*)(string)+4*10);
				print_data_float(data1[i].velocityActual[1],  	(uint8_t*)(string)+4*11);
				print_data_float(data1[i].positionTarget[1],  	(uint8_t*)(string)+4*12);
				print_data_float(data1[i].positionActual[1],   	(uint8_t*)(string)+4*13);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, 4*14);
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
