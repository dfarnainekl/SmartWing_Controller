#include "logic.h"
#include "swdriver.h"
#include "tmc6200/TMC6200.h"
#include "tmc6200/TMC6200_highLevel.h"
#include "tmc4671/TMC4671_highLevel.h"
#include "as5047U.h"
#include "usart.h"
#include "tim.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define SWEEP_TA	0.001
#define SWEEP_N		10240

#define MATLAB 1


sweep_t sweep = {	.Ta = SWEEP_TA,
									.N = SWEEP_N,
									.U = 1,
									.omegaStart = 2*M_PI*1,
									.omegaEnd = 2*M_PI*50,
									.k = 0,
									.mode = AIL
								};

__attribute__((section(".ram_d1"))) data1_t data1[SWEEP_N] = {0};
__attribute__((section(".ram_d2"))) data2_t data2[SWEEP_N] = {0};

volatile uint16_t systick_counter = 0;
volatile uint16_t systick_counter_2 = 0;
volatile uint8_t 	systick_counter_3 = 0;

volatile uint8_t rx_byte_new = 0;
uint8_t rx_byte;

volatile uint16_t pwm_in[4] = {1500, 1500, 1500, 1500};
volatile bool pwm_updated = false;
volatile float angleOut[4] = {0, 0, 0, 0};
uint16_t angle[4];




bool chirp = false;
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
		#endif
	 } //TODO: do-while()
	 rx_byte_new = 0;


	// // initialize left wing
	// TMC4671_highLevel_initEncoder_new(0);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(0);
	// TMC4671_highLevel_positionMode_rampToZero(0);
	// TMC4671_highLevel_stoppedMode(0);
	// HAL_Delay(100);
	// TMC4671_highLevel_initEncoder_new(1);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(1);
	// TMC4671_highLevel_positionMode_rampToZero(1);
	// TMC4671_highLevel_positionMode_fluxTorqueRamp(0);
	// TMC4671_highLevel_positionMode_rampToZero(0);

	HAL_Delay(100);

	// initialize right wing
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




	// pwm inputs //FIXME: timer interrupt priority lower than spi?
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	// HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	for(i=0; i<4; i++)	TMC4671_highLevel_setPositionFilter(i, false);
	//for(i=0; i<4; i++)	TMC4671_highLevel_setCurrentLimit(i, 15000);



	//for(i=0; i<4; i++)	TMC4671_highLevel_setIntegralPosition(i, 20);

	// HAL_Delay(5000);
	//for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
	//for(i=0; i<4; i++) 	TMC4671_highLevel_pwmOff(i);


	#if MATLAB
		snprintf(string, 500, 	"fininit\n");
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
	#endif
}


void logic_loop(void)
{
	static int32_t positionTarget[4] = {0, 0, 0, 0};
	static float angleIn[4] = {0, 0, 0, 0};
	static uint16_t i;
	static char string[1500];

	static bool button_stop=true;
	button_stop = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	if(button_stop == false)
	{
		for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
		for(i=0; i<4; i++)	TMC4671_highLevel_pwmOff(i);
		chirp = false;
	}



	if(rx_byte_new)
	{
		rx_byte_new = 0;

		switch(rx_byte)
		{
			case ' ':
				for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
				for(i=0; i<4; i++)	TMC4671_highLevel_pwmOff(i);
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

			case 'g':
				if(proportional_vel>=500)
				{
					proportional_vel -= 500;
					for(i=0; i<4; i++)	TMC4671_highLevel_setProportionalVelocity(i, proportional_vel);
				}
				break;

			case 't':
				if(proportional_vel<=20000)
				{
					proportional_vel += 500;
					for(i=0; i<4; i++)	TMC4671_highLevel_setProportionalVelocity(i, proportional_vel);
				}
				break;

			case 'j':
				if(integral_vel>=500)
				{
						integral_vel -= 500;
					for(i=0; i<4; i++)	TMC4671_highLevel_setIntegralVelocity(i, integral_vel);
				}
				break;

			case 'u':
				if(integral_vel<=20000)
				{
					integral_vel += 500;
					for(i=0; i<4; i++)	TMC4671_highLevel_setIntegralVelocity(i, integral_vel);
				}
				break;

			case 'h':
				if(proportional_pos>=50)
				{
					proportional_pos -= 50;
					for(i=0; i<4; i++)	TMC4671_highLevel_setProportionalPosition(i, proportional_pos);
				}
				break;

			case 'z':
				if(proportional_pos<=2000)
				{
					proportional_pos += 50;
					for(i=0; i<4; i++)	TMC4671_highLevel_setProportionalPosition(i, proportional_pos);
				}
				break;

			case 'k':
				if(integral_pos>=10)
				{
					integral_pos -= 10;
					for(i=0; i<4; i++)	TMC4671_highLevel_setIntegralPosition(i, integral_pos);
				}
				break;

			case 'i':
				if(integral_pos<=150)
				{
					integral_pos += 10;
					for(i=0; i<4; i++)	TMC4671_highLevel_setIntegralPosition(i, integral_pos);
				}
				break;

			case 'o':
				for(i=0; i<4; i++)  TMC4671_highLevel_stoppedMode(i);
				break;

			case 'p':
				TMC4671_highLevel_positionMode_rampToZero(1);
				TMC4671_highLevel_positionMode_rampToZero(3);
				TMC4671_highLevel_positionMode2(0);
				TMC4671_highLevel_positionMode2(2);
				break;

			case 'a':
				stats = false;
				break;

			case 'y':
					stats = true;
				break;

			case '+':
				sweep.U += 1;
				break;

			case '-':
				if (sweep.U >1 )
					sweep.U -= 1;
				break;

			case '1': // testcase 1: sweep aileron
				chirp = true;
				systick_counter_3 = 0;
				sweep.k = 0;
				sweep.U = 5;
				sweep.mode = AIL;
				break;

			case '2': // testcase 2: sweep flap
				chirp = true;
				systick_counter_3 = 0;
				sweep.k = 0;
				sweep.U = 1;
				sweep.mode = FLP;
				break;

			case '3': // testcase 3: jump aileron
				chirp = true;
				systick_counter_3 = 0;
				sweep.k = 0;
				sweep.U = 5;
				sweep.mode = JMP_AIL;
				break;

			case '4': // testcase 3: jump flaps
				chirp = true;
				systick_counter_3 = 0;
				sweep.k = 0;
				sweep.U = 3;
				sweep.mode = JMP_FLP;
				break;

			case '5':
				chirp = true;
				systick_counter_3 = 0;
				sweep.k = 0;
				sweep.U = 2;
				sweep.mode = SWING_INNER;
				TMC4671_highLevel_stoppedMode(0);
				TMC4671_highLevel_stoppedMode(2);
				TMC4671_highLevel_pwmOff(0);
				TMC4671_highLevel_pwmOff(2);
				break;

			case '6':
				chirp = true;
				systick_counter_3 = 0;
				sweep.k = 0;
				sweep.U = 2;
				sweep.mode = SWING_OUTER;
				TMC4671_highLevel_stoppedMode(1);
				TMC4671_highLevel_stoppedMode(3);
				TMC4671_highLevel_pwmOff(1);
				TMC4671_highLevel_pwmOff(3);
				break;

			case '0': // stop sweep
				chirp = false;
				systick_counter_3 = 0;
				sweep.k = 0;
				for(i=0; i<4; i++)	TMC4671_highLevel_stoppedMode(i);
				for(i=0; i<4; i++)	TMC4671_highLevel_pwmOn(i);
				break;

			case 'r':
				for(i=0; i<4; i++)	angle[i] = as5047U_getAngle(i);

				snprintf(string, 1500, 	"%s", clear_string);
				//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(0));
				//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(1));
				snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(2));
				snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(3));
				snprintf(string+strlen(string), 1500, "pwm_in:     %d %d %d %d\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
				snprintf(string+strlen(string), 1500, "angleIn:    % 2.1f % 2.1f % 2.1f % 2.1f\n", angleIn[0], angleIn[1], angleIn[2], angleIn[3]);
				snprintf(string+strlen(string), 1500, "angleOut:   % 2.1f % 2.1f % 2.1f % 2.1f\n", angleOut[0], angleOut[1], angleOut[2], angleOut[3]);
				snprintf(string+strlen(string), 1500, "---------------------------\n");
				snprintf(string+strlen(string), 1500, "enc0: %5d\tenc1: %5d\tenc2: %5d\tenc3: %5d\n", angle[0], angle[1], angle[2], angle[3]);
				snprintf(string+strlen(string), 1500, "---------------------------\n");
				snprintf(string+strlen(string), 1500, "[o] ... stopped mode\n[p] ... position mode\n[SPACE] ... STOP\n");
				snprintf(string+strlen(string), 1500, "fin\n");
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				break;

			case 'l': // print data
				sweep.len = snprintf(sweep.string,800,"%s", clear_string);
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)sweep.string, sweep.len);
				uint32_t n = 0;
				if(sweep.mode==JMP_FLP || sweep.mode==JMP_AIL)
					n = 5000;
				else
					n = SWEEP_N;

				for(i=0; i<n; i++)
				{
					// sweep.len = snprintf(sweep.string, 800,"%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%ld;%ld;%ld;%ld;%ld;%ld;%ld;%ld\n", i,
					// data2[i].torqueTarget[0], 	data2[i].torqueTarget[1], 	data2[i].torqueTarget[2], 	data2[i].torqueTarget[3],
					// data1[i].torqueActual[0], 	data1[i].torqueActual[1], 	data1[i].torqueActual[2], 	data1[i].torqueActual[3],
					// data2[i].velocityTarget[0],	data2[i].velocityTarget[1], data2[i].velocityTarget[2], data2[i].velocityTarget[3],
					// data1[i].velocityActual[0],	data1[i].velocityActual[1], data1[i].velocityActual[2], data1[i].velocityActual[3],
					// data1[i].posTarget[0], 			data1[i].posTarget[1], 			data1[i].posTarget[2], 			data1[i].posTarget[3],
					// data1[i].posActual[0],			data1[i].posActual[1], 			data1[i].posActual[2], 			data1[i].posActual[3]	);
					sweep.len = snprintf(sweep.string, 200,"%d;%d;%d;%d;%d;%d;%d;%d;%d;%ld;%ld;%ld;%ld\n", i,
					data2[i].torqueTarget[2], 	data2[i].torqueTarget[3],
					data1[i].torqueActual[2], 	data1[i].torqueActual[3],
					data2[i].velocityTarget[2], data2[i].velocityTarget[3],
					data1[i].velocityActual[2], data1[i].velocityActual[3],
					data1[i].posTarget[2], 			data1[i].posTarget[3],
					data1[i].posActual[2], 			data1[i].posActual[3]	);
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)sweep.string, sweep.len);
					HAL_Delay(2);
				}
				#if MATLAB
					snprintf(string, 200, 	"finplot\n");
					HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
				#endif
				break;

			default:
				break;
		} // end of: switch(rx_byte)
	} // end of: if(rx_byte_new)


	if(pwm_updated && chirp == false)
	{
		pwm_updated = false;

		for(i=0; i<4; i++) 	angleIn[i] =  ( (float)(pwm_in[i] - 1500)/ 500.0 * ANGLE_MAX_ALPHA_DEGREE ); // in degree
		for(i=0; i<4; i++)	positionTarget[i] = clacAngle(i, angleIn);
		for(i=0; i<4; i++) 	TMC4671_highLevel_setPosition_nonBlocking(i, positionTarget[i]);
	}


	if(systick_counter_3 && chirp)
	{
		static float pos, r, t;
		systick_counter_3 = 0;
		sweep.k++;

		t = sweep.k * sweep.Ta;
		r = sat(10*(float)sweep.k/sweep.N)*sat(10*(float)(sweep.N-sweep.k)/sweep.N);
		pos = sweep.U*r*sin(sweep.omegaStart*t	+ (sweep.omegaEnd-sweep.omegaStart)/(sweep.N*sweep.Ta*2)*(t*t) );

		if(	sweep.mode == AIL)//aileron sweep
		{
			angleIn[0]= pos;
			angleIn[1]= 0;
			angleIn[2]= -pos;
			angleIn[3]= 0;
		}
		else if(	sweep.mode == FLP)//flap sweep
		{
			angleIn[0]= 0;
			angleIn[1]= pos;
			angleIn[2]= 0;
			angleIn[3]= pos;
		}
		else if (  sweep.mode == JMP_AIL  )
		{
				if(sweep.k < 1000 || sweep.k > 3000)
				{
					angleIn[0]= 0;
					angleIn[1]= 0;
					angleIn[2]= 0;
					angleIn[3]= 0;
				}
				else
				{
					angleIn[0]= sweep.U;
					angleIn[1]= 0;
					angleIn[2]= -sweep.U;
					angleIn[3]= 0;
				}
		}
		else if (  sweep.mode == JMP_FLP  )
		{
				if(sweep.k < 1000 || sweep.k > 3000)
				{
					angleIn[0]= 0;
					angleIn[1]= 0;
					angleIn[2]= 0;
					angleIn[3]= 0;
				}
				else
				{
					angleIn[0]= 0;
					angleIn[1]= sweep.U;
					angleIn[2]= 0;
					angleIn[3]= sweep.U;
				}
		}
		else if (sweep.mode == SWING_INNER )
		{
			angleIn[0]= 0;
			angleIn[1]= pos;
			angleIn[2]= 0;
			angleIn[3]= pos;

		}
		else if ( sweep.mode == SWING_OUTER )
		{
			angleIn[0]= pos;
			angleIn[1]= 0;
			angleIn[2]= pos;
			angleIn[3]= 0;
		}

		for(i=0; i<4; i++)	positionTarget[i] = clacAngle(i, angleIn);
		for(i=0; i<4; i++) 	TMC4671_highLevel_setPosition(i, positionTarget[i]);
		for(i=2; i<4; i++) data1[sweep.k].posTarget[i] 			= positionTarget[i];
		for(i=2; i<4; i++) data1[sweep.k].posActual[i] 			= TMC4671_highLevel_getPositionActual(i);
		for(i=2; i<4; i++) data1[sweep.k].torqueActual[i] 	= TMC4671_highLevel_getTorqueActual(i);
		for(i=2; i<4; i++) data1[sweep.k].velocityActual[i] = TMC4671_highLevel_getVelocityActual(i);
		for(i=2; i<4; i++) data2[sweep.k].velocityTarget[i] = TMC4671_highLevel_getVelocityTarget(i);
		for(i=2; i<4; i++) data2[sweep.k].torqueTarget[i] 	= TMC4671_highLevel_getTorqueActual(i);

		// snprintf(string, 500, 	"%s%ld\t%ld\n", clear_string, positionTarget[2], positionTarget[3]);
		// HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));


		if(sweep.k >= sweep.N-1 || ((sweep.mode==JMP_FLP || sweep.mode==JMP_AIL) && sweep.k>5000) )
		{
			chirp = false;
			sweep.k = 0;
			pos  = 0;

			for(i=0; i<4; i++)	TMC4671_highLevel_stoppedMode(i);
			for(i=0; i<4; i++)	TMC4671_highLevel_pwmOn(i);

			#if MATLAB
				snprintf(string, 1500, 	"fintest\n");
				HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
			#endif
		}
	}

	if(systick_counter >= 20) //50Hz
	{
		systick_counter = 0;
	}

	if(systick_counter_2 >= 200 && chirp == false && stats == true) //5Hz
	{
		systick_counter_2 = 0;


		#if MATLAB == 0
		for(i=0; i<4; i++)	angle[i] = as5047U_getAngle(i);

		snprintf(string, 1500, 	"%s", clear_string);
		//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(0));
		//snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(1));
		snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(2));
		snprintf(string+strlen(string), 1500, "%s",  TMC4671_highLevel_getStatus(3));
		snprintf(string+strlen(string), 1500, "pwm_in:     %d %d %d %d\r\n",	pwm_in[0], pwm_in[1], pwm_in[2], pwm_in[3]);
		snprintf(string+strlen(string), 1500, "angleIn:    % 2.1f % 2.1f % 2.1f % 2.1f\n", angleIn[0], angleIn[1], angleIn[2], angleIn[3]);
		snprintf(string+strlen(string), 1500, "angleOut:   % 2.1f % 2.1f % 2.1f % 2.1f\n", angleOut[0], angleOut[1], angleOut[2], angleOut[3]);
		snprintf(string+strlen(string), 1500, "---------------------------\n");
		snprintf(string+strlen(string), 1500, "enc0: %5d\tenc1: %5d\tenc2: %5d\tenc3: %5d\n", angle[0], angle[1], angle[2], angle[3]);
		snprintf(string+strlen(string), 1500, "---------------------------\n");
		snprintf(string+strlen(string), 1500, "[o] ... stopped mode\n[p] ... position mode\n[SPACE] ... STOP\n\n");

		HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, strlen(string));
		#endif
	} // end of: if(systick_counter_2 >= 200) //5Hz

} // end of: void logic_loop(void)


void HAL_SYSTICK_Callback(void)
{
	systick_counter_3++;
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
