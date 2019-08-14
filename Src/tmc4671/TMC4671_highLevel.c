#include "tmc4671/TMC4671_highLevel.h"
#include "tmc4671/TMC4671.h"
#include "stm32h7xx_hal.h"


void TMC4671_highLevel_openLoopTest(uint8_t drv)
{
	// Motor type &  PWM configuration
	tmc4671_writeInt(drv, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030006); //BLDC, 6 pole pairs
	tmc4671_writeInt(drv, TMC4671_PWM_POLARITIES, 0x00000000); //LS and HS polarity off
	tmc4671_writeInt(drv, TMC4671_PWM_MAXCNT, 0x000003E7); //999 --> 100kHz PWM
	tmc4671_writeInt(drv, TMC4671_PWM_BBM_H_BBM_L, 0x00001919); //LS and HS 250ns BBM
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, 0x00000007); //centered PWM for FOC, Space Vector PWM disabled

	// ADC configuration
	tmc4671_writeInt(drv, TMC4671_ADC_I_SELECT, 0x24000100); //adcs for current measurement, default assignment (ux=0 v=1 wy=2)
	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010); //CLK (100 MHz) is used
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_A, 0x20000000); //clock frequency
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_B, 0x00000000); //clock frequency 0 --> adc group b off
	tmc4671_writeInt(drv, TMC4671_dsADC_MDEC_B_MDEC_A, 0x00540054); //?
	tmc4671_writeInt(drv, TMC4671_ADC_I0_SCALE_OFFSET, 0xFC40881D); //offset and scale
	tmc4671_writeInt(drv, TMC4671_ADC_I1_SCALE_OFFSET, 0xFC40841A); //offset and scale

	// Open loop settings
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_MODE, 0x00000000); //openloop phi positive
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_ACCELERATION, 60); //openloop phi acceleration
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0); //velocity target 0

	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 0x00000002); //phi e openloop
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, 0x000008FF); //ud=8FF uq=0

	// ===== Open loop test drive =====

	// Switch to open loop velocity mode
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008); // uq_ud_ext

	// Rotate right
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 60); //velocity target 60
	HAL_Delay(5000);

	// Rotate left
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, -60); //velocity target -60
	HAL_Delay(5000);

	// Stop
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000); //velocity target 0
	HAL_Delay(1000);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, 0x00000000); //ud=0 uq=0
	HAL_Delay(100);
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, 0x00000000); //PWM off
}
