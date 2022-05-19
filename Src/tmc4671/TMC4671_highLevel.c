#include "tmc4671/TMC4671_highLevel.h"
#include "tmc4671/TMC4671.h"
#include "stm32h7xx_hal.h"
#include "swdriver.h"
#include "usart.h"
#include "as5047U.h"
#include <stdio.h>

//TODO: use functions from TMC4671.c, use masking
void TMC4671_highLevel_init(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // stopped_mode

	// Motor type &  PWM configuration
	tmc4671_writeInt(drv, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (3 << TMC4671_MOTOR_TYPE_SHIFT) | (POLE_PAIRS << TMC4671_N_POLE_PAIRS_SHIFT)); // BLDC, pole pairs
	tmc4671_writeInt(drv, TMC4671_PWM_POLARITIES, 0); // LS and HS polarity off
	tmc4671_writeInt(drv, TMC4671_PWM_MAXCNT, 3999); // 3999 --> 25kHz PWM
	tmc4671_writeInt(drv, TMC4671_PWM_BBM_H_BBM_L, (30 << TMC4671_PWM_BBM_H_SHIFT) | (30 << TMC4671_PWM_BBM_L_SHIFT)); // LS and HS 300ns BBM
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, (0 << TMC4671_PWM_SV_SHIFT) | (7 << TMC4671_PWM_CHOP_SHIFT)); // Space Vector PWM disabled, centered PWM for FOC

	// ADC configuration
	tmc4671_writeInt(drv, TMC4671_ADC_I_SELECT, 0x24000100); // adcs for current measurement, default assignment (ux=0 v=1 wy=2)
//	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0); // internal ds-modulator, MCLK for both groups FIXME make this work
	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010); // internal ds-modulator, CLK (100MHz) for both groups FIXME bits might actually be inverted
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_A, (1 << 29)); // group a clock frequency 25MHz
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_B, 0); // group b clock frequency 0 --> off
	tmc4671_writeInt(drv, TMC4671_dsADC_MDEC_B_MDEC_A, (1000 << TMC4671_DSADC_MDEC_B_SHIFT) | (1000 << TMC4671_DSADC_MDEC_A_SHIFT)); // decimation ratio FIXME adapt to clock

	HAL_Delay(100);

	uint16_t i;
	uint32_t adcOffs0 = 0;
	uint32_t adcOffs1 = 0;
	for(i=0; i<256; i++)
	{
		adcOffs0 += TMC4671_getAdcRaw0(drv);
		adcOffs1 += TMC4671_getAdcRaw1(drv);
		HAL_Delay(1);
	}
	adcOffs0 /= 256;
	adcOffs1 /= 256;

	tmc4671_writeInt(drv, TMC4671_ADC_I0_SCALE_OFFSET, (-490 << TMC4671_ADC_I0_SCALE_SHIFT) | (adcOffs0 << TMC4671_ADC_I0_OFFSET_SHIFT)); // offset, scale 2mA/lsb
	tmc4671_writeInt(drv, TMC4671_ADC_I1_SCALE_OFFSET, (-490 << TMC4671_ADC_I1_SCALE_SHIFT) | (adcOffs1 << TMC4671_ADC_I1_OFFSET_SHIFT)); // offset, scale 2mA/lsb

	// ABN encoder settings
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_MODE, 0); // standard polarity and count direction, don't clear at n pulse
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PPR, 2048); // decoder pulses per revolution

	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 3); // phi_e_abn
	tmc4671_writeInt(drv, TMC4671_VELOCITY_SELECTION, (9 << TMC4671_VELOCITY_SELECTION_SHIFT) | (1 << TMC4671_VELOCITY_METER_SELECTION_SHIFT)); // phi_m_abn, advanced velocity meter (time difference measurement)
	tmc4671_writeInt(drv, TMC4671_POSITION_SELECTION, 9); // phi_m_abn

	// Limits
	tmc4671_writeInt(drv, TMC4671_PIDOUT_UQ_UD_LIMITS, 23169); // UQ/UD limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 32767); // torque/flux ddt limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, 20000); // torque/flux limit 40A TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_ACCELERATION_LIMIT, 10000); // acceleration limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_LIMIT, 200); // velocity limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_LIMIT_LOW, -65535 * 5); // position lower limit, -5 turns * 65535 TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_LIMIT_HIGH, 65535 * 5); // position upper limit, 5 turns * 65535 TODO optimize
	// PI settings
	tmc4671_writeInt(drv, TMC4671_PID_FLUX_P_FLUX_I, (100 << TMC4671_PID_FLUX_P_SHIFT) | (800 << TMC4671_PID_FLUX_I_SHIFT)); // flux PI TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_P_TORQUE_I, (100 << TMC4671_PID_TORQUE_P_SHIFT) | (800 << TMC4671_PID_TORQUE_I_SHIFT)); // torque PI TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_P_VELOCITY_I, (4000 << TMC4671_PID_VELOCITY_P_SHIFT) | (8000 << TMC4671_PID_VELOCITY_I_SHIFT)); // velocity PI TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_P_POSITION_I, (300 << TMC4671_PID_POSITION_P_SHIFT) | (0 << TMC4671_PID_POSITION_I_SHIFT)); // position PI TODO optimize

	// Actual Velocity Biquad settings (lowpass 2nd order, f=200, d=1.0)
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 9); // biquad_v_a_1
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1021092885);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 10); // biquad_v_a_2
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, -485512745);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 12); // biquad_v_b_0
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 322693);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 13); // biquad_v_b_1
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 645386);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 14); // biquad_v_b_2
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 322693);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 15); // biquad_v_enable
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 0); //none

	// Target Position Biquad settings (lowpass 2nd order, f=100, d=1.0)
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 1); // biquad_x_a_1
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1047090657);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 2); // biquad_x_a_2
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, -510550497);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 4); // biquad_x_b_0
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 82688);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 5); // biquad_x_b_1
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 165376);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 6); // biquad_x_b_2
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 82688);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 7); // biquad_x_enable
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 0); //none
}


void TMC4671_highLevel_pwmOff(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, 0x00000000); // PWM off
}

void TMC4671_highLevel_stoppedMode(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // stopped_mode
}


void TMC4671_highLevel_positionMode(uint8_t drv) 	// Switch to position mode
{
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 0); // target position 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3); // position_mode
}

void TMC4671_highLevel_positionMode2(uint8_t drv)
{
	// Switch to position mode without setting target position to 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3); // position_mode
}


void TMC4671_highLevel_setPosition(uint8_t drv, int32_t position)
{
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, position); // position target
}


void TMC4671_highLevel_setPosition_nonBlocking(uint8_t drv, int32_t position)
{
	tmc4671_writeInt_nonBlocking(drv, TMC4671_PID_POSITION_TARGET, position); // position target
}


void TMC4671_highLevel_printOffsetAngle(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 1);  // phi_e_ext
	tmc4671_writeInt(drv, TMC4671_PHI_E_EXT, 0);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (2000 << TMC4671_UD_EXT_SHIFT)); // uq=0, ud=2000
	HAL_Delay(1000);

	char string[64];
	// uint16_t angle = as5147_getAngle(drv); // will be printed as int16!!!
	// uint16_t len = snprintf(string, 64, "\n\rdriver %d encoder zero angle: %d (11bit) %d (16bit)\n\r", drv, angle, (int16_t)(angle << 5));
	uint16_t angle = as5047U_getAngle(drv); // will be printed as int16!!!
	uint16_t len = snprintf(string, 64, "\n\rdriver %d encoder zero angle: %d (16bit)\n\r", drv, angle);
	HAL_UART_Transmit(&huart3, (uint8_t*)string, len, 100000000);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0
}


uint16_t TMC4671_getAdcRaw0(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_ADC_RAW_ADDR, 0);
	return tmc4671_readInt(drv, TMC4671_ADC_RAW_DATA) & 0xFFFF;
}

uint16_t TMC4671_getAdcRaw1(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_ADC_RAW_ADDR, 0);
	return (tmc4671_readInt(drv, TMC4671_ADC_RAW_DATA) >> 16) & 0xFFFF;
}


void TMC4671_highLevel_initEncoder(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 1);  // phi_e_ext
	tmc4671_writeInt(drv, TMC4671_PHI_E_EXT, 0);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (5000 << TMC4671_UD_EXT_SHIFT)); // uq=0, ud=5000
	HAL_Delay(500);
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_COUNT, 0);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // off
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 3); // phi_e_abn
}

void TMC4671_highLevel_initEncoder_new(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8);
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0);
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 1);
	tmc4671_writeInt(drv, TMC4671_PHI_E_EXT, 0);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (2500 << TMC4671_UD_EXT_SHIFT));
	HAL_Delay(2000);
	uint16_t angle = as5047U_getAngle(drv);
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_COUNT, 0);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0);

	int32_t position =  (int32_t)( (int16_t)( (angle) - swdriver[drv].ofs_pos0)  );
	// static char string[128];
	// uint16_t len = snprintf(string, 128, "-->driver %d encoder angle: %d (11bit) %d (16bit) %ld\n\r", drv, angle, (angle << 5), position );
	// HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, len);
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 3); // phi_e_abn
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_ACTUAL, position );
}

void TMC4671_highLevel_positionMode_fluxTorqueRamp(uint8_t drv) // TODO read actual position before torque ramp, ramp position from actual to 0 afterwards
{
	uint16_t torque_flux_limit = 5000;
	uint16_t torque_flux = 0;
	uint8_t i;

	torque_flux_limit = tmc4671_readRegister16BitValue(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, BIT_0_TO_15);

	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, 0);
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3); // position_mode

	for(i=0; i<100; i++)
	{
		torque_flux = (uint16_t)( (float)i/100.0*(float)torque_flux_limit );
		tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, torque_flux);
		HAL_Delay(1);
	}
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, torque_flux_limit);
}

void TMC4671_highLevel_positionMode_rampToZero(uint8_t drv)
{
	uint8_t i;
	int32_t position = tmc4671_readInt(drv, TMC4671_PID_POSITION_ACTUAL);

	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3);

	for(i=100; i> 0; i--)
	{
		//TMC4671_highLevel_setPosition(drv, position);
		tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, (int32_t)((float)position*(float)i/100.0) );
		HAL_Delay(10);
	}
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 0);
}

void TMC4671_highLevel_setPositionFilter(uint8_t drv, bool status)
{
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 7);
	//bool enabled = (bool)tmc4671_readInt(drv, TMC4671_CONFIG_DATA);
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, status);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 0);
}

void TMC4671_highLevel_setCurrentLimit(uint8_t drv, uint16_t torque_flux_limit)
{
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, torque_flux_limit);
}

void TMC4671_highLevel_setIntegralPosition(uint8_t drv, uint16_t integral)
{
	tmc4671_writeRegister16BitValue(drv, TMC4671_PID_POSITION_P_POSITION_I, BIT_0_TO_15, integral);
}

int32_t TMC4671_highLevel_getPositionTarget(uint8_t drv)
{
	return tmc4671_readInt(drv, TMC4671_PID_POSITION_TARGET);
}

int32_t TMC4671_highLevel_getPositionActual(uint8_t drv)
{
	return tmc4671_readInt(drv, TMC4671_PID_POSITION_ACTUAL);
}

int16_t TMC4671_highLevel_getTorqueActual(uint8_t drv)
{
	return tmc4671_readRegister16BitValue(drv,TMC4671_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
}

int16_t TMC4671_highLevel_getVelocityActual(uint8_t drv)
{
	// tmc4671_writeInt(drv, TMC4671_INTERIM_ADDR, 24);
	// INTERIM_ADDR
	//  INTERIM_DATA
	// return tmc4671_readRegister16BitValue(drv,TMC4671_PID_VELOCITY_ACTUAL_LSB, BIT_0_TO_15);
	return (int16_t) tmc4671_readInt(drv, TMC4671_PID_VELOCITY_ACTUAL);
}

int16_t TMC4671_highLevel_getTorqueTarget(uint8_t drv)
{
	// INTERIM_ADDR = 0 => PIDIN_TARGET_TORQUE
 tmc4671_writeInt(drv, TMC4671_INTERIM_ADDR, 0);
 return (int16_t)tmc4671_readInt(drv, TMC4671_INTERIM_DATA);
}

int16_t TMC4671_highLevel_getVelocityTarget(uint8_t drv)
{
	// INTERIM_ADDR = 2 => PIDIN_TARGET_VELOCITY
 tmc4671_writeInt(drv, TMC4671_INTERIM_ADDR, 2);
 return (int16_t)tmc4671_readInt(drv, TMC4671_INTERIM_DATA);
}

char* TMC4671_highLevel_getStatus(uint8_t drv)
{
	static char string[4][200];
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 7);
	bool enabled = (bool)tmc4671_readInt(drv, TMC4671_CONFIG_DATA);
	uint16_t torque_flux_limit = tmc4671_readRegister16BitValue(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, BIT_0_TO_15);
	uint16_t position_i = tmc4671_readRegister16BitValue(drv, TMC4671_PID_POSITION_P_POSITION_I, BIT_0_TO_15);
	snprintf(&string[drv][0], 200,
		"Drive %d\n\r"
		"Position-Filter [f]: %s\r\n"
		"Torque-Limit    [c]: %d\r\n"
		"Position-I:     [i]: %d\r\n"
		"I0:                  %d\n\r"
		"I1:                  %d\n\r"
		"---------------------------\n\r",
			drv, enabled?"on":"off", torque_flux_limit, position_i, tmc4671_getAdcI0Offset(drv),  tmc4671_getAdcI1Offset(drv));

	return &string[drv][0];
}





void TMC4671_highLevel_positionTest(uint8_t drv)
{
	// Switch to position mode
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 0); // position target 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3); // position_mode

	// move
	uint8_t i;
	for(i=0; i<4; i++)
	{
		tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 65535); // position target 65535
		HAL_Delay(2000);
		tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 0); // position target 0
		HAL_Delay(2000);
	}

	// Stop
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // stopped_mode
}

void TMC4671_highLevel_velocityTest(uint8_t drv)
{
	// Switch to velocity mode
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_TARGET, (0 << TMC4671_PID_VELOCITY_TARGET_SHIFT)); // velocity target 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 2); // velocity_mode

	// Rotate right
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_TARGET, (60 << TMC4671_PID_VELOCITY_TARGET_SHIFT)); // velocity target
	HAL_Delay(5000);

	// Rotate left
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_TARGET, (-60 << TMC4671_PID_VELOCITY_TARGET_SHIFT)); // velocity target
	HAL_Delay(5000);

	// Stop
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_TARGET, (0 << TMC4671_PID_VELOCITY_TARGET_SHIFT)); // velocity target 0
}

void TMC4671_highLevel_torqueTest(uint8_t drv)
{
	// Switch to torque mode
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (0 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 1); // torque_mode

	// Rotate right
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (200 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target 200 (400mA)
	HAL_Delay(3000);

	// Rotate left
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (-200 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target -200 (-400mA)
	HAL_Delay(3000);

	// Stop
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (0 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target 0
}


void TMC4671_highLevel_openLoopTest(uint8_t drv)
{
	// Open loop settings
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_MODE, 0x00000000); // openloop phi positive
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_ACCELERATION, 60); // openloop phi acceleration
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0); // velocity target 0

	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 2); // phi_e_openloop
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (2000 << TMC4671_UD_EXT_SHIFT)); // uq=0, ud=2000

	// ===== Open loop test drive =====

	// Switch to open loop velocity mode
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext

	// Rotate right
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 1); // velocity target 1
	HAL_Delay(20000);

	// Rotate left
	//tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, -1); // velocity target -1
	//HAL_Delay(5000);

	// Stop
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0); // velocity target 0
	HAL_Delay(2000);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0
}


void TMC4671_highLevel_openLoopTest2(uint8_t drv) // to verify correct encoder initialisation
{
	// Open loop settings
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_MODE, 0x00000000); // openloop phi positive
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_ACCELERATION, 60); // openloop phi acceleration
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0); // velocity target 0

	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 2); // phi_e_openloop
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (3000 << TMC4671_UD_EXT_SHIFT)); // uq=0, ud=3000

	// Switch to open loop velocity mode
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 10); // rotate right, velocity target 1
}


void TMC4671_highLevel_openLoopTest3(uint8_t drv) // low duty cycle operation for high current without overheating
{
	// Open loop settings
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_MODE, 0x00000000); // openloop phi positive
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_ACCELERATION, 60); // openloop phi acceleration
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0); // velocity target 0

	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 2); // phi_e_openloop
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (5000 << TMC4671_UD_EXT_SHIFT)); // uq=0, ud=5000 --> ~40-50A peak phase @ 20Vin

	// Switch to open loop velocity mode
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext
	tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 1); // rotate right, velocity target 1

	while(1)
	{
		HAL_Delay(10000);
		tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // stopped
		tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 0); // velocity target 0
		HAL_Delay(50000);
		tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext
		tmc4671_writeInt(drv, TMC4671_OPENLOOP_VELOCITY_TARGET, 1); // rotate right, velocity target 1
	}
}


void TMC4671_highLevel_referenceEndStop(uint8_t drv)
{
	int32_t torque = 5000; // 10A

	uint32_t torqueOld = tmc4671_readInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS);

	//lower torque limit
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, torque); // torque/flux limit

	// Switch to velocity mode
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_TARGET, (0 << TMC4671_PID_VELOCITY_TARGET_SHIFT)); // velocity target 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 2); // velocity_mode

	// drive into end stop
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_TARGET, (-100 << TMC4671_PID_VELOCITY_TARGET_SHIFT)); // velocity target

	HAL_Delay(200);

	//wait until stopped
	uint8_t torqueHighCount = 0;
	while(torqueHighCount < 100)
	{
		if(abs(TMC4671_highLevel_getTorqueActual(drv)) > torque * 0.8) torqueHighCount++;
		else torqueHighCount = 0;
		HAL_Delay(1);
	}

	tmc4671_writeInt(drv, TMC4671_PID_POSITION_ACTUAL, -16384); // -1/4 rotation

	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); //stopped_mode

	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, torqueOld); // reset limit

	tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 0);

	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3); // position_mode
}
