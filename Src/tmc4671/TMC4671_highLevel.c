#include "tmc4671/TMC4671_highLevel.h"


//TODO: use functions from TMC4671.c, use masking
void TMC4671_highLevel_init(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // stopped_mode

	// Motor type &  PWM configuration
	tmc4671_writeInt(drv, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (3 << TMC4671_MOTOR_TYPE_SHIFT) | (7 << TMC4671_N_POLE_PAIRS_SHIFT)); // BLDC, 7 pole pairs
	tmc4671_writeInt(drv, TMC4671_PWM_POLARITIES, 0); // LS and HS polarity off
	tmc4671_writeInt(drv, TMC4671_PWM_MAXCNT, 3999); // 3999 --> 25kHz PWM
	// tmc4671_writeInt(drv, TMC4671_PWM_MAXCNT,  999); //  999 --> 100kHz PWM
	tmc4671_writeInt(drv, TMC4671_PWM_BBM_H_BBM_L, (25 << TMC4671_PWM_BBM_H_SHIFT) | (25 << TMC4671_PWM_BBM_L_SHIFT)); // LS and HS 100ns BBM
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, (0 << TMC4671_PWM_SV_SHIFT) | (7 << TMC4671_PWM_CHOP_SHIFT)); // Space Vector PWM disabled, centered PWM for FOC

	// ADC configuration
	tmc4671_writeInt(drv, TMC4671_ADC_I_SELECT, 0x24000100); // adcs for current measurement, default assignment (ux=0 v=1 wy=2)
//	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0); // internal ds-modulator, MCLK for both groups FIXME make this work
	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010); // internal ds-modulator, CLK (100MHz) for both groups FIXME bits might actually be inverted
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_A, (1 << 29)); // group a clock frequency 25MHz
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_B, 0); // group b clock frequency 0 --> off
	tmc4671_writeInt(drv, TMC4671_dsADC_MDEC_B_MDEC_A, (1000 << TMC4671_DSADC_MDEC_B_SHIFT) | (1000 << TMC4671_DSADC_MDEC_A_SHIFT)); // decimation ratio FIXME adapt to clock

	uint8_t i;
	uint32_t adcOffs0 = 0;
	uint32_t adcOffs1 = 0;
	for(i=0; i<32; i++) {adcOffs0 += TMC4671_getAdcRaw0(drv); HAL_Delay(1);}
	for(i=0; i<32; i++) {adcOffs1 += TMC4671_getAdcRaw1(drv); HAL_Delay(1);}
	adcOffs0 /= 32;
	adcOffs1 /= 32;

	tmc4671_writeInt(drv, TMC4671_ADC_I0_SCALE_OFFSET, (-490 << TMC4671_ADC_I0_SCALE_SHIFT) | (adcOffs0 << TMC4671_ADC_I0_OFFSET_SHIFT)); // offset, scale 2mA/lsb
	tmc4671_writeInt(drv, TMC4671_ADC_I1_SCALE_OFFSET, (-490 << TMC4671_ADC_I1_SCALE_SHIFT) | (adcOffs1 << TMC4671_ADC_I1_OFFSET_SHIFT)); // offset, scale 2mA/lsb

	// ABN encoder settings
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_MODE, 0); // standard polarity and count direction, don't clear at n pulse
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PPR, 16384); // decoder pulses per revolution
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_COUNT, 0); // decoder angle 0 FIXME: writing anything else doesn't work but writing current angle would allow for more elegant solution. 3 lines below could be deleted, see git history
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_ACTUAL, 0);

	//funtktioniert nicht
	// tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0);
	//tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M, 0);
	tmc4671_writeRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, BIT_16_TO_31, 0);
	tmc4671_writeRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, BIT_0_TO_15, 0);
	tmc4671_writeRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31, 0);
	tmc4671_writeRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15, 0);


	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 3); // phi_e_abn
	tmc4671_writeInt(drv, TMC4671_VELOCITY_SELECTION, (9 << TMC4671_VELOCITY_SELECTION_SHIFT) | (1 << TMC4671_VELOCITY_METER_SELECTION_SHIFT)); // phi_m_abn, advanced velocity meter (time difference measurement)
	tmc4671_writeInt(drv, TMC4671_POSITION_SELECTION, 9); // phi_m_abn

	// Limits
	tmc4671_writeInt(drv, TMC4671_PIDOUT_UQ_UD_LIMITS, 23169); // UQ/UD limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 32767); // torque/flux ddt limit 10A TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, 5000); // torque/flux limit 10A TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_ACCELERATION_LIMIT, 1000000); // acceleration limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_LIMIT, 2000); // velocity limit TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_LIMIT_LOW, -16383); // position lower limit, -90°  TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_LIMIT_HIGH, 16383); // position upper limit, 90°  TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_LIMIT_LOW, -65536); // position lower limit, -90°  TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_LIMIT_HIGH, 65536); // position upper limit, 90°  TODO optimize


	// PI settings
	tmc4671_writeInt(drv, TMC4671_PID_FLUX_P_FLUX_I, (100 << TMC4671_PID_FLUX_P_SHIFT) | (2600 << TMC4671_PID_FLUX_I_SHIFT)); // flux PI TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_P_TORQUE_I, (140 << TMC4671_PID_TORQUE_P_SHIFT) | (2900 << TMC4671_PID_TORQUE_I_SHIFT)); // torque PI TODO optimize
	// tmc4671_writeInt(drv, TMC4671_PID_TORQUE_P_TORQUE_I, (100 << TMC4671_PID_TORQUE_P_SHIFT) | (2600 << TMC4671_PID_TORQUE_I_SHIFT));
	tmc4671_writeInt(drv, TMC4671_PID_VELOCITY_P_VELOCITY_I, (8000 << TMC4671_PID_VELOCITY_P_SHIFT) | (500 << TMC4671_PID_VELOCITY_I_SHIFT)); // velocity PI TODO optimize
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_P_POSITION_I, (600 << TMC4671_PID_POSITION_P_SHIFT) | (0 << TMC4671_PID_POSITION_I_SHIFT)); // velocity PI TODO optimize


	// Actual Velocity Biquad settings (lowpass 2nd order, f=400, d=1.0)
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 9); // biquad_v_a_1
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 970963714);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 10); // biquad_v_a_2
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, -439011740);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 12); // biquad_v_b_0
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1229735);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 13); // biquad_v_b_1
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 2459469);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 14); // biquad_v_b_2
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1229735);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 15); // biquad_v_enable
	tmc4671_writeInt(drv, TMC4671_CONFIG_DATA, 1);
	tmc4671_writeInt(drv, TMC4671_CONFIG_ADDR, 0); //none

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


void TMC4671_highLevel_pwmOff(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, 0x00000000); // PWM off
}

void TMC4671_highLevel_pwmOn(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, (0 << TMC4671_PWM_SV_SHIFT) | (7 << TMC4671_PWM_CHOP_SHIFT)); // Space Vector PWM disabled, centered PWM for FOC
}

void TMC4671_highLevel_stoppedMode(uint8_t drv)
{
	tmc4671_switchToMotionMode(drv, TMC4671_MOTION_MODE_STOPPED); 		// tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0); // stopped_mode
}

void TMC4671_highLevel_torqueMode(uint8_t drv)
{
	tmc4671_switchToMotionMode(drv, TMC4671_MOTION_MODE_TORQUE);
}


void TMC4671_highLevel_initEncoder(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8);
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0);
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 1);
	tmc4671_writeInt(drv, TMC4671_PHI_E_EXT, 0);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (2500 << TMC4671_UD_EXT_SHIFT));
	HAL_Delay(2000);
	uint16_t angle = as5047U_getAngle(drv);

	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_COUNT, 0);
	HAL_Delay(100);
	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 0);

	int32_t position =  (int32_t)( (int16_t)( angle - swdriver[drv].ofs_pos0)  );
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 3); // phi_e_abn
	tmc4671_writeInt(drv, TMC4671_PID_POSITION_ACTUAL, position );


	// static char string[128];
	// uint16_t len = snprintf(string, 128, "-->driver [%d]\nangle:            %d\npos_soll          %d\nangle - pos_soll  %d\nposition:  %ld\n", drv, angle,swdriver[drv].ofs_pos0, (int16_t)( angle - swdriver[drv].ofs_pos0), position );
	// HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, len);
}





void TMC4671_highLevel_setCurrentLimit(uint8_t drv, uint16_t torque_flux_limit)
{
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, torque_flux_limit);
}

void TMC4671_highLevel_setTorqueTarget(uint8_t drv, int16_t targetTorque)
{
	tmc4671_setTargetTorque_raw(drv, targetTorque);
}


void TMC4671_highLevel_setTorqueTargetA(uint8_t drv, float targetTorque)
{
	tmc4671_setTargetTorque_raw(drv, (int32_t)(500.0*targetTorque));
}


void TMC4671_highLevel_setFluxTarget(uint8_t drv, int16_t targetFlux)
{
	tmc4671_setTargetFlux_raw(drv, targetFlux);
}



int16_t TMC4671_highLevel_getTorqueActual(uint8_t drv)
{
	//return tmc4671_readRegister16BitValue(drv,TMC4671_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
	return tmc4671_getActualTorque_raw(drv);
}

int16_t TMC4671_highLevel_getTorqueTarget(uint8_t drv)
{
	return tmc4671_getTargetTorque_raw(drv);
}

int16_t TMC4671_highLevel_getPhiM(uint8_t drv)
{
	return tmc4671_readRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15);
}

int16_t TMC4671_highLevel_getPhiMOffset(uint8_t drv)
{
	return tmc4671_readRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, BIT_0_TO_15);
}


int16_t TMC4671_highLevel_getPhiE(uint8_t drv)
{
	return tmc4671_readRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31);
}

int16_t TMC4671_highLevel_getPhiEOffset(uint8_t drv)
{
	return tmc4671_readRegister16BitValue(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, BIT_16_TO_31);
}


int32_t TMC4671_highLevel_getVelocityActual(uint8_t drv)
{
	return tmc4671_getActualVelocity(drv);
}

int32_t TMC4671_highLevel_getPositionActual(uint8_t drv)
{
	return tmc4671_getActualPosition(drv);
}

float TMC4671_highLevel_getPositionActualRad(uint8_t drv)
{
	return (float)TMC4671_highLevel_getPositionActual(drv)/65536.0*2*M_PI;
}



char* TMC4671_highLevel_getStatus(uint8_t drv)
{
	static char string[4][500];
	uint16_t torque_flux_limit = tmc4671_readRegister16BitValue(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, BIT_0_TO_15);

	snprintf(&string[drv][0], 500, 	"Drive %d\n", drv);
	snprintf(&string[drv][0]+strlen(&string[drv][0]), 500-strlen(&string[drv][0]), "Torque-Limit    [c]: %d\n",  torque_flux_limit);
	snprintf(&string[drv][0]+strlen(&string[drv][0]), 500-strlen(&string[drv][0]), "Encoder            : %d\n",   as5047U_getAngle(drv));
	snprintf(&string[drv][0]+strlen(&string[drv][0]), 500-strlen(&string[drv][0]), "Position           : %ld\n",  TMC4671_highLevel_getPositionActual(drv));
	snprintf(&string[drv][0]+strlen(&string[drv][0]), 500-strlen(&string[drv][0]), "Velocity           : %ld\n",  TMC4671_highLevel_getVelocityActual(drv));
	snprintf(&string[drv][0]+strlen(&string[drv][0]), 500-strlen(&string[drv][0]), "---------------------------\n");

	return &string[drv][0];
}


// void TMC4671_highLevel_positionMode_rampToZero(uint8_t drv)
// {
// 	uint8_t i;
// 	int32_t position = tmc4671_readInt(drv, TMC4671_PID_POSITION_ACTUAL);
//
// 	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 3);
//
// 	for(i=100; i> 0; i--)
// 	{
// 		//TMC4671_highLevel_setPosition(drv, position);
// 		tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, (int32_t)((float)position*(float)i/100.0) );
// 		HAL_Delay(10);
// 	}
// 	tmc4671_writeInt(drv, TMC4671_PID_POSITION_TARGET, 0);
// }
