#include "tmc4671/TMC4671_highLevel.h"
#include "tmc4671/TMC4671.h"
#include "stm32h7xx_hal.h"
#include "swdriver.h"
#include "usart.h"
#include "as5147.h"

void TMC4671_highLevel_init(uint8_t drv)
{
	// Motor type &  PWM configuration
	tmc4671_writeInt(drv, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, (3 << TMC4671_MOTOR_TYPE_SHIFT) | (7 << TMC4671_N_POLE_PAIRS_SHIFT)); // BLDC, 7 pole pairs
	tmc4671_writeInt(drv, TMC4671_PWM_POLARITIES, 0); // LS and HS polarity off
	tmc4671_writeInt(drv, TMC4671_PWM_MAXCNT, 3999); // 3999 --> 25kHz PWM
	tmc4671_writeInt(drv, TMC4671_PWM_BBM_H_BBM_L, (10 << TMC4671_PWM_BBM_H_SHIFT) | (10 << TMC4671_PWM_BBM_L_SHIFT)); // LS and HS 100ns BBM
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, (0 << TMC4671_PWM_SV_SHIFT) | (7 << TMC4671_PWM_CHOP_SHIFT)); // Space Vector PWM disabled, centered PWM for FOC

	// ADC configuration
	tmc4671_writeInt(drv, TMC4671_ADC_I_SELECT, 0x24000100); // adcs for current measurement, default assignment (ux=0 v=1 wy=2)
//	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0); // internal ds-modulator, MCLK for both groups FIXME make this work
	tmc4671_writeInt(drv, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010); // internal ds-modulator, CLK (100MHz) for both groups FIXME bits might actually be inverted
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_A, (1 << 29)); // group a clock frequency 25MHz
	tmc4671_writeInt(drv, TMC4671_dsADC_MCLK_B, 0); // group b clock frequency 0 --> off
	tmc4671_writeInt(drv, TMC4671_dsADC_MDEC_B_MDEC_A, (1000 << TMC4671_DSADC_MDEC_B_SHIFT) | (1000 << TMC4671_DSADC_MDEC_A_SHIFT)); // decimation ratio FIXME adapt to clock
	tmc4671_writeInt(drv, TMC4671_ADC_I0_SCALE_OFFSET, (-490 << TMC4671_ADC_I0_SCALE_SHIFT) | (swdriver[drv].ofs_i0 << TMC4671_ADC_I0_OFFSET_SHIFT)); // offset, scale 2mA/lsb
	tmc4671_writeInt(drv, TMC4671_ADC_I1_SCALE_OFFSET, (-490 << TMC4671_ADC_I1_SCALE_SHIFT) | (swdriver[drv].ofs_i1 << TMC4671_ADC_I1_OFFSET_SHIFT)); // offset, scale 2mA/lsb

	// ABN encoder settings
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_MODE, 0); // standard polarity and count direction, don't clear at n pulse
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PPR, 2048); // decoder pulses per revolution
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_COUNT, 0); // decoder angle 0 FIXME: writing anything else doesn't work but writing current angle would allow for more elegant solution. the 2 lines below could be deleted, see git history
	uint16_t angle_current = as5147_getAngle(drv);  // current decoder angle
	swdriver[drv].ofs_enc_phim += (angle_current << 5);
	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, ((uint16_t)swdriver[drv].ofs_phim_phie << TMC4671_ABN_DECODER_PHI_E_OFFSET_SHIFT) | ((uint16_t)swdriver[drv].ofs_enc_phim << TMC4671_ABN_DECODER_PHI_M_OFFSET_SHIFT)); // TODO

	// Limits
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_LIMITS, 5000); // torque/flux limit 10A TODO: increase

	// PI settings
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_P_TORQUE_I, (379 << TMC4671_PID_TORQUE_P_SHIFT) | (3853 << TMC4671_PID_TORQUE_I_SHIFT)); // torque PI
	tmc4671_writeInt(drv, TMC4671_PID_FLUX_P_FLUX_I, (379 << TMC4671_PID_FLUX_P_SHIFT) | (3853 << TMC4671_PID_FLUX_I_SHIFT)); // flux PI
}


void TMC4671_highLevel_pwmOff(uint8_t drv)
{
	tmc4671_writeInt(drv, TMC4671_PWM_SV_CHOP, 0x00000000); // PWM off
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
	uint16_t angle = as5147_getAngle(drv);
	uint16_t len = snprintf(string, 64, "\n\n\ndriver %d encoder zero angle: %d (11bit) %d (16bit)\n", drv, angle, (int16_t)(angle << 5));
	HAL_UART_Transmit(&huart3, (uint8_t*)string, len, 100000000);

	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0
}


void TMC4671_highLevel_torqueTest(uint8_t drv)
{
//	// Init encoder (mode 0)
//	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 8); // uq_ud_ext
//	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
//	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 1);  // phi_e_ext
//	tmc4671_writeInt(drv, TMC4671_PHI_E_EXT, 0);
//	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (2000 << TMC4671_UD_EXT_SHIFT)); // uq=0, ud=2000
//	HAL_Delay(1000);
//	tmc4671_writeInt(drv, TMC4671_ABN_DECODER_COUNT, 0);
//	tmc4671_writeInt(drv, TMC4671_UQ_UD_EXT, (0 << TMC4671_UQ_EXT_SHIFT) | (0 << TMC4671_UD_EXT_SHIFT)); // ud=0 uq=0

	// Feedback selection
	tmc4671_writeInt(drv, TMC4671_PHI_E_SELECTION, 3); // phi_e_abn
	tmc4671_writeInt(drv, TMC4671_VELOCITY_SELECTION, 9); // phi_m_abn

	// Switch to torque mode
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (0 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target 0
	tmc4671_writeInt(drv, TMC4671_MODE_RAMP_MODE_MOTION, 1); // torque_mode

	// Rotate right
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (1000 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target 1000 (2A)
	HAL_Delay(3000);

	// Rotate left
	tmc4671_writeInt(drv, TMC4671_PID_TORQUE_FLUX_TARGET, (-1000 << TMC4671_PID_TORQUE_TARGET_SHIFT)); // torque target -1000 (-2A)
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
