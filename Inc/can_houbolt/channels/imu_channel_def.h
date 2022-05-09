#ifndef IMU_CHANNEL_DEF_H_
#define IMU_CHANNEL_DEF_H_

#include "cmds.h"

#define IMU_DATA_N_BYTES 12

typedef enum
{
	IMU_MEASUREMENT,
	IMU_REFRESH_DIVIDER
} IMU_VARIABLES;

typedef enum
{
	IMU_REQ_RESET_SETTINGS = COMMON_REQ_RESET_SETTINGS,	// NO payload
	IMU_RES_RESET_SETTINGS = COMMON_RES_RESET_SETTINGS,	// NO payload
	IMU_REQ_STATUS = COMMON_REQ_STATUS,					// NO payload
	IMU_RES_STATUS = COMMON_RES_STATUS,					// TODO: some status msg
	IMU_REQ_SET_VARIABLE = COMMON_REQ_SET_VARIABLE,		// SetMsg_t
	IMU_RES_SET_VARIABLE = COMMON_RES_SET_VARIABLE,		// SetMsg_t
	IMU_REQ_GET_VARIABLE = COMMON_REQ_GET_VARIABLE,		// GetMsg_t
	IMU_RES_GET_VARIABLE = COMMON_RES_GET_VARIABLE,		// SetMsg_t
	//COMMON_TOTAL_CMDS,				// NO payload

	IMU_TOTAL_CMDS
} IMU_CMDs;

typedef struct __attribute__((__packed__))
{
	uint16_t x_accel;	// x-axis linear acceleration
	uint16_t y_accel;	// y-axis linear acceleration
	uint16_t z_accel;	// z-axis linear acceleration
	uint16_t x_alpha;	// x-axis angular acceleration
	uint16_t y_alpha;	// y-axis angular acceleration
	uint16_t z_alpha;	// z-axis angular acceleration
} IMUMsg_t;

#endif
