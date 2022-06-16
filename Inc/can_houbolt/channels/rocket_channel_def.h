#ifndef ROCKET_CHANNEL_DEF_H_
#define ROCKET_CHANNEL_DEF_H_

#include "cmds.h"

#define ROCKET_DATA_N_BYTES 2

typedef enum
{
	ROCKET_CURRENT_STATE,
	ROCKET_CHAMBER_PRESSURE_TARGET,
	ROCKET_REFRESH_DIVIDER
} ROCKET_VARIABLES;

typedef enum
{
	ROCKET_REQ_RESET_SETTINGS = COMMON_REQ_RESET_SETTINGS,	// NO payload
	ROCKET_RES_RESET_SETTINGS = COMMON_RES_RESET_SETTINGS,	// NO payload
	ROCKET_REQ_STATUS = COMMON_REQ_STATUS,					// NO payload
	ROCKET_RES_STATUS = COMMON_RES_STATUS,					// TODO: some status msg
	ROCKET_REQ_SET_VARIABLE = COMMON_REQ_SET_VARIABLE,		// SetMsg_t
	ROCKET_RES_SET_VARIABLE = COMMON_RES_SET_VARIABLE,		// SetMsg_t
	ROCKET_REQ_GET_VARIABLE = COMMON_REQ_GET_VARIABLE,		// GetMsg_t
	ROCKET_RES_GET_VARIABLE = COMMON_RES_GET_VARIABLE,		// SetMsg_t
	ROCKET_REQ_AUTO_SEQUENCE = COMMON_TOTAL_CMDS,			// NO payload
	ROCKET_RES_AUTO_SEQUENCE,
	ROCKET_REQ_ABORT,
	ROCKET_RES_ABORT,
	ROCKET_REQ_ENDOFFLIGHT,
	ROCKET_RES_ENDOFFLIGHT,

	ROCKET_TOTAL_CMDS
} ROCKET_CMDs;

typedef enum
{
	PAD_IDLE,
	AUTO_CHECK,
	IGNITION_SEQUENCE,
	HOLD_DOWN,
	POWERED_ASCENT,
	UNPOWERED_ASCENT,
	DEPRESS,
	ABORT
} ROCKET_STATE;

#endif