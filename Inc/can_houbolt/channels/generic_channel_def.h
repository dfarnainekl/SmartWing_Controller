#ifndef GENERIC_CMDS_H__
#define GENERIC_CMDS_H__

#include "cmds.h"

#define GENERIC_CHANNEL_ID 0x3F

#define MAX_CHANNELS 32

typedef enum
{
	EQUALS = 0,
	LESS_THAN,
	GREATER_THAN
} Operation_t;

typedef enum
{
	GENERIC_BUS1_VOLTAGE,
	GENERIC_BUS2_VOLTAGE,
	GENERIC_PWR_VOLTAGE,
	GENERIC_PWR_CURRENT,
	GENERIC_UART_ENABLED,
	GENERIC_REFRESH_DIVIDER,
	GENERIC_REFRESH_RATE
} GENERIC_VARIABLES;

typedef enum
{
	GENERIC_REQ_RESET_ALL_SETTINGS = COMMON_REQ_RESET_SETTINGS,	// NO payload
	GENERIC_RES_RESET_ALL_SETTINGS = COMMON_RES_RESET_SETTINGS,	// NO payload
	GENERIC_REQ_STATUS = COMMON_REQ_STATUS,					// NO payload
	GENERIC_RES_STATUS = COMMON_RES_STATUS,					// TODO: some status msg
	GENERIC_REQ_SET_VARIABLE = COMMON_REQ_SET_VARIABLE,		// SetMsg_t
	GENERIC_RES_SET_VARIABLE = COMMON_RES_SET_VARIABLE,		// SetMsg_t
	GENERIC_REQ_GET_VARIABLE = COMMON_REQ_GET_VARIABLE,		// GetMsg_t
	GENERIC_RES_GET_VARIABLE = COMMON_RES_GET_VARIABLE,		// SetMsg_t
	GENERIC_REQ_SYNC_CLOCK = COMMON_TOTAL_CMDS,				// NO FUCKING IDEA
	GENERIC_RES_SYNC_CLOCK,									// NO FUCKING IDEA
	GENERIC_REQ_DATA,										// NO payload
	GENERIC_RES_DATA,										// DataMsg_t
	GENERIC_REQ_NODE_INFO,									// NO payload
	GENERIC_RES_NODE_INFO,									// NodeInfoMsg_t
	GENERIC_REQ_NODE_STATUS,								// NO payload
	GENERIC_RES_NODE_STATUS,								// NodeStatusMsg_t
	GENERIC_REQ_SPEAKER,									// SpeakerMsg_t
	GENERIC_REQ_THRESHOLD,									// ThresholdMsg_t

	GENERIC_TOTAL_CMDS
} GENERIC_CMDs;

typedef enum
{
	ERROR_FLAG_CAN_COMM1 = 0x0001,
	ERROR_FLAG_CAN_COMM2 = 0x0002,
	ERROR_FLAG_PWR_BUS1 = 0x0004,
	ERROR_FLAG_PWR_BUS2 = 0x0008,
	ERROR_FLAG_OVERCURRENT_THLD1 = 0x0010

} ERROR_FLAG;

typedef uint32_t ErrorFlag_t;

typedef struct __attribute__((__packed__))
{
	uint32_t firmware_version;
	uint32_t channel_mask;
	uint8_t channel_type[MAX_CHANNELS];
} NodeInfoMsg_t;

typedef struct __attribute__((__packed__))
{
	ErrorFlag_t node_error_flags;
	uint32_t channel_error_mask;
	ChannelStatus_t channel_status[MAX_CHANNELS];
} NodeStatusMsg_t;

typedef struct __attribute__((__packed__))
{
	uint32_t channel_mask;
	uint8_t uint8[MAX_DATA_SIZE - sizeof(uint32_t)];
} DataMsg_t;

typedef struct __attribute__((__packed__))
{
	uint16_t tone_frequency;	// in Hz
	uint16_t on_time;			// in ms
	uint16_t off_time;			// in ms
	uint8_t count;				// number of beeps
} SpeakerMsg_t;

typedef struct __attribute__((__packed__))
{
	uint8_t channel_id;
	uint8_t threshold_id;
	uint8_t enabled;			// (0,1)
	uint8_t var_id;
	Operation_t compare_id;
	int32_t threshold;
	CHANNEL_STATUS result;
	uint8_t or_threshold_id;
	uint8_t and_threshold_id;
} ThresholdMsg_t;

#endif
