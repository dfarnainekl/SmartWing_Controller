#ifndef CAN_CMDS_H_
#define CAN_CMDS_H_

#include <stdint.h>

#define MAX_DATA_SIZE 64

#define CAN_MSG_LENGTH(payload_length) ( sizeof(Can_MessageDataInfo_t) + sizeof(uint8_t) + payload_length )



typedef enum
{
	MASTER2NODE_DIRECTION, NODE2MASTER_DIRECTION,
} CAN_MessageDirection_t;

typedef enum
{
	URGENT_PRIORITY, HIGH_PRIORITY, STANDARD_PRIORITY, LOW_PRIORITY
} CAN_MessagePriority_t;

typedef enum
{
	ABORT_SPECIAL_CMD, CLOCK_SYNC_SPECIAL_CMD,	// DIR = MASTER2NODE_DIRECTION
	ERROR_SPECIAL_CMD = CLOCK_SYNC_SPECIAL_CMD, // DIR = NODE2MASTER_DIRECTION
	INFO_SPECIAL_CMD,
	STANDARD_SPECIAL_CMD,
} CAN_MessageSpecialCmd_t;

typedef enum
{
	DIRECT_BUFFER, ABSOLUTE_BUFFER, RELATIVE_BUFFER, RESERVED_BUFFER
} CAN_MessageBuffer_t;

typedef union
{
	struct __attribute__((__packed__))
	{
		uint32_t direction :1;		//bit:    0   | CAN_MessageDirection_t
		uint32_t node_id :6;		//bit:  6-1   | Node ID: 0 - 63
		uint32_t special_cmd :2;	//bit:  8-7   | CAN_MessageSpecialCmd_t
		uint32_t priority :2;		//bit: 10-9   | CAN_MessagePriority_t
	} info;
	uint32_t uint32;
} Can_MessageId_t;

typedef struct __attribute__((__packed__))
{
	uint32_t channel_id :6;			//bit:  5-0   | Channel ID: 0 - 63
	CAN_MessageBuffer_t buffer :2;	//bit:  7-6   | CAN_MessageBuffer_t
} Can_MessageDataInfo_t;

typedef union
{
	struct __attribute__((__packed__))
	{
		Can_MessageDataInfo_t info;
		uint8_t cmd_id;
		union
		{
			uint8_t uint8[(MAX_DATA_SIZE - CAN_MSG_LENGTH(0))];
			uint32_t uint32[(MAX_DATA_SIZE - CAN_MSG_LENGTH(0)) / 4];
		} data;
	} bit;
	uint8_t uint8[MAX_DATA_SIZE];
} Can_MessageData_t;

#endif
