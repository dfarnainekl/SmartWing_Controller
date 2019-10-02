#ifndef AS5047U_H_
#define AS5047U_H_


#include <stdint.h>


#define AS5047U_READ			1
#define AS5047U_WRITE			0

//volatile memory
#define ADDR_NOP          	0x0000
#define ADDR_ERRFL		    	0x0001
#define ADDR_PROG		     	  0x0003
#define ADDR_VEL		      	0x3FFC
#define ADDR_ANGLEUNC	    	0x3FFE
#define ADDR_ANGLECOM	    	0x3FFF
#define ADDR_ECC_CHECK	  	0x00D1
#define ADDR_DIAG           0x3ff5

//non-volatile memory
#define ADDR_NV_DISABLE	  		0x0016
#define ADDR_NV_ZPOSM		  	0x0016
#define ADDR_NV_ZPOSL		  	0x0017
#define ADDR_NV_SETTINGS1		0x0018
#define ADDR_NV_SETTINGS2		0x0019
#define ADDR_NV_SETTINGS3		0x001A
#define ADDR_NV_ECC			  	0x001B


uint32_t as5047U_sendCommand(uint8_t drv, uint8_t rw, uint16_t address);

uint16_t as5047U_getData(uint8_t drv, uint32_t data);
uint32_t as5047U_sendData(uint8_t drv, uint16_t data);

uint16_t as5047U_getAngle(uint8_t drv);
uint16_t as5047U_getAngle_fast(uint8_t drv);
void as5047U_setABIResolution14Bit(uint8_t drv);


#endif /* AS5047U_H_ */
