#include "as5047U.h"
#include "swdriver.h"
#include "spi.h"
#include "usart.h"
#include <limits.h>
#include <string.h>

#define PRINT_ERRORS 0

static void spiMode_set(uint8_t drv)
{
	swdriver[drv].SPI->Init.CLKPolarity = SPI_POLARITY_LOW;
	swdriver[drv].SPI->Init.CLKPhase = SPI_PHASE_2EDGE;

	if (HAL_SPI_Init(swdriver[drv].SPI) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	//HAL_Delay(2);
}

static void spiMode_reset(uint8_t drv)
{
	swdriver[drv].SPI->Init.CLKPolarity = SPI_POLARITY_HIGH;
	swdriver[drv].SPI->Init.CLKPhase = SPI_PHASE_2EDGE;

	if (HAL_SPI_Init(swdriver[drv].SPI) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	//HAL_Delay(2);
}

static uint8_t gencrc(uint8_t *data, uint8_t len)
{
    uint8_t crc =0xb7;
    size_t i, j;
    for (i = 0; i < len; i++)
		{
        crc ^= data[i];
        for (j = 0; j < 8; j++)
				{
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x1d);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static uint32_t as5047U_sendCommand(uint8_t drv, uint8_t rw, uint16_t address)
{
	uint16_t data_send = 0;
	uint32_t data_receive = 0;

	uint8_t txData[3];
	uint8_t rxData[3];

	address &= 0x3FFF;
	data_send = address | (rw<<14);

	txData[0] = (uint8_t)( (data_send & 0xff00) >> 8);
	txData[1] = (uint8_t)(data_send & 0x00ff) ;
	txData[2] = gencrc(txData, 2);

	spiMode_set(drv);
	//HAL_Delay(1);
	swdriver_setCsnEncoder(drv, false);
	HAL_SPI_TransmitReceive(swdriver[drv].SPI, txData, rxData, 3, HAL_MAX_DELAY);
	swdriver_setCsnEncoder(drv, true);
	//HAL_Delay(1);
	spiMode_reset(drv);

	data_receive =  ((uint32_t)rxData[0]<<16) | ((uint32_t)rxData[1]<<8) | (uint32_t)rxData[2];
	return data_receive;
}


static void diag(uint8_t drv, uint8_t warning, uint8_t error)
{
#if PRINT_ERRORS
	static char string[500];
	static char string2[50];
#endif

	uint32_t data32 = 0;
	uint16_t data16 = 0;

#if PRINT_ERRORS
	sprintf(string, "\n\n");

	if(error)
		strcat (string,"ERROR\t");
	else if (warning)
		strcat (string,"WARNING\t");

	sprintf(string2, "drv %d\n", drv);
	strcat (string,string2);
#endif

	as5047U_sendCommand(drv, AS5047U_READ, ADDR_ERRFL);
	data32 = as5047U_sendCommand(drv, AS5047U_READ, ADDR_ERRFL);
	data16 =(uint16_t) ((data32 & 0x003FFF00)>>8);

	if((data16>>0)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"AGC-warning\n");
		#endif
	}
	else if((data16>>1)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"MagHalf\n");
		#endif
	}
	else if((data16>>2)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"P2ram_warning\n");
		#endif
	}
	else if((data16>>3)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"P2ram_error\n");
		#endif
	}
	else if((data16>>4)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"Framing error\n");
		#endif
	}
	else if((data16>>5)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"Command error\n");
		#endif
	}
	else if((data16>>6)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"CRC error\n");
		#endif
	}
	else if((data16>>7)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"WDTST\n");
		#endif
	}
	else if((data16>>9)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"OffCompNotFinished\n");
		#endif
	}
	else if((data16>>10)&0x01)
	{
		as5074uErrorCounter[drv]++;
		#if PRINT_ERRORS
		strcat (string,"CORDIC_Overflow\n");
		#endif
	}

	#if PRINT_ERRORS
	HAL_UART_Transmit_IT(&huart3, (uint8_t*)string, 500);
	HAL_Delay(300);
	#endif
}


static uint32_t as5047U_sendData(uint8_t drv, uint16_t data)
{
	return as5047U_sendCommand(drv, AS5047U_WRITE, data);
}


static uint16_t as5047U_getData(uint8_t drv, uint32_t data)
{
	uint8_t warning 	=  ((uint32_t)(data & 0x00800000)>>23);
	uint8_t error 		=  ((uint32_t)(data & 0x00400000)>>22);

	if(warning || error)
		diag(drv, warning, error);

	return (uint16_t) ((data & 0x003FFF00)>>8);
}





uint16_t as5047U_getAngle(uint8_t drv) //returns 16 bit value (with 14 bit resolution)
{
	uint32_t data_received = 0;
	uint16_t data_raw = 0;
	data_received = as5047U_sendCommand(drv, AS5047U_READ, ADDR_ANGLECOM);
	data_raw = as5047U_getData(drv, data_received);
	data_received = as5047U_sendCommand(drv, AS5047U_READ, ADDR_NOP);
	data_raw = as5047U_getData(drv, data_received);
	return ((uint16_t)data_raw<<2);
}

int16_t as5047U_getVelocity(uint8_t drv) //returns 16 bit value (with 14 bit resolution)
{
	uint32_t data_received = 0;
	uint16_t data_raw = 0;
	data_received = as5047U_sendCommand(drv, AS5047U_READ, ADDR_VEL);
	data_raw = as5047U_getData(drv, data_received);
	data_received = as5047U_sendCommand(drv, AS5047U_READ, ADDR_NOP);
	data_raw = as5047U_getData(drv, data_received);
	return ((int16_t)(data_raw<<2)/4);
}


uint16_t as5047U_getAngle_fast(uint8_t drv) //returns 16 bit value (with 14 bit resolution)
{
	spiMode_set(drv);

	uint8_t txData[2];
	txData[0] = (1 << 7) | (1 << 6) | 0x3F; // parity 1, read, address upper 6 bits
	txData[1] = 0xFF; // address lower 6 bits

	swdriver_setCsnEncoder(drv, false);
	HAL_SPI_Transmit(swdriver[drv].SPI, txData, 2, HAL_MAX_DELAY);
	swdriver_setCsnEncoder(drv, true);
	//HAL_Delay(2);

	txData[0] = 0;
	txData[1] = 0;
	uint8_t rxData[2];

	swdriver_setCsnEncoder(drv, false);
	HAL_SPI_TransmitReceive(swdriver[drv].SPI, txData, rxData, 2, HAL_MAX_DELAY);
	swdriver_setCsnEncoder(drv, true);

	spiMode_reset(drv);

	return ((((((uint16_t)rxData[0]) & 0x3F) << 8 ) | rxData[1] ) << 2);
}




void as5047U_setABIResolution14Bit(uint8_t drv)
{
	uint32_t data_received = 0;

	data_received = as5047U_sendCommand(drv, AS5047U_WRITE, ADDR_NV_SETTINGS3);
	as5047U_getData(drv, data_received);
	data_received = as5047U_sendData(drv, 0b10000000);
	data_received = as5047U_sendCommand(drv, AS5047U_READ, ADDR_NV_SETTINGS3);
	data_received = as5047U_sendCommand(drv, AS5047U_READ, ADDR_NOP);
	as5047U_getData(drv, data_received);
}
