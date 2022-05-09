#ifndef BLMB_H_
#define BLMB_H_



//#define BLMB_ERROR_THRESHOLD (1<<16)
#define BLMB_ERROR_THRESHOLD (1<<5)
#define BLMB_ERROR_TIMER 50
#define BLMB_REDUCTION 4
#define MAX_BLMB_CHANNELS 1

#define ENDPOINT_THRESHOLD 6554

#define BLMB_SERVO_CHANNEL 0
#define BLMB_SENSOR_CHANNEL 1
#define BLMB_TORQUE_CHANNEL 2

#define BLMB_MAIN_CAN_BUS 0


#define EN_GPIO_Port		GPIOD
#define EN_Pin				LL_GPIO_PIN_7

#define STATUS_GPIO_Port	GPIOC
#define STATUS_Pin			LL_GPIO_PIN_12

#define FAULT_GPIO_Port		GPIOD
#define FAULT_Pin			LL_GPIO_PIN_4

#define CSN_CTR_GPIO_Port	GPIOC
#define CSN_CTR_Pin			LL_GPIO_PIN_6

#define CSN_DRV_GPIO_Port	GPIOD
#define CSN_DRV_Pin			LL_GPIO_PIN_11

#define CSN_ENC_GPIO_Port	GPIOD
#define CSN_ENC_Pin		LL_GPIO_PIN_14


void BLMB_main(void);

#endif
