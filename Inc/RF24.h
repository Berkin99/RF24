/*
 *	nrf24.h
 *
 *  Created on: Fab 10, 2024
 *      Author: BerkN
 *
 *  nRF24l01 transceiver driver library for stm32 microcontrollers.
 *  HAL Library integration.
 *
 *  10.02.2024 : Created.
 *
 *	References:
 *  [0] nRF24L01+ Single Chip 2.4GHz Transceiver Preliminary Product Specification v1.0
 *	[1] github.com/nRF24/RF24
 *
 */

#include <stdint.h>
#include "main.h"
//#include "sysconfig.h"


typedef enum
{
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;

typedef enum
{
	RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

typedef enum
{
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;


void RF24_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* ce_gpio, uint16_t ce_pin, GPIO_TypeDef* cs_gpio, uint16_t cs_pin);

uint8_t RF24_getStatus(void);
uint8_t RF24_readRegister(uint8_t target, uint8_t* pRxBuffer, uint8_t length);
void RF24_writeRegister(uint8_t target, uint8_t* pData, uint8_t length);

void RF24_writePayload(const void* pData, uint8_t length, uint8_t writeType);
void RF24_readPayload(void* pBuffer, uint8_t length);

void RF24_setChannel(uint8_t channel);
void RF24_setPayloadSize(uint8_t size);
uint8_t RF24_getPayloadSize(void);
void RF24_setAddressWidth(uint8_t a_width);
uint8_t RF24_available(void);
uint8_t RF24_pipeAvailable(uint8_t* pipe_num);
void RF24_setRetries(uint8_t delay, uint8_t count);
void RF24_powerUp(void);

uint8_t RF24_begin(void);
void RF24_openReadingPipe(uint8_t child, uint8_t* address);
void RF24_closeReadingPipe(uint8_t pipe);
void RF24_startListening(void);
void RF24_stopListening(void);

void RF24_read(void* pBuffer, uint8_t length);
