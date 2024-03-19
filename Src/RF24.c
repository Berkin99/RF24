/*
 *	nrf24.c
 *
 *  Created on: Fab 10, 2024
 *      Author: BerkN
 *
 *  nRF24l01 transceiver driver library for stm32 microcontrollers.
 *  HAL Library integration.
 *
 *  10.02.2024 : Created.
 *  19.03.2024 : Write Function added.
 *
 *	References:
 *  [0] nRF24L01+ Single Chip 2.4GHz Transceiver Preliminary Product Specification v1.0
 *	[1] github.com/nRF24/RF24
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf24l01.h"
#include "RF24.h"

#include "timer.h"


#if !defined(RF24_POWERUP_DELAY)
    #define RF24_POWERUP_DELAY 5
#endif

#ifndef RF24_SPI_SPEED
    #define RF24_SPI_SPEED 10000000
#endif

#define rf24_max(a, b) (a > b ? a : b)
#define rf24_min(a, b) (a < b ? a : b)
#define _BV(x) (1 << (x))


static SPI_HandleTypeDef* RF24_hspi;
static GPIO_TypeDef* _ce_gpio;
static uint16_t _ce_pin;
static GPIO_TypeDef* _cs_gpio;
static uint16_t _cs_pin;

static uint32_t spi_speed = RF24_SPI_SPEED;   /* SPI Bus Speed */
static uint8_t NOP = RF24_NOP;

static uint8_t status;                   /* The status byte returned from every SPI transaction */
static uint8_t payload_size;             /* Fixed size of payloads */
static uint8_t pipe0_reading_address[5]; /* Last address set on pipe 0 for reading. */
static uint8_t config_reg;               /* For storing the value of the NRF_CONFIG register */
static bool _is_p_variant;               /* For storing the result of testing the toggleFeatures() affect */
static bool _is_p0_rx;                   /* For keeping track of pipe 0's usage in user-triggered RX mode. */

static const uint8_t child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

static uint8_t addr_width;
static bool dynamic_payloads_enabled;
static bool ack_payloads_enabled;
static uint32_t csDelay;
static uint32_t txDelay;

void RF24_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* ce_gpio, uint16_t ce_pin, GPIO_TypeDef* cs_gpio, uint16_t cs_pin){
	RF24_hspi = hspi;
	_ce_gpio  = ce_gpio;
	_ce_pin   = ce_pin;
	_cs_gpio  = cs_gpio;
	_cs_pin   = cs_pin;

	pipe0_reading_address[0] = 0;

	payload_size = 32;
	_is_p_variant= false;
	_is_p0_rx = false;

	addr_width = 5;
	dynamic_payloads_enabled = true;
	csDelay = 5;

	pipe0_reading_address[0] = 0;
	if (spi_speed <= 35000) { //Handle old BCM2835 speed constants, default to RF24_SPI_SPEED
		spi_speed = 10000000;
	}
}

void beginTransaction(void){
	HAL_GPIO_WritePin(_cs_gpio, _cs_pin, GPIO_PIN_RESET);
	delayMicroseconds(csDelay);
}

void endTransaction(void){
	HAL_GPIO_WritePin(_cs_gpio, _cs_pin, GPIO_PIN_SET);
	delayMicroseconds(csDelay);
}

uint8_t flush_rx(void){
	RF24_writeRegister(FLUSH_RX, &NOP, 0);
    return status;
}

uint8_t flush_tx(void){
	RF24_writeRegister(FLUSH_TX, &NOP, 0);
    return status;
}

uint8_t RF24_getStatus(void)
{
    RF24_writeRegister(RF24_NOP, &NOP, 0);
    return status;
}


uint8_t RF24_readRegister(uint8_t target, uint8_t* pRxBuffer, uint8_t length){
    beginTransaction();
    uint8_t reg = (R_REGISTER | target);
    HAL_SPI_TransmitReceive(RF24_hspi, &reg, &status, 1, 50);

    while(length--){
    	HAL_SPI_TransmitReceive(RF24_hspi, &NOP, pRxBuffer++, 1, 50);
    }

    endTransaction();
    return status;
}

void RF24_writeRegister(uint8_t target, uint8_t* pData, uint8_t length){
    beginTransaction();
    uint8_t reg = (W_REGISTER | target);
    HAL_SPI_TransmitReceive(RF24_hspi, &reg, &status, 1, 50);
    if(length > 0) HAL_SPI_Transmit(RF24_hspi, pData, length, 50);

    endTransaction();
}

void RF24_writePayload(const void* pData, uint8_t length, uint8_t writeType){

	uint8_t* current = (uint8_t*)pData;
    uint8_t blank_len = !length ? 1 : 0;

    if (!dynamic_payloads_enabled) {
    	length = rf24_min(length, payload_size);
        blank_len = payload_size - length;
    }
    else length = rf24_min(length, 32);

    beginTransaction();
    HAL_SPI_TransmitReceive(RF24_hspi, &writeType, &status, 1, 50);

	while (length--) {
		HAL_SPI_Transmit(RF24_hspi, current++, 1, 50);
	}

	uint8_t empty = 0;

	while (blank_len--) {
		HAL_SPI_Transmit(RF24_hspi, &empty, 1, 50);
	}
    endTransaction();
}

void RF24_readPayload(void* pBuffer, uint8_t length){

	uint8_t* current = (uint8_t*) pBuffer;
    uint8_t blank_len = 0;
    if (!dynamic_payloads_enabled) {
    	length = rf24_min(length, payload_size);
        blank_len = (uint8_t)(payload_size - length);
    }
    else length = rf24_min(length, 32);

    beginTransaction();

    uint8_t temp = R_RX_PAYLOAD;
    HAL_SPI_TransmitReceive(RF24_hspi, &temp, &status, 1, 50);

    while (length--) {
        HAL_SPI_TransmitReceive(RF24_hspi, &NOP, current++, 1, 50);
    }

    while (blank_len--) {
		HAL_SPI_Transmit(RF24_hspi, &NOP, 1, 50);
    }

    endTransaction();
}

void RF24_setChannel(uint8_t channel)
{
    const uint8_t max_channel = 125;
    uint8_t ch = rf24_min(channel, max_channel);
    RF24_writeRegister(RF_CH,&ch,1);
}


void RF24_setPayloadSize(uint8_t size){
    // payload size must be in range [1, 32]
    payload_size = (uint8_t)(rf24_max(1, rf24_min(32, size)));

    // write static payload size setting for all pipes
    for (uint8_t i = 0; i < 6; ++i) {
        RF24_writeRegister((uint8_t)(RX_PW_P0 + i), &payload_size, 1);
    }
}

uint8_t RF24_getPayloadSize(void){
	return payload_size;
}

void RF24_setAddressWidth(uint8_t a_width){
    a_width = (uint8_t)(a_width - 2);
    if (a_width) {

    	uint8_t temp = (a_width % 4);
    	RF24_writeRegister(SETUP_AW, &temp,1);
        addr_width = ((a_width % 4) + 2);
    }
    else {
    	uint8_t temp = 0;
    	RF24_writeRegister(SETUP_AW, &temp,1);
        addr_width = 2;
    }
}

uint8_t RF24_available(void){
    uint8_t pipe = 0;
    return RF24_pipeAvailable(&pipe);
}


uint8_t RF24_pipeAvailable(uint8_t* pipe_num)
{
    // get implied RX FIFO empty flag from status byte
    uint8_t pipe = (RF24_getStatus() >> RX_P_NO) & 0x07;
    if (pipe > 5) return 0;

    // If the caller wants the pipe number, include that
    if (pipe_num) *pipe_num = pipe;
    return 1;
}


void RF24_setRetries(uint8_t delay, uint8_t count){
	uint8_t temp =  (uint8_t)(rf24_min(15, delay) << ARD | rf24_min(15, count));
	RF24_writeRegister(SETUP_RETR,&temp,1);
}

uint8_t _data_rate_reg_value(rf24_datarate_e speed)
{
    txDelay = 85;
    if (speed == RF24_250KBPS) {
        txDelay = 155;
        return _BV(RF_DR_LOW);
    }
    else if (speed == RF24_2MBPS) {
        txDelay = 65;
        return _BV(RF_DR_HIGH);
    }
    // HIGH and LOW '00' is 1Mbs - our default
    return 0;
}


uint8_t RF24_setDataRate(rf24_datarate_e speed)
{
    uint8_t result = 0;
    uint8_t setup = 0;

    RF24_readRegister(RF_SETUP,&setup,1);

    // HIGH and LOW '00' is 1Mbs - our default
    setup = (uint8_t)(setup & ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)));
    setup |= _data_rate_reg_value(speed);

    RF24_writeRegister(RF_SETUP, &setup,1);
    RF24_readRegister(RF_SETUP, &result, 1);

    return (result==setup);
}


void toggle_features(){
	beginTransaction();

	uint8_t reg = ACTIVATE;
	HAL_SPI_TransmitReceive(RF24_hspi, &reg, &status, 1, 50);
	reg = 0x73;
    HAL_SPI_Transmit(RF24_hspi, &reg, 1, 50);
    endTransaction();
}

void RF24_powerUp(void)
{
    // if not powered up then power up and wait for the radio to initialize
    if (!(config_reg & _BV(PWR_UP))) {
        config_reg |= _BV(PWR_UP);
        RF24_writeRegister(NRF_CONFIG, &config_reg, 1);
        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        delayMicroseconds(RF24_POWERUP_DELAY);
    }
}


uint8_t RF24_begin(void)
{

	HAL_GPIO_WritePin(_ce_gpio, _ce_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_cs_gpio, _cs_pin, GPIO_PIN_SET);

	HAL_Delay(5);

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See datasheet for a more complete explanation.
	RF24_setRetries(5, 15);

	// Then set the data rate to the slowest (and most reliable) speed supported by all hardware.
	RF24_setDataRate(RF24_1MBPS);

	// detect if is a plus variant & use old toggle features command accordingly
	uint8_t before_toggle = 0;
	RF24_readRegister(FEATURE,&before_toggle,1);
	toggle_features();
	uint8_t after_toggle = 0;
	RF24_readRegister(FEATURE,&after_toggle,1);

	_is_p_variant = before_toggle == after_toggle;

	uint8_t temp = 0;
	if (after_toggle) {
		if (_is_p_variant) {
			// module did not experience power-on-reset (#401)
			toggle_features();
		}
		// allow use of multicast parameter and dynamic payloads by default
		RF24_writeRegister(FEATURE, &temp, 1);
	}

	ack_payloads_enabled = false; // ack payloads disabled by default

	RF24_writeRegister(DYNPD, &temp, 1);
	dynamic_payloads_enabled = false;

	temp = 0x3F;
	RF24_writeRegister(EN_AA, &temp, 1);

	temp = 0x03;
	RF24_writeRegister(EN_RXADDR, &temp, 1);


	RF24_setPayloadSize(32);           // set static payload size to 32 (max) bytes by default
	RF24_setAddressWidth(5);           // set default address length to (max) 5 bytes

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	RF24_setChannel(76);

	// Reset current status
	// Notice reset and flush is the last thing we do

	temp = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);
	RF24_writeRegister(NRF_STATUS, &temp,1);

	// Flush buffers
	flush_rx();
	flush_tx();

	// Clear CONFIG register:
	//      Reflect all IRQ events on IRQ pin
	//      Enable PTX
	//      Power Up
	//      16-bit CRC (CRC required by auto-ack)
	// Do not write CE high so radio will remain in standby I mode
	// PTX should use only 22uA of power


	temp = (_BV(EN_CRC) | _BV(CRCO));
	RF24_writeRegister(NRF_CONFIG, &temp,1);
	RF24_readRegister(NRF_CONFIG, &config_reg, 1);

	RF24_powerUp();

	// if config is not set correctly then there was a bad response from module
	return config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)) ? 1 : 0;
}

void RF24_openReadingPipe(uint8_t child, uint8_t* address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, address, addr_width);
        _is_p0_rx = true;
    }

    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            RF24_writeRegister(child_pipe[child], address, addr_width);
        }
        else {
            RF24_writeRegister(child_pipe[child], address, 1);
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.

        uint8_t temp = 0;
        RF24_readRegister(EN_RXADDR,&temp,1);
        temp |= _BV(child_pipe_enable[child]);

        RF24_writeRegister(EN_RXADDR, &temp, 1);
    }
}

void RF24_closeReadingPipe(uint8_t pipe)
{
	uint8_t temp = 0;
	RF24_readRegister(EN_RXADDR,&temp,1);
    temp &= ~_BV(child_pipe_enable[pipe]);
    RF24_writeRegister(EN_RXADDR,&temp,1);

    if (!pipe) {
        // keep track of pipe 0's RX state to avoid null vs 0 in addr cache
        _is_p0_rx = false;
    }
}

void RF24_openWritingPipe(uint8_t* address){
	RF24_writeRegister(RX_ADDR_P0, address, addr_width);
	RF24_writeRegister(TX_ADDR, address, addr_width);
}

void RF24_startListening(void)
{
    RF24_powerUp();

    config_reg |= _BV(PRIM_RX);
    RF24_writeRegister(NRF_CONFIG,&config_reg,1);


	uint8_t temp = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);

    RF24_writeRegister(NRF_STATUS, &temp,1);

    HAL_GPIO_WritePin(_ce_gpio, _ce_pin, GPIO_PIN_SET);

    // Restore the pipe0 address, if exists
    if (_is_p0_rx) {
    	RF24_writeRegister(RX_ADDR_P0, pipe0_reading_address, addr_width);
    }
    else {
        RF24_closeReadingPipe(0);
    }
}

void RF24_stopListening(void){

    HAL_GPIO_WritePin(_ce_gpio, _ce_pin, GPIO_PIN_RESET);

    delayMicroseconds(txDelay);
	if (ack_payloads_enabled) {
		flush_tx();
	}

	config_reg = config_reg & ~_BV(PRIM_RX);
	RF24_writeRegister(NRF_CONFIG, &config_reg,1);

	uint8_t temp = 0;
	RF24_readRegister(EN_RXADDR, &temp, 1);
	temp |= _BV(child_pipe_enable[0]);

	RF24_writeRegister(EN_RXADDR, &temp,1); // Enable RX on pipe0
}

void RF24_read(void* pBuffer, uint8_t length)
{
    // Fetch the payload
    RF24_readPayload(pBuffer, length);
    //Clear the only applicable interrupt flags
    uint8_t temp = (uint8_t)_BV(RX_DR);
    RF24_writeRegister(NRF_STATUS,&temp,1);
}


uint8_t RF24_write(const void* pBuffer, uint8_t length){

	uint8_t multicast = 0;
    RF24_writePayload(pBuffer, length, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    HAL_GPIO_WritePin(_ce_gpio, _ce_pin, GPIO_PIN_SET);

    uint32_t timer = millis();
    while (!(RF24_getStatus() & (_BV(TX_DS) | _BV(MAX_RT)))) {
		if (millis() - timer > 95) {
			return 0;
		}
	}

	HAL_GPIO_WritePin(_ce_gpio, _ce_pin, GPIO_PIN_RESET);

	uint8_t data = (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	RF24_writeRegister(NRF_STATUS, &data, 1);
	if (status & _BV(MAX_RT)) {
		flush_tx(); // Only going to be 1 packet in the FIFO at a time using this method, so just flush
		return 0;
	}

	return 1;
}
