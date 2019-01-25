/* Copyright (c) 2007 Fabian Greif
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------


#include <avr/pgmspace.h>
#include <util/delay.h>

#include <stdint.h>
#include "digital_pin.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"
#include "spi.h"

#define MCP2515_CS(x) write_pin(portB, PORTB2, x)
#define MCP2515_INT_INIT configure_pin(portD, PORTD2, INPUT)
#define MCP2515_INT(x) write_pin(portD, PORTD2, x)
#define MCP2515_INT_HIGH read_pin(portD, PIND2)
#define LED2_INIT configure_pin(portB, PORTB0, OUTPUT_INIT_LOW)
#define LED2(x) write_pin(portB, PORTB0, x)
#define TRUE 1
#define FALSE 0
#define HIGH 1
#define LOW 0

// -------------------------------------------------------------------------
void mcp2515_write_register( uint8_t adress, uint8_t data )
{
	MCP2515_CS(LOW);

	spi_xchg(SPI_WRITE);
	spi_xchg(adress);
	spi_xchg(data);

	MCP2515_CS(HIGH);
}

// -------------------------------------------------------------------------
uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;

	MCP2515_CS(LOW);

	spi_xchg(SPI_READ);
	spi_xchg(adress);

	data = spi_xchg(0xff);

	MCP2515_CS(HIGH);

	return data;
}

// -------------------------------------------------------------------------
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	MCP2515_CS(LOW);

	spi_xchg(SPI_BIT_MODIFY);
	spi_xchg(adress);
	spi_xchg(mask);
	spi_xchg(data);

	MCP2515_CS(HIGH);
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;

	MCP2515_CS(LOW);

	spi_xchg(type);
	data = spi_xchg(0xff);

	MCP2515_CS(HIGH);

	return data;
}

// -------------------------------------------------------------------------
uint8_t mcp2515_init(uint8_t speed)
{
	/* SPI interface is already configured
     *
     * SET_INPUT(MCP2515_INT);
     * SET(MCP2515_INT);
     */

	// Initialize the MCP2515 interrupt line as input
	MCP2515_INT_INIT;
	// Initialize LED2 B0 [Arduino D8] so we can flash status on it
	LED2_INIT;

	// reset MCP2515 by software reset.
	// After this he is in configuration mode.
	MCP2515_CS(LOW);
	spi_xchg(SPI_RESET);
	MCP2515_CS(HIGH);

	// wait a little bit until the MCP2515 has restarted
	_delay_us(20);

	// load CNF1..3 Register
	MCP2515_CS(LOW);
	spi_xchg(SPI_WRITE);
	spi_xchg(CNF3);

/*	spi_xchg((1<<PHSEG21));		// Bitrate 125 kbps at 16 MHz
	spi_xchg((1<<BTLMODE)|(1<<PHSEG11));
	spi_xchg((1<<BRP2)|(1<<BRP1)|(1<<BRP0));
*/
/*
	spi_xchg((1<<PHSEG21));		// Bitrate 250 kbps at 16 MHz
	spi_xchg((1<<BTLMODE)|(1<<PHSEG11));
	spi_xchg((1<<BRP1)|(1<<BRP0));
*/
	// TODO: Revisit how to parameterize this
	// Bitrate 500 kbps at 16 MHz
	spi_xchg(0x05);		// CNF3 - SOF = off, WAKFIL = off, PS2 = 6xTq
	spi_xchg(0xE3); 	// CNF2 - BTL = on, SAM = 3x, PS1 = 5xTq, PropSeg = 4xTq
    spi_xchg(0x40);		// CNF1 - SJW = 2xTq, BRP = 2x

	// activate interrupts
	spi_xchg(_BV(RX1IE) | _BV(RX0IE));

	MCP2515_CS(HIGH);

	// test if we could read back the value => is the chip accessible?
	if (mcp2515_read_register(CNF1) != speed) {
		// Blink LED2 5 times
		for (int n = 0; n < 5; n++) {
			LED2(1);
			_delay_ms(250);
			LED2(0);
			_delay_ms(250);
		}
		return FALSE;
	}

	// deaktivate the RXnBF Pins (High Impedance State)
	mcp2515_write_register(BFPCTRL, 0);

	// set TXnRTS as inputs
	mcp2515_write_register(TXRTSCTRL, 0);

	// turn off filters => receive any message
	mcp2515_write_register(RXB0CTRL, _BV(RXM1) | _BV(RXM0));
	mcp2515_write_register(RXB1CTRL, _BV(RXM1) | _BV(RXM0));

	// reset device to normal mode
	mcp2515_write_register(CANCTRL, 0);
	LED2(HIGH);
	return TRUE;
}

// ----------------------------------------------------------------------------
// check if there are any new messages waiting

uint8_t mcp2515_check_message(void) {
	return (!MCP2515_INT_HIGH);
}

// ----------------------------------------------------------------------------
// check if there is a free buffer to send messages

uint8_t mcp2515_check_free_buffer(void)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);

	// all buffers used?
	return ((status & 0x54) == 0x54);}

// ----------------------------------------------------------------------------
uint8_t mcp2515_get_message(tCAN *message)
{
	// read status
	uint8_t status = mcp2515_read_status(SPI_RX_STATUS);
	uint8_t addr;
	uint8_t t;
	if (bit_is_set(status,6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
	}
	else if (bit_is_set(status,7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
	}
	else {
		// Error: no message available
		return 0;
	}

	MCP2515_CS(LOW);
	spi_xchg(addr);

	// read id
	message->id = (uint16_t) spi_xchg(0xff) << 3;
	message->id |= spi_xchg(0xff) >> 5;

	spi_xchg(0xff);
	spi_xchg(0xff);

	// read DLC
	uint8_t length = spi_xchg(0xff) & 0x0f;

	message->header.length = length;
	message->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;

	// read data
	for (t=0;t<length;t++) {
		message->data[t] = spi_xchg(0xff);
	}
	MCP2515_CS(LOW);

	// clear interrupt flag
	if (bit_is_set(status, 6)) {
		mcp2515_bit_modify(CANINTF, _BV(RX0IF), 0);
	}
	else {
		mcp2515_bit_modify(CANINTF, _BV(RX1IF), 0);
	}

	return (status & 0x07) + 1;
}

// ----------------------------------------------------------------------------
uint8_t mcp2515_send_message(tCAN *message)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);

	/* Statusbyte:
	 *
	 * Bit	Function
	 *  2	TXB0CNTRL.TXREQ
	 *  4	TXB1CNTRL.TXREQ
	 *  6	TXB2CNTRL.TXREQ
	 */
	uint8_t address;
	uint8_t t;
	LED2(LOW);
	if (bit_is_clear(status, 2)) {
		address = 0x00;
	}
	else if (bit_is_clear(status, 4)) {
		address = 0x02;
	}
	else if (bit_is_clear(status, 6)) {
		address = 0x04;
	}
	else {
		// all buffer used => could not send message
		return 0;
	}

	MCP2515_CS(LOW);
	spi_xchg(SPI_WRITE_TX | address);

	spi_xchg(message->id >> 3);
    spi_xchg(message->id << 5);

	spi_xchg(0);
	spi_xchg(0);

	uint8_t length = message->header.length & 0x0f;

	if (message->header.rtr) {
		// a rtr-frame has a length, but contains no data
		spi_xchg(_BV(RTR) | length);
	}
	else {
		// set message length
		spi_xchg(length);

		// data
		for (t=0;t<length;t++) {
			spi_xchg(message->data[t]);
		}
	}
	MCP2515_CS(HIGH);

	_delay_us(1);

	// send message
	MCP2515_CS(LOW);
	address = (address == 0) ? 1 : address;
	spi_xchg(SPI_RTS | address);
	MCP2515_CS(HIGH);

	return address;
}

