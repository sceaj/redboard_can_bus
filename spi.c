// ----------------------------------------------------------------------------
/* Copyright (c) 2019 Jeff Rosen
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

#include "spi.h"
#include <stdio.h>
#include <avr/pgmspace.h>
#include "digital_pin.h"
#include "xitoa.h"

/* Initialize the SPI module
 *
 * Parameters:
 *     master - 0 for slave mode, otherwise master mode
 *     spiMode - SPI clock polarity and phase
 *     clkDivider - FOSC divider used to generate SPI clock
 */
void spi_init(int master, spi_mode spiMode, spi_clock_div clkDivider) {

	uint8_t spcr = _BV(SPE) | spiMode | (clkDivider & 0x03);
	if (master) {
		spcr |= _BV(MSTR);
	}
	uint8_t spsr = (clkDivider >> 2);
#if defined (SPI_DEBUG)
	xprintf(PSTR("SPI control register = 0x%X\n"), spcr);
	xprintf(PSTR("SPI status request = 0x%X\n"), spsr);
#endif

	/* Bring all the SPI pins low */
	write_pin(portB, PORTB3, 0);
	write_pin(portB, PORTB4, 0);
	write_pin(portB, PORTB5, 0);

	/* Configure MOSI/MISO/SCLK pins */
	// MOSI = PORTB3, MISO = PORTB4, SCLK = PORTB5
	DDRB &= ~_BV(DDB4);              // MISO configured as input
	DDRB |= _BV(DDB3) | _BV(DDB5);   // MOSI, and SCLK as output
#if defined (SPI_DEBUG)
	xprintf(PSTR("SPI Pins configured - PORTB = 0x%X [0x%X]\n"), PINB, DDRB);
#endif

	/* Enable SPI module in SPI mode 0 with 64x clock divider (SPI clock runs at 250kHz) */
	SPCR = spcr;
	SPSR = spsr;

	// Clear SPI Interrupt Flag by reading SPSR and SPDR
	xprintf(PSTR("SPI status request = 0x%X\n"), SPSR);
	xprintf(PSTR("SPI data register = 0x%X\n"), SPDR);
}

/* Update the SPI clock divisor
 *
 * Parameters:
 *     clkDivider - FOSC divider used to generate SPI clock
 */
void spi_set_clk_div(spi_clock_div clkDivider) {
	uint8_t spsr = SPSR;
	if (clkDivider > 0x03) {
		spsr |= (clkDivider >> 2);
	}
	SPSR = spsr;
	uint8_t spcr = SPCR;
	spcr &= 0xFC;
	spcr |= (clkDivider & 0x03);
	SPCR = spcr;
}

/* Send a block of data over SPI *
 */
void spi_disable() {
	/* Disable SPI function */
    SPCR = 0;
}

/* Send a block of data over SPI *
 *
 * Parameters:
 *     dataBuffer: pointer to Data block to be sent
 *     dataSize: number of bytes in Data block
 */
void spi_block_tx(const uint8_t* dataBuffer, uint16_t dataSize) {
	do {
		SPDR = *dataBuffer++;
		loop_until_bit_is_set(SPSR, SPIF);
		SPDR = *dataBuffer++;
		loop_until_bit_is_set(SPSR, SPIF);
	} while (dataSize -= 2);
}

/* Receive a block of data over SPI *
 *
 * Parameters:
 *     dataBuffer: pointer to buffer that receives data from SPI
 *     dataSize: number of bytes to receive
 */
void spi_block_rx (uint8_t* dataBuffer, uint16_t dataSize)
{
	do {
		SPDR = 0xFF;
		loop_until_bit_is_set(SPSR, SPIF);
		*dataBuffer++ = SPDR;
		SPDR = 0xFF;
		loop_until_bit_is_set(SPSR, SPIF);
		*dataBuffer++ = SPDR;
	} while (dataSize -= 2);
}

/* Send/Receive data in a single operation.
 *
 * Data will be transmitted from txBuffer and received data will be put in rxBuffer.  This function
 * will always transmit enough bytes to populate rxBuffer with rxSize bytes.  The population of
 * rxBuffer does not begin until rxDelay bytes have been transmitted. Since it is common that useful
 * data won't be returned from the other device until some number of bytes have been transmitted,
 * rxDelay provides control over this.
 *
 * Once txSize bytes have been transmitted, the value 0x55 is transmitted until rxSize bytes have been
 * put in rxBuffer.  If for some reason txSize is greater than rxSize + rxDelay, transmission will end
 * after rxBuffer has been fully populated and the remaining data in txBuffer will not be sent.
 *
 * When rxSize   For example, let's assume that the remote device is an EEPROM device where you send
 * the READ cmd byte followed by a 2 byte address and the as the 4th byte is transmitted it starts
 * returning the requested data and continues to return data for sequential addresses as long as the
 * master continues to supply clocks on the SPI bus.  Let's also assume that we want to get a 64 byte
 * block starting at address 0x200.
 *
 * We would code that up as:
 *
 *     uint8_t command[] = { READ_CMD, 0x20, 0x00 };
 *     uint8_t dataBlock[64];
 *     spi_block_xchg(command, 3, dataBlock, (sizeof dataBlock / sizeof dataBlock[0]), 3);
 *
 * When control returns from spi_block_xchg dataBlock would hold the data from the EEPROM.
 *
 * Parameters:
 *     txBuffer: pointer to buffer that holds the data to be transmitted
 *     txSize: the number of bytes
 *
 */
void spi_block_xchg(const uint8_t* txBuffer, int16_t txSize, uint8_t* rxBuffer, int16_t rxSize, uint8_t rxDelay)
{
	for (int16_t idx = 0; idx < (rxSize + rxDelay); idx++) {
		uint8_t txByte = 0x55;
		if (idx < txSize) txByte = *(txBuffer + idx);
		uint8_t rxByte = spi_xchg(txByte);
		if ((idx - rxDelay) >= 0) {
			*(rxBuffer + (idx - rxDelay)) = rxByte;
		}
	}
}

/* Exchange a byte
 *
 * Parameters:
 *     data: the Data byte to be sent
 * Returns
 *     The byte read back from SPI
 */
uint8_t spi_xchg(uint8_t data) {
#if defined (SPI_DEBUG)
	xprintf(PSTR("Sending byte: 0x%X\n"), data);
#endif
	SPDR = data;
	loop_until_bit_is_set(SPSR, SPIF);
	uint8_t read = SPDR;
#if defined (SPI_DEBUG)
	xprintf(PSTR("Read byte: 0x%X\n"), read);
#endif
	return read;
}

