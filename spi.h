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

#ifndef SPI_H_
#define SPI_H_

#include <inttypes.h>

typedef enum { MODE0 = 0x00, MODE1 = 0x04, MODE3 = 0x08, MODE4 = 0x0C } spi_mode;
typedef enum { F_DIV4, F_DIV16, F_DIV64, F_DIV128, F_DIV2, F_DIV8, F_DIV32 } spi_clock_div;

void spi_init(int master, spi_mode spiMode, spi_clock_div clkDivider);
void spi_set_clk_div(spi_clock_div clkDivider);
void spi_disable(void);

void spi_block_tx(const uint8_t* dataBuffer, uint16_t dataSize);
void spi_block_rx(uint8_t* dataBuffer, uint16_t dataSize);
void spi_block_xchg(const uint8_t* txBuffer, int16_t txSize, uint8_t* rxBuffer, int16_t rxSize, uint8_t rxDelay);

uint8_t spi_xchg(uint8_t data);

#endif /* SPI_H_ */
