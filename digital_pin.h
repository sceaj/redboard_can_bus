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

#ifndef DIGITAL_PIN_H_
#define DIGITAL_PIN_H_

#include <stdio.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

typedef enum { AVR_PORTA, AVR_PORTB, AVR_PORTC, AVR_PORTD } avr_port;
typedef enum { INPUT = 2, INPUT_WITH_PULLUP = 3, OUTPUT_INIT_LOW = 4, OUTPUT_INIT_HIGH = 5 } pin_mode;

typedef struct avr_port {
  volatile uint8_t* ctrlReg;
  volatile uint8_t* portReg;
  volatile uint8_t* pinReg;
} avr_port_t;

extern avr_port_t avrPortB;
extern avr_port_t* portB;
extern avr_port_t avrPortC;
extern avr_port_t* portC;
extern avr_port_t avrPortD;
extern avr_port_t* portD;

void disable_pullups(int disableAll);

void configure_pin(avr_port_t* port, uint8_t pin, pin_mode mode);

uint8_t read_pin(avr_port_t* port, uint8_t pin);

void write_pin(avr_port_t* port, uint8_t pin, int on);

#endif /* DIGITAL_PIN_H_ */
