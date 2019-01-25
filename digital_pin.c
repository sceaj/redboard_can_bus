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

#include "digital_pin.h"

avr_port_t avrPortB = {
    .ctrlReg = &DDRB,
	.portReg = &PORTB,
	.pinReg = &PINB
};
avr_port_t* portB = &avrPortB;

avr_port_t avrPortC = {
	.ctrlReg = &DDRC,
	.portReg = &PORTC,
	.pinReg = &PINC
};
avr_port_t* portC = &avrPortC;

avr_port_t avrPortD = {
	.ctrlReg = &DDRD,
	.portReg = &PORTD,
	.pinReg = &PIND
};
avr_port_t* portD = &avrPortD;


void disable_pullups(int disableAll) {
	if (disableAll) {
		MCUCR |= _BV(PUD);
	} else {
		MCUCR &= ~_BV(PUD);
	}
}

void configure_pin(avr_port_t* port, uint8_t pinNo, pin_mode mode) {
	*(port->portReg) &= ~_BV(pinNo);
	if (mode & INPUT) {
		*port->ctrlReg &= ~_BV(pinNo);
		if (mode & 0x01) {
			// Enable pin's internal pull-up
			*port->portReg |= _BV(pinNo);
		}
	} else if (mode & OUTPUT_INIT_LOW) {
		*port->ctrlReg |= _BV(pinNo);
		if (mode & 0x01) {
			*port->portReg |= _BV(pinNo);
		}
	}
}

uint8_t read_pin(avr_port_t* port, uint8_t pinNo) {
	return *port->pinReg & _BV(pinNo);
}

void write_pin(avr_port_t* port, uint8_t pinNo, int on) {
  if (on) {
    *port->portReg |= _BV(pinNo);
  } else {
    *port->portReg &= ~_BV(pinNo);
  }
}
