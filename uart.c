// ----------------------------------------------------------------------------
/* for NerdKits with ATmega168, 14.7456 MHz clock
 * mrobbins@mit.edu
 * Modifications copyright (c) 2019 Jeff Rosen
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

#include <stdlib.h>
#include <avr/io.h>
#include "uart.h"

//static FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void uart_init(long baud) {

  uint16_t baud_setting;
  int use_u2x = 1;

#if F_CPU == 16000000UL
  if (baud == 57600l) use_u2x = 0;
#endif

  if (use_u2x) {
    UCSR0A = _BV(U2X0);
    baud_setting = (F_CPU / 8 / baud) - 1;
  } else {
    UCSR0A = 0;
    baud_setting = (F_CPU / 16 / baud) - 1;
  }

  // set baud rate
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;

  // enable uart RX and TX
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  // set 8N1 frame format
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

  // set up STDIO handlers so you can use printf, etc
  fdevopen(&uart_putchar, &uart_getchar);
  //uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
  //  stdin = stdout = &uart_stream;
}


void uart_write(char x) {
  // wait for empty receive buffer
  while ((UCSR0A & (1<<UDRE0))==0);
  // send
  UDR0 = x;
}

uint8_t uart_char_is_waiting() {
  // returns 1 if a character is waiting
  // returns 0 if not
  return (UCSR0A & (1<<RXC0));
}

char uart_read() {
  // wait
  while(!uart_char_is_waiting());
  char x = UDR0;
  return x;
}

int uart_putchar(char c, FILE *stream) {
  uart_write(c);
  return 0;
}

int uart_getchar(FILE *stream) {
  int x = uart_read();
  return x;
}




