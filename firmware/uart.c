#include <avr/io.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 19200
#endif
#include <util/setbaud.h>

/* based on http://www.cs.mun.ca/~rod/Winter2007/4723/notes/serial/serial.html */

// compatibility with ATMEGA8
#ifndef UBRR0H
#define UBRR0H UBRRH
#endif

#ifndef UBRR0L
#define UBRR0L UBRRL
#endif

#ifndef UCSR0A
#define UCSR0A UCSRA
#endif

#ifndef UCSR0B
#define UCSR0B UCSRB
#endif

#ifndef U2X0
#define U2X0 U2X
#endif

#ifndef RXEN0
#define RXEN0 RXEN
#endif

#ifndef TXEN0
#define TXEN0 TXEN
#endif

#ifndef UDR0
#define UDR0 UDR
#endif

#ifndef UDRE0
#define UDRE0 UDRE
#endif

#ifndef RXC0
#define RXC0 RXC
#endif

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif


#ifndef URSRC
	UCSRC=_BV(URSEL)|_BV(UCSZ1) | _BV(UCSZ0);  // Data format
#else
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
#endif
	
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */    
}

void uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

char uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}