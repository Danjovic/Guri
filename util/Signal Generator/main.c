/* Name: main.c
 * Project: Avr-Fun base project
 * Author: Daniel Jose Viana
 * Creation Date: 13/08/2015
 * Tabsize: 4	
 * Copyright: (c) 2007 by Daniel Jose Viana
 * License: GNU GPL v2 (see License.txt)
 * This Revision: 
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "bsp.h"



///////////////////////////////////////////
///

#define SNES_CLOCK   (1<<PB1)  // Pino CLOCK mantido em nível baixo para gerar sinal latch
                              // atraves de rede rc. Shift ocorre em pulsos de clock rapidos 
#define SNES_DATA    (1<<PB7)
 
#define SNES_CLOCK_LOW()	PORTB &= ~(SNES_CLOCK);
#define SNES_CLOCK_HIGH()	PORTB |= SNES_CLOCK;
#define SNES_GET_DATA()		(PINB & SNES_DATA)

#define MAX_REPORTS 4  // 4 gamepads 
#define REPORT_SIZE 3  // Size of report: ID + 2 bytes of data

#define uchar uint8_t
#define MAX_RPM 7000
#define MIN_RPM 150

static int16_t RPM=900;

ISR (TIMER1_COMPA_vect ) {
	PORTB |= (1<<3);
}

ISR (TIMER1_COMPB_vect ) {
	PORTB &= ~(1<<3);
}


void	show_RPM(uint16_t rpm){
	char st[6];
	
	LCD_puts ("\fRPM:\0");
	LCD_puts (itoa(rpm,st,10));
//	LCD_putc ("-");
//	LCD_puts (itoa((uint16_t)(7500000/rpm),st,10));
	
}

void add_RPM (int16_t incr_rpm){
	uint16_t T1_top,T1_mid;
	
	if (incr_rpm ==0)
		RPM=900;
	else 
		RPM += incr_rpm;
		
	if (RPM>MAX_RPM) RPM=MAX_RPM;
	if (RPM<MIN_RPM) RPM=MIN_RPM;
	

    show_RPM (RPM);


	T1_top = (uint16_t)(7500000/RPM);
	T1_mid = (uint16_t)(2500000/RPM);
	
	cli(); 
	// Stop Timer1
		TCCR1B = (1<<WGM12);
	TCNT1 = 0;
	
	// Change values
	OCR1A = T1_top;
	OCR1B = T1_mid;
	
	// Restart Timer1
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10) ;
	TCNT1 = 0;
	
	TIFR = (1<<OCF1A) | (1<<OCF1B); // clear any pending interrupts
	
	sei();
	
	
	
}
	
int main (void) {


	uint8_t tb1,tb2,tb3,tb4,tb5=0;
	
	
	// Setup output pin (PB3)
	DDRB |= (1<<3);
	PORTB &= ~(1<<3);
	
	// Setup Timer1, prescaler 64, CTC mode
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10) ;
	TCNT1 = 0;
	
	TIMSK = (1<<OCIE1A) | (1<<OCIE1B);
	
	
	
	LCD_Init();
    add_RPM(0);
	show_RPM(RPM);
	sei();
	
	
	// Main loop
	for (;;) {
		// action for button 1
		if (Read_Button(BUTTON1)) {
			if (tb1<255) tb1++;
		
		} else { // release
			if (tb1 > 10) {
				add_RPM(-100);
			}
			tb1=0;
		}

		// action for button 2
		if (Read_Button(BUTTON2)) {
			if (tb2<255) tb2++;
		
		} else { // release
			if (tb2 > 10) {
				add_RPM(-50);
			}
			tb2=0;
		}

		// action for button 3
		if (Read_Button(BUTTON3)) {
			if (tb3<255) tb3++;
		
		} else { // release
			if (tb3 > 10) {
				add_RPM(0);  // set 900 RPM 
			}
			tb3=0;
		}

		// action for button 4
		if (Read_Button(BUTTON4)) {
			if (tb4<255) tb4++;
		
		} else { // release
			if (tb4 > 10) {
				add_RPM(50);
			}
			tb4=0;
		}

		// action for button 5
		if (Read_Button(BUTTON5)) {
			if (tb5<255) tb5++;
		
		} else { // release
			if (tb5 > 10) {
				add_RPM(100);
			}
			tb5=0;
		}
	
		_delay_ms(10);
	
	
	}



}