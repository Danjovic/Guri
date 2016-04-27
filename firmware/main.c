/*
   Guri: Ignition Timing Controller for VW Air Cooled engines
   
   Danjovic 2016 - Danjovic@hotmail.com
 

                    ATMega8
                   +---_---+  
        nRESET/PC6-|1    28|- PC5/ADC5/SCL - POT2
   DEBUG - RXD/PD0-|2    27|- PC4/ADC4/SDA - POT1
   DEBUG - TXD/PD1-|3    26|- PC3/ADC3 - VBAT
 IGNTSTS -INT0/PD2-|4    25|- PC2/ADC2
 OVERCUR  INT1/PD3-|5    24|- PC1/ADC1
        XCK\T0/PD4-|6    23|- PC0/ADC0 - VACUUM
               VCC-|7    22|- GND 
               GND-|8    21|- AREF
           XT1/PB6-|9    20|- AVCC
           XT2/PB7-|10   19|- PB5/SCK
            T1/PD5-|11   18|- PB4/MISO
          AIN0/PD6-|12   17|- PB3/MOSI/OC2
          AIN1/PD7-|13   16|- PB2/nSS/OC1B - FF_CLK
          ICP1/PB0-|14   15|- PB1/OC1A - FF_DAT
                   +-------+
   
*/

#define DEBUG

// *******************************Libraries************************
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/wdt.h> 

#ifdef DEBUG
#include <stdio.h>
#include "uart.h"
#include "uart.c"

#endif
// *******************************Definitions**********************
#define FF_CLK (1<<PB2)
#define FF_DAT (1<<PB1)

//


//F_CPU 12000000

#define TIME_FACTOR 64000000UL/F_CPU  // us
#define RPM_FACTOR  7500000UL
#define MIN_RPM      150
#define MAX_RPM     6200

#define _t_max_rpm  RPM_FACTOR/MAX_RPM     
#define _t_min_rpm  62500    //  120 RPM
#define _t_base     2000 / TIME_FACTOR // 2ms
#define _t_burn     500 / TIME_FACTOR  // 0,5ms 

#define _v_8V       114   // ADC reading for 8Volts
#define _v_15V      214   // ADC reading for 15Volts
#define _delta_Vbat ( _v_15V - _v_8V ) // delta


#define _v_127mmHg       114   // ADC reading for 127.5 mmHg
#define _v_270mmHg      214   // ADC reading for 270 mmHg
#define _delta_Vacuum ( _v_270mmHg - _v_127mmHg ) // delta


#define _v_factor      8    // Multiply factor to reduce loss of bits
                         // in the interpolation
#define _v_interval    15   // ending interval 


// *******************************Variables************************
static volatile uint8_t Vbat=0;
static volatile uint8_t Vacuum=0;


const uint16_t PROGMEM Tcharge[16] = { 600,575,550,525,500,475,450,425,400,375,350,325,300,275,250,225 };
const uint16_t PROGMEM Tvacuum[16] = {   0, 25, 50, 75,100,125,150,175,200,225,250,275,300,325,350,375 };
//const uint16_t PROGMEM constants[] = {_t_max_rpm, _t_min_rpm, _t_base, _t_burn } ;

// *******************************Function Prototypes**************
void init_hw(void);
uint16_t read_adc(uint8_t channel);


// *******************************Main Loop************************
int main (void) {

	uint8_t v;
	uint16_t temp16;

	init_hw();      // Initialize hardware
	                // Initialize variables
#ifdef DEBUG
	uart_init();    // Initialize USART, 19200 8N1
    stdout = &uart_output;
    stdin  = &uart_input;
	printf_P(PSTR("Guri:Debug mode\n"));
			   
#endif			   


    while(1) {
        puts("Press a Key");
        v = getchar();
		temp16=read_adc(0); printf("vacuum: %u\n",temp16);  
		temp16=read_adc(3); printf("vbat  : %u\n",temp16);  
		temp16=read_adc(4); printf("pot1  : %u\n",temp16);  
		temp16=read_adc(5); printf("pot2  : %u\n",temp16); 
 		
    }
		   
		   
			   
	//sei();     // start interrupts
	
	wdt_reset();
	wdt_disable();
	wdt_enable(WDTO_500MS);
	
	for (;;) {
		// reset watchdog
		wdt_reset();
		
		// read Vbat
		ADCSRA &= ~7; // select channel 0 (VBAT)
		ADCSRA |= ADSC; // start conversion
		while  	bit_is_set(ADCSRA, ADSC); // wait end of conversion
		v = ADCH; // read Battery voltage
		if (v < _v_8V ) v  =  _v_8V;  // low value limit
		if (v > _v_15V ) v = _v_15V;  // high value limit
		// convert value into index to lookup table		
		temp16 = (v - _v_8V) * ( _v_factor * _v_interval);
		temp16 = temp16 / (_v_factor * _delta_Vbat);
		v = (uint8_t)temp16;
		Vbat = v;
		
	
		// read Vaccum sensor
		ADCSRA |= 1; // select channel 1 (VACUUM)
		ADCSRA |= ADSC; // start conversion
		while  	bit_is_set(ADCSRA, ADSC); // wait end of conversion
		v = ADCH; // read vacuum at Manifold	
		if (v < _v_127mmHg ) v  =  _v_127mmHg;  // low value limit
		if (v > _v_270mmHg ) v = _v_270mmHg;  // high value limit
		// convert value into index to lookup table
		temp16 = (v - _v_127mmHg) * ( _v_factor * _v_interval);
		temp16 = temp16 / (_v_factor * _delta_Vacuum);
		v = (uint8_t)temp16;		
		Vacuum = v;
		
	}

}

// ******************************Initialize Hardware***************
void init_hw(void){
	
	// Initialize I/O
	DDRB = (FF_CLK | FF_DAT);
	PORTB = 0;
	PORTB |= FF_CLK;  // pulse clock to turn off coil 
	PORTB = 0;
	
	// Initialize ADC
	ADMUX =  (1<<REFS0); // Aref=AVcc
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescalar div factor =128
  
	
	// Initialize Timer
	TCCR1A=0;          // Timer 1 in Normal mode, OC1A/B disconnected
	TCNT1  = 0;        // Clear Timer1
	TCCR1B=(3<CS10);   // Prescaler 64, overflow 262ms
	
	// Initialize Interrupts
	TIMSK = (1<<TOIE1) + (1<<OCIE1A) + (1<<OCIE1B) ;// Overflow, compare A and B
	MCUCR = (3<<ISC01);// INT0 on rising edge
	GICR  = (1<<INT0); // INT0  
	//sei();
}

// ******************************Perform one Analog reading********
uint16_t read_adc(uint8_t channel) {
		ADMUX &= ~7;     //clear 3 lsb from channel selection
		ADMUX |= ( channel & 7); // range check in argument
		ADCSRA |= (1<<ADSC); // start conversion
		while  	bit_is_set(ADCSRA, ADSC); // wait end of conversion
		return ADC; // read Battery voltage, 8 bits
}

// ******************************Timer 1 Overflow******************
ISR (TIMER1_OVF_vect) {
	
	PORTB &= ~FF_DAT;   // Turn off coil and timers
	asm ("nop");		
	PORTB |= FF_CLK;
	asm ("nop");
	PORTB &= ~FF_CLK;
	
	TCCR1B = 0;           // Stop Timer1
	TCNT1  = 0;           // Clear Timer1
	
	TIMSK = (1<<TOIE1);	// Disable compare interrupts
}

// ******************************Timer 1 Compare A*****************
ISR (TIMER1_COMPA_vect) {     // Start Charging the Coil 
	PORTB |= FF_DAT;
	asm ("nop");	
	PORTB |= FF_CLK;
	asm ("nop");
	PORTB &= ~FF_CLK;
}

// ******************************Timer 1 Compare B*****************
ISR (TIMER1_COMPB_vect) {     // Stop Charge, generate Spark
	PORTB &= ~FF_DAT;
	asm ("nop");	
	PORTB |= FF_CLK;
	asm ("nop");
	PORTB &= ~FF_CLK;
}

// ******************************External Interrupt****************
ISR (INT0_vect) {             // Measure Tcycle
	uint16_t t_cycle=TCNT1;
	TCNT1=0;
	TCCR1B=(3<CS10);   // (re)start timer, Prescaler 64	
	
	PORTB &= ~FF_DAT;  // turn off coil if we reach here 
	asm ("nop");	   // before Compare B interrupt
	PORTB |= FF_CLK;
	asm ("nop");
	PORTB &= ~FF_CLK;
	
	
	// Check if RPM is valid. This function has a side effect of
	// limiting maximum RPM by not programming a spark either on 
	// too high or too low engine speeds
	if ((t_cycle > _t_max_rpm) && (t_cycle < _t_min_rpm) ) { 
	
		// compute spark time
		t_cycle -= _t_base ;                    // base time
		t_cycle -= pgm_read_word (&Tvacuum[Vacuum]); // vaccum advance time
		OCR1B = t_cycle;
		
		// compute Start of charge time but leave room for the mixture 
		// to burn. This may reduce coil charging time.
		t_cycle -= pgm_read_word(&Tcharge[Vbat]);  // compute charge time
		if (t_cycle < _t_burn) t_cycle = _t_burn; // leave room
		OCR1A = t_cycle;
		
		// (re)program compare interrupts
		TIMSK = (1<<TOIE1) + (1<<OCIE1A) + (1<<OCIE1B) ;
		
	} else { 
		// Disable compare interrupts
		TIMSK = (1<<TOIE1);
	}	

}
