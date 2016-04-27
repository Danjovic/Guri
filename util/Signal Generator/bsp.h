/*
	Board Support Package for AVR-Fun prototyping board
	
	Daniel Jose Viana - 2015
		
	This library contains drivers for the following peripherals
	
	 # Peripheral        Version   Date    Status
	 1 LCD 16x2             -    20150815   pre
	 2 LEDS 1-5             -    20150815    ok  
	 3 Buttons 1-5          -    20150816    ok
	 4 RS232                -    20150816    ok
	 5 RS485                -    20150816   pre
	 6 Buzzer/Audio         -    20150817   pre
	 7 Video
	 8 SD Card 
	 9 DS2433 (1wire)
	10 PCF8563 (RTC I2C)
	11 24C256 (EEPROM I2C)
	12 IR emitter
	13 IR Receiver
	14 LDR
	15 PS/2 Keyboard

*/

// Definitions: change according to the project

#define USE_LCD     1
#define USE_LEDS    0
#define USE_BUTTONS 1
#define USE_RS232   0
#define USE_RS485   0
#define USE_AUDIO   0
#define USE_VIDEO   0
#define USE_SDCARD  0
#define USE_1WIRE   0
#define USE_I2C     0
#define USE_IR_SEND 0
#define USE_IR_RECV 0
#define USE_LDR     0
#define USE_PS2     0



#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef USART_BAUDRATE
#define USART_BAUDRATE 9600
#endif



#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>




///////////////////////////////////////////////////////////////////////////////////////////////////
// LCD 
//
#if USE_LCD==1

#define LCD_PORT PORTC
#define LCD_DDR  DDRC

#define LCD_D7_PIN	PC7
#define LCD_D6_PIN	PC6
#define LCD_D5_PIN	PC5
#define LCD_D4_PIN	PC4
#define LCD_RS_PIN  PC3
#define LCD_E_PIN	PC2


// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// LCD Functions
void LCD_Init(void);
void LCD_putc(uint8_t c);
void LCD_puts(uint8_t *s);

void LCD_Write_Nibble(uint8_t nibble);
void LCD_Write_Command(uint8_t command);
void LCD_Write_Data(uint8_t Data);
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////
// LEDS
//
#if USE_LEDS==1

#define LED_PORT PORTC
#define LED_DDR  DDRC

#define LED1 (1<<7) // led 1 at pin C7
#define LED2 (1<<6) // led 2 at pin C6
#define LED3 (1<<5) // led 3 at pin C5
#define LED4 (1<<4) // led 4 at pin C4
#define LED5 (1<<3) // led 5 at pin C3

void Led_ON (uint8_t led);
void Led_OFF (uint8_t led);

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// BUTTONS
//
#if USE_BUTTONS==1

#define BUTTON_PIN  PINA
#define BUTTON_PORT PORTA
#define BUTTON_DDR  DDRA

#define BUTTON1 (1<<7)  // button 1 at pin A7
#define BUTTON2 (1<<6)  // button 1 at pin A6
#define BUTTON3 (1<<5)  // button 1 at pin A5
#define BUTTON4 (1<<4)  // button 1 at pin A4
#define BUTTON5 (1<<3)  // button 1 at pin A3

uint8_t Read_Button(uint8_t button);

#endif
 
///////////////////////////////////////////////////////////////////////////////////////////////////
// RS232
//
#if USE_RS232==1

#define USART_PORT PORTD
#define USART_DDR  DDRD

#define USART_TX_PIN (1<<1)  // TX at pin D1
#define USART_RX_PIN (1<<0)  // RX at pin D0

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define OCR0_VALUE (F_CPU / (2 * 64 * 1000 ))-1;


void USARTInit(void);
int  USARTSendByte(char u8Data, FILE *stream);
int  USARTReceiveByte(FILE *stream);
uint8_t kbhit(void);

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// RS485
//
#if USE_RS485==1

#define RS485_TXEN_PORT PORTD
#define RS485_TXEN_DDR  DDRD

#define RS485_TXEN_PIN (1<<4)  // TX Enable at PIN D4

#define RS485_Enable_TX() RS485_TXEN_PORT |=  RS485_TXEN_PIN
#define RS485_Enable_RX() RS485_TXEN_PORT &= ~RS485_TXEN_PIN

void RS485_USARTInit(void);

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// BUZZER/AUDIO
//
#if USE_AUDIO==1

#define AUDIO_PORT PORTB
#define AUDIO_DDR  DDRB

#define AUDIO_PIN (1<<3)  // Audio Out at PIN B3

void AUDIO_Init(void);
void Sound(uint8_t note, uint16_t time_ms);  // blocking!
void Note(uint8_t  note);
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// VIDEO
//
#if USE_VIDEO==1
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////
// LDR
//
#if USE_LDR==1
#define LDR_PORT PORTA
#define LDR_DDR  DDRA
#define LDR_PIN  PINA

#define LDR_CH   2   // LDR At pin PA2 
uint8_t Read_LDR8(void);



#endif

