/*
	Board Support Package for AVR-Fun prototyping board
	
	Daniel Jose Viana - 2015
	
	Functions Implementation

*/

#include "bsp.h"
#include "note_table.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// LCD Functions Implementation
// 
#if USE_LCD==1
void LCD_Init(void) {

	uint8_t LedState = LCD_PORT;  //preserve leds state

	// configure the microprocessor pins for data and control lines
    LCD_DDR = ( 1<<LCD_D7_PIN |1<<LCD_D6_PIN |1<<LCD_D5_PIN |1<<LCD_D4_PIN |1<<LCD_RS_PIN |1<<LCD_E_PIN );

	// initial Power-up delay
	_delay_ms(100);   

	LCD_PORT &= ~(1<<LCD_RS_PIN | 1<<LCD_E_PIN); // select Instruction Register (RS low)
												   // and make sure E is initially low
	// Reset the LCD controller  
    LCD_Write_Nibble(3);               
    _delay_ms(10);                        
    LCD_Write_Nibble(3);                
    _delay_us(200);                         
	LCD_Write_Nibble(3);                 
    _delay_us(200);                                
	
	// set 4-bit mode 
	LCD_Write_Nibble(2);              
    _delay_us(80);                     

	// Finish setup
    LCD_Write_Command(lcd_FunctionSet4bit);   // set mode, lines, and font
    _delay_us(80);                                

    LCD_Write_Command(lcd_DisplayOff);        // turn display OFF
    _delay_us(80);                             

    LCD_Write_Command(lcd_Clear);             // clear display RAM
    _delay_ms(4);                                

    LCD_Write_Command(lcd_EntryMode);         // Entry Mode and shift setup
    _delay_us(80); 

                                
	// finished the LCD controller, now turn the display back ON.
    LCD_Write_Command(lcd_DisplayOn);        
    _delay_us(80);                              
		
	LCD_PORT = LedState;           // restore Led states
}

void LCD_Write_Nibble(uint8_t nibble){

	uint8_t temp;

	temp = LCD_PORT & 0x0f;      // Preserve lower 4 bits from LCD Port 
	temp |= (nibble<<4);	      
	LCD_PORT = temp ;            // write nibble at higher 4 bits of LCD Port
  	
    LCD_PORT |= (1<<LCD_E_PIN); // Pulse E line  
    _delay_us(1);                      
	LCD_PORT &= ~(1<<LCD_E_PIN);    
    _delay_us(1);                       	
}

void LCD_Write_Command(uint8_t command ){
	LCD_PORT &= ~(1<<LCD_RS_PIN);    // RS low (Status/Command)
	LCD_PORT &= ~(1<<LCD_E_PIN);     // E low
	LCD_Write_Nibble (command>>4);   // write upper 4-bits
	LCD_Write_Nibble (command);      // write lower 4-bits
}


void LCD_Write_Data(uint8_t data){
    LCD_PORT |= (1<<LCD_RS_PIN);    // RS high (Data)
    LCD_PORT &= ~(1<<LCD_E_PIN);    // E low
    LCD_Write_Nibble (data >> 4);   // write upper 4-bits
    LCD_Write_Nibble (data);        // write lower 4-bits
}


void LCD_puts(uint8_t *s){
    	
    while (*s)
    {
        LCD_putc(*s);
        s++;
        _delay_us(80);                              // 40 uS delay (min)
    }

}


void LCD_putc(uint8_t c){
	
	//preserve leds state
	uint8_t LedState = LCD_PORT;
 	
	if (c=='\f') {
		    LCD_Write_Command(lcd_Clear);  // clear display RAM
			_delay_ms(4); 
	} else
		LCD_Write_Data(c);

	// restore Led states
	LCD_PORT = LedState;

}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// LED Functions Implementation
//
#if USE_LEDS==1
void Led_ON (uint8_t led){
	uint8_t temp = led & 0xf8; // discard bits 2..0
    LED_DDR |= ( LED1 | LED2 | LED3 | LED4 | LED5 );
	LED_PORT|=temp;
}

void Led_OFF (uint8_t led) {
	uint8_t temp = led & 0xf8; // discard bits 2..0
	LED_DDR |= ( LED1 | LED2 | LED3 | LED4 | LED5 );
	LED_PORT&=~(temp);
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
// BUTTONS Functions implementation
//
#if USE_BUTTONS==1
uint8_t Read_Button(uint8_t button){
		
	button &= 0xf8;         // mask button bits	
	BUTTON_DDR &= ~button;  // set desired bits as inputs
	BUTTON_PORT |= button;  // turn on pullups on required pins
	return ((BUTTON_PIN & button)==0);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// RS232 Functions Implementation
//
#if USE_RS232==1


void USARTInit(void)
{
  // setup IO Ports
  USART_DDR |=  USART_TX_PIN;   // set TX pin as output
  USART_DDR &=  ~USART_RX_PIN;  // set RX pin as input


  // Set baud rate
   UBRRH = (uint8_t)(UBRR_VALUE>>8);
   UBRRL = (uint8_t)UBRR_VALUE;
   // Set frame format to 8 data bits, no parity, 1 stop bit
   UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
   //enable transmission and reception
   UCSRB |= (1<<RXEN)|(1<<TXEN);
}

int USARTSendByte(char u8Data, FILE *stream)
{
   if(u8Data == '\n')
   {
      USARTSendByte('\r', 0);
   }
   //wait while previous byte is completed
   while(!(UCSRA&(1<<UDRE))){};
   // Transmit data
   UDR = u8Data;
   return 0;
}

int USARTReceiveByte(FILE *stream)
{
   uint8_t u8Data;
   // Wait for byte to be received
   while(!(UCSRA&(1<<RXC))){};
   u8Data=UDR;
   //echo input data
//   USART0SendByte(u8Data,stream);
   // Return received data
   return u8Data;

}

uint8_t kbhit (void)
{
    if (UCSRA&(1<<RXC))
        return 1;
    else
        return 0;
}

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
// RS485 Functions Implementation
//
#if USE_RS485==1

void RS485_USARTInit(void){
	RS485_TXEN_DDR|= RS485_TXEN_PIN;    // configure TXEN as output
	RS485_TXEN_PORT &= ~RS485_TXEN_PIN;  // dixable TXEN (start as RX)
	USARTInit();                        // start UART as in RS232C  
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// BUZZER/AUDIO
//
#if USE_AUDIO==1

void AUDIO_Init(void){

	// configure Audio Out pin as output
	AUDIO_DDR |= AUDIO_PIN;
	AUDIO_PORT &= ~AUDIO_PIN; // port default as low level

	// Timer0 in CTC mode, 
	TCCR0 = ( (1<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00) );

}

void Sound(uint8_t note, uint16_t time_ms){ // blocking!
	uint16_t t=time_ms;

	Note (note);
	while (t) {
		_delay_ms(1);
		t--;
	}

}

void Note(uint8_t  note){
	
	TCCR0 =0 ;     //  stop audio
	TCNT0 = 0;      
		
	if ( (note>11) && (note<=108) ) { // Valid note intervall 	
		if (note<=35) {
			TCCR0 |= ( (1<<CS02)|(0<<CS01)|(1<<CS00) ); // prescaler = 1024
	
		} else if ( (note<=59)){
			TCCR0 |= ( (1<<CS02)|(0<<CS01)|(0<<CS00) ); // prescaler = 256

		} else if ( (note<=96)) {
			TCCR0 |= ( (0<<CS02)|(1<<CS01)|(1<<CS00) ); // prescaler = 64	

		} else { // if note<=108 
			TCCR0 |= ( (0<<CS02)|(1<<CS01)|(0<<CS00) );// prescaler = 8
		
		}
		OCR0 = pgm_read_byte(&(note_table[note]));
		TCCR0|= ( (1<<WGM01)|(0<<WGM00)|(0<<COM01)|(1<<COM00) ); // turn on CTC mode
	} 
}
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

uint8_t Read_LDR8(void) {
	// Initialize ADC
  // init ADC
  uint8_t r;
  
  LDR_DDR &= ~(1<<LDR_CH);


  ADMUX =   (1 << ADLAR) |  LDR_CH ; 
  ADCSRA =  (1<<ADEN) | (7 << ADPS0) ;   // set prescaler to 128
  while (ADCSRA & (1 << ADSC) );      // wait for conversion to complete 
  r=ADCH;
  return r;
}



#endif
