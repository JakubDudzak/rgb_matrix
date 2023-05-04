#include <avr/io.h>
#include "defines.h"
#include "setup.h"

inline void setup_timer(void){
	// Set up Timer 1 for column refreshing
	TCCR1A = 0b00000000; // Set Timer/Counter1 Control Register A to all 0s
	TCCR1B = 0b00001011; // Set Timer/Counter1 Control Register B:
	// - Bit 3 (WGM12) to 1 to set CTC mode and call interrupt on counter match
	// - Bits 0 and 1 (CS10 and CS11) to 1 to set clock prescaler to 64 (16MHz/64=250kHz)
	TIMSK1 = 0b00000010; // Set Timer/Counter1 Interrupt Mask Register:
	// - Bit 1 (OCIE1A) to 1 to enable interrupt on OCR1A match
	OCR1A = OCR_VAL;         // Set output compare register to 30 (interrupt will be called every (30+1)*4us=124us)
}
inline void setup_SPI(void){
	// Enable SPI, set as Master, and set clock rate to fck/2
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}
inline void setup_pins(void){
	DDRB |= (1 << LATCH_PIN) | (1 << DATA_PIN) | (1 << CLOCK_PIN)| (1 << BLANK_PIN); 	// Set MOSI, SCK, blank and latch as output
	DDRD |= (1 << LAYER_A) | (1 << LAYER_B) | (1 << LAYER_C);			 				// Set as output
	PORTB |= 1 << LATCH_PIN;
	PORTB &= ~((1 << LATCH_PIN)); 									// Set outputs to LOW (1 << LATCH_PIN) | 
	PORTD &= ~((1 << LAYER_A) | (1 << LAYER_B) | (1 << LAYER_C)); 						// Set outputs to LOW
}
inline void setup_ADC(void)
{
	ADMUX |= (1 << REFS0) | (1 << ADLAR); 					// use AVcc as the reference voltage and left-adjust the result
	ADMUX &= ~(0b00001111);                 				// select ADC input pin A0
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  	// ADC clock prescaler = 128
	ADCSRA |= (1 << ADIE);              					// enable ADC interrupt
	ADCSRA |= (1 << ADEN);             		 				// enable ADC
  	ADCSRA |= (1 << ADSC);               					// start first conversion
}
void setup_USART(uint16_t ubrr)
{
    UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

