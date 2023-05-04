#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "setup.h"
#include "i2c_lcd.h"
#include "utils.h"
#include "animations.h"
#include "bitmasks.h"
#include "colours.h"



//gets updated each time PWM_level reaches PWM_SIZE and becomes 0
volatile uint8_t potentiometer_value = 0;
uint8_t matrix_rgb[PWM_SIZE][NUM_OF_ROWS][BYTES_PER_ROW] = {0};



char received_str[20];
volatile uint8_t receiving_str = 0;

ISR(TIMER1_COMPA_vect){
	static uint8_t current_row=0;
	static uint8_t pwm_level=0;


	PORTB |= (1 << BLANK_PIN); //Blank pin HIGH
	current_row = (!pwm_level) ? ((current_row + 1) & ROWS_MAX) : current_row;
	write_row_to_spi_rgb(pwm_level,current_row);
	pwm_level = (pwm_level + 1) & PWM_MAX;
	PORTB |= (1 << LATCH_PIN);
	PORTB &= ~(1 << LATCH_PIN); // Latch that new value (up and down)
	PORTB &= ~(1 << BLANK_PIN);	// Turn LEDS on
	PORTD = (current_row)<<2;	//(PORTD & ~(7 << 2)) | ((current_row) << 2);
}
ISR(ADC_vect)
{
  	potentiometer_value = ADCH;
  	ADCSRA |= (1 << ADSC); // start next conversion
}
// ISR(USART_RX_vect) {
// 	static uint8_t index = 0;
// 	char received_byte = UDR0;
// 	if (index == NUM_OF_USART_BYTES) { // received_byte=='\n' if the newline character is received, terminate the string
// 		index = 0;
// 		receiving_str = 1; // set the flag to indicate that a string is being received
		
// 		} 
// 	else {
// 		uint8_t row = index / (8 * 3);
// 		uint8_t col = index % (8 * 3) / 3;
// 		if (index % 3 == 0)
// 			update_LED_color_r(row,col,received_byte);
// 		else if (index % 3 == 1)
// 			update_LED_color_g(row,col,received_byte);
// 		else if (index % 3 == 2)
// 			update_LED_color_b(row,col,received_byte);

// 		index++;
// 	}
// }

ISR(USART_RX_vect) {
	char c = UDR0;
	static uint8_t index = 0;
	if (c=='\r') { // if the newline character is received, terminate the string
		received_str[index] = '\0';
		index = 0;
		receiving_str = 1; // set the flag to indicate that a string is being received
		} else {
		received_str[index++] = c;
		if (index >= 20) {
			index = 0;
		}
	}
}

int main(void){
	cli();
	setup_pins();
	setup_SPI();
	setup_timer();
 	setup_USART(UBRR);
	setup_i2c();
	setup_lcd();
	setup_ADC();
	sei();

	matrix_write_letter('A',"RED","PURPLE");
	for (char i = 'A'; i <= 'Z'; i++)
	{
		matrix_write_letter(i,"GREEN","MAGENTA");
		_delay_ms(10);
	}
	
	while (1) {
		char letter_colour[15];
		char background_colour[15];
		char letter;
		if (receiving_str){
			sscanf(received_str,"%[^,],%c,%s",letter_colour,&letter,background_colour);
			matrix_write_letter(letter,background_colour,letter_colour);
			USART_transmit_string("\n\rletter colour is:    ");
			USART_transmit_string(letter_colour);
			USART_transmit_string("\n\rletter is:    ");
			USART_transmit(letter);
			USART_transmit_string("\n\rbackground colour s
			is:    ");
			USART_transmit_string(background_colour);
			USART_transmit_string("\n\r############################");
			lcd_clear();
			lcd_send_string(1,0,letter_colour);
			lcd_send_string(2,0,background_colour);
			lcd_send_char(1,15,letter);
			lcd_send_number_at_end(2,potentiometer_value);
			receiving_str = 0; // reset the flag to indicate that we're ready to receive a new message
		}
	}
	
}




 