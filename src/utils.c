/*
 * utils.c
 *
 * Created: 29. 3. 2023 22:50:34
 *  Author: denyz
 */ 
#include "utils.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


static inline uint8_t get_pwm_bits(uint8_t value){
	uint16_t num_of_bits = (value*8)/255;
	switch (num_of_bits)
	{
	case 8:
		return 0b11111111;
		break;
	case 7:
		return 0b01111111;
		break;
	case 6:
		return 0b00111111;
		break;
	case 5:
		return 0b00011111;
		break;
	case 4:
		return 0b00001111;
		break;
	case 3:
		return 0b00000111;
		break;
	case 2:
		return 0b00000011;
		break;
	case 1:
		return 0b00000001;
		break;
	default:
		return 0;
		break;
	}
}


inline void SPI_transfer(uint8_t data){
	SPDR = data;
	while(!(SPSR & (1 << SPIF)));
}
void write_row_to_spi_rgb(uint8_t pwm_level,uint8_t row){
	for(int register_no = BYTES_PER_ROW-1; register_no>=0; register_no--){
		SPI_transfer(matrix_rgb[pwm_level][row][register_no]);
	}
}



void update_LED_color_r(uint8_t row, uint8_t column, uint8_t red) {
	red = get_pwm_bits(red);
    for (uint8_t i = 0; i < PWM_SIZE; i++) {
        uint8_t pwm_bit = (red >> i) & 1;
        uint8_t byte_position = (column * 3) / 8;
        matrix_rgb[i][row][byte_position] &= ~(1 << ((column*3)&7));
        matrix_rgb[i][row][byte_position] |= ((1 & pwm_bit)<<((column*3)&7));
    }
}
void update_LED_color_g(uint8_t row, uint8_t column, uint8_t green) {
	green = get_pwm_bits(green);
    for (uint8_t i = 0; i < PWM_SIZE; i++) {
        uint8_t pwm_bit = (green >> i) & 1;
        uint8_t byte_position = (column * 3 + 1) / 8;
        matrix_rgb[i][row][byte_position] &= ~(1 << ((column*3 + 1)&7));
        matrix_rgb[i][row][byte_position] |= ((1 & pwm_bit) << ((column*3 + 1)&7));
    }
}
void update_LED_color_b(uint8_t row, uint8_t column, uint8_t blue) {
	blue = get_pwm_bits(blue);
    for (uint8_t i = 0; i < PWM_SIZE; i++) {
        uint8_t pwm_bit = (blue >> i) & 1;
        uint8_t byte_position = (column * 3 + 2) / 8;
        matrix_rgb[i][row][byte_position] &= ~(1 << ((column*3 + 2)&7));
        matrix_rgb[i][row][byte_position] |= ((1 & pwm_bit) << ((column*3 + 2)&7));
    }
}

inline void update_row_rgb(uint8_t row, uint8_t red, uint8_t green, uint8_t blue){
	for (uint8_t col = 0; col < NUM_OF_COLS; col++)
	{
		update_LED_color_rgb(row,col,red,green,blue);
	}
}
inline void update_col_rgb(uint8_t col, uint8_t red, uint8_t green, uint8_t blue){
	for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
	{
		update_LED_color_rgb(row,col,red,green,blue);
	}
}
inline void update_matrix_rgb(uint8_t red, uint8_t green, uint8_t blue){
	for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
	{
		for (uint8_t col = 0; col < NUM_OF_COLS; col++)
		{
			update_LED_color_rgb(row,col,red,green,blue);
		}	
	}
}
inline void update_LED_color_rgb(uint8_t row, uint8_t column, uint8_t red, uint8_t green, uint8_t blue){
	update_LED_color_r(row,column,red);
	update_LED_color_g(row,column,green);
	update_LED_color_b(row,column,blue);
}




void update_LED_color_hsv(uint8_t row, uint8_t column, uint8_t hue, uint8_t saturation, uint8_t value) {
	// Convert HSV to RGB
	uint8_t r, g, b;
	hsv_to_rgb(hue, saturation, value, &r, &g, &b);

	// Update LED color using RGB values
	update_LED_color_rgb(row, column, r, g, b);
}
void update_row_hsv(uint8_t row,uint8_t hue, uint8_t saturation, uint8_t value){
	for (uint8_t col = 0; col < NUM_OF_COLS; col++)
	{
		update_LED_color_hsv(row,col,hue,saturation,value);
	}
}
void update_col_hsv(uint8_t col,uint8_t hue, uint8_t saturation, uint8_t value){
	for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
	{
		update_LED_color_hsv(row,col,hue,saturation,value);
	}
}
void update_matrix_hsv(uint8_t hue, uint8_t saturation, uint8_t value){
	for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
	{
		for (uint8_t col = 0; col < NUM_OF_COLS; col++)
		{
			update_LED_color_hsv(row,col,hue,saturation,value);
		}	
	}
}


void USART_transmit(uint8_t byte)
{
    while (!(UCSR0A & (1 << UDRE0))); // wait for the transmit buffer to be empty
    UDR0 = byte; // write the byte to the USART data register
}

void USART_transmit_dec(int number){
	//Send sensor value to PC
    char buffer[5];
    itoa(potentiometer_value, buffer, 10);
    strcat(buffer, "\n");
    for (int i = 0; i < strlen(buffer); i++) {
      USART_transmit(buffer[i]);
    }
}

void USART_transmit_string(uint8_t *str){
	while (*str) {
		USART_transmit(*str++);
	}
}


void hsv_to_rgb(uint8_t hue, uint8_t saturation, uint8_t value, uint8_t* r, uint8_t* g, uint8_t* b) {
	// Convert HSV to RGB algorithm
	double h = (double)hue / 256.0;
	double s = (double)saturation / 256.0;
	double v = (double)value / 256.0;

	double c = v * s;
	double h_deg = h * 360.0;
	double x = c * (1 - fabs(fmod(h_deg / 60.0, 2) - 1));

	double r1, g1, b1;

	if (h_deg >= 0 && h_deg < 60) {
		r1 = c;
		g1 = x;
		b1 = 0;
		} else if (h_deg >= 60 && h_deg < 120) {
		r1 = x;
		g1 = c;
		b1 = 0;
		} else if (h_deg >= 120 && h_deg < 180) {
		r1 = 0;
		g1 = c;
		b1 = x;
		} else if (h_deg >= 180 && h_deg < 240) {
		r1 = 0;
		g1 = x;
		b1 = c;
		} else if (h_deg >= 240 && h_deg < 300) {
		r1 = x;
		g1 = 0;
		b1 = c;
		} else {
		r1 = c;
		g1 = 0;
		b1 = x;
	}

	double m = v - c;
	*r = round((r1 + m) * 255);
	*g = round((g1 + m) * 255);
	*b = round((b1 + m) * 255);
}
void delay_ms(uint16_t ms)
{
    // calculate the number of clock cycles per millisecond
    uint32_t cycles = F_CPU / 1000;
    
    // loop for the specified number of milliseconds
    while (ms--) {
        // wait for one millisecond
        _delay_loop_2(cycles);
    }
}


inline void clear_matrix(){
	update_matrix_rgb(0,0,0);
}