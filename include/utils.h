/*
 * utils.h
 *
 * Created: 29. 3. 2023 18:42:05
 *  Author: denyz
 */ 

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include "defines.h"
void SPI_transfer(uint8_t data);



void update_LED_color_r(uint8_t row, uint8_t column, uint8_t red);
void update_LED_color_g(uint8_t row, uint8_t column, uint8_t green);
void update_LED_color_b(uint8_t row, uint8_t column, uint8_t blue);
void update_LED_color_rgb(uint8_t row, uint8_t column, uint8_t red, uint8_t green, uint8_t blue);
void hsv_to_rgb(uint8_t hue, uint8_t saturation, uint8_t value, uint8_t* r, uint8_t* g, uint8_t* b);
void update_row_rgb(uint8_t row, uint8_t red, uint8_t green, uint8_t blue);
void update_col_rgb(uint8_t col, uint8_t red, uint8_t green, uint8_t blue);



void update_LED_color_hsv(uint8_t row, uint8_t column, uint8_t hue, uint8_t saturation, uint8_t value);
void update_matrix_hsv(uint8_t hue, uint8_t saturation, uint8_t value);
void update_col_hsv(uint8_t col,uint8_t hue, uint8_t saturation, uint8_t value);
void update_row_hsv(uint8_t row,uint8_t hue, uint8_t saturation, uint8_t value);



void USART_transmit_dec(int number);
void USART_transmit(uint8_t byte);
void USART_transmit_string(uint8_t *string);



void clear_matrix();

void update_matrix_rgb(uint8_t red, uint8_t green, uint8_t blue);
void write_row_to_spi_rgb(uint8_t pwm_level,uint8_t row);
void delay_ms(uint16_t ms);
extern volatile uint8_t potentiometer_value;
extern uint8_t matrix_rgb[PWM_SIZE][NUM_OF_ROWS][BYTES_PER_ROW];

#endif /* UTILS_H_ */
