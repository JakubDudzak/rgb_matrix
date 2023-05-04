#include <util/delay.h>
#include "utils.h"
#include "defines.h"
#include "animations.h"
#define ROW 4

static enum choices{
	LCD,
	LCD_USART,
	USART
};
#define WHERE_TO_SEND LCD

static char buffer[5] = "";
static inline void send_potentiometer_value(uint8_t choice){
	itoa(potentiometer_value, buffer, 10);
	switch (choice)
	{
	case LCD:
		lcd_clear();
		lcd_send_string(0,0,buffer);
		_delay_ms(50);
		break;
	case USART:
    	strcat(buffer, "\n");
		USART_transmit_string(buffer);
		buffer[strlen(buffer) - 1] = '\0';
		break;
	case LCD_USART:
		lcd_clear();
		lcd_send_string(0,0,buffer);
		_delay_ms(50);
		strcat(buffer, "\n");
		USART_transmit_string(buffer);
		buffer[strlen(buffer) - 1] = '\0';
		break;
	default:
		break;
	}
		
}

void kolotoc(uint8_t brightness){
	for(uint8_t row = 0; row<NUM_OF_ROWS;row++){
		for(uint8_t column = 0; column < NUM_OF_COLS;column++){
			update_LED_color_rgb(row,column,brightness,0,0);
			delay_ms(potentiometer_value);
		}
		for(uint8_t column = 0; column < NUM_OF_COLS;column++){
				update_LED_color_rgb(row,column,0,brightness,0);
				delay_ms(potentiometer_value);
		}
		for(uint8_t column = 0; column < NUM_OF_COLS;column++){
				update_LED_color_rgb(row,column,0,0,brightness);
				delay_ms(potentiometer_value);
		}
	}
}
void pulsing_animation(uint8_t brightness) {
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blue = 0;
	uint8_t step = 5;
	while (red < brightness) {
		for (uint8_t col = 0; col < NUM_OF_COLS; col++) {
			for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
			{
				update_LED_color_rgb(row, col, red, green, blue);
			}
		}
		red += step;
		delay_ms(potentiometer_value);
	}

	while (red > 0) {
		for (uint8_t col = 0; col < NUM_OF_COLS; col++) {
			for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
			{
				update_LED_color_rgb(row, col, red, green, blue);
			}
		}

		red -= step;
		delay_ms(potentiometer_value);
	}
	// Set all LEDs to off
	for (uint8_t col = 0; col < NUM_OF_COLS; col++) {
		for (uint8_t row = 0; row < NUM_OF_ROWS; row++)
		{
			update_LED_color_rgb(row, col, 0, 0, 0);
		}
	}
}
void rainbow_animation(uint8_t brightness) {
    // amount to shift hue value on each iteration
    uint8_t hue[NUM_OF_COLS]; // initial hue values for each LED
    for (uint8_t i = 0; i < NUM_OF_COLS; i++) {
        hue[i] = i * (255 / NUM_OF_COLS);
	}
    
    for (uint8_t i = 0; i < 20; i++) {
        for (uint8_t row = 0; row < NUM_OF_ROWS; row++) {
            for (uint8_t col = 0; col < NUM_OF_COLS; col++) {
                hue[col] += 2; // shift hue value for each LED
                if (hue[col] >= 255) hue[col] = 0; // reset hue value if it exceeds 255
                update_LED_color_hsv(row, col, hue[col], 255, brightness); // set LED color with the updated hue value and full saturation/brightness
            }
        }
		send_potentiometer_value(LCD);
        delay_ms(potentiometer_value/8); // adjust delay time as desired
    }
}

