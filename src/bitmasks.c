#include "bitmasks.h"
#include "utils.h"
#include "colours.h"


void matrix_write_letter(char letter,const char* background_colour,const char* letter_colour){
    Colour background_colour_rgb = get_colour(background_colour);
    Colour letter_colour_rgb = get_colour(letter_colour);
    // iterate through each row and column of the bitmask and display the corresponding pixel on the LED matrix
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (charset[letter][row] & (1 << col)) {
                // the current bit is set, so set the pixel to the letter colour
                update_LED_color_rgb(row, col, letter_colour_rgb.red,letter_colour_rgb.green,letter_colour_rgb.blue);
            } else {
                // the current bit is not set, so set the pixel to the background colour
                update_LED_color_rgb(row, col, background_colour_rgb.red,background_colour_rgb.green,background_colour_rgb.blue);
            }
        }
    }
}