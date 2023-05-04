#ifndef BITMASKS_H
#define BITMASKS_H

#include <stdint.h>

void matrix_write_letter(char letter, const char* background_colour, const char* letter_colour);
uint8_t* get_letter_bits(char letter);
const char charset[128][8];
#endif // BITMASKS_H
