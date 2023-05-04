#ifndef COLOURS_H_
#define COLOURS_H_

#include <stdint.h>

typedef struct {
    const char *name;
    uint8_t red, green, blue;
} Colour;



Colour get_colour(const char *colour_name);

#endif /* COLOURS_H_ */
