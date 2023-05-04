#include "colours.h"
#include <stdint.h>



static Colour colours[] = {
    {"RED", 255, 0, 0},
    {"GREEN", 0, 255, 0},
    {"BLUE", 0, 0, 255},
    {"YELLOW", 255, 255, 0},
    {"CYAN", 0, 255, 255},
    {"MAGENTA", 255, 0, 255},
    {"ORANGE", 255, 128, 0},
    {"PURPLE", 128, 0, 255},
    {"LIME", 128, 255, 0},
    {"TEAL", 0, 128, 128},
    {"PINK", 255, 0, 128},
    {"OLIVE", 128, 128, 0},
    {"LAVENDER", 204, 153, 255},
    {"BROWN", 153, 102, 51},
    {"BEIGE", 255, 204, 153},
    {"MAROON", 128, 0, 0},
    {"NAVY", 0, 0, 128},
    {"FOREST_GREEN", 0, 128, 0},
    {"GRAY", 128, 128, 128},
    {"DARK_RED", 224, 0, 0},
    {"DARK_GREEN", 0, 64, 0},
    {"DARK_BLUE", 0, 0, 64},
    {"DARK_CYAN", 0, 64, 64},
    {"DARK_MAGENTA", 64, 0, 64},
    {"DARK_YELLOW", 128, 128, 0},
    {"LIGHT_YELLOW", 255, 255, 128},
    {"LIGHT_CYAN", 128, 255, 255},
    {"LIGHT_MAGENTA", 255, 128, 255},
    {"LIGHT_GREEN", 128, 255, 128},
    {"LIGHT_BLUE", 128, 128, 255},
    {"LIGHT_GRAY", 192, 192, 192},
    {"DARK_GRAY", 64, 64, 64},
    {"BEIGE2", 204, 204, 153},
    {"OLIVE_DRAB", 107, 142, 35},
    {"KHAKI", 240, 230, 140},
    {"SANDY_BROWN", 244, 164, 96},
    {"SIENNA", 160, 82, 45},
    {"CORAL", 255, 127, 80},
    {"SALMON", 250, 128, 114},
    {"TOMATO", 255, 99, 71},
    {"ORCHID", 218, 112, 214},
    {"THISTLE", 216, 191, 216},
    {"PLUM", 221, 160, 221},
    {"DEEP_PINK", 255, 20, 147},
};

Colour get_colour(const char *colour_name) {
    for (int i = 0; i < sizeof(colours)/sizeof(colours[0]); i++) {
        if (strcmp(colour_name, colours[i].name) == 0) {
            return colours[i];
        }
    }
    // Return black color if the input color name is not found in the COLORS array
    return (Colour){"BLACK", 0, 0, 0};
}
