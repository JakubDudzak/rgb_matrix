#include "bitmasks.h"
#include "utils.h"
#include "colours.h"



static const uint8_t A[8] = { 
    0b00111100, 
    0b01100110, 
    0b11000011, 
    0b11000011, 
    0b11111111, 
    0b11000011, 
    0b11000011, 
    0b11000011 
};

static const uint8_t B[8] = { 
    0b11111110, 
    0b11000011, 
    0b11000011, 
    0b11111110, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11111110 
};

static const uint8_t C[8] = { 
    0b00011110, 
    0b01100011, 
    0b11000001, 
    0b11000000, 
    0b11000000, 
    0b11000001, 
    0b01100011, 
    0b00011110 
};

static const uint8_t D[8] = { 
    0b11111100, 
    0b11000110, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000110, 
    0b11111100 
};

static const uint8_t E[8] = { 
    0b11111111, 
    0b11000000, 
    0b11000000, 
    0b11111111, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11111111 
};

static const uint8_t F[8] = { 
    0b11111111, 
    0b11000000, 
    0b11000000, 
    0b11111111, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11000000 
};

static const uint8_t G[8] = { 
    0b00011110, 
    0b01100011, 
    0b11000001, 
    0b11000000, 
    0b11000111, 
    0b11000011, 
    0b01100111, 
     0b00011110\
};

static const uint8_t H[8] = { 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11111111, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011 
};

static const uint8_t I[8] = { 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000 
};

static const uint8_t J[8] = { 
    0b11111110, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b11011000, 
    0b11110000, 
    0b01100000 
};

static const uint8_t K[8] = { 
    0b11000110, 
    0b11001100, 
    0b11011000, 
    0b11110000, 
    0b11110000, 
    0b11011000, 
    0b11001100, 
    0b11000110 
};

static const uint8_t L[8] = { 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11111111 
};

static const uint8_t M[8] = { 
    0b11000011, 
    0b11100111, 
    0b11111111, 
    0b11011011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011 
};

static const uint8_t N[8] = { 
    0b11000011, 
    0b11100011, 
    0b11110011, 
    0b11011011, 
    0b11001111, 
    0b11000111, 
    0b11000011, 
    0b11000011 
};

static const uint8_t O[8] = { 
    0b00111100, 
    0b01000010, 
    0b10000001, 
    0b10000001, 
    0b10000001, 
    0b10000001, 
    0b01000010, 
    0b00111100 
};

static const uint8_t P[8] = { 
    0b11111110, 
    0b11000011, 
    0b11000011, 
    0b11111110, 
    0b11000000, 
    0b11000000, 
    0b11000000, 
    0b11100000 
};

static const uint8_t Q[8] = { 
    0b00111100, 
    0b01000010, 
    0b10000001, 
    0b10000001, 
    0b10000001, 
    0b10100001, 
    0b01011010, 
    0b00111101 
};

static const uint8_t R[8] = { 
    0b11111110, 
    0b11000011, 
    0b11000011, 
    0b11111110, 
    0b11011000, 
    0b11001100, 
    0b11000110, 
    0b11000011 
};

static const uint8_t S[8] = { 
    0b01111110, 
    0b11000011, 
    0b11000000, 
    0b01111110, 
    0b00000011, 
    0b11000011, 
    0b11000011, 
    0b01111110 
};

static const uint8_t T[8] = { 
    0b11111111, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000 
};

static const uint8_t U[8] = { 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11111111 
};

static const uint8_t V[8] = { 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b01100110, 
    0b01100110, 
    0b00111100 
};

static const uint8_t W[8] = { 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11000011, 
    0b11011011, 
    0b11011011, 
    0b01111110, 
    0b01100110 
};

static const uint8_t X[8] = { 
    0b11000011, 
    0b01100110, 
    0b00111100, 
    0b00011000, 
    0b00111100, 
    0b01100110, 
    0b11000011, 
    0b11000011 
};

static const uint8_t Y[8] = { 
    0b11000011, 
    0b01100110, 
    0b00111100, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000, 
    0b00011000 
};

static const uint8_t Z[8] = { 
    0b11111111, 
    0b11000011, 
    0b10000110, 
    0b00001100, 
    0b00011000, 
    0b00110000, 
    0b01100011, 
    0b11111111 
};


uint8_t* get_letter_bits(char letter) {
    switch (letter) {
        case 'A':
            return A;
        case 'B':
            return B;
        case 'C':
            return C;
        case 'D':
            return D;
        case 'E':
            return E;
        case 'F':
            return F;
        case 'G':
            return G;
        case 'H':
            return H;
        case 'I':
            return I;
        case 'J':
            return J;
        case 'K':
            return K;
        case 'L':
            return L;
        case 'M':
            return M;
        case 'N':
            return N;
        case 'O':
            return O;
        case 'P':
            return P;
        case 'Q':
            return Q;
        case 'R':
            return R;
        case 'S':
            return S;
        case 'T':
            return T;
        case 'U':
            return U;
        case 'V':
            return V;
        case 'W':
            return W;
        case 'X':
            return X;
        case 'Y':
            return Y;
        case 'Z':
            return Z;
        default:
            return 0;
    }
}


void matrix_write_letter(char letter,const char* background_colour,const char* letter_colour){
    uint8_t* letter_bits; // pointer to the bitmask for the requested letter
    letter_bits = get_letter_bits(letter);
    Colour background_colour_rgb = get_colour(background_colour);
    Colour letter_colour_rgb = get_colour(letter_colour);
    // iterate through each row and column of the bitmask and display the corresponding pixel on the LED matrix
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (letter_bits[row] & (1 << col)) {
                // the current bit is set, so set the pixel to the letter colour
                update_LED_color_rgb(row, 7 - col, letter_colour_rgb.red,letter_colour_rgb.green,letter_colour_rgb.blue);
            } else {
                // the current bit is not set, so set the pixel to the background colour
                update_LED_color_rgb(row, 7 - col, background_colour_rgb.red,background_colour_rgb.green,background_colour_rgb.blue);
            }
        }
    }
}