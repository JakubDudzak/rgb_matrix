#ifndef DEFINES_H
#define DEFINES_H
// ##############################PINS################################
#define BLANK_PIN PB1    // Arduino digital pin 9  =>  3 zelena 
#define LATCH_PIN PB2    // Arduino digital pin 10 (SS, but used as I/O)    =>  1 fialova
#define DATA_PIN PB3     // Arduino digital pin 11 (MOSI)                   =>  4 modra
#define CLOCK_PIN PB5    // Arduino digital pin 13 (SCK)                    =>  2 zlta
#define LAYER_A PD2      // Arduino digital pin 2                           =>
#define LAYER_B PD3      // Arduino digital pin 3                           => 
#define LAYER_C PD4      // Arduino digital pin 4                           =>
// ##############################PINS################################


// ###########################SETTINGS###############################
#define F_CPU 16000000UL
#define BAUDRATE 9600     
#define PWM_SIZE 8
#define MATRIX_SIZE 8
#define OCR_VAL 30
#define LCD__I2C_ADDR 0x27
#define PCF8574_I2C_ADDR 0x4E //0x7E
// ###########################SETTINGS###############################





// #############AUTOMATICALLY COMPUTED DO NOT CHANGE#################
#define NUM_OF_COLS MATRIX_SIZE
#define NUM_OF_ROWS MATRIX_SIZE
#define NUM_OF_USART_BYTES MATRIX_SIZE*MATRIX_SIZE*3
#define MATRIX_MAX (MATRIX_SIZE - 1)                //Max matrix index
#define PWM_MAX (PWM_SIZE - 1)                      //Max PWM index
#define ROWS_MAX (NUM_OF_ROWS - 1)                  //Max row index
#define COLS_MAX (NUM_OF_COLS - 1)                  //Max col index
#define BYTES_PER_ROW ((MATRIX_SIZE * 3 + 7) / 8)   //If matrix_size == 8 ,je to 3
#define UBRR ((F_CPU / (16UL * BAUDRATE)) - 1)
// #############AUTOMATICALLY COMPUTED DO NOT CHANGE#################
#endif