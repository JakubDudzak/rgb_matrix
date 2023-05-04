#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2c_lcd.h"


void setup_i2c(void)
{
    // nastavenie rýchlosti I2C komunikácie
    TWSR = 0x00;
    TWBR = ((F_CPU / 100000UL) - 16) / 2;
}

static void i2c_start(void)
{
    // poslanie start bitu
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    // čakanie na dokončenie operácie
    while (!(TWCR & (1 << TWINT)));
}

static void i2c_stop(void)
{
    // poslanie stop bitu
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static void i2c_write(uint8_t data)
{
    // zápis dát do registra
    TWDR = data;

    // poslanie dát
    TWCR = (1 << TWINT) | (1 << TWEN);

    // čakanie na dokončenie operácie
    while (!(TWCR & (1 << TWINT)));
}

static uint8_t i2c_receive_ack(void) {
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	return (TWDR);
}

static uint8_t i2c_receive_nack(void) {
	TWCR = (1<<TWINT)|(1<<TWEN)|(0<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	return (TWDR);
}

static uint8_t i2c_component (uint8_t address)
{
	uint8_t st;
	
	i2c_start();	
    i2c_write(address);
	st = TWI_STATUS;
	i2c_stop();
	return (st);
}

static void i2c_send_byte (char data) {

	i2c_start();
	i2c_write(PCF8574_I2C_ADDR); 
	i2c_write(data);
	i2c_stop();
}

static void i2c_send_string (uint8_t *datas) {
	uint8_t	i;
	
	i2c_start();
	i2c_write(PCF8574_I2C_ADDR);
	for (i=0; i<strlen(datas); i++) {
        i2c_write(datas[i]);
    }
	i2c_stop();
}



//------------------------------------------------------

void lcd_send_command(uint8_t cmd){
    uint8_t lcd_data;
    // Nastavenie hornych 4 bitov commandu
    lcd_data = (cmd & 0xF0);
    // Nastavenie RS na LOW pre command
    lcd_data &= RS_L;
    // Nastavenie EN a BL na HIGH, EN umožňuje prijímať príkazy alebo dáta, ktoré sa majú zobraziť na displeji, BL zapina podsvietenie
    // Začiatok komunikácie s LCD modulom
    lcd_data |= (EN_H |BL_H);
    // Nastavenie RW na LOW pre zapis
    lcd_data &= RW_L;
    // Odoslanie horných 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);
    // Nastavenie EN na LOW
    lcd_data &= EN_L;
    // Odoslanie dolnych 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);

    // Nastavenie dolnych 4 bitov commandu
    lcd_data = ((cmd & 0x0F)*0x10);
    // Nastavenie RS na LOW pre command
    lcd_data &= RS_L;
    // Nastavenie EN a BL na HIGH
    lcd_data |= (EN_H |BL_H);
    // Nastavenie RW na LOW pre zapis
    lcd_data &= RW_L;
    // Odoslanie horných 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);
    // Nastavenie EN na LOW
    lcd_data &= EN_L;
    // Odoslanie dolnych 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);
}

void lcd_send_byte(uint8_t chr)
{
	uint8_t lcd_data;
	// Nastavenie horných 4 bitov dat na LCD data
    lcd_data = (chr & 0xF0);
    // Nastavenie signálov RS, EN a BL na HIGH pre odoslanie príkazu na LCD a LCD osvetlenie
    lcd_data |= (RS_H | EN_H | BL_H);
    // Nastavenie signálu RW na LOW pre zápis na displej
    lcd_data &= RW_L;
    // Odoslanie horných 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);
    // Nastavenie signálu EN na LOW, aby sa mohli odoslať dolné 4 bity dat
    lcd_data &= EN_L;
    // Odoslanie dolných 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);


    // Nastavenie dolných 4 bitov dat (súčin & 0x0F) na LCD data
    lcd_data = ((chr & 0x0F)*0x10);
    // Nastavenie signálov RS, EN a BL na HIGH pre odoslanie príkazu na LCD a LCD osvetlenie
    lcd_data |= (RS_H | EN_H | BL_H);
    // Nastavenie signálu RW na LOW pre zápis na displej
    lcd_data &= RW_L;
    // Odoslanie horných 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);
    // Nastavenie signálu EN na LOW, aby sa mohli odoslať dolné 4 bity dat
    lcd_data &= EN_L;
    // Odoslanie dolných 4 bitov dat na PCF8574 pomocou I2C
    i2c_send_byte(lcd_data);
}

void lcd_send_string(uint8_t row, uint8_t col, char *str)
{
	unsigned char i;
	
	switch(row)
	{
		case 1:row=0x00;break;
		case 2:row=0x40;break;
		case 3:row=0x14;break;
		case 4:row=0x54;break;
	}
	lcd_send_command(SET_DDRAM | row | col);
	
	for (i=0; i< strlen(str); i++)	{
		lcd_send_byte(str[i]);
	}
}

void lcd_send_char(uint8_t row, uint8_t col, char character)
{
	unsigned char i;
	
	switch(row)
	{
		case 1:row=0x00;break;
		case 2:row=0x40;break;
		case 3:row=0x14;break;
		case 4:row=0x54;break;
	}
	lcd_send_command(SET_DDRAM | row | col);
	
	lcd_send_byte(character);
}

static int count_digits(int n) {
    int count = 0;
    if (n < 0) {
        n = -n;
    }
    while (n != 0) {
        n /= 10;
        ++count;
    }
    return count;
}

inline void lcd_send_number_at_end(uint8_t row, int number)
{
    uint8_t num_of_digits = count_digits(number);
	char buffer[10];
    itoa(number,buffer,10);
    buffer[num_of_digits] = '\0';
    lcd_send_string(row,16-num_of_digits,buffer);
}


inline void lcd_clear(void){
    lcd_send_command(CLEAR_DISP);
    _delay_ms(2);
}

void setup_lcd(void) {
	//I2C-byte: D7 D6 D5 D4 BL EN RW RS
	lcd_send_byte(0x00);
	_delay_ms(40);

	i2c_send_byte(0b00110100); // D7=0, D6=0, D5=1, D4=1, BL=0, EN=1, RW=0, RS=0
	_delay_us(5);
	i2c_send_byte(0b00110000); // D7=0, D6=0, D5=1, D4=1, BL=0, EN=0, RW=0, RS=0
	_delay_ms(5);
	i2c_send_byte(0b00110100); // D7=0, D6=0, D5=1, D4=1, BL=0, EN=1, RW=0, RS=0
	_delay_us(5);
	i2c_send_byte(0b00110000); // D7=0, D6=0, D5=1, D4=1, BL=0, EN=0, RW=0, RS=0
	_delay_us(100);
	i2c_send_byte(0b00110100); // D7=0, D6=0, D5=1, D4=1, BL=0, EN=1, RW=0, RS=0
	_delay_us(5);
	i2c_send_byte(0b00110000); // D7=0, D6=0, D5=1, D4=1, BL=0, EN=0, RW=0, RS=0
	_delay_us(100);

    i2c_send_byte(0b00100100);
    _delay_us(5);
    i2c_send_byte(0b00100000);
	_delay_us(50);

	lcd_send_command(FUN_SET|0x08);		//DL=0, N=1, F=0
	_delay_us(50);
	lcd_send_command(DISP_ON_OFF|0x04);	//D2=display_on, D1=cursor_on, D0=cursor_blink
	_delay_us(50);
	lcd_send_command(CLEAR_DISP);
	_delay_us(2000);
	lcd_send_command(MODE_SET|0x02);		//I/D=1
	lcd_send_string(1,0,"Mikroprocesorova");
	lcd_send_string(2,0,"----Technika----");
}





