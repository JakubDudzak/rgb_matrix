#ifndef ANIMATIONS_H_
#define ANIMATIONS_H_
#define PCF8574_I2C_ADDR 0x4E //0x7E
typedef unsigned char uint8_t;
/****************************************************************************
  TWI Status/Control register definitions
****************************************************************************/
#define TWI_BUFFER_SIZE 8   // Set this to the largest message size that will
							// be sent including address byte.
#define TWI_TWBR	0x48   	// TWI Bit rate Register setting.	100KHz CLK
//#define TWI_TWPS	0x00    // This driver presumes prescaler = 00

/****************************************************************************
  Function definitions
****************************************************************************/



void setup_i2c(void); // inicializacia I2C komunikacie
// void i2c_start(void); // poslanie start bitu
// void i2c_stop(void); // poslanie stop bitu
// void i2c_write(uint8_t data); // zapis dát do registra
// uint8_t i2c_receive_ack(void); // prijatie dát so ziadostou o potvrdenie prijatia
// uint8_t i2c_receive_nack(void); // prijatie dát bez ziadosti o potvrdenie prijatia
// uint8_t i2c_component (uint8_t address); // overenie spojenia s komponentom
// void i2c_send_byte (char data); // odoslanie bajtu pomocou I2C
// void i2c_send_string (uint8_t *datas); // odoslanie reťazca pomocou I2C
void lcd_send_command(uint8_t cmd); // odoslanie príkazu na LCD displej
void lcd_send_byte(uint8_t chr); // odoslanie znaku na LCD displej
void lcd_send_string(uint8_t row, uint8_t col, char *str);
void setup_lcd(void);
void lcd_clear(void);
void lcd_send_char(uint8_t row, uint8_t col, char character);
void lcd_send_number_at_end(uint8_t row, int number);

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master staus codes                      
#define TWI_START                  0x08  // START has been transmitted  
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter staus codes                      
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

// TWI Master Receiver staus codes  
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

// TWI Slave Transmitter staus codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver staus codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = “0”
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#define TWI_STATUS					(TWSR&0xF8)





// LED 

//RS ->	P0
//RW ->	P1
//EN ->	P2
//D4 -> P4
//D5 -> P5
//D6 -> P6
//D7 -> P7

#define RS_H	0b00000001  //data
#define RS_L	0b11111110  //command
#define RW_H	0b00000010  //read
#define RW_L	0b11111101  //write
#define EN_H	0b00000100  
#define EN_L	0b11111011
#define BL_H	0b00001000
#define BL_L	0b11110111



#define CLEAR_DISP	0x01     // vyčistenie displeja (vymazanie obsahu)
#define RETURN_H	0x02     // návrat kurzoru na začiatok (horný riadok)
#define MODE_SET	0x04	 // nastavenie režimu: I/D (inkrementácia), S (scroll)
#define DISP_ON_OFF	0x08	 // zapnutie alebo vypnutie displeja, kurzoru a blikania kurzoru
#define CUR_DISP	0x10	 // zapnutie alebo vypnutie kurzoru a posúvanie displeja
#define FUN_SET		0x20	 // nastavenie základných parametrov: DL (8/4-bitový prenos dát), N (2/1 riadky), F (5x11/5x8 font)
#define SET_CGRAM	0x40	 // nastavenie adresy pamäte CGRAM (0-63)
#define SET_DDRAM	0x80	 // nastavenie adresy pamäte DDRAM (0-127)



#endif 