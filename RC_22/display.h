/*----------------------------------------------------------------------------
 Copyright:
 Author:         Radig Ulrich
 Remarks:        
 known Problems: none
 Version:        21.11.2009
 Description:    EA DOG (M/L)128-6
------------------------------------------------------------------------------*/

#include <stdio.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>

#ifndef _DISPLAY_H
	#define _DISPLAY_H

// nicht von font.h????
#define FONT_WIDTH 	6
//#define FONT_HEIGHT 8  //8, 16, 32, 64, 128


#define DISPLAY_OFFSET		2
	
#define LINE1				5
	
#define LINE2				1



/*
#define DOG_CS       5
#define DOG_RST      4
#define DOG_A0       3
#define DOG_SCL      2
#define DOG_DATA     1
#define DOG_PWM      0
*/

// defines teensy3.5
#define DOG_CS       7
#define DOG_RST      6
#define DOG_A0       5
#define DOG_SCL      4
#define DOG_DATA     3
#define DOG_PWM      2

#define UPDATESCREEN    5 // Bit in status wird gesetzt wenn eine Taste gedrueckt ist, reset wenn update ausgefuehrt

#define SETTINGWAIT     6  // Bit in status wird gesetzt bis Taste 5 3 * gedrueckt ist

//#define MANUELL         7   // Bit 7 von Status

#define MINWAIT         3 // Anzahl loops von loopcount1 bis einschalten



#define MAXSPANNUNG  756
#define MINSPANNUNG  500



// SOFT-SPI defines

//#define SOFT_SPI_PORT   PORTC
//#define SOFT_SPI_DDR    DDRC


// http://www.cczwei-forum.de/cc2/thread.php?postid=49733#post49733
#define DISPOFF      0xAE
#define DISPON       0xAF
#define DISPSTART    0x40
#define PAGEADR      0xB0
#define COLADRL      0x00
#define COLADRH      0x10
#define ADCNORMAL    0xA0
#define ADCREVERSE   0xA1
#define COMNORMAL    0xC0
#define COMREVERSE   0xC8
#define DISPNORMAL   0xA6
#define DISPREVERSE  0xA7
#define LCDBIAS9     0xA2
#define LCDBIAS7     0xA3
#define RESET        0xE2
#define SETPOWERCTRL 0x28
#define REGRESISTOR  0x20
#define SETCONTRAST  0x81
#define STATINDMODE  0xAC
#define BOOSTERRATIO 0xF8



// Textausrichtung 12:00: 4 zu x-koord addieren
#define OFFSET_6_UHR 4



/*
volatile uint8_t itemtab[10] = {8+OFFSET_6_UHR,32+OFFSET_6_UHR,48+OFFSET_6_UHR,60+OFFSET_6_UHR,72+OFFSET_6_UHR,84+OFFSET_6_UHR,96+OFFSET_6_UHR,108+OFFSET_6_UHR,116+OFFSET_6_UHR,0+OFFSET_6_UHR};
*/
/*
volatile uint8_t cursortab[10] = {0+OFFSET_6_UHR,24+OFFSET_6_UHR+OFFSET_6_UHR,40+OFFSET_6_UHR,52+OFFSET_6_UHR,64+OFFSET_6_UHR,76+OFFSET_6_UHR,88+OFFSET_6_UHR,100+OFFSET_6_UHR,108+OFFSET_6_UHR,0};
*/
const uint8_t   expoarray25[3][26] ={
{0x00,0x01,0x01,0x02,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0F,0x10,0x12,0x14,0x15,0x17,0x19,0x1B,0x1E,0x20},
{0x00,0x00,0x01,0x01,0x01,0x02,0x02,0x03,0x03,0x04,0x04,0x05,0x06,0x07,0x08,0x09,0x0B,0x0C,0x0E,0x0F,0x12,0x14,0x16,0x19,0x1C,0x20},
{0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x09,0x0B,0x0D,0x0F,0x12,0x16,0x1B,0x20}
};

void sethomescreen(void);
void display_init(void);
void display_soft_init(void);
void display_mem(char* pointer);
void display_clear(void);
void display_go_to (unsigned char, unsigned char);
void display_back_char (void);
void display_write_byte(unsigned cmd_data, unsigned char data);

void display_write_char(unsigned char);
void display_write_propchar(unsigned char c, uint8_t prop);
void display_write_simple_propchar(unsigned char c, uint8_t prop, uint8_t offset);


void display_write_str(char *str, uint8_t prop);
void display_write_inv_str(char *str, uint8_t prop);

void display_write_P (const char *Buffer,...);

void display_write_int(uint8_t zahl, uint8_t prop);
void display_inverse(uint8_t inv);
uint8_t spi_out(uint8_t dataout);
void display_write_symbol(const char symbol);
void display_write_propsymbol(const char symbol);

void display_pfeilvollrechts(uint8_t col, uint8_t page);
void display_write_min_sek(uint16_t rawsekunde, uint8_t prop);
void display_write_zeit(uint8_t sekunde,uint8_t minute,uint8_t stunde,uint8_t prop);
void display_write_stopzeit(uint8_t sekunde,uint8_t minute,uint8_t prop);
void display_write_stopzeit_BM(uint8_t sekunde,uint8_t minute);

void display_writeprop_str(uint8_t page, uint8_t column, uint8_t inverse, const uint8_t *pChain);

void display_write_prop_str(uint8_t page, uint8_t column, uint8_t inverse, const uint8_t *pChain, uint8_t prop);

void display_write_spannung(uint16_t rawspannung, uint8_t prop); // eine Dezimale
void display_akkuanzeige (uint16_t spannung);
void display_trimmanzeige_horizontal (uint8_t char_x0, uint8_t char_y0, uint8_t device, int8_t mitteposition);
void display_trimmanzeige_vertikal (uint8_t char_x0, uint8_t char_y0, uint8_t device, int8_t mitteposition);

void r_uitoa8(int8_t zahl, char* string);

uint8_t update_screen(void);

uint8_t display_diagramm (uint8_t char_x, uint8_t char_y, uint8_t stufea,uint8_t stufeb, uint8_t typ);
uint8_t display_kanaldiagramm (uint8_t char_x, uint8_t char_y, uint8_t level, uint8_t expo, uint8_t typ );
uint8_t display_kanaldiagramm_var(uint8_t char_x0, uint8_t char_y0, uint8_t level, uint8_t expo, uint8_t typ );
void display_cursorweg(void);

void display_set_LED(uint8_t state);





//	#define display_write(format, args...)   display_write_P(PSTR(format) , ## args)
/*
volatile unsigned char char_x;
volatile unsigned char char_y;
volatile unsigned char char_height_mul;
volatile unsigned char char_width_mul;
*/
//write to lc-display command or data register
	
	
	
	//Befehlstabelle EA DOGM128-6 Seite 5
	// (1) Display ON/OFF
	#define DISPLAY_ON       			0xAF  //LCD_DISPLAY_ON
	#define DISPLAY_OFF      			0xAE  //LCD_DISPLAY_OFF

	// (2) Display start line set
	

	// (3) Page address set
	#define DISPLAY_PAGE_ADDRESS		0xB0

	// (4) Column address set upper bit
	#define DISPLAY_COL_ADDRESS_MSB		0x10
	// (4) Column address set lower bit
	#define DISPLAY_COL_ADDRESS_LSB		0x00  

	// (5) Status read (doesn't work in SPI mode)

	// (6) Display data write
	
	// (7) Display data read (doesn't work in SPI mode)
	
	// (8) ADC select
	#define DISPLAY_BOTTOMVIEW			0xA0  
	#define DISPLAY_TOPVIEW				0xA1  

	// (9) Display normale/reverse
	#define DISPLAY_NORMAL   			0xA6
	#define DISPLAY_REVERSE				0xA7

	// (10) Display all points ON/OFF
	#define DISPLAY_SHOW_NORMAL			0xA4
	#define DISPLAY_SHOW_ALL_POINTS		0xA5

	// (11) LCD bias set
	#define DISPLAY_BIAS_1_9			0xA2
	#define DISPLAY_BIAS_1_7			0xA3

	// (12) Read-modify-write (doesn't work in SPI mode)

	// (13) End clear read/modify/write (doesn't work in SPI mode)

	// (14) RESET
	#define DISPLAY_RESET_CMD			0xE2

	// (15) Common output mode select
	#define DISPLAY_SCAN_DIR_NORMAL		0xC0  
	#define DISPLAY_SCAN_DIR_REVERSE    0xC8

	// (16) Power control set
	#define DISPLAY_POWER_CONTROL       0x28
	#define DISPLAY_POWER_LOW_POWER		0x2F
	#define DISPLAY_POWER_WIDE_RANGE    0x2F 
	#define DISPLAY_POWER_LOW_VOLTAGE	0x2B 

	// (17) V0 voltage regulator internal resistor ratio set
	#define DISPLAY_VOLTAGE          	0x20

	// (18) Electronic volume mode set
	#define DISPLAY_VOLUME_MODE_1    	0x81
	// (18) Register
	#define DISPLAY_VOLUME_MODE_2    	0x00

	// (19) Static indicator ON/OFF
	#define DISPLAY_INDICATOR_ON       	0xAD  
	#define DISPLAY_INDICATOR_OFF      	0xAC  
	// (19) Static indicator register set
	#define DISPLAY_INDICATOR_MODE_OFF 	0x00
	#define DISPLAY_INDICATOR_MODE_1HZ 	0x01
	#define DISPLAY_INDICATOR_MODE_2HZ 	0x10
	#define DISPLAY_INDICATOR_MODE_ON  	0x11

	// (20) Booster ratio set
	#define DISPLAY_BOOSTER_SET      	0xF8
	#define DISPLAY_BOOSTER_234      	0x00
	#define DISPLAY_BOOSTER_5        	0x01
	#define DISPLAY_BOOSTER_6        	0x03

	// (21) Power save

	// (22) NOP
	#define LCD_NOP              		0xE3
	
	// (23) Test Command for IC test. Do not use this command.
	


#endif //_DISPLAY_H





