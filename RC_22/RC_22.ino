///
/// @mainpage	RC_22
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		___ORGANIZATIONNAME___
/// @date		04.02.2022 10:59
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2022
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		RC_22.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		___ORGANIZATIONNAME___
/// @date		04.02.2022 10:59
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2022
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///

#include "spi_eeprom.h"
#include "Arduino.h"
#include <ADC.h>
#include <SPI.h>
#include "lcd.h"
#include "settings.h"
//#include "spi_eeprom.h"
#include "display.h"

// Display




// Define structures and classes


// Define variables and constants
uint8_t loopLED;
#define USB_DATENBREITE 64

#define TEST 1

#define NUM_SERVOS 8

int8_t r;

// USB
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
volatile uint16_t          usb_recv_counter=0;
volatile uint16_t          cnc_recv_counter=0;
// end USB

// RC 
volatile uint8_t           timerstatus=0;
volatile uint8_t           code=0;
volatile uint8_t           servostatus=0;



#define RUN 0
#define PAUSE        1
#define PAKET   2
#define IMPULS    3
#define ADC_OK 4
#define USB_OK 5
#define ENDEPAKET  7

volatile uint8_t           tastaturstatus=0;
#define TASTEOK   1
#define AKTIONOK 2
#define UPDATEOK 3
elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMicros sinceusb;
uint16_t cncdelaycounter = 0;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;

elapsedMillis sincelastseccond;

elapsedMicros sincelastimpuls;

static volatile uint8_t buffer[USB_DATENBREITE]={};   // Daten von usb
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};// Daten nach usb

static volatile uint8_t drillbuffer[USB_DATENBREITE]={};// Daten fuer Drill, beim Start geladen

#  pragma mark intervaltimer

IntervalTimer servopaketTimer;
IntervalTimer servoimpulsTimer;
IntervalTimer kanalimpulsTimer;

volatile uint8_t                 programmstatus=0x00;

uint16_t impulsdelaycounter = 0;
uint16_t impulsdauer = 0;
uint8_t impulscounter = 0;

IntervalTimer microTimer; 
uint16_t microcounter = 0;

#define IMPULSPIN  1

//IntervalTimer              delayTimer;

volatile uint8_t           servoindex = 0;


// Utilities

volatile uint16_t impulstimearray[NUM_SERVOS] = {};

volatile uint8_t adcpinarray[NUM_SERVOS] = {};
// Prototypes
ADC *adc = new ADC(); // adc object

#define POTLO  10
#define POTHI  4092
#define PPMLO  500
#define PPMHI  1500

#define SHIFT 0xFFFFF

#define IMPULSBREITE 50
// Pot
/*
volatile uint16_t potlo = POTLO; // min pot
volatile uint16_t pothi = POTHI; // max pot
volatile uint16_t ppmlo = PPMLO; // min ppm
volatile uint16_t ppmhi = PPMHI; // max ppm
*/

float potlo = POTLO; // min pot
float pothi = POTHI; // max pot
float ppmlo = PPMLO; // min ppm
float ppmhi = PPMHI; // max ppm

volatile unsigned char char_x = 0;
volatile unsigned char char_y = 0;
volatile unsigned char char_height_mul = 0;
volatile unsigned char char_width_mul = 0;

//uint8_t spieeprom_rdbyte(uint16_t addr);


volatile float quot = (ppmhi - ppmlo)/(pothi - potlo);

// display

volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint8_t                 settingstartcounter=0; // Counter fuer Klicks auf Taste 5

uint8_t testdata = 0;
volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor

volatile uint8_t              curr_levelarray[8];
volatile uint8_t              curr_expoarray[8];
volatile uint8_t              curr_mixarray[8]={};
volatile uint8_t              curr_funktionarray[8];
volatile uint8_t             curr_devicearray[8] = {};
volatile uint8_t             curr_ausgangarray[8];
volatile int8_t              curr_trimmungarray[8];

volatile uint16_t                 blink_cursorpos=0xFFFF;

volatile uint16_t stopsekunde=0;
volatile uint16_t stopminute=0;
volatile uint16_t motorsekunde=0;
volatile uint16_t motorminute=0;
volatile uint8_t motorstunde=0;

volatile uint16_t sendesekunde=0;
volatile uint16_t sendeminute=0;
volatile uint8_t sendestunde=0;


volatile uint16_t batteriespannung =0;
 
volatile uint8_t                 curr_model=0; // aktuelles modell
volatile uint8_t                 speichermodel=0;
volatile uint8_t                 curr_kanal=0; // aktueller kanal
volatile uint8_t                 curr_richtung=0; // aktuelle richtung
volatile uint8_t                 curr_impuls=0; // aktueller impuls

volatile uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t                          speichersetting=0;

volatile uint8_t                 curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
volatile uint8_t                 curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal


volatile uint8_t                 curr_screen=0; // aktueller screen

volatile uint8_t                 curr_page=7; // aktuelle page
volatile uint8_t                 curr_col=0; // aktuelle colonne

volatile uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
volatile uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
volatile uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
volatile uint8_t                 last_cursorspalte=0; // letzte colonne des cursors


volatile uint8_t                  masterstatus = 0;
volatile uint8_t                   eepromsavestatus = 0;
volatile uint16_t                updatecounter=0; // Zaehler fuer Einschalten
volatile uint16_t                manuellcounter=0;

// Tastatur
volatile uint8_t                 Tastenindex=0;
volatile uint16_t                Tastenwert=0;
volatile uint16_t                Trimmtastenwert=0;
volatile uint8_t                 adcswitch=0;
volatile uint16_t                lastTastenwert=0;
volatile int16_t                 Tastenwertdiff=0;
volatile uint16_t                tastaturcounter=0;


volatile uint8_t  eeprom_indata=0;
volatile uint8_t  eeprom_errcount=0;

volatile uint8_t levelwert=0x32;
volatile uint8_t levelb=0x12;

volatile uint8_t expowert=0;
volatile uint8_t expob=0;


// USB
volatile uint8_t  usbtask = 0;


// Functions

void OSZI_A_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,LOW);
}

void OSZI_A_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,HIGH);
}

void OSZI_A_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZI_B_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,LOW);
}

void OSZI_B_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,HIGH);
}


void OSZI_C_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C,LOW);
}

void OSZI_C_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C,HIGH);
}

void OSZI_D_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_D,LOW);
}


void OSZI_D_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_D,HIGH);
}


void EE_CS_HI(void)
{
   digitalWriteFast(SPI_EE_CS_PIN,HIGH);
}


void EE_CS_LO(void)
{
      digitalWriteFast(SPI_EE_CS_PIN,LOW);
}

// MARK: Proto
uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte);
uint8_t eeprombytelesen(uint16_t readadresse); // 300 us ohne lcd_anzeige
uint8_t eeprompartlesen(uint16_t readadresse); //   us ohne lcd_anzeige
uint16_t eeprompartschreiben(void); // 23 ms
//void spieeprom_wrbyte(uint16_t addr, uint8_t data);


void read_eeprom_zeit(void);
void write_eeprom_zeit(void);
void write_eeprom_status(void);



void kanalimpulsfunktion(void) // kurze HI-Impulse beenden
{
   digitalWriteFast(IMPULSPIN,LOW);
   kanalimpulsTimer.end();
   servoindex++;
   // next impuls schon laden
   servoimpulsTimer.update(impulstimearray[servoindex]);
}

void servoimpulsfunktion(void) // 
{ 
  // servoindex++; // Version B: end und neu begin 
   if (servoindex <= NUM_SERVOS)
   { 
      //Version B:
      //servoimpulsTimer.end();
      //servoimpulsTimer.begin(servoimpulsfunktion,impulstimearray[servoindex]);
      
      // Version A
      //servoimpulsTimer.update(impulstimearray[servoindex]); // zu spaet: timer ist schon in neuem Intervall
      digitalWriteFast(IMPULSPIN,HIGH); // neuer impuls
      OSZI_B_HI();
   }
   else  // Paket beenden
   {
      servoimpulsTimer.end();
      servostatus |= (1<<PAUSE);
      servostatus |= (1<<ADC_OK); // ADCFenster starten
      //OSZI_B_HI();
      //OSZI_C_LO();

   }
   kanalimpulsTimer.begin(kanalimpulsfunktion,IMPULSBREITE); // neuer Kanalimpuls
   
}

// MARK: readSettings
uint8_t eeprombytelesen(uint16_t readadresse) ;// 300 us ohne lcd_anzeige

void read_Ext_EEPROM_Settings(void)
{
   uint8_t modelindex =0;
   modelindex = buffer[3]; // welches model soll gelesen werden
   uint16_t readstartadresse=0;

   uint8_t pos=0, verbose=buffer[4];
   
   //EE_CS_LO;
   _delay_us(LOOPDELAY);
  // uint16_t readstartadresse=0;
   //uint8_t modelindex = curr_model; // welches model soll gelesen werden
   // uint8_t pos=0;
   
    // Level lesen
   cli();
    readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
   sei();
    // startadresse fuer Settings des models
    for (pos=0;pos<8;pos++)
    {
       curr_levelarray[pos] = eeprombytelesen(readstartadresse+pos);
       
    }
    _delay_us(100);
   
    // Expo lesen
    readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
       curr_expoarray[pos] = eeprombytelesen(readstartadresse+pos);
       
    }
    _delay_us(100);
   
   
   // Mix lesen
   cli();
    readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
   sei();
   /*
   lcd_gotoxy(0,0);
   //lcd_putc('+');
   //lcd_putint1(modelindex);
   //lcd_putc('+');
   lcd_putint12(readstartadresse);
   lcd_putc('*');
   lcd_puthex((readstartadresse & 0xFF00)>>8);
   lcd_puthex((readstartadresse & 0x00FF));
 */
   
    for (pos=0;pos<8;pos++)
    {
       if (pos==0)
       {
       //OSZI_D_LO;
       }
       //cli();
       curr_mixarray[pos] = eeprombytelesen(readstartadresse+pos);
       //OSZI_D_HI;

    }
   
   _delay_us(EE_READ_DELAY);
   
   // Funktion lesen
   cli();
   readstartadresse = TASK_OFFSET  + FUNKTION_OFFSET + modelindex*SETTINGBREITE;
   sei();
   /*
    lcd_gotoxy(0,0);
    //lcd_putc('+');
    //lcd_putint1(modelindex);
    //lcd_putc('+');
    lcd_putint12(readstartadresse);
    lcd_putc('*');
    lcd_puthex((readstartadresse & 0xFF00)>>8);
    lcd_puthex((readstartadresse & 0x00FF));
    */
   
   for (pos=0;pos<8;pos++)
   {
      if (pos==0)
      {
         //OSZI_D_LO;
      }
      //cli();
      curr_funktionarray[pos] = eeprombytelesen(readstartadresse+pos);
      //OSZI_D_HI;
      
   }
   
   /*
   lcd_gotoxy(0,1);
   
   lcd_puthex(curr_funktionarray[0]);
   lcd_putc('$');
   lcd_puthex(curr_funktionarray[1]);
   lcd_putc('$');
   lcd_puthex(curr_funktionarray[2]);
   lcd_putc('$');
   lcd_puthex(curr_funktionarray[3]);
   lcd_putc('$');
    */
   
   _delay_us(EE_READ_DELAY);

   
   //EE_CS_HI;
}

// MARK: writeSettings
void write_Ext_EEPROM_Settings(void)
{
   // Halt einschalten
   masterstatus |= (1<<HALT_BIT); // Halt-Bit aktiviert Task bei ausgeschaltetem Slave
//   MASTER_PORT &= ~(1<<SUB_BUSY_PIN);
   
   
   lcd_clr_line(1);
   lcd_putint(eepromsavestatus);
   //EE_CS_LO;
   _delay_us(LOOPDELAY);
   uint16_t writestartadresse=0;
   uint8_t modelindex = curr_model; // welches model soll gelesen werden
   uint8_t pos=0;
   
   if (eepromsavestatus & (1<<SAVE_LEVEL))
   {
      eepromsavestatus &= ~(1<<SAVE_LEVEL);
      // Level schreiben
      cli();
      writestartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
      sei();
      // startadresse fuer Settings des models
      for (pos=0;pos<8;pos++)
      {
  //       lcd_gotoxy(4+2*pos,1);
         //lcd_putc(' ');
  //       lcd_puthex(curr_levelarray[pos]);
         eeprombyteschreiben(0xB0,writestartadresse+pos,curr_levelarray[pos]);
         
      }
      _delay_us(100);
   }
   
   if (eepromsavestatus & (1<<SAVE_EXPO))
   {
      eepromsavestatus &= ~(1<<SAVE_EXPO);
      
      // Expo schreiben
      cli();
      writestartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
      sei();
      
      for (pos=0;pos<8;pos++)
      {
         eeprombyteschreiben(0xB0,writestartadresse+pos,curr_expoarray[pos]);
      }
      _delay_us(100);
   }
   
   if (eepromsavestatus & (1<<SAVE_MIX))
   {
      eepromsavestatus &= ~(1<<SAVE_MIX);
      
      // Mix schreiben
      cli();
      writestartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
      sei();
      /*
       lcd_gotoxy(0,0);
       //lcd_putc('+');
       //lcd_putint1(modelindex);
       //lcd_putc('+');
       lcd_putint12(readstartadresse);
       lcd_putc('*');
       lcd_puthex((readstartadresse & 0xFF00)>>8);
       lcd_puthex((readstartadresse & 0x00FF));
       */
      
      for (pos=0;pos<8;pos++)
      {
         if (pos==0)
         {
            //OSZI_D_LO;
         }
         cli();
         eeprombyteschreiben(0xB0,writestartadresse+pos,curr_mixarray[pos]);
         //OSZI_D_HI;
         
      }
      sei();
      _delay_us(EE_READ_DELAY);
   }
   
   if (eepromsavestatus & (1<<SAVE_FUNKTION))
   {
      eepromsavestatus &= ~(1<<SAVE_FUNKTION);
      
      // Funktion schreiben
      cli();
      writestartadresse = TASK_OFFSET  + FUNKTION_OFFSET + modelindex*SETTINGBREITE;
      sei();
      /*
       lcd_gotoxy(0,0);
       //lcd_putc('+');
       //lcd_putint1(modelindex);
       //lcd_putc('+');
       lcd_putint12(readstartadresse);
       lcd_putc('*');
       lcd_puthex((readstartadresse & 0xFF00)>>8);
       lcd_puthex((readstartadresse & 0x00FF));
       */
      
      for (pos=0;pos<8;pos++)
      {
         if (pos==0)
         {
            //OSZI_D_LO;
         }
         cli();
         eeprombyteschreiben(0xB0,writestartadresse+pos,curr_funktionarray[pos]);
         //OSZI_D_HI;
         
      }
      sei();
      _delay_us(EE_READ_DELAY);
   }
   //EE_CS_HI;
   //Halt reseten
   // RAM_SEND_PPM_STATUS schicken: Daten haben geaendert
   
   

   masterstatus &= ~(1<<HALT_BIT); // Halt-Bit aktiviert Task bei ausgeschaltetem Slave
//   MASTER_PORT |= (1<<SUB_BUSY_PIN);
   _delay_us(100);
   
   masterstatus |= (1<<DOGM_BIT);
   
   // Löst in der loop das Setzen von task_out aus.
   // Umständlich, aber sonst nicht machbar.

   // das ist in loop verschoben
   //task_out |= (1<< RAM_SEND_DOGM_TASK);
   //task_outdata = curr_model;//modelindex;
}

uint8_t eeprombytelesen(uint16_t readadresse) // 300 us ohne lcd_anzeige
{
   //OSZI_B_LO;
   cli();
//   SUB_EN_PORT &= ~(1<<SUB_EN_PIN);
 //  _delay_us(EE_READ_DELAY);
 //  spi_start();
 //  _delay_us(EE_READ_DELAY);
//   SPI_PORT_Init();
   _delay_us(EE_READ_DELAY);
 //  spieeprom_init();
   _delay_us(EE_READ_DELAY);
   
   
   //lcd_gotoxy(1,0);
   //lcd_putc('r');
   //lcd_putint12(readadresse);
   //lcd_putc('*');
   
   eeprom_indata = 0xaa;
   uint8_t readdata=0xaa;
   
   // Byte  read 270 us
   EE_CS_LO();
   _delay_us(EE_READ_DELAY);
   readdata = spieeprom_rdbyte(readadresse);
   _delay_us(EE_READ_DELAY);
   _delay_us(10);
   EE_CS_HI;
  
   /*
   sendbuffer[0] = 0xD5;
   
   sendbuffer[1] = readadresse & 0x00FF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   // wird fuer Darstellung der Read-Ergebnisse im Interface benutzt.
   
//   usb_rawhid_send((void*)sendbuffer, 50);
   */
   sei();
   //OSZI_B_HI;
   //lcd_putc('*');
   return readdata;
}

uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte) //   1 ms ohne lcd-anzeige
{
   //OSZI_B_LO;
   uint8_t byte_errcount=0;
   uint8_t checkbyte=0;
   cli();
//   SUB_EN_PORT &= ~(1<<SUB_EN_PIN);
//   spi_start();
//   SPI_PORT_Init();
   /*
      lcd_gotoxy(3,0);
      lcd_putc('w');
      lcd_putint12(writeadresse);
      lcd_putc('*');
   */
   //spieeprom_init();
   
   // Test 131210
   
   // WREN schicken: Write ermoeglichen
   
   _delay_us(LOOPDELAY);
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   spieeprom_wren();
   _delay_us(LOOPDELAY);
   EE_CS_HI; // SS HI End
   
   //  lcd_putc('a');
   // End Test
   
   
   // Data schicken
   EE_CS_LO;
   
   spieeprom_wrbyte(writeadresse,eeprom_writedatabyte);
   
   EE_CS_HI;
   uint8_t w=0;
   
    
   //   lcd_putc('c');
   
   // Kontrolle
   _delay_us(LOOPDELAY);
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   
   //checkbyte = spieeprom_rdbyte(writeadresse);
   checkbyte = eeprombytelesen(writeadresse);
   
   //   lcd_putc('d');
   _delay_us(LOOPDELAY);
   EE_CS_HI;
   
   //lcd_putc('*');
   
   if ((eeprom_writedatabyte - checkbyte)||(checkbyte - eeprom_writedatabyte))
   {
      byte_errcount++;
      eeprom_errcount ++;
   }
   //   lcd_putc('e');
   
   //OSZI_B_LO;
   // Notewndig fuer schreiben der Expo-Settings (???)
   _delay_ms(4);
   
   /*
   lcd_gotoxy(0,1);
   lcd_putc('e');
   lcd_puthex(byte_errcount);
   lcd_putc(' ');
   lcd_puthex(eeprom_writedatabyte);
   lcd_putc(' ');
   lcd_puthex(checkbyte);
   */
   //OSZI_B_HI;
   
   sendbuffer[1] = writeadresse & 0xFF;
   sendbuffer[2] = (writeadresse & 0xFF00)>>8;
   sendbuffer[3] = byte_errcount;
   sendbuffer[4] = eeprom_writedatabyte;
   sendbuffer[5] = checkbyte;
   sendbuffer[6] = w;
   sendbuffer[7] = 0x00;
   sendbuffer[8] = 0xF9;
   sendbuffer[9] = 0xFA;
   
   sendbuffer[0] = code;
   //eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_WRITE_BYTE_TASK);
   
   //lcd_putc('+');
   usb_rawhid_send((void*)sendbuffer, 50);
   //lcd_putc('+');
   
   sei();
   // end Daten an EEPROM
   //OSZI_D_HI ;
   
   return byte_errcount;
}

void servopaketfunktion(void) // start Abschnitt
{ 
   servostatus &= ~(1<<PAUSE);
   servostatus |= (1<<PAKET);// neues Paket starten
   servostatus |= (1<<IMPULS);
   servoindex = 0; // Index des aktuellen impulses
 
   servoimpulsTimer.begin(servoimpulsfunktion,impulstimearray[servoindex]);
   
   kanalimpulsTimer.begin(kanalimpulsfunktion, IMPULSBREITE); // neuer Kanalimpuls
   digitalWriteFast(IMPULSPIN,HIGH);
   OSZI_B_LO();
   
   
}






void displayinit()
{
   /*
#define A0_HI        SOFT_SPI_PORT |= (1<<DOG_A0)
#define A0_LO        SOFT_SPI_PORT &= ~(1<<DOG_A0)

#define RST_HI        SOFT_SPI_PORT |= (1<<DOG_RST)
#define RST_LO        SOFT_SPI_PORT &= ~(1<<DOG_RST)


#define CS_HI        SOFT_SPI_PORT |= (1<<DOG_CS)
#define CS_LO        SOFT_SPI_PORT &= ~(1<<DOG_CS)

#define SCL_HI       SOFT_SPI_PORT |= (1<<DOG_SCL)
#define SCL_LO       SOFT_SPI_PORT &= ~(1<<DOG_SCL)

#define DATA_HI      SOFT_SPI_PORT |= (1<<DOG_DATA)
#define DATA_LO      SOFT_SPI_PORT &= ~(1<<DOG_DATA)
*/
   
   /*
   pinMode(DOG_CS, OUTPUT);
   digitalWriteFast(DOG_CS, 1);
   
   pinMode(DOG_RST, OUTPUT);
   digitalWriteFast(DOG_RST, 1);

   pinMode(DOG_A0, OUTPUT);
   digitalWriteFast(DOG_A0, 1);
   
   pinMode(DOG_SCL, OUTPUT);
   digitalWriteFast(DOG_SCL, 0);
   
   pinMode(DOG_DATA, OUTPUT);
   digitalWriteFast(DOG_DATA, 0);
   
   pinMode(DOG_PWM, OUTPUT);
   digitalWriteFast(DOG_PWM, 1);
*/


   
}

uint8_t Tastenwahl(uint16_t Tastaturwert)
{
  
   
   
   // Tastatur2 // Reihenfolge anders
   //   

   if (Tastaturwert < WERT1) // 76
      return 2;
   if (Tastaturwert < WERT2) // 124
      return 1;
   if (Tastaturwert < WERT3) // 200
      return 4;
   if (Tastaturwert < WERT4) // 276
      return 8;
   if (Tastaturwert < WERT5) // 354
      return 7;
   if (Tastaturwert < WERT6) // 442
      return 6;
   if (Tastaturwert < WERT7) // 557
      return 3;
   if (Tastaturwert < WERT8) // 672
      return 9;
   if (Tastaturwert < WERT9) // 861
      return 5;
   
   return 0xFF;
}


// Add setup code
void setup()
{
   Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }
  // pinMode(LOOPLED, OUTPUT);
   pinMode(LOOPLED, OUTPUT);
//   dog_7565R DOG;
   
   
   // void initialize (byte p_cs, byte p_si, byte p_clk, byte p_a0, byte p_res, byte type);
   // CS-Pin; MOSI-Pin; SCK-PIN; A0-Pin (data or command), Reset-Pin, 1=EA
   //DOGM128-6 2=EA DOGL128-6 3=EA DOGM132-5
//   dog.initialize(

   //displayinit();
   
   adc->adc0->setAveraging(4); // set number of averages 
   adc->adc0->setResolution(12); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);

   //volatile float quot = (ppmhi - ppmlo)/(pothi - potlo);
   //volatile float quot = (PPMHI - PPMLO)/(POTHI - POTLO);
   uint32_t shiftquot = uint32_t(quot * SHIFT);
   Serial.printf("quot: %.6f shiftquot: %d\n",quot, shiftquot);
                 
   for (int i=0;i<NUM_SERVOS;i++)
   {
    int wert = 500 + i * 50;
      wert = 750;
       impulstimearray[i] = wert; // mittelwert
   //Serial.printf("i: %d wert:\t %d\n",i,wert);
      adcpinarray[i] = 0xFF;
   }
   // init Pins
   
   pinMode(pot0_PIN, INPUT);
   adcpinarray[0] = pot0_PIN;
   pinMode(pot1_PIN, INPUT);
   adcpinarray[1] = pot1_PIN;
   pinMode(pot2_PIN, INPUT);
   adcpinarray[2] = pot2_PIN;
   pinMode(pot3_PIN, INPUT);
   adcpinarray[3] = pot3_PIN;
   adcpinarray[NUM_SERVOS-1] = 0xEF;// letzten Puls kennzeichnen
   
  
   
   pinMode(IMPULSPIN, OUTPUT);
   digitalWriteFast(IMPULSPIN,LOW);
   
   // Servopakete starten
   servoimpulsTimer.priority(3);
   kanalimpulsTimer.priority(2);
   servopaketTimer.priority(1);
   
   servopaketTimer.begin(servopaketfunktion, 25000);
   
   if (TEST)
   {
      pinMode(OSZI_PULS_A, OUTPUT);
      digitalWriteFast(OSZI_PULS_A, HIGH); 

      pinMode(OSZI_PULS_B, OUTPUT);
      digitalWriteFast(OSZI_PULS_B, HIGH); 

      pinMode(OSZI_PULS_C, OUTPUT);
      digitalWriteFast(OSZI_PULS_C, HIGH); 
      
      pinMode(OSZI_PULS_D, OUTPUT);
      digitalWriteFast(OSZI_PULS_D, HIGH); 

   }
   
   
   // EEPROM
   
   pinMode(SPI_EE_CS_PIN, OUTPUT);
   digitalWriteFast(SPI_EE_CS_PIN, HIGH); 
   
   delay(100);
   
   /* initialize the LCD */
  lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

    
   // Display
   pinMode(DOG_CS, OUTPUT);
   digitalWriteFast(DOG_CS, 1);
   
   pinMode(DOG_RST, OUTPUT);
   digitalWriteFast(DOG_RST, 1);

   pinMode(DOG_A0, OUTPUT);
   digitalWriteFast(DOG_A0, 1);
   
   pinMode(DOG_SCL, OUTPUT);
   digitalWriteFast(DOG_SCL, 0);
   
   pinMode(DOG_DATA, OUTPUT);
   digitalWriteFast(DOG_DATA, 0);
   
   pinMode(DOG_PWM, OUTPUT);
   digitalWriteFast(DOG_PWM, 1);
   _delay_ms(50);
   display_soft_init();
   _delay_ms(50);
   display_clear();
   _delay_us(50);
   
  
   sethomescreen();
   //display_write_str("abc",2);

   servostatus &= ~(1<<RUN);
   
   // Tastatur
   
   pinMode(TASTATURPIN, INPUT);
   //Serial.printf("W1: %d W2: %d W3: %d W4: %d W5: %d W6: %dW7: %d W8: %d W9: %d \n",WERT1, WERT2, WERT3, WERT4, WERT5, WERT6, WERT7, WERT8, WERT9);
}

// Add loop code
void loop()
{

   if (!(servostatus & (1<<RUN))) // first run
   {
      Serial.printf("first run\n");
      servostatus |= (1<<RUN);
   }
   
   if (sincelastseccond > 1000)
   {
      sincelastseccond = 0;
      sendesekunde++;
      update_sendezeit();
 
      //Serial.printf("motorsekunde: %d programmstatus: %d manuellcounter: %d\n",motorsekunde, programmstatus, manuellcounter);
      
      if (sendesekunde == 60)
      {
         sendeminute++;
         sendesekunde = 0;
      }
      if (sendeminute == 60)
      {
         sendestunde++;
         sendeminute = 0;
      }
      Serial.printf("sendesekunde: %d programmstatus: %d servostatus: %d manuellcounter: %d curr_screen: %d\n",sendesekunde, programmstatus,servostatus,  manuellcounter, curr_screen);

      
      if (programmstatus & (1<<MOTOR_ON))
      {
         motorsekunde++;
         if (motorsekunde==60)
         {
            motorminute++;
            motorsekunde=0;
         }
         if (motorminute >= 60)
         {
            motorminute = 0;
         }
         update_time();
      }
      
      if (programmstatus & (1<<STOP_ON))
      {
      //   lcd_gotoxy(15,0);
      //   lcd_putint2(stopsekunde);

         stopsekunde++;
         if (stopsekunde == 60)
         {
            stopminute++;
            stopsekunde=0;
         }
         if (stopminute >= 60)
         {
            stopminute = 0;
         }
         update_time();
      }

   } // 1000
   
   if (sinceblink > 500) 
   {   
      //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
      manuellcounter++;
      //Serial.printf("manuellcounter: %d\n",manuellcounter);
      sinceblink = 0;
      if (programmstatus & (1<<SETTINGWAIT))
      {
         startcounter++;
         /*
         if (startcounter > 5) // Irrtum, kein Umschalten
         {
           // lcd_gotoxy(0,1);
            //lcd_putc('X');
            programmstatus &= ~(1<< SETTINGWAIT);
            settingstartcounter=0;
            startcounter=0;
            manuellcounter = 0;
         }
          */
      }
      else
      {
         //startcounter = 0;
      }

      if (digitalRead(LOOPLED) == 1)
      {
         digitalWriteFast(LOOPLED, 0);
         
      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
         
        // Serial.printf("display_data %d\n",testdata);
         /*
         display_go_to(4,4);
         _delay_us(50);
         display_write_str("abc",2);
         //display_write_byte(DATEN,testdata++);
          */
         //Tastenindex = Tastenwahl(Tastenwert); // taste pressed
         //Serial.printf("Tastenwert: %d Tastenindex: %d\n",Tastenwert,Tastenindex);
      }
      //Serial.printf("servo potwert 0: %d 1: %d\n", impulstimearray[0],impulstimearray[1]); 
      impulscounter++;
      if (impulscounter > 5)
      {
         //Serial.printf("servo potwert 0: %d 1: %d\n", impulstimearray[0],impulstimearray[1]); 
         /*
         for (uint8_t i=0;i<NUM_SERVOS;i++)
         {
            Serial.printf("\t%d\t%d",i,impulstimearray[i]);
         }
         Serial.printf("\n");
          */
         impulscounter = 0;
      }
   
      
      //
      if ((manuellcounter > MANUELLTIMEOUT) )
      {
         {
            
            programmstatus &= ~(1<< LEDON);
            display_set_LED(0);
            manuellcounter=1;
            
            if (curr_screen) // nicht homescreen
            {
               display_clear();
               curr_screen=0;
               curr_cursorspalte=0;
               curr_cursorzeile=0;
               last_cursorspalte=0;
               last_cursorzeile=0;
               settingstartcounter=0;
               startcounter=0;
               eepromsavestatus=0;
        //       read_Ext_EEPROM_Settings();// zuruecksetzen
               
               sethomescreen();
               programmstatus &= ~(1<<UPDATESCREEN);
            }
           else 
           {
              programmstatus &= ~(1<< SETTINGWAIT);
              startcounter=0;
              settingstartcounter=0;
              /*
              lcd_gotoxy(0,2);
              lcd_putc(' ');
              lcd_putc(' ');
              lcd_putc(' ');
*/
           }
         }
         //
      }
   
   
   
   
   }// sinceblink
   
   
   if (servostatus & (1<<ADC_OK)) // 20us pro kanal ohne printf
   {
      //manuellcounter++;
      //Serial.printf("A");
      OSZI_C_LO();
      for (uint8_t i=0;i<NUM_SERVOS;i++)
      {
         
         //Serial.printf("i: %d pin: %d\n",i,adcpinarray[i]);
         if (adcpinarray[i] < 0xFF) // PIN belegt
         {
            if (adcpinarray[i] == 0xEF) // last
            {
               impulstimearray[i] = 1000;
            }
            else 
            {
            uint16_t potwert = adc->adc0->analogRead(adcpinarray[i]);
            float ppmfloat = PPMLO + quot *(float(potwert)-POTLO);
            uint16_t ppmint = uint16_t(ppmfloat);
               /*
            if (i == 0)
            {
               Serial.printf("servo %d potwert: %d   ppmfloat: %.6f\n",i,potwert,ppmfloat);    
            }                 
            */
            impulstimearray[i] = ppmint;
            //impulstimearray[i] = potwert;
            }
         }
         
         
      }
      servostatus &= ~(1<<ADC_OK);
      
      // MARK:  Tastatur ADC
      Tastenwert=(adc->adc0->analogRead(TASTATURPIN))>>2;
      //Tastenwert = 0;
      if (Tastenwert>5)
      {
         if (!(tastaturstatus & (1<<TASTEOK)))
         {
            //Serial.printf("A");
            //Tastenindex = Tastenwahl(Tastenwert); // taste pressed
            Tastenwertdiff = Tastenwert - lastTastenwert;
            if (Tastenwert > lastTastenwert)
            {
               Tastenwertdiff = Tastenwert - lastTastenwert;
            }
            else 
            {
               Tastenwertdiff = lastTastenwert - Tastenwert;
            }
            lastTastenwert = Tastenwert;
            //Serial.printf("B");
            if (Tastenwertdiff < 6)
            {
               //Serial.printf("C");
               if (tastaturcounter < ADCTIMEOUT)
               {
                  //Serial.printf("D");
                  tastaturcounter++;
                  
                  if (tastaturcounter == ADCTIMEOUT)
                  {
                     
                     Tastenindex = Tastenwahl(Tastenwert); // taste pressed
                     Serial.printf("Tastenwert: %d Tastenindex: %d\n",Tastenwert,Tastenindex);
                     tastaturstatus |= (1<<TASTEOK);
                     tastaturstatus |= (1<<AKTIONOK);
                     programmstatus |= (1<< LEDON);
                     
                     display_set_LED(1);
                     
                  }
               }
             
            } // if Tastenwertdiff 
             
            else
            {
               Serial.printf("F");
               tastaturcounter = 0;
            }
            
         }   
         
      }
      else
      {
         //Serial.printf("H");
         tastaturstatus &= ~(1<<TASTEOK);
         tastaturcounter = 0;
         Tastenindex = 0;
//           Trimmtastenwert=adc_read(TRIMMTASTATURPIN)>>2;
      }
      //Serial.printf("End\n");
      // end Tastatur
      OSZI_C_HI();
      
      
      
      //servostatus |= (1<<USB_OK);
   }
   
   
   
   if (tastaturstatus & (1<<TASTEOK))
   {
      Serial.printf("U Tastenindex: %d\n",Tastenindex);
      programmstatus |= (1<<UPDATESCREEN);
      //tastaturstatus &= ~(1<<TASTEOK);
      //Tastenindex = 0;
      switch (Tastenindex)
      {
#pragma mark Taste 0
         case 0:// Schalter auf Null-Position
         {
            
            {
               manuellcounter=0;
            }
            
         }break;
            
         case 1:
         {
#pragma mark Taste 1

            if (manuellcounter)
            {
               //Serial.printf("MOTOR_ON\n");
               if (tastaturstatus & (1<<AKTIONOK))
               {
                  programmstatus ^= (1<<MOTOR_ON);
                  tastaturstatus &=  ~(1<<AKTIONOK);
               }
               manuellcounter=0;
             }

         }break;
            
         case 2://
         {
#pragma mark Taste 2
            if (tastaturstatus & (1<<AKTIONOK))
            {
               tastaturstatus &=  ~(1<<AKTIONOK);
               
               switch (curr_screen)
               {
                  case HOMESCREEN: // home
                  {
                     if (manuellcounter)
                     {
                        //substatus |= (1<<SETTINGS_READ);; // Settings beim Start lesen
                        manuellcounter=0;
                        //        display_clear();
                        //        sethomescreen();
                        
                     }
                     
                     
                     
                  }break;
                     
                  case TRIMMSCREEN: // Trimmung
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        //lcd_gotoxy(0,1);
                        if (curr_cursorzeile ) // curr_cursorzeile ist >0,
                        {
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile--;
                           //lcd_putc('+');
                        }
                        else
                        {
                           
                           //lcd_putc('-');
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter)
                     {
                        /*
                         lcd_gotoxy(0,1);
                         lcd_puthex((blink_cursorpos & 0xFF00)>>8);
                         lcd_putc('*');
                         lcd_puthex((blink_cursorpos & 0x00FF));
                         */
                        //switch((blink_cursorpos & 0xFF00)>>8) // Blink-Zeile
                        switch(curr_cursorzeile) // Blink-Zeile
                        {
                           case 0: // vertikal
                           {
                              //switch (blink_cursorpos & 0x00FF)
                              switch (curr_cursorspalte)
                              {
                                 case 0:
                                 {
                                    //lcd_putc('0');
                                    if (curr_model )
                                    {
                                       curr_model--;
                                    }
                                    
                                 }break;
                                    
                                 case 1:
                                 {
                                    //lcd_putc('1');
                                    if (curr_setting )
                                    {
                                       curr_setting--;
                                    }
                                    
                                 }break;
                              } // switch Spalte
                              
                           }break;
                              
                           case  1: // horizontal
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0:
                                 {
                                    //lcd_putc('0');
                                    if (curr_model )
                                    {
                                       curr_model--;
                                    }
                                    
                                 }break;
                                    
                                 case 1:
                                 {
                                    //lcd_putc('1');
                                    if (curr_setting )
                                    {
                                       curr_setting--;
                                    }
                                    
                                 }break;
                              } // switch Spalte
                              
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                  }break; // trimmscreen
                     
                     
                     
                     
                  case SETTINGSCREEN: // Settings
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        //lcd_gotoxy(0,1);
                        if (curr_cursorzeile ) // curr_cursorzeile ist >0,
                        {
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile--;
                           //lcd_putc('+');
                        }
                        else
                        {
                           
                           //lcd_putc('-');
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter)
                     {
                        /*
                         lcd_gotoxy(0,1);
                         lcd_puthex((blink_cursorpos & 0xFF00)>>8);
                         lcd_putc('*');
                         lcd_puthex((blink_cursorpos & 0x00FF));
                         */
                        //switch((blink_cursorpos & 0xFF00)>>8) // Blink-Zeile
                        switch(curr_cursorzeile) // Blink-Zeile
                        {
                           case 0: // modell
                           {
                              //switch (blink_cursorpos & 0x00FF)
                              switch (curr_cursorspalte)
                              {
                                 case 0:
                                 {
                                    //lcd_putc('0');
                                    if (curr_model )
                                    {
                                       curr_model--;
                                    }
                                    
                                 }break;
                                    
                                 case 1:
                                 {
                                    //lcd_putc('1');
                                    if (curr_setting )
                                    {
                                       curr_setting--;
                                    }
                                    
                                 }break;
                              } // switch Spalte
                              
                           }break;
                              
                           case  1:
                           {
                              
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                  }break;
                     
                  case KANALSCREEN: // Kanalsettings
                  {
                     /*
                      lcd_gotoxy(5,1);
                      lcd_puthex(curr_cursorzeile);
                      lcd_putc('*');
                      lcd_puthex((blink_cursorpos & 0xFF00)>>8); // Zeile
                      lcd_putc('*');
                      //lcd_putc('*');
                      */
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                        
                     {
                        if (curr_cursorzeile )//
                        {
                           if (curr_cursorzeile<8)
                           {
                              char_height_mul=1;
                           }
                           else
                           {
                              char_height_mul=1;
                           }
                           
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile--;
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc('+');
                        }
                        else
                        {
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc('-');
                        }
                        //lcd_putint2(curr_cursorzeile);
                        
                        //lcd_putc(' ');
                        
                        
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        
                        //switch((blink_cursorpos & 0xFF00)>>8) // zeile
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanal
                           {
                              //uint8_t tempspalte = (blink_cursorpos & 0x00FF);
                              //lcd_puthex(curr_cursorspalte);
                              //blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Kanalnummer
                                 {
                                    if (curr_kanal )
                                    {
                                       curr_kanal--;
                                    }
                                 }break;
                                    
                                 case 1: // Richtung
                                 {
                                    eepromsavestatus |= (1<<SAVE_EXPO);
                                    //if (curr_settingarray[curr_kanal][1] & 0x80)
                                    if (curr_expoarray[curr_kanal] & 0x80)
                                    {
                                       curr_expoarray[curr_kanal] &= ~0x80;
                                    }
                                    else
                                    {
                                       curr_expoarray[curr_kanal] |= 0x80;
                                    }
                                 }break;
                                    
                                 case 2: // Funktion
                                 {
                                    //lcd_gotoxy(5,1);
                                    //lcd_putc('*');
                                    //Bezeichnung von: FunktionTable[curr_funktionarray[curr_kanal]]
                                    //uint8_t tempfunktion = curr_funktionarray[curr_kanal]& 0x07; // Bit 0-3
                                    //lcd_puthex(tempfunktion);
                                    //lcd_putc('*');
                                    
                                    //tempfunktion--; // decrementieren
                                    //tempfunktion &= 0x07; // Begrenzen auf 0-7
                                    //lcd_puthex(tempfunktion);
                                    //curr_funktionarray[curr_kanal] |= tempfunktion; // cycle in FunktionTable
                                    eepromsavestatus |= (1<<SAVE_FUNKTION);
                                    uint8_t tempfunktion = curr_funktionarray[curr_kanal]&0x07; //bit 0-2
                                    tempfunktion--;
                                    tempfunktion &= 0x07;
                                    
                                    curr_funktionarray[curr_kanal] = (curr_funktionarray[curr_kanal]&0xF0)|tempfunktion; // cycle in FunktionTable
                                    
                                 }break;
                                    
                                    
                                    
                              }// switch tempspalte
                              
                           }break;
                              
                           case  1: // level
                           {
                              eepromsavestatus |= (1<<SAVE_LEVEL);
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Levelwert A
                                 {
                                    if ((curr_levelarray[curr_kanal] & 0x70)>>4)
                                    {
                                       curr_levelarray[curr_kanal] -= 0x10;
                                    }
                                    
                                 }break;
                                 case 1: // Levelwert B
                                 {
                                    if ((curr_levelarray[curr_kanal] & 0x07))
                                    {
                                       curr_levelarray[curr_kanal] -= 0x01;
                                    }
                                    
                                 }break;
                                    
                                 case 2: //
                                 {
                                    curr_cursorspalte = 1; // fehler, back
                                    
                                 }break;
                                    
                              }// switch tempspalte
                              
                           }break;
                              
                           case  2: // Expo
                           {
                              eepromsavestatus |= (1<<SAVE_EXPO);
                              switch (curr_cursorspalte)
                              {
                                 case 0: // expowert A
                                 {
                                    if ((curr_expoarray[curr_kanal] & 0x70)>>4)
                                    {
                                       curr_expoarray[curr_kanal] -= 0x10;
                                    }
                                 }break;
                                    
                                 case 1: // Expowert B
                                 {
                                    if ((curr_expoarray[curr_kanal] & 0x07))
                                    {
                                       curr_expoarray[curr_kanal] -= 0x01;
                                    }
                                 }break;
                                    
                                 case 2: //
                                 {
                                    curr_cursorspalte = 1; // fehler, back
                                 }break;
                                    
                              }// switch tempspalte
                           }break;
                              
                           case  4:
                           {
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     } // if manuellcounter
                  }break; // canalscreen
                     
                  case MIXSCREEN:
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (curr_cursorzeile )//
                        {
                           if (curr_cursorzeile<8)
                           {
                              char_height_mul=1;
                           }
                           else
                           {
                              char_height_mul=1;
                           }
                           
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile--;
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        eepromsavestatus |= (1<<SAVE_MIX);
                        switch (curr_cursorspalte)
                        {
                           case 0: // Mix weiterschalten
                           {
                              
                              if (curr_mixarray[2*curr_cursorzeile+1])
                              {
                                 curr_mixarray[2*curr_cursorzeile+1] -= 0x01;// Mix ist auf ungerader zeile
                              }
                              
                           }break;
                              
                           case 1: // Kanal A zurueckschalten
                           {
                              uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0xF0)>>4;// kanal a ist auf gerader zeile in bit 4-6, 8 ist OFF
                              
                              if (tempdata) //
                              {
                                 curr_mixarray[2*curr_cursorzeile] -= 0x10;
                              }
                              
                              
                           }break;
                              
                           case 2: // Kanal B zurueckschalten
                           {
                              uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0x0F);// kanal b ist auf gerader zeile in bit 0-2, 8 ist OFF
                              
                              if (tempdata)
                              {
                                 curr_mixarray[2*curr_cursorzeile] -= 0x01;
                              }
                              
                           }break;
                              
                        }// switch curr_cursorspalte
                        
                        manuellcounter = 0;
                     }
                  }break; // mixscreen
                     
                     
                  case ZUTEILUNGSCREEN:
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (curr_cursorzeile )//
                        {
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile--;
                           //lcd_puthex(curr_cursorzeile);
                           //lcd_putc('+');
                        }
                        else
                        {
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                        /*
                         const char funktion0[] PROGMEM = "Seite  \0";
                         const char funktion1[] PROGMEM = "Hoehe  \0";
                         const char funktion2[] PROGMEM = "Quer   \0";
                         const char funktion3[] PROGMEM = "Motor  \0";
                         const char funktion4[] PROGMEM = "Quer L\0";
                         const char funktion5[] PROGMEM = "Quer R\0";
                         const char funktion6[] PROGMEM = "Lande  \0";
                         const char funktion7[] PROGMEM = "Aux    \0";
                         
                         */
                        /*
                         lcd_gotoxy(0,0);
                         lcd_puthex(curr_cursorzeile);
                         lcd_putc(' ');
                         lcd_puthex(curr_cursorspalte);
                         lcd_putc(' ');
                         */
                        eepromsavestatus |= (1<<SAVE_DEVICE);
                        switch (curr_cursorzeile)
                        {
                           case 0: // pitch vertikal
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // L_V index 1
                                 {
                                    // Kanalnummer decrement
                                    if (((curr_devicearray[1]& 0x07)))
                                    {
                                       curr_devicearray[1]-= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_V index 3
                                 {
                                    if (((curr_devicearray[3]& 0x07)))
                                    {
                                       curr_devicearray[3]-= 0x01;
                                    }
                                 }break;
                              }// switch curr_cursorspalte
                           }break; // pitch v
                              
                           case 1: // pitch horizontal
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // L_H index 0
                                 {
                                    if (((curr_devicearray[0]& 0x07)))
                                    {
                                       // Kanalnummer fuer Device decrement
                                       curr_devicearray[0]-= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_H index 2
                                 {
                                    if (((curr_devicearray[2]& 0x07)))
                                    {
                                       curr_devicearray[2]-= 0x01;
                                    }
                                 }break;
                              }// switch curr_cursorspalte
                           }break; // case spalte
                              
                           case 2: // schieber
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // S_L index 4
                                 {
                                    if (((curr_devicearray[4]& 0x07)))
                                    {
                                       // Kanalnummer fuer Device increment
                                       curr_devicearray[4]-= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // S_R index 5
                                 {
                                    if (((curr_devicearray[5]& 0x07)))
                                    {
                                       curr_devicearray[5]-= 0x01;
                                    }
                                 }break;
                              }// switch curr_cursorspalte
                           }break; // case spalte
                              
                        }//switch curr_cursorzeile
                        manuellcounter = 0;
                        
                     } // else if manuellcounter
                     
                  }break; // zuteilungscreen
                     
                  case AUSGANGSCREEN:
                  {
#pragma mark 2 AUSGANGSCREEN
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (curr_cursorzeile)// noch nicht zuoberst
                        {
                           char_height_mul=1;
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           if ((curr_impuls < 4) || (curr_impuls > 4))
                           {
                              curr_cursorzeile--;
                           }
                           else // zurueckscrollen
                           {
                              curr_cursorzeile = 3;
                           }
                           
                           curr_impuls--;
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                        /*
                         const char funktion0[] PROGMEM = "Seite  \0";
                         const char funktion1[] PROGMEM = "Hoehe  \0";
                         const char funktion2[] PROGMEM = "Quer   \0";
                         const char funktion3[] PROGMEM = "Motor  \0";
                         const char funktion4[] PROGMEM = "Quer L\0";
                         const char funktion5[] PROGMEM = "Quer R\0";
                         const char funktion6[] PROGMEM = "Lande  \0";
                         const char funktion7[] PROGMEM = "Aux    \0";
                         */
                        /*
                         lcd_gotoxy(0,0);
                         lcd_puthex(curr_cursorzeile);
                         lcd_putc(' ');
                         lcd_puthex(curr_cursorspalte);
                         lcd_putc(' ');
                         */
                        eepromsavestatus |= (1<<SAVE_AUSGANG);
                        switch (curr_cursorspalte)
                        {
                           case 0: // Kanal
                           {
                              // Kanalnummer im Devicearray increment
                              if (((curr_ausgangarray[curr_cursorzeile]& 0x07)))
                              {
                                 curr_ausgangarray[curr_cursorzeile]-= 0x01;
                              }
                           }break;
                              
                           case 1: // Zeile  nach oben verschieben
                           {
                              if (((curr_ausgangarray[curr_cursorzeile]& 0x07))<8)
                              {
                                 uint8_t tempzeilenwert =curr_ausgangarray[curr_cursorzeile];
                                 if (curr_impuls) // nicht erste Zeile, auf erster Seite
                                 {
                                    if ((curr_cursorzeile < 4) && (curr_impuls < 4)) // Noch vor scrollen, auf erster Seite
                                    {
                                       tempzeilenwert =curr_ausgangarray[curr_impuls];
                                       
                                       curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls-1]; // Wert von naechster zeile
                                       curr_ausgangarray[curr_impuls -1] = tempzeilenwert;
                                       // cursorzeile verschieben
                                       display_cursorweg();
                                       
                                       curr_cursorzeile--;
                                       // blink-cursorzeile verschieben
                                       blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                       
                                    }
                                    else  if ((curr_cursorzeile == 1) && (curr_impuls == 4))// zweite Zeile auf Seite 2, scrollen.
                                    {
                                       tempzeilenwert =curr_ausgangarray[curr_impuls];
                                       
                                       curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls-1]; // Wert von naechster zeile, noch auf dieser Seite
                                       curr_ausgangarray[curr_impuls -1] = tempzeilenwert;
                                       display_cursorweg();
                                       curr_cursorzeile = 3; // Scroll
                                       // blink-cursorzeile verschieben
                                       blink_cursorpos = cursorpos[3][curr_cursorspalte];
                                    }
                                    else  if ((curr_cursorzeile) && (curr_impuls >4))// zweite Zeile oder mehr auf zweiter Seite
                                    {
                                       tempzeilenwert =curr_ausgangarray[curr_impuls];
                                       curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls-1]; // Wert von naechster zeile
                                       curr_ausgangarray[curr_impuls -1] = tempzeilenwert;
                                       // cursorzeile verschieben
                                       display_cursorweg();
                                       
                                       curr_cursorzeile--;
                                       // blink-cursorzeile verschieben
                                       blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                       
                                    }
                                    curr_impuls--;
                                 }
                                 else // letzte Zeile, mit erster zeile vertauschen
                                 {
                                    /*
                                     tempzeilenwert =curr_ausgangarray[curr_impuls];
                                     curr_ausgangarray[curr_impuls] =curr_ausgangarray[0]; // Wert von erster zeile
                                     curr_ausgangarray[0] = tempzeilenwert;
                                     display_cursorweg();
                                     curr_cursorzeile=0;
                                     curr_impuls =0;
                                     blink_cursorpos = cursorpos[0][curr_cursorspalte];
                                     */
                                 }
                              }
                              
                           }break;
                              
                              
                        }// switch curr_cursorspalte
                        manuellcounter = 0;
                        
                     } // else if manuellcounter
                     
                  }break; // case ausgang
                     
               }// switch
            }// if AKTIONOK
         }break;
            
         case 3: //
         {
#pragma mark Taste 3

            if (manuellcounter)
            {
               if (tastaturstatus & (1<<AKTIONOK))
               {
                  programmstatus ^= (1<<STOP_ON);
                  tastaturstatus &=  ~(1<<AKTIONOK);
               }

                manuellcounter=0;

            }
         }break;
            
         case 4:// nach links
         {
#pragma mark Taste 4
            if (tastaturstatus & (1<<AKTIONOK))
            {
               tastaturstatus &=  ~(1<<AKTIONOK);
               
               switch (curr_screen)
               {
                  case HOMESCREEN: // home
                  {
                     //lcd_gotoxy(14,2);
                     // lcd_puts("*H4*");
                  }break;
                     
                  case SAVESCREEN: // save
                  {
                     if (curr_cursorspalte)
                     {
                        display_cursorweg();
                        
                        char_height_mul=1;
                        last_cursorspalte =curr_cursorspalte;
                        curr_cursorspalte--;
                        //lcd_putc('+');
                        blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                        
                     }
                     
                  }break;
                     
                     
                  case SETTINGSCREEN: // Settings
                  {
                     lcd_gotoxy(14,2);
                     lcd_puts("*S4*");
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Modellname
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Modellname
                                 {
                                    
                                 }   break;
                                    
                                 case 1: // Set text
                                 {
                                    display_cursorweg();
                                    char_height_mul=1;
                                    last_cursorspalte =curr_cursorspalte;
                                    curr_cursorspalte--;
                                 }break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  2: // Kanal
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // levelwert A, nicht weiter nach links
                                 {
                                    
                                 }   break;
                                    
                                 case 1: // Levelwert B
                                 {
                                    display_cursorweg();
                                    char_height_mul=1;
                                    last_cursorspalte =curr_cursorspalte;
                                    curr_cursorspalte--;
                                 }break;
                              }
                           }break;
                              
                           case  3: // Mix
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // expowert A, nicht weiter nach links
                                 {
                                    
                                 }   break;
                                    
                                 case 1: // expowert B
                                 {
                                    display_cursorweg();
                                    char_height_mul=1;
                                    last_cursorspalte =curr_cursorspalte;
                                    
                                    curr_cursorspalte--;
                                    
                                 }break;
                              }
                              
                           }break;
                              //
                              
                        }// switch
                        
                     }
                     else
                     {
                        switch(curr_cursorzeile) // zeile
                           
                        {
                           case 0: // modell
                           {
                              
                           }break;
                           case  1:
                           {
                           }break;
                              
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                     }
                     
                     
                  }break;
                     
                  case KANALSCREEN: // Kanal
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanaltext
                                 {
                                 }   break;
                                    
                                 default: // Richtung
                                 {
                                    display_cursorweg();
                                    char_height_mul=1;
                                    last_cursorspalte =curr_cursorspalte;
                                    
                                    curr_cursorspalte--;
                                    
                                    
                                 }break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1: // Level
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // levelwert A, nicht weiter nach links
                                 {
                                    
                                 }   break;
                                    
                                 default: // Levelwert B
                                 {
                                    display_cursorweg();
                                    char_height_mul=1;
                                    last_cursorspalte =curr_cursorspalte;
                                    
                                    curr_cursorspalte--;
                                    
                                 }break;
                              }
                           }break;
                              
                           case  2: // Expo
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // expowert A, nicht weiter nach links
                                 {
                                    
                                 }   break;
                                    
                                 case 1: // expowert B
                                 {
                                    display_cursorweg();
                                    char_height_mul=1;
                                    last_cursorspalte =curr_cursorspalte;
                                    
                                    curr_cursorspalte--;
                                    
                                 }break;
                              }
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                     else if (manuellcounter)
                     {
                        //switch((blink_cursorpos & 0xFF00)>>8) // zeile
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0:
                           {
                              
                              
                           }break;
                           case  1:
                           {
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                  }break; // Kanalscreen
                     
                  case MIXSCREEN: // Mixing
                  {
                     lcd_gotoxy(0,0);
                     
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (curr_cursorspalte)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte--;
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc(' ');
                           lcd_puthex(curr_cursorspalte);
                           lcd_putc(' ');
                           lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                        }
                        manuellcounter=0;
                        
                     }
                     else
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                     }
                     
                     
                  }break;
                     
                  case ZUTEILUNGSCREEN: // Zuteilung
                  {
                     lcd_gotoxy(0,0);
                     
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (curr_cursorspalte)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte--;
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc(' ');
                           lcd_puthex(curr_cursorspalte);
                           lcd_putc(' ');
                           lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                        }
                        manuellcounter=0;
                        
                     }
                     else if (manuellcounter)
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                     
                     
                  }break; // case Zuteilungscreen
                     
                  case AUSGANGSCREEN: // Ausgang
                  {
                     lcd_gotoxy(0,0);
                     
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (curr_cursorspalte)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte--;
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc(' ');
                           lcd_puthex(curr_cursorspalte);
                           lcd_putc(' ');
                           lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                        }
                        manuellcounter=0;
                        
                     }
                     else if (manuellcounter)
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                     
                     
                  }break; // case Ausgangscreen
                     
                     
                     
               }// switch curr_screen
            } // if AKTIONOK
            
         }break;
            
         case 5://
         {
#pragma mark Taste 5
            switch (curr_screen)
            {
#pragma mark Taste 5 HOMESCREEN
               case HOMESCREEN:
               {
                  // lcd_gotoxy(14,2);
                  // lcd_puts("*H5*");
                  Serial.printf("H5 \t");
                  if (tastaturstatus & (1<<AKTIONOK))
                  { 
                     tastaturstatus &=  ~(1<<AKTIONOK);
                     //lcd_putint2(startcounter);
                     //lcd_putc('*');
                     if ((startcounter == 0) && (manuellcounter)) // Settings sind nicht aktiv
                     {
                        //lcd_gotoxy(0,2);
                        //lcd_putc('1');
                        //lcd_putc(' ');
                        {
                           programmstatus |= (1<< SETTINGWAIT);
                           settingstartcounter=1;
                           manuellcounter = 1;
                        }
                     }
                     
                     else 
                        if (startcounter > 3) // Irrtum, kein Umschalten
                        {
                           //lcd_gotoxy(0,2);
                           //lcd_putc(' ');
                           // lcd_putc(' ');
                           programmstatus &= ~(1<< SETTINGWAIT);
                           settingstartcounter=0;
                           startcounter=0;
                           manuellcounter = 1;
                        }
                     
                        else
                        {
                           if ((programmstatus & (1<< SETTINGWAIT))&& (manuellcounter)) // Umschaltvorgang noch aktiv
                           {
                              //lcd_gotoxy(1,2);
                              //lcd_putc('G');
                              //lcd_putc('A'+settingstartcounter);
                              //lcd_putint2(settingstartcounter);
                              //lcd_gotoxy(2,2);
                              //lcd_putint1(settingstartcounter);
                              settingstartcounter++; // counter fuer klicks
                              Serial.printf("settingstartcounter: %d\n",settingstartcounter);
                              if (settingstartcounter == 3)
                              {
                                 //lcd_gotoxy(2,2);
                                 //lcd_putc('3');
                                 programmstatus &= ~(1<< SETTINGWAIT);
                                 programmstatus |=(1<<UPDATESCREEN);
                                 settingstartcounter=0;
                                 startcounter=0;
                                 eepromsavestatus = 0;
                                 // Umschalten
                                 display_clear();
                                 //lcd_putc('D');
                                 setsettingscreen();
                                 //lcd_putc('E');
                                 curr_screen = SETTINGSCREEN;
                                 curr_cursorspalte=0;
                                 curr_cursorzeile=0;
                                 last_cursorspalte=0;
                                 last_cursorzeile=0;
                                 blink_cursorpos=0xFFFF;
                                 
                                 manuellcounter = 1;
                              } // if settingcounter <
                              //manuellcounter = 0;
                           }
                        }
                     /*
                      if (startcounter > 3) // Irrtum, kein Umschalten
                      {
                      lcd_gotoxy(3,2);
                      lcd_putc('*');
                      
                      programmstatus &= ~(1<< SETTINGWAIT);
                      settingstartcounter=0;
                      startcounter=0;
                      manuellcounter = 1;
                      }
                      */
                  }
               }break;
                  
               case SAVESCREEN:
               {
               #pragma mark  5 SAVESCREEN
                 switch (curr_cursorspalte)
                  {
                     case 0: // sichern
                     {
                        
                        write_Ext_EEPROM_Settings();// neue Einstellungen setzen
                        
                        // In write_Ext_EEPROM_Settings wird masterstatus & 1<<DOGM_BIT gesetzt.
                        //  In der Loop wird damit
                        //    task_out |= (1<< RAM_SEND_DOGM_TASK);
                        //    task_outdata = curr_model;//modelindex;
                        // ausgeloest.

                     }break;
                        
                     case 1: // abbrechen
                     {
                        eepromsavestatus=0;
                        read_Ext_EEPROM_Settings();// zuruecksetzen
                     
                     }break;

                  }// switch curr_cursorspalte
                  
                  display_clear();
                  curr_screen=0;
                  curr_cursorspalte=0;
                  curr_cursorzeile=0;
                  last_cursorspalte=0;
                  last_cursorzeile=0;
                  blink_cursorpos = 0xFFFF;
                  
                  sethomescreen();
                  
                  
                  
               }break;

                  
               case SETTINGSCREEN: // 5 setting
               {
                  #pragma mark  5 SETTINGSCREEN
                  if (manuellcounter)
                  {
                     switch (curr_cursorzeile)
                     {
                        case 0: // Modell
                        {
                           // lcd_gotoxy(0,0);
                           //lcd_puthex(curr_cursorzeile);
                           //lcd_putc('*');
                           //lcd_puthex(curr_cursorspalte);
                           if (manuellcounter)
                           {
                              blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                              manuellcounter=0;
                           } // if manuellcounter
                        }break;
                           
                           
                        case 1: // Kanal
                        {
                          
                           // Zu Kanal-Screen
                           //blink_cursorpos =  cursorpos[2][0]; // canalcursor
                           if (manuellcounter)
                           {
                              display_clear();
                              
                              curr_screen = KANALSCREEN;
                              blink_cursorpos=0xFFFF;
                              curr_cursorspalte=0;
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              setcanalscreen();
                              manuellcounter=0;
                           }
                           
                           
                        }break;
                        case 2: // Mix
                        {
                           //zu Mix-Screen
                           if (manuellcounter)
                           {
                              display_clear();
                              
                              curr_screen = MIXSCREEN;
                              blink_cursorpos=0xFFFF;
                              curr_cursorspalte=0;
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              setmixscreen();
                              manuellcounter=0;
                           }
                           
                        }break;
                           
                        case 3: // Zuteilung
                        {
                           //zu Zuteilung-Screen
                           if (manuellcounter)
                           {
                              display_clear();
                              
                              curr_screen = ZUTEILUNGSCREEN;
                              blink_cursorpos=0xFFFF;
                              curr_cursorspalte=0;
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              setzuteilungscreen();
                              manuellcounter=0;
                           }
                           
                        }break;
                           
                        case 4: // Ausgang
                        {
                           //zu Zuteilung-Screen
                           if (manuellcounter)
                           {
                              display_clear();
                              
                              curr_screen = AUSGANGSCREEN;
                              blink_cursorpos=0xFFFF;
                              curr_cursorspalte=0;
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              setausgangscreen();
                              manuellcounter=0;
                           }
                           
                        }break;
                           
                           
                     }// switch curr_cursorzeile
                  } // if manuellcounter
                  
               }break;
                  
               case KANALSCREEN: // Kanal
               {
                  #pragma mark  5 KANALSCREEN
                  if (manuellcounter)
                  {
                     blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                     manuellcounter=0;
                  } // if manuellcounter
                  
                  /*
                   if (manuellcounter)
                   {
                   switch (curr_cursorzeile)
                   {
                   case 0: // Kanal
                   {
                   
                   switch (curr_cursorspalte)
                   {
                   case 0:
                   {
                   blink_cursorpos =  cursorpos[0][0]; // kanalcursor
                   }break;
                   case 1: // Richtung
                   {
                   
                   blink_cursorpos =  cursorpos[0][1]; // richtungpfeilcursor
                   
                   }break;
                   case 2: // funktion
                   {
                   blink_cursorpos =  cursorpos[0][2];
                   }break;
                   
                   
                   } // switch curr_cursorspalte
                   }break;// case 0
                   
                   case 1: // Level
                   {
                   if (curr_cursorspalte < 2)
                   {
                   switch (curr_cursorspalte)
                   {
                   case 0:// kanalwert A
                   {
                   
                   blink_cursorpos =  cursorpos[1][0]; // kanalwert A
                   }break;
                   
                   case 1: // kanalwert B
                   
                   {
                   blink_cursorpos =  cursorpos[1][1]; // kanalwert B
                   
                   }break;
                   case 2:
                   {
                   blink_cursorpos =  1; //fehler, back
                   }break;
                   
                   } //  case curr_cursorspalte
                   }
                   }break;
                   
                   case 2: // Expo
                   {
                   if (curr_cursorspalte < 2)
                   {
                   switch (curr_cursorspalte)
                   {
                   case 0:// expowert A
                   {
                   
                   blink_cursorpos =  cursorpos[2][0]; // expowert A
                   }break;
                   
                   case 1: // expowert B
                   
                   {
                   
                   blink_cursorpos =  cursorpos[2][1]; // expowert B
                   
                   }break;
                   case 2:
                   {
                   blink_cursorpos =  1; //fehler, back
                   
                   }break;
                   
                   } //  case curr_cursorspalte
                   }
                   }break;
                   
                   case 3:
                   {
                   
                   }break;
                   
                   
                   }// switch cursorzeile                        }break;
                   manuellcounter=0;
                   } // if manuellcounter
                   
                   //display_kanaldiagramm (char_x, uchar_y, level, expo, uint8_t typ )
                   // level: 0-3 expo: 0-3
                   //display_kanaldiagramm (64, 7, curr_levelarray[curr_kanal], curr_expoarray[curr_kanal], 1);
                   
                   //manuellcounter=0;
                   */
                  
               }break; // case kanalscreen
                  
                  
               case MIXSCREEN: // Mixing
               {
                  #pragma mark  5 MIXSCREEN
                  if (manuellcounter)
                  {
                     blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                     manuellcounter=0;
                  } // if manuellcounter
                  
                  /*
                   if (manuellcounter)
                   {
                   lcd_gotoxy(0,0);
                   lcd_puthex(curr_cursorzeile);
                   lcd_putc(' ');
                   lcd_puthex(curr_cursorspalte);
                   lcd_putc(' ');
                   //lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                   
                   switch (curr_cursorzeile)
                   {
                   case 0: // Mix 0
                   {
                   
                   switch (curr_cursorspalte)
                   {
                   case 0:
                   {
                   blink_cursorpos =  cursorpos[0][0]; // Typ
                   }break;
                   
                   case 1: // Kanal A
                   {
                   blink_cursorpos =  cursorpos[0][1]; // richtungpfeilcursor
                   
                   }break;
                   case 2: // Kanal B
                   {
                   blink_cursorpos =  cursorpos[0][2];
                   }break;
                   
                   
                   } // switch curr_cursorspalte
                   }break;// case 0
                   
                   case 1: // Mix 1
                   {
                   
                   switch (curr_cursorspalte)
                   {
                   case 0:// Typ
                   {
                   
                   blink_cursorpos =  cursorpos[1][0]; // Typ
                   }break;
                   
                   case 1: // kanalwert A
                   
                   {
                   blink_cursorpos =  cursorpos[1][1]; // Kanal A
                   
                   }break;
                   case 2:
                   {
                   blink_cursorpos =  cursorpos[1][2];// Kanal B
                   }break;
                   
                   } //  case curr_cursorspalte
                   
                   }break;
                   
                   case 2: //
                   {
                   }break;
                   
                   case 3:
                   {
                   
                   }break;
                   
                   
                   }// switch cursorzeile                        }break;
                   manuellcounter=0;
                   } // if manuellcounter
                   */
                  
                  
               }break; // case mixscreen
                  
               case ZUTEILUNGSCREEN: // Zuteilung
               case AUSGANGSCREEN:
               {
                  #pragma mark  5 AUSGANGSCREEN
                  if (manuellcounter)
                  {
                     blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                     manuellcounter=0;
                  } // if manuellcounter
                  
                  
                  
               }break; // case zuteilungscreen
                  
                  
            }// switch curr_screen
         } break; // 5
            
            
         case 6:// cursor nach rechts
         {
#pragma mark Taste 6
            if (tastaturstatus & (1<<AKTIONOK))
            {
               tastaturstatus &=  ~(1<<AKTIONOK);
               
               switch (curr_screen)
               {
                  case HOMESCREEN: // home
                  {
                     //lcd_gotoxy(14,2);
                     //lcd_puts("*H6*");
                  }break;
                     
                  case SAVESCREEN: // save
                  {
                     if (posregister[curr_cursorzeile][curr_cursorspalte+1]<0xFFFF)
                     {
                        display_cursorweg();
                        
                        char_height_mul=1;
                        last_cursorspalte =curr_cursorspalte;
                        curr_cursorspalte++;
                        blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                        //lcd_putc('+');
                     }
                     
                  }break;
                     
                  case SETTINGSCREEN: // Settings
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        //lcd_gotoxy(0,1);
                        //if (curr_cursorspalte <1) // curr_cursorzeile ist >0,
                        if (posregister[curr_cursorzeile][curr_cursorspalte+1]<0xFFFF)
                        {
                           if (curr_cursorzeile ==0)
                           {
                              char_height_mul=2;
                           }
                           display_cursorweg();
                           
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte++;
                           //lcd_putc('+');
                        }
                        else
                        {
                           
                           //lcd_putc('-');
                        }
                        manuellcounter=0;
                     }
                     else
                     {
                        switch((blink_cursorpos & 0xFF00)>>8) // zeile
                        {
                           case 0: // modell
                           {
                              
                           }break;
                           case  4:
                           {
                              blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                              //
                              
                        }// switch
                     }
                     
                     
                  }break;
                     
                  case KANALSCREEN: // Kanal
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile][curr_cursorspalte+1]<0xFFFF)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte++;
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter)
                     {
                        switch((blink_cursorpos & 0xFF00)>>8) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (blink_cursorpos & 0x00FF) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     }
                     
                     
                  }break;
                     
                  case MIXSCREEN: // Mixing
                  {
                     lcd_gotoxy(0,0);
                     
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile][curr_cursorspalte+1]< 0xFFFF)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte++;
                           
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc(' ');
                           lcd_puthex(curr_cursorspalte);
                           lcd_putc(' ');
                           lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                        }
                        manuellcounter=0;
                        
                     }
                     else
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                     }
                     
                     
                  }break;
                     
                     
                  case ZUTEILUNGSCREEN: // Zuteilung der Kanaele
                  {
                     lcd_gotoxy(0,0);
                     
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile][curr_cursorspalte+1]< 0xFFFF)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte++;
                           
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc(' ');
                           lcd_puthex(curr_cursorspalte);
                           lcd_putc(' ');
                           lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                        }
                        manuellcounter=0;
                        
                     }
                     else
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                     }
                     
                     
                  }break;
                     
                     
                  case AUSGANGSCREEN: // Ausgang
                  {
                     lcd_gotoxy(0,0);
                     lcd_puthex(curr_cursorzeile);
                     lcd_putc(' ');
                     lcd_puthex(curr_cursorspalte);
                     lcd_putc(' ');
                     
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile][curr_cursorspalte+1]< 0xFFFF)
                        {
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorspalte =curr_cursorspalte;
                           
                           curr_cursorspalte++;
                           
                           lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                        }
                        manuellcounter=0;
                        
                     }
                     else
                     {
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanaltext
                           {
                              switch (curr_cursorspalte) // cursorspalte
                              {
                                 case 0: // Kanalnummer
                                 {
                                    
                                 }   break;
                              }
                              // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                              
                              
                           }break;
                              
                              
                           case  1:
                           {
                              //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                           }break;
                              
                           case  2:
                           {
                              
                           }break;
                           case  3:
                           {
                              
                           }break;
                              //
                              
                        }// switch
                     }
                     
                     
                  }break;
                     
               }// switch
            } // if AKTIONOK
            manuellcounter=0;
            
            
         }break;
            
            
         case 7://home, in wenn 3* click aus default
         {
#pragma mark Taste 7
            //manuellcounter=0; // timeout zuruecksetzen
            //lcd_gotoxy(14,2);
            //lcd_puts("*7*");
            if (curr_screen) // nicht homescreen
            {
               if (tastaturstatus & (1<<AKTIONOK))
               {
                  tastaturstatus &=  ~(1<<AKTIONOK);
                  
                  switch (curr_screen)
                  {
                     case HOMESCREEN:
                     {
                        display_clear();
                        
                        curr_cursorspalte=0;
                        curr_cursorzeile=0;
                        last_cursorspalte=0;
                        last_cursorzeile=0;
                        blink_cursorpos = 0xFFFF;
                        
                        
                        
                        // curr_screen=HOMESCREEN;
                        sethomescreen();
                        
                        
                     }break;
                        
                     case TRIMMSCREEN:
                     {
                        display_clear();
                        
                        curr_cursorspalte=0;
                        curr_cursorzeile=0;
                        last_cursorspalte=0;
                        last_cursorzeile=0;
                        blink_cursorpos = 0xFFFF;
                        
                        
                        // curr_screen=HOMESCREEN;
                        sethomescreen();
                        programmstatus &= ~(1<<UPDATESCREEN);
                        
                     }break;
                        
                        
                     case SAVESCREEN:
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           display_clear();
                           curr_cursorspalte=0;
                           curr_cursorzeile=0;
                           last_cursorspalte=0;
                           //last_cursorzeile=0;
                           blink_cursorpos = 0xFFFF;
                           settingstartcounter=0;
                           startcounter=0;
                           
                           
                           curr_screen=SAVESCREEN;
                           //setsavescreen();
                           
                           
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                     }break;
                        
                        
                     case SETTINGSCREEN: // Settings
                     {
                        programmstatus &= ~(1<< SETTINGWAIT);
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           display_clear();
                           
                           curr_cursorspalte=0;
                           //curr_cursorzeile=0;
                           last_cursorspalte=0;
                           //last_cursorzeile=0;
                           blink_cursorpos = 0xFFFF;
                           settingstartcounter=0;
                           startcounter=0;
                           ///          
                           eepromsavestatus=0;
                           ///          
                           if (eepromsavestatus)
                           {
                              
                              curr_screen=SAVESCREEN;
                              setsavescreen();
                           }
                           else
                           {
                              Serial.printf("H\n");
                              //
                              //
                              curr_screen=HOMESCREEN;
                              sethomescreen();
                              programmstatus &= ~(1<<UPDATESCREEN);
                           }
                           
                           
                           manuellcounter=1;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                        
                     }break;
                        
                     case KANALSCREEN: // Settings
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           manuellcounter=0;
                           
                           blink_cursorpos = 0xFFFF;
                           //
                           if (curr_cursorspalte==0) // position am linken Rand
                           {
                              display_clear();
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              curr_screen=SETTINGSCREEN;
                              setsettingscreen();
                           }
                           else
                           {
                              
                           }
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                     }break;
                        
                        
                     case LEVELSCREEN: // Level einstellen
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           manuellcounter=0;
                           display_clear();
                           curr_screen=KANALSCREEN;
                           curr_cursorspalte=0;
                           curr_cursorzeile=1; // Zeile Level
                           last_cursorspalte=0;
                           last_cursorzeile=0;
                           blink_cursorpos = 0xFFFF;
                           setcanalscreen();
                           
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                        
                     }break;
                        
                     case EXPOSCREEN: // Level einstellen
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           manuellcounter=0;
                           display_clear();
                           curr_screen=KANALSCREEN;
                           curr_cursorspalte=0;
                           curr_cursorzeile=2; //Zeile Expo
                           last_cursorspalte=0;
                           last_cursorzeile=0;
                           blink_cursorpos = 0xFFFF;
                           setcanalscreen();
                           
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                        
                     }break;
                        
                     case MIXSCREEN: // Settings
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           manuellcounter=0;
                           
                           blink_cursorpos = 0xFFFF;
                           //
                           if (curr_cursorspalte==0) // position am linken Rand
                           {
                              display_clear();
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              curr_screen=SETTINGSCREEN;
                              setsettingscreen();
                           }
                           else
                           {
                              
                           }
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                     }break;
                        
                     case ZUTEILUNGSCREEN: // Settings
                        
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           manuellcounter=0;
                           
                           blink_cursorpos = 0xFFFF;
                           //
                           if (curr_cursorspalte==0) // position am linken Rand
                           {
                              display_clear();
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              curr_screen=SETTINGSCREEN;
                              setsettingscreen();
                           }
                           else
                           {
                              
                           }
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                     }break;
                        
                     case AUSGANGSCREEN: // Settings
                        
                     {
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           manuellcounter=0;
                           
                           blink_cursorpos = 0xFFFF;
                           //
                           if (curr_cursorspalte==0) // position am linken Rand
                           {
                              
                              display_clear();
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              curr_screen=SETTINGSCREEN;
                              setsettingscreen();
                           }
                           else
                           {
                              
                           }
                           manuellcounter=0;
                        }
                        else if (manuellcounter)
                        {
                           
                           blink_cursorpos = 0xFFFF;
                           manuellcounter=0;
                        }
                     }break;
                        
                  }// switch
               }
            }
            else // schon homescreen, motorzeit reset
            {
               startcounter = 0;
               /*
               lcd_gotoxy(0,2);
               lcd_putc(' ');
               lcd_putc(' ');
               lcd_putc(' ');
               lcd_gotoxy(14,2);
               lcd_puts("*H7*");
                */
               if (manuellcounter) // kurz warten
               {
                  if (tastaturstatus & (1<<AKTIONOK))
                  {
                     tastaturstatus &=  ~(1<<AKTIONOK);
                     programmstatus &= ~(1<<MOTOR_ON);
                     motorsekunde=0;
                     motorminute=0;
                     //update_time();
                     update_motorzeit();
                     
                  }
                  manuellcounter=0; // timeout zuruecksetzen
                  //update_time();
               }
            }
         }break;
            
            
         case 8://
         {
#pragma mark Taste 8
            if (tastaturstatus & (1<<AKTIONOK))
            {
               tastaturstatus &=  ~(1<<AKTIONOK);
               tastaturstatus |= (1<<UPDATEOK);
               switch (curr_screen)
               {
                  case HOMESCREEN: // home
                  {
                     // lcd_gotoxy(14,2);
                     // lcd_puts("*H8*");
                     break; // trimmscreen ev. korrupt
                     
                     if (manuellcounter)
                     {
                        display_clear();
                        
                        curr_screen = TRIMMSCREEN;
                        blink_cursorpos=0xFFFF;
                        curr_cursorspalte=0;
                        curr_cursorzeile=0;
                        last_cursorspalte=0;
                        last_cursorzeile=0;
                        settrimmscreen();
                        manuellcounter=0;
                     }
                     
                     
                     
                  }break;
#pragma mark 8 SETTINGSCREEN                   
                  case SETTINGSCREEN: // Settings
                  {
                     if ((blink_cursorpos == 0xFFFF) && manuellcounter) // kein Blinken
                     {
                        uint8_t cur = (posregister[curr_cursorzeile+1][curr_cursorspalte]&0xFF00)>>8;
                        Serial.printf("H8 curr_cursorzeile: %d curr_cursorspalte: %d\n",curr_cursorzeile,curr_cursorspalte);
                        /*
                         lcd_gotoxy(5,1);
                         lcd_puthex(curr_cursorzeile);
                         lcd_putc('*');
                         lcd_puthex((posregister[curr_cursorzeile+1][curr_cursorspalte]&0xFF00)>>8);
                         lcd_putc('*');
                         lcd_puthex((posregister[curr_cursorzeile+1][curr_cursorspalte]&0x00FF));
                         lcd_putc('*');
                         */
                        //if (curr_cursorzeile < 3 )//
                        if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)
                        {
                           if (curr_cursorzeile == 0)
                           {
                              char_height_mul = 2;
                           }
                           display_cursorweg();
                           char_height_mul = 1;
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile++;
                           
                           //lcd_gotoxy(19,1);
                           //lcd_putc('+');
                        }
                        else
                        {
                           
                           lcd_putc('-');
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter)
                     {
                        /*
                         lcd_gotoxy(0,1);
                         lcd_puthex((blink_cursorpos & 0xFF00)>>8);
                         lcd_putc('*');
                         lcd_puthex((blink_cursorpos & 0x00FF));
                         lcd_putc(' ');
                         */
                        //switch((blink_cursorpos & 0xFF00)>>8) // zeile
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // modell
                           {
                              //switch (blink_cursorpos & 0x00FF) // spalte
                              switch (curr_cursorspalte) // spalte
                              {
                                 case 0:
                                 {
                                    //lcd_putc('0');
                                    if (curr_model <8)
                                    {
                                       curr_model++;
                                    }
                                    
                                 }break;
                                    
                                 case 1:
                                 {
                                    //lcd_putc('1');
                                    if (curr_setting <4)
                                    {
                                       curr_setting++;
                                    }
                                    
                                 }break;
                                    
                                 case 2: //
                                 {
                                    
                                    
                                 }break;
                                    
                              } // switch Spalte
                              manuellcounter=0;
                           }break;
                              
                           case  1:
                           {
                              //lcd_putc('1');
                              
                           }break;
                              
                           case  2:
                           {
                              //lcd_putc('2');
                           }break;
                           case  3:
                           {
                              // lcd_putc('3');
                           }break;
                              //
                              
                        }// switch
                        manuellcounter =0;
                     }
                  }break;
#pragma mark 8 KANALSCREEN
                  case KANALSCREEN: // Kanalsettings
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                        {
                           if (curr_cursorzeile==1)
                           {
                              char_height_mul=2;
                           }
                           else
                           {
                              char_height_mul=1;
                           }
                           
                           display_cursorweg();
                           char_height_mul=1;
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile++;
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc('+');
                        }
                        else
                        {
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc('-');
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter)
                     {
                        
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanal
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Kanalnummer
                                 {
                                    if (curr_kanal < 7)
                                    {
                                       curr_kanal++;
                                    }
                                 }break;
                                 case 1: // Richtung
                                 {
                                    eepromsavestatus |= (1<<SAVE_EXPO);
                                    if (curr_expoarray[curr_kanal] & 0x80)
                                    {
                                       curr_expoarray[curr_kanal] &= ~0x80;
                                    }
                                    else
                                    {
                                       curr_expoarray[curr_kanal] |= 0x80;
                                    }
                                 }break;
                                    
                                 case 2: // Funktion
                                 {
                                    eepromsavestatus |= (1<<SAVE_FUNKTION);
                                    //lcd_gotoxy(5,1);
                                    //lcd_putc('*');
                                    //Bezeichnung von: FunktionTable[curr_funktionarray[curr_kanal]]&0x07
                                    // Funktion ist bit 0-2, Steuerdevice ist bit 4-6!!
                                    uint8_t tempfunktion = curr_funktionarray[curr_kanal]&0x07; //bit 0-2
                                    tempfunktion++;
                                    tempfunktion &= 0x07;
                                    
                                    //lcd_puthex(tempfunktion);
                                    curr_funktionarray[curr_kanal] = (curr_funktionarray[curr_kanal]&0xF0)|tempfunktion; // cycle in FunktionTable
                                    
                                    
                                    /*
                                     if (tempfunktion<8)
                                     {
                                     curr_funktionarray[curr_kanal] += 1;
                                     }
                                     */
                                 }break;
                                    
                              }// switch tempspalte
                              
                           }break;
                              
                           case  1: // Level
                           {
                              eepromsavestatus |= (1<<SAVE_LEVEL);
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Levelwert A
                                 {
                                    if (((curr_levelarray[curr_kanal] & 0x70)>>4)<5)
                                    {
                                       curr_levelarray[curr_kanal] += 0x10;
                                    }
                                    
                                 }break;
                                 case 1: // Levelwert B
                                 {
                                    if (((curr_levelarray[curr_kanal] & 0x07))<5)
                                    {
                                       curr_levelarray[curr_kanal] += 0x01;
                                    }
                                    
                                 }break;
                                    
                                 case 2: //
                                 {
                                    curr_cursorspalte = 1; // fehler, back
                                    
                                 }break;
                              }// switch tempspalte
                           }break;
                              
                           case  2: // Expo
                           {
                              eepromsavestatus |= (1<<SAVE_EXPO);
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Expowert A
                                 {
                                    if (((curr_expoarray[curr_kanal] & 0x70)>>4)<3)
                                    {
                                       curr_expoarray[curr_kanal] += 0x10;
                                    }
                                 }break;
                                    
                                 case 1: // Expowert B
                                 {
                                    if (((curr_expoarray[curr_kanal] & 0x07))<3)
                                    {
                                       curr_expoarray[curr_kanal] += 0x01;
                                    }
                                 }break;
                                    
                                 case 2: //
                                 {
                                    curr_cursorspalte = 1; // fehler, back
                                    
                                 }break;
                              }// switch tempspalte
                           }break;
                           case  4: // Typ
                           {
                              
                           }break;
                        }// switch
                        manuellcounter=0;
                     }
                  }break; // kanalscreen
                     
                  case MIXSCREEN:
                  {
#pragma mark 8 MIXSCREEN
                     /*
                      lcd_gotoxy(5,1);
                      lcd_puthex(curr_cursorzeile);
                      lcd_putc('*');
                      lcd_puthex((blink_cursorpos & 0xFF00)>>8); // Zeile
                      lcd_putc('*');
                      lcd_putc('*');
                      */
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                        {
                           char_height_mul=1;
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile++;
                           
                           //lcd_puthex(curr_cursorzeile);
                           //lcd_putc('+');
                        }
                        else
                        {
                           //lcd_puthex(curr_cursorzeile);
                           //lcd_putc('-');
                        }
                        //lcd_putint2(curr_cursorzeile);
                        
                        //lcd_putc(' ');
                        
                        
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        /*
                         lcd_gotoxy(0,0);
                         lcd_puthex(curr_cursorzeile);
                         lcd_putc(' ');
                         lcd_puthex(curr_cursorspalte);
                         lcd_putc(' ');
                         */
                        eepromsavestatus |= (1<<SAVE_MIX);
                        switch (curr_cursorspalte)
                        {
                           case 0: // Mix weiterschalten
                           {
                              if (curr_mixarray[2*curr_cursorzeile+1]<3)
                              {
                                 curr_mixarray[2*curr_cursorzeile+1] += 0x01;// Mix ist auf ungerader zeile
                              }
                           }break;
                              
                           case 1: // Kanal A weiterschalten
                           {
                              uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0xF0)>>4;// kanal a ist auf gerader zeile in bit 4-6, 8 ist OFF
                              
                              if (tempdata < 8) //
                              {
                                 curr_mixarray[2*curr_cursorzeile] += 0x10;
                                 
                              }
                              
                              
                           }break;
                              
                           case 2: // Kanal B weiterschalten
                           {
                              uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0x0F);// kanal b ist auf gerader zeile in bit 0-2, 8 ist OFF
                              
                              if (tempdata < 8)
                              {
                                 curr_mixarray[2*curr_cursorzeile] += 0x01;
                              }
                              
                           }break;
                              
                        }// switch curr_cursorspalte
                        
                        manuellcounter = 0;
                        
                     }
                  }break; // mixscreen
                     
                     
                  case ZUTEILUNGSCREEN:
                  {
#pragma mark 8 ZUTEILUNGSCREEN
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                        {
                           char_height_mul=1;
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           
                           curr_cursorzeile++;
                           
                           //lcd_puthex(curr_cursorzeile);
                           //lcd_putc('+');
                        }
                        else
                        {
                           //lcd_puthex(curr_cursorzeile);
                           //lcd_putc('-');
                        }
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                        /*
                         const char funktion0[] PROGMEM = "Seite  \0";
                         const char funktion1[] PROGMEM = "Hoehe  \0";
                         const char funktion2[] PROGMEM = "Quer   \0";
                         const char funktion3[] PROGMEM = "Motor  \0";
                         const char funktion4[] PROGMEM = "Quer L\0";
                         const char funktion5[] PROGMEM = "Quer R\0";
                         const char funktion6[] PROGMEM = "Lande  \0";
                         const char funktion7[] PROGMEM = "Aux    \0";
                         
                         */
                        
                        lcd_gotoxy(0,0);
                        lcd_puthex(curr_cursorzeile);
                        lcd_putc(' ');
                        lcd_puthex(curr_cursorspalte);
                        lcd_putc(' ');
                        eepromsavestatus |= (1<<SAVE_DEVICE);
                        switch (curr_cursorzeile)
                        {
                           case 0: // pitch vertikal
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // L_V index 1
                                 {
                                    // Kanalnummer im Devicearray increment
                                    if (((curr_devicearray[1]& 0x07))<8)
                                    {
                                       curr_devicearray[1]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_V index 3
                                 {
                                    if (((curr_devicearray[3]& 0x07))<8)
                                    {
                                       curr_devicearray[3]+= 0x01;
                                    }
                                 }break;
                              }// switch curr_cursorspalte
                           }break; // pitch v
                              
                           case 1: // pitch horizontal
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // L_H index 0
                                 {
                                    if (((curr_devicearray[0]& 0x07))<8)
                                    {
                                       // Kanalnummer fuer Device increment
                                       curr_devicearray[0]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_H index 2
                                 {
                                    if (((curr_devicearray[2]& 0x07))<8)
                                    {
                                       curr_devicearray[2]+= 0x01;
                                    }
                                 }break;
                              }// switch curr_cursorspalte
                           }break; // case spalte
                              
                              
                              
                           case 2: // schieber
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // S_L index 4
                                 {
                                    if (((curr_devicearray[4]& 0x07))<8)
                                    {
                                       // Kanalnummer fuer Device increment
                                       curr_devicearray[4]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // S_R index 5
                                 {
                                    if (((curr_devicearray[5]& 0x07))<8)
                                    {
                                       curr_devicearray[5]+= 0x01;
                                    }
                                 }break;
                              }// switch curr_cursorspalte
                           }break; // case spalte
                              
                        }//switch curr_cursorzeile
                        manuellcounter = 0;
                        
                     } // else if manuellcounter
                     
                  }break; // case zuteilung
                     
                  case AUSGANGSCREEN:
                  {
#pragma mark 8 AUSGANGSCREEN
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                        {
                           char_height_mul=1;
                           display_cursorweg();
                           last_cursorzeile =curr_cursorzeile;
                           if ((curr_cursorzeile < 3) || (curr_impuls > 3)) // Noch vor scrollen oder nach umschalten
                           {
                              curr_cursorzeile++;
                           }
                           else
                           {
                              curr_cursorzeile = 1; // Scroll
                           }
                           curr_impuls++;
                        }
                        
                        manuellcounter=0;
                     }
                     else if (manuellcounter) // blinken ist on
                     {
                        //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                        /*
                         const char funktion0[] PROGMEM = "Seite  \0";
                         const char funktion1[] PROGMEM = "Hoehe  \0";
                         const char funktion2[] PROGMEM = "Quer   \0";
                         const char funktion3[] PROGMEM = "Motor  \0";
                         const char funktion4[] PROGMEM = "Quer L\0";
                         const char funktion5[] PROGMEM = "Quer R\0";
                         const char funktion6[] PROGMEM = "Lande  \0";
                         const char funktion7[] PROGMEM = "Aux    \0";
                         */
                        
                        lcd_gotoxy(0,0);
                        lcd_puthex(curr_cursorzeile);
                        lcd_putc(' ');
                        lcd_puthex(curr_cursorspalte);
                        lcd_putc(' ');
                        eepromsavestatus |= (1<<SAVE_AUSGANG);
                        switch (curr_cursorspalte)
                        {
                           case 0: // Kanal
                           {
                              // Kanalnummer im Devicearray increment
                              if (((curr_ausgangarray[curr_cursorzeile]& 0x07))<8)
                              {
                                 curr_ausgangarray[curr_cursorzeile]+= 0x01;
                              }
                           }break;
                              
                           case 1: // Zeile nach unten verschieben
                           {
                              if (((curr_ausgangarray[curr_cursorzeile]& 0x07))<8)
                              {
                                 uint8_t tempzeilenwert =curr_ausgangarray[curr_cursorzeile];
                                 if (curr_impuls < 7) // nicht letzte Zeile
                                 {
                                    if ((curr_cursorzeile < 3) && (curr_impuls < 3)) // Noch vor scrollen, auf erster Seite
                                    {
                                       tempzeilenwert =curr_ausgangarray[curr_impuls];
                                       
                                       curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls+1]; // Wert von naechster zeile
                                       curr_ausgangarray[curr_impuls +1] = tempzeilenwert;
                                       // cursorzeile verschieben
                                       display_cursorweg();
                                       
                                       curr_cursorzeile++;
                                       // blink-cursorzeile verschieben
                                       blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                       
                                    }
                                    else  if ((curr_cursorzeile == 3) && (curr_impuls == 3))// zweitunterste Zeile, scrollen.
                                    {
                                       tempzeilenwert =curr_ausgangarray[curr_impuls];
                                       curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls+1]; // Wert von naechster zeile, noch auf dieser Seite
                                       curr_ausgangarray[curr_impuls +1] = tempzeilenwert;
                                       display_cursorweg();
                                       curr_cursorzeile = 1; // Scroll
                                       // blink-cursorzeile verschieben
                                       blink_cursorpos = cursorpos[1][curr_cursorspalte];
                                    }
                                    else  if ((curr_cursorzeile < 4) && (curr_impuls >3))// zweite Zeile oder mehr auf zweiter Seite
                                    {
                                       tempzeilenwert =curr_ausgangarray[curr_impuls];
                                       curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls+1]; // Wert von naechster zeile
                                       curr_ausgangarray[curr_impuls +1] = tempzeilenwert;
                                       // cursorzeile verschieben
                                       display_cursorweg();
                                       
                                       curr_cursorzeile++;
                                       // blink-cursorzeile verschieben
                                       blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                       
                                    }
                                    curr_impuls++;
                                 }
                                 else // letzte Zeile, mit erster zeile vertauschen
                                 {
                                    /*
                                     tempzeilenwert =curr_ausgangarray[curr_impuls];
                                     curr_ausgangarray[curr_impuls] =curr_ausgangarray[0]; // Wert von erster zeile
                                     curr_ausgangarray[0] = tempzeilenwert;
                                     display_cursorweg();
                                     curr_cursorzeile=0;
                                     curr_impuls =0;
                                     blink_cursorpos = cursorpos[0][curr_cursorspalte];
                                     */
                                 }
                              }
                              
                           }break;
                              
                              
                        }// switch curr_cursorspalte
                        manuellcounter = 0;
                        
                     } // else if manuellcounter
                     
                  }break; // case ausgang
                     
               }// switch
               if (tastaturstatus & (1<<UPDATEOK))
               {
                  tastaturstatus &= ~(1<<UPDATEOK);
                  Serial.printf("H8 update curscreen: %d\n",curr_screen);
                  update_screen();
               }
            }//if AKTIONOK
            
            
            
         }break; // case 8
            
         case 9://set, out wenn auf home
         {
#pragma mark Taste 9
           // lcd_gotoxy(14,2);
           // lcd_puts("*9*");

            if (manuellcounter) // kurz warten
            {
               if (tastaturstatus & (1<<AKTIONOK))
               {
                  tastaturstatus &=  ~(1<<AKTIONOK);
                  
                  programmstatus &= ~(1<<STOP_ON);
                  stopsekunde=0;
                  stopminute=0;
                  //update_time();
                  update_stopzeit();
               }
               manuellcounter=0; // timeout zuruecksetzen
               
            }
            
         }break;
            
         case 10:// *Manuell einschalten
         {
#pragma mark Taste 10
         }break;
            
         case 11://
         {
#pragma mark Taste 11
         }break;
            
         case 12: // # Normalbetrieb einschalten
         {
#pragma mark Taste 12
            
         }break;
         
         default:
            break;
      }
      Serial.printf("V\n");
      programmstatus |= (1<<UPDATESCREEN);
      tastaturstatus &= ~(1<<TASTEOK);
 
   } // if Tastaturok
       
   // 
   if (servostatus & (1<<USB_OK))
   {
#pragma mark start_usb
      if (sinceusb > 100)   
      {
         //Serial.printf("usb\n");
         sinceusb = 0;
         r = RawHID.recv(buffer, 0); 
         
         code = 0;
         if (r > 0) // 
         {
            Serial.printf("usb r: %d\n",r);
         //   noInterrupts();
            
            code = buffer[24];
            
            
            Serial.printf("\n***************************************  --->    rawhid_recv start code HEX: %02X\n",code);
            //Serial.printf("code: %d\n",code);
            usb_recv_counter++;
            uint8_t device = buffer[32];
            sendbuffer[24] =  buffer[32];
            
            switch (code)
            {   
               case 0xA4:
               {
                  Serial.printf("A4 clear\n");
               }break;
                  
      #pragma mark A5  GO HOME         
               case 0xA5: //  go home
               {
                  
               }break;
        
      #pragma mark default
               default:
               {
                  
               }break; // default
                  
                  
                  
            } // switch code
            interrupts();
       //     code=0;
         }// r > 0
         /**   End USB-routinen   ***********************/
       
      } // since usb

      servostatus &= ~(1<<USB_OK);
   }// usb
} // loop
