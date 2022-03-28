///
/// @mainpage   RC_22
///
/// @details   Description of the project
/// @n
/// @n
/// @n @a      Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author      Ruedi Heimlicher
/// @author      ___ORGANIZATIONNAME___
/// @date      04.02.2022 10:59
/// @version   <#version#>
///
/// @copyright   (c) Ruedi Heimlicher, 2022
/// @copyright   GNU General Public Licence
///
/// @see      ReadMe.txt for references
///


///
/// @file      RC_22.ino
/// @brief      Main sketch
///
/// @details   <#details#>
/// @n @a      Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author      Ruedi Heimlicher
/// @author      ___ORGANIZATIONNAME___
/// @date      04.02.2022 10:59
/// @version   <#version#>
///
/// @copyright   (c) Ruedi Heimlicher, 2022
/// @copyright   GNU General Public Licence
///
/// @see      ReadMe.txt for references
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
#include "expo.h"

#include <EEPROM.h>
// Display


extern const char *FunktionTable[];

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




volatile uint8_t           tastaturstatus=0;
#define TASTEOK   1
#define AKTIONOK 2
#define UPDATEOK 3
elapsedMillis zeitintervall;
uint8_t sekundencounter = 0;
//elapsedMillis sincelcd;
elapsedMicros sinceusb;




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
uint16_t displaycounter = 0;

#define IMPULSPIN  1

//IntervalTimer              delayTimer;

volatile uint8_t           servoindex = 0;


// Utilities

volatile uint16_t impulstimearray[NUM_SERVOS] = {};

volatile uint8_t adcpinarray[NUM_SERVOS] = {};
volatile uint16_t servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

// Prototypes
ADC *adc = new ADC(); // adc object

#define POT0LO 848
#define POT0HI 3600

#define POT1LO 660
#define POT1HI 3450


#define POTLO   1300
#define POTHI  2900
#define PPMLO  850
#define PPMHI  2150

#define SHIFT 0xFFFFF

#define IMPULSBREITE 20
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

volatile uint16_t pot0 = 0;

volatile unsigned char char_x = 0;
volatile unsigned char char_y = 0;
volatile unsigned char char_height_mul = 0;
volatile unsigned char char_width_mul = 0;

//uint8_t spieeprom_rdbyte(uint16_t addr);
uint16_t potgrenzearray[NUM_SERVOS][2]; // obere und untere Grenze von adc

volatile float quot = (ppmhi - ppmlo)/(pothi - potlo);

volatile float expoquot = (ppmhi - ppmlo)/2/0x200; // umrechnen der max expo (512) auf PPM  

volatile float quotarray[NUM_SERVOS] = {}; // Umrechnungsfaktor pro Pot

// display

volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint8_t                 settingstartcounter=0; // Counter fuer Klicks auf Taste 5

uint8_t testdata = 0;
volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor

volatile uint8_t              curr_levelarray[8] = {};//{0x11,0x22,0x33,0x44,0x00,0x00,0x00,0x00};
volatile uint8_t              curr_expoarray[8] = {0x33,0x22,0x33,0x44,0x00,0x00,0x00,0x00};
volatile uint8_t              curr_mixarray[8] = {};//{0x11,0x22,0x33,0x44,0x00,0x00,0x00,0x00};
uint8_t                       curr_mixstatusarray[4] = {24,96,0,0};//

uint8_t                       curr_mixkanalarray[4] = {16,50,0,0};//


volatile uint8_t              curr_funktionarray[8] = {}; //{0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
volatile uint8_t             curr_statusarray[8] = {};//{0x11,0x22,0x33,0x44,0x00,0x00,0x00,0x00};
volatile uint8_t             curr_ausgangarray[8] = {};//{0x11,0x22,0x33,0x44,0x00,0x00,0x00,0x00};
volatile uint8_t             curr_devicearray[8] = {};
volatile int8_t              curr_trimmungarray[8];
volatile int8_t              curr_richtung; // Bits fuer Richtung
volatile int8_t              curr_on; // Bits fuer on

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

volatile uint8_t                 displaystatus=0x00; // Tasks fuer Display

volatile uint8_t                  masterstatus = 0;
volatile uint8_t                  eepromsavestatus = 0;
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

uint16_t                paketcounter=0;
uint16_t                loopcounter=0;
uint8_t                adccounter=0;

volatile uint8_t levelwert=0x32;
volatile uint8_t levelb=0x12;

volatile uint8_t expowert=0;
volatile uint8_t expob=0;

// MARK: EEPROM Var
// EEPROM
volatile uint8_t  eeprom_indata=0;
volatile uint8_t  eeprom_errcount=0;
volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

static volatile uint8_t kontrollbuffer[USB_DATENBREITE]={};

static volatile uint8_t eeprombuffer[USB_DATENBREITE]={};

volatile uint8_t kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] = {};

uint8_t mixingsettingarray[ANZAHLMODELLE][4][2] = {};



uint8_t readdata=0xaa;


// USB
volatile uint8_t  usbtask = 0;

// MARK: Functions
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
      kanalimpulsTimer.begin(kanalimpulsfunktion,IMPULSBREITE); // neuer Kanalimpuls
   }
   else  // Paket beenden
   {
      servoimpulsTimer.end();
      servostatus |= (1<<PAUSE);
      servostatus |= (1<<ADC_OK); // ADCFenster starten
      //digitalWriteFast(IMPULSPIN,LOW);
   }
   //kanalimpulsTimer.begin(kanalimpulsfunktion,IMPULSBREITE); // neuer Kanalimpuls
   
}


void servopaketfunktion(void) // start Abschnitt
{ 
   servostatus &= ~(1<<PAUSE);
   servostatus |= (1<<PAKET);// neues Paket starten
   servostatus |= (1<<IMPULS);
   servoindex = 0; // Index des aktuellen impulses
   if (servostatus & (1<<RUN))
   {
      servoimpulsTimer.begin(servoimpulsfunktion,impulstimearray[servoindex]);
      
      kanalimpulsTimer.begin(kanalimpulsfunktion, IMPULSBREITE); // neuer Kanalimpuls
      digitalWriteFast(IMPULSPIN,HIGH);
      OSZI_B_LO();
      //paketcounter++;
   }
   paketcounter++;
}


// MARK: readSettings
void load_EEPROM_Settings(uint8_t model)
{
   Serial.printf("load_EEPROM_Settings model: %d\n",model);
   uint8_t pos = USB_DATA_OFFSET;
   for (uint8_t kanal = 0;kanal < 8;kanal++) // kanal mit je 4 bytes: status(modeöl, on, kanalindex, Ri), level. expo, device(Fkt, device)
   {
      //Serial.printf("load_EEPROM_Settings kanal: %d\n",kanal);
      for (uint8_t dataindex = 0;dataindex < 4;dataindex++)
      {
         // kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] 
         
         uint8_t data = EEPROM.read(model *  MODELSETTINGBREITE  + kanal * KANALSETTINGBREITE + dataindex);
         //Serial.printf("load_EEPROM_Settings kanal: %d dataindex: %d **  data: %d \n",kanal, dataindex, data);
         kanalsettingarray[model][kanal][dataindex] = data;
         
         switch (dataindex)
         {
            case 0: // status
            {
               curr_statusarray[kanal] = data;
               //Serial.printf("dataindex: %d curr_status: %d\n",dataindex,curr_statusarray[kanal]);
            }break;
            case 1: //leveld: 
            {
               curr_levelarray[kanal] = data;
            }break;
            case 2: // expo
            {
               curr_expoarray[kanal] = data;
            }break;
            case 3: // device
            {
               curr_devicearray[kanal] = data;
            }break;
         }// switch kanal
         
         //Serial.printf("load_EEPROM_Settings nach data: %d\n",data);
         
         //if (kanal==0)
         {
           // Serial.printf("load_EEPROM_Settings kanal: %d dataindex: %d **  data: %d \n",kanal, dataindex, data);
         }
         
      }// for dataindex
      
      //Serial.printf("load_EEPROM_Settings kanal: %d status; %d level: %d\n", kanal, curr_statusarray);
 
         
      pos += KANALSETTINGBREITE;
   } // for kanal 
   
   
   for (uint8_t i=0;i<8;i++)
   {
      Serial.printf("load_EEPROM_Settings i: %d status: %d level: %d expo: %d device: %d\n",i,curr_statusarray[i],curr_levelarray[i], curr_expoarray[i], curr_devicearray[i]);
   }

   
      
  }


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
    readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*EEPROM_MODELSETTINGBREITE;
   sei();
    // startadresse fuer Settings des models
    for (pos=0;pos<8;pos++)
    {
       curr_levelarray[pos] = EEPROM.read(readstartadresse+pos);
       
    }
    _delay_us(100);
   
    // Expo lesen
    readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
       curr_expoarray[pos] = EEPROM.read(readstartadresse+pos);
       
    }
    _delay_us(100);
   
   
   // Mix lesen
   //cli();
    readstartadresse = TASK_OFFSET  + FUNKTION_OFFSET + modelindex*SETTINGBREITE;
   //sei();
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
       curr_mixarray[pos] = EEPROM.read(readstartadresse+pos);
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
  // masterstatus |= (1<<HALT_BIT); // Halt-Bit aktiviert Task bei ausgeschaltetem Slave
//   MASTER_PORT &= ~(1<<SUB_BUSY_PIN);
   
   //   EEPROM.write(modelindex * MODELSETTINGBREITE + kanal * KANALSETTINGBREITE + dataindex,buffer[USB_DATA_OFFSET + kanal * KANALSETTINGBREITE + dataindex]);


   
//   lcd_clr_line(1);
//   lcd_putint(eepromsavestatus);
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
         
    //     eeprombyteschreiben(0xB0,writestartadresse+pos,curr_levelarray[pos]);
         EEPROM.write(writestartadresse+pos,curr_levelarray[pos]);
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
         //eeprombyteschreiben(0xB0,writestartadresse+pos,curr_expoarray[pos]);
         EEPROM.write(writestartadresse+pos,curr_expoarray[pos]);
      }
      _delay_us(100);
   }
   
   if (eepromsavestatus & (1<<SAVE_MIX))
   {
      eepromsavestatus &= ~(1<<SAVE_MIX);
      
      // Mix schreiben
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
         //eeprombyteschreiben(0xB0,writestartadresse+pos,curr_mixarray[pos]);
         EEPROM.write(writestartadresse+pos,curr_mixarray[pos]);
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
         //eeprombyteschreiben(0xB0,writestartadresse+pos,curr_funktionarray[pos]);
         EEPROM.write(writestartadresse+pos,curr_funktionarray[pos]);
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
   readdata = EEPROM.read(readadresse);
   //OSZI_B_HI;
   return readdata;

   
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
   
   //lcd_putc('*');

   return readdata;
}

uint8_t eeprompartlesen(uint16_t readadresse) //   us ohne lcd_anzeige
{
   //OSZI_B_LO;
    
   _delay_us(LOOPDELAY);
   //     OSZI_B_LO;
   
   uint8_t i=0;
   for (i=0;i<EE_PARTBREITE;i++)
   {      
       _delay_us(LOOPDELAY);
         readdata = (uint8_t)EEPROM.read(readadresse+i); // 220 us
         sendbuffer[EE_PARTBREITE+i] = readdata;
         _delay_us(LOOPDELAY);
      
   }
   //     OSZI_B_HI;
   
   //OSZI_C_HI;
   
   sendbuffer[0] = 0xDB;
    
   sendbuffer[1] = readadresse & 0x00FF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   sendbuffer[4] = 0xDB;
   
   //eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   usb_rawhid_send((void*)sendbuffer, 50);
   
   sei();
   //OSZI_B_HI;
   return readdata;
}

uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte) //   1 ms ohne lcd-anzeige
{
   //return;
   //OSZI_B_LO;
   EEPROM.write(writeadresse,eeprom_writedatabyte);
   
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
   /*
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
   */
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
   sendbuffer[6] = 0;
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

uint16_t eeprompartschreiben(void) // 23 ms
{
   //return;
   //OSZI_B_LO;
     uint16_t result = 0;
   
   eeprom_errcount=0;
   
   
   uint16_t abschnittstartadresse = eepromstartadresse ; // Ladeort im EEPROM
   
   /*
   lcd_gotoxy(4,1);
   lcd_putint12(abschnittstartadresse);
   lcd_putc(' ');
   lcd_puthex(eeprombuffer[32]);
   lcd_puthex(eeprombuffer[33]);
   lcd_putc(' ');
   lcd_puthex(eeprombuffer[34]);
   lcd_puthex(eeprombuffer[35]);
   */
   
    
   uint8_t w=0;
   uint8_t i=0;
   for (i=0;i<EE_PARTBREITE;i++)
   {
      uint16_t tempadresse = abschnittstartadresse+i;
      uint8_t databyte = eeprombuffer[EE_PARTBREITE+i]& 0xFF; // ab byte 32
      {
         sendbuffer[EE_PARTBREITE+i] = databyte;
      }
      result += databyte;
       
      _delay_us(LOOPDELAY);

       
      // Byte 0-31: codes
      // Byte 32-63: data
      
      EEPROM.write(tempadresse,databyte); // an abschnittstartadresse und folgende

       //spieeprom_wrbyte(0,13); // an abschnittstartadresse und folgende
      _delay_us(LOOPDELAY);
      
      
      eeprom_indata = (uint8_t)EEPROM.read(tempadresse);
      kontrollbuffer[EE_PARTBREITE+i] = eeprom_indata;
      
 
      if ((databyte - eeprom_indata)||(eeprom_indata - databyte))
      {
         eeprom_errcount++;
      }
   }
   _delay_us(LOOPDELAY);
   
   
   
   kontrollbuffer[0] = 0xCB;
   kontrollbuffer[1] = abschnittstartadresse & 0xFF;
   kontrollbuffer[2] = (abschnittstartadresse & 0xFF00)>>8;
   kontrollbuffer[3] = eeprom_errcount;
   kontrollbuffer[8] = 0xA1;
   kontrollbuffer[9] = 0xA2;
    
   usb_rawhid_send((void*)kontrollbuffer, 50);
   
   sei();
    //OSZI_B_HI;
   return result;
}

uint8_t* encodeEEPROMChannelSettings(uint8_t modelindex)
{
   uint8_t usbarray[USB_DATENBREITE] = {};
    // Kanal-Settings  
   for (uint8_t kanal = 0;kanal < 8;kanal++)
   {
      for (uint8_t dataindex = 0;dataindex < 4;dataindex++)
      {
         uint8_t data = EEPROM.read(modelindex * EEPROM_MODELSETTINGBREITE + kanal * KANALSETTINGBREITE + dataindex);
         sendbuffer[USB_DATA_OFFSET + kanal * KANALSETTINGBREITE + dataindex] = data;        usbarray[USB_DATA_OFFSET + kanal * KANALSETTINGBREITE + dataindex] = data;
         //Serial.printf("kanal: %d dataindex: %d data: %d \n", kanal, dataindex,data);
      } // for dataindex
   } // for kanal
   
   // Mixing-Settings
   
   
   return usbarray;
}

uint8_t* encodeEEPROMMixingSettings(uint8_t modelindex)
{
   Serial.printf("encodeEEPROMMixingSettings modelindex: %d\n",modelindex);
   uint8_t mixingarray[8] = {};
   for (uint8_t mixindex = 0;mixindex < 4;mixindex++)
   {
      uint8_t data0 = EEPROM.read(modelindex * EEPROM_MODELSETTINGBREITE + MODELSETTINGBREITE + 2*mixindex);
      mixingarray[2*mixindex] = data0;
      sendbuffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 2*mixindex] = data0;
      uint8_t data1 = EEPROM.read(modelindex * EEPROM_MODELSETTINGBREITE + MODELSETTINGBREITE + 2*mixindex+1);
      mixingarray[2*mixindex+1] = data1;
      sendbuffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 2*mixindex + 1] = data1;
   }

   for (uint8_t mixindex = 0;mixindex < 4;mixindex++)
   {
      Serial.printf("mixindex: %d data0: %d data1: %d\n",mixindex,mixingarray[2*mixindex], mixingarray[2*mixindex+1]);
   }

}

// current settings in USB
uint8_t encodeCurrentChannelSettings(uint8_t kanalindex, uint8_t modelindex)
{
   uint8_t usbarray[USB_DATENBREITE] = {0};
   Serial.printf("encodeCurrentChannelSettings curr_statusarray: %d curr_levelarray: %d curr_expoarray: %d curr_devicearray: %d\n",curr_statusarray[kanalindex], curr_levelarray[kanalindex],curr_expoarray[kanalindex],curr_devicearray[kanalindex]);
   
   
   sendbuffer[USB_DATA_OFFSET] = curr_statusarray[kanalindex];
   sendbuffer[USB_DATA_OFFSET + 1] = curr_levelarray[kanalindex];
   sendbuffer[USB_DATA_OFFSET + 2] = curr_expoarray[kanalindex];
   sendbuffer[USB_DATA_OFFSET + 3] = curr_devicearray[kanalindex];

   /*
   Serial.printf("*encodeCurrentChannelSettings\n");   
   for (uint8_t i = 0;i<USB_DATENBREITE;i++)
   {
      Serial.printf("\t%d",sendbuffer[i]);
   }
   Serial.printf("*\n");    
    */
   return 0;

}// encodeCurrentChannelSettings

uint8_t encodeChannelSettings( uint8_t modelindex)
{
   uint8_t usbarray[USB_DATENBREITE] = {0};
   for (uint8_t kanalindex = 0;kanalindex<8;kanalindex++)
   {
     // Serial.printf("encodeChannelSettings curr_statusarray: %d curr_levelarray: %d curr_expoarray: %d curr_devicearray: %d\n",curr_statusarray[kanalindex], curr_levelarray[kanalindex],curr_expoarray[kanalindex],curr_devicearray[kanalindex]);
      
      
      sendbuffer[USB_DATA_OFFSET + kanalindex * KANALSETTINGBREITE] = curr_statusarray[kanalindex];
      sendbuffer[USB_DATA_OFFSET + 1] = curr_levelarray[kanalindex];
      sendbuffer[USB_DATA_OFFSET + 2] = curr_expoarray[kanalindex];
      sendbuffer[USB_DATA_OFFSET + 3] = curr_devicearray[kanalindex];
   }
   /*
   Serial.printf("*encodeCurrentChannelSettings\n");   
   for (uint8_t i = 0;i<USB_DATENBREITE;i++)
   {
      Serial.printf("\t%d",sendbuffer[i]);
   }
   Serial.printf("*\n");    
    */
   return 0;

}// encodeCurrentChannelSettings

uint8_t decodeUSBMixingSettings(uint8_t buffer[USB_DATENBREITE])
{
   Serial.printf("decodeUSBMixingSettings\n");
   Serial.printf("mix data0 : %d data 1: %d\n",buffer[USB_DATA_OFFSET + MODELSETTINGBREITE],buffer[USB_DATA_OFFSET + MODELSETTINGBREITE +1 ]);
   // uint8_t mixingsettingarray[ANZAHLMODELLE][4][2] = {};
   
   uint8_t mix0 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE] ; // Bit 3
   uint8_t mix1 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 1]; // Bit 3
   
   uint8_t modelindex = mix0 & 0x03; // bit 0,1
   Serial.printf("modelindex: %d \n",modelindex);
   uint8_t mixart = (mix0 & 0x30) >> 4; // bit 4,5
   Serial.printf("mixart: %d \n",mixart);
   uint8_t mixnummer = (mix0 & 0xC0) >> 6; // bit 6,7
   Serial.printf("mixnummer: %d \n",mixnummer);
   uint8_t mixon = (mix0 & 0x08) >> 3; // Bit 3
   Serial.printf("mixon: %d \n",mixon);
   
   
   
   uint8_t mixkanala = mix1 & 0x07 ; // Bit 0-3
   Serial.printf("mixkanala: %d \n",mixkanala);
   uint8_t mixkanalb = (mix1 & 0x70) >> 4; // Bit 4-6
   Serial.printf("mixkanalb: %d \n", mixkanalb);
   
   
   for (uint8_t mixindex = 0;mixindex < 3;mixindex++)
   {
      uint8_t mix0 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 2 * mixindex] ; // mixstatus
      uint8_t mix1 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 2 * mixindex + 1]; // mixkanal
      Serial.printf("mixindex: %d mix0: %d mix1: %d\n",mixindex,mix0,mix1);
      mixingsettingarray[modelindex][mixindex][0] = mix0;
      mixingsettingarray[modelindex][mixindex][1] = mix1;
      
      curr_mixstatusarray[mixindex] = mix0;
      curr_mixkanalarray[mixindex] = mix1;

      EEPROM.write(((modelindex)* EEPROM_MODELSETTINGBREITE + MODELSETTINGBREITE + 2 * mixindex),mix0); // 
      EEPROM.write(((modelindex) * EEPROM_MODELSETTINGBREITE + MODELSETTINGBREITE  + 2 * mixindex + 1),mix1);

   }
   
   size_t n = sizeof(mixingsettingarray) / sizeof(mixingsettingarray[0]);
   Serial.printf("mixingsettingarray anz: %d: \n",n);
   for (uint8_t i=0;i<n;i++)
   {
      Serial.printf("mix0: %d  mix1: %d\n",mixingsettingarray[modelindex][i][0],mixingsettingarray[modelindex][i][1]);
   
   }
   return 0;
}

// USB in EEPROM und curr_einstellungen
uint8_t  decodeUSBChannelSettings(uint8_t buffer[USB_DATENBREITE])
{
   // uint8_t kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] = {};
   Serial.printf("decodeUSBChannelSettings\n");
   uint8_t modelindex = buffer[USB_DATA_OFFSET + modelindex * MODELSETTINGBREITE] & 0x03; // Bit 0,1 welches model soll gelesen werden

   uint8_t kanalindex =  buffer[USB_DATA_OFFSET + modelindex * KANALSETTINGBREITE] & 0x70; // Bits 4,5,6
   
   uint8_t on = (buffer[USB_DATA_OFFSET + modelindex * KANALSETTINGBREITE] & 0x08) >>8; // Bit 3
   uint8_t pos = USB_DATA_OFFSET;
    // Byte 0: Modell 3b, ON 1b, Kanalindex3b, RI 1b
   
   for (uint8_t kanal = 0;kanal < 8;kanal++)
   {
      for (uint8_t dataindex = 0;dataindex < 4;dataindex++)
      {
         // kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] 
         kanalsettingarray[modelindex][kanal][dataindex] = buffer[pos + dataindex];
         
         EEPROM.write(modelindex *  MODELSETTINGBREITE  + kanal * KANALSETTINGBREITE + dataindex,buffer[pos + dataindex]);
         if (kanal==0)
         {
            Serial.printf("kanal: %d dataindex: %d **  data: %d \n",kanal, dataindex, buffer[pos + dataindex]);
         }
      }
      
      if (kanal==0)
      {
         // aktuelle Werte setzen
         Serial.printf("curr_statusarray: %d\t",buffer[pos] );
         curr_statusarray[kanal] = buffer[pos ]; // Modell, ON, Kanal, RI
         
         Serial.printf("curr_levelarray: %d\t",buffer[pos+ 1]);
         curr_levelarray[kanal] = buffer[pos + 1]; // level A, level B
         
         Serial.printf("curr_expoarray: %d\t",buffer[pos + 2]);
         curr_expoarray[kanal] = buffer[pos + 2]; // expo A, expo B
         
         Serial.printf("curr_funktionarray: %d\n",buffer[pos + 3]);
         curr_devicearray[kanal] = buffer[pos + 3]; // funktion, device
         
         
         
      }
      pos += KANALSETTINGBREITE;
   }
   //uint8_t mixpos = USB_DATA_OFFSET + MODELSETTINGBREITE;
   //Serial.printf("*** pos nach for kanal: %d mixpos: %d\n",pos , mixpos);
   
   //_delay_ms(1);
   //mixsettingarray[ANZAHLMODELLE][4][2]
   
   /*
    uint8_t mix0 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE] ; // Bit 3
    uint8_t mix1 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 1]; // Bit 3
    
    uint8_t mixon = (mix0 & 0x08) >> 3; // Bit 3
    Serial.printf("mixon: %d \n",mixon);
    uint8_t mixart = (mix0 & 0x30) >> 4; // Bit 4,5
    Serial.printf("mixart: %d \n",mixart);
    
     uint8_t mixkanala = mix1 & 0x07 ; // Bit 0-3
    Serial.printf("mixkanala: %d \n",mixkanala);
    uint8_t mixkanalab = (mix1 & 0x70) >> 4; // Bit 4-6
    Serial.printf("mixkanalb: %d \n", mixkanalb);

    */

   
   
   
   // Test: rueckwaerts
  // uint8_t* eepromarray = encodeEEPROMChannelSettings(0);
   sendbuffer[0] = 0xF5;
   
   Serial.printf("USB \n");
   for (uint8_t i = 0;i<USB_DATENBREITE;i++)
   {
      //Serial.printf("i: %d usbdata: %d\t",i,sendbuffer[i]);
      Serial.printf("%d\t",sendbuffer[i]);
   }
   
   Serial.printf("\n");
   uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 50);
   Serial.printf("decodeUSBChannelSettings senderfolg: %d\n",senderfolg);
   return senderfolg;
   
  
   
}// decodeUSBChannelSettings





void displayinit()
{
   
 

   
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
 // while (!Serial) {
 //   yield();
 // }
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
      
      potgrenzearray[i][0] = potlo;
      potgrenzearray[i][1] = pothi;
      quotarray[i] = quot;
   //Serial.printf("i: %d wert:\t %d\n",i,wert);
      adcpinarray[i] = 0xFF;
      servomittearray[i] = 1500;
      
   }

   // init Pins
   
   // Servo 0
   potgrenzearray[0][0] = POT0LO;
   potgrenzearray[0][1] = POT0HI;
   pinMode(pot0_PIN, INPUT);
   adcpinarray[0] = pot0_PIN;
   
   quotarray[0] = float((ppmhi - ppmlo))/float((potgrenzearray[0][1] - potgrenzearray[0][0]));
   Serial.printf("potgrenzearray[0][0]: %d potgrenzearray[0][1]: %d  quotarray[0]: %.3f\n",potgrenzearray[0][0],potgrenzearray[0][1],quotarray[0]);
   // Servo 1
   potgrenzearray[1][0] = POT1LO;
   potgrenzearray[1][1] = POT1HI;
   pinMode(pot1_PIN, INPUT);
   adcpinarray[1] = pot1_PIN;
   quotarray[1] = (ppmhi - ppmlo)/(potgrenzearray[1][1] - potgrenzearray[1][0]);

   // Servo 2 
   pinMode(pot2_PIN, INPUT);
   adcpinarray[2] = pot2_PIN;
   
   // Servo 3
   pinMode(pot3_PIN, INPUT);
   adcpinarray[3] = pot3_PIN;
   adcpinarray[NUM_SERVOS-1] = 0xEF;// letzten Puls kennzeichnen
   
  
   
   pinMode(IMPULSPIN, OUTPUT);
   digitalWriteFast(IMPULSPIN,LOW);
   
   // Servopakete starten
   
   servoimpulsTimer.priority(3);
   kanalimpulsTimer.priority(2);
   servopaketTimer.priority(1);
   
   servopaketTimer.begin(servopaketfunktion, 20000);
   
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
   
 //  load_EEPROM_Settings(0);
   
   
   curr_levelarray[0] = 0x30;
   curr_levelarray[1] = 0x22;
   curr_levelarray[2] = 0x11;
   curr_levelarray[3] = 0x00;

   curr_expoarray[0] = 0x30;
   curr_expoarray[1] = 0x22;
   curr_expoarray[2] = 0x11;
   curr_expoarray[3] = 0x00;
   
   /*
   for (uint8_t i=0;i<8;i++)
   {
      Serial.printf("load_EEPROM_Settings i: %d status: %d level: %d expo: %d device: %d\n",i,curr_statusarray[i],curr_levelarray[i], curr_expoarray[i], curr_devicearray[i]);
   }
*/
   
   //curr_levelarray[0] = 1;
   sethomescreen();
   //display_write_str("abc",2);

   servostatus &= ~(1<<RUN);
   
   //curr_mixstatusarray = {24,96,0,0};
   //curr_mixkanalarray = [16,50,0,0];
   //mixingsettingarray[0][0][0] = curr_mixstatusarray;
   for (uint8_t i=0;i<4;i++)
   {
      mixingsettingarray[0][i][0] = curr_mixstatusarray[0];
      mixingsettingarray[0][i][1] = curr_mixkanalarray[0];
   }
   // Tastatur
   
   pinMode(TASTATURPIN, INPUT);
   //Serial.printf("W1: %d W2: %d W3: %d W4: %d W5: %d W6: %dW7: %d W8: %d W9: %d \n",WERT1, WERT2, WERT3, WERT4, WERT5, WERT6, WERT7, WERT8, WERT9);
   pinMode(24, OUTPUT);
   digitalWriteFast(24, 0);
}

// Add loop code
void loop()
{
  
   if (!(servostatus & (1<<RUN))) // first run
   {
      Serial.printf("run start\n");
      servostatus |= (1<<RUN);
      // Mitte lesen
      //Serial.printf("Mitte lesen quot: %.4f expoquot:  %.4f\n",quot, expoquot);
      for (uint8_t i=0;i<NUM_SERVOS;i++)
      {
         if (adcpinarray[i] < 0xFF) // PIN belegt
         {
            //Pot 0
            uint16_t potwert = adc->adc0->analogRead(adcpinarray[i]);
            
            float potmitte = (potgrenzearray[i][1] + potgrenzearray[i][0])/2;
            
            float ppmfloat = PPMLO + quot *(float(potwert)-POTLO);
            uint16_t ppmint = uint16_t(ppmfloat);
            //Serial.printf("i: %d potwert: %d ppmint: %d potmitte: %.4f\n",i,potwert,ppmint,potmitte);
            //Serial.printf("i: %d potgrenzearray[i][0]: %d potgrenzearray[i][1]: %d quotarray[]: %.3f\n",i,potgrenzearray[i][0],potgrenzearray[i][1],quotarray[i]);

            servomittearray[i] = ppmint;

            
         }
         
      } // for servo
      for (uint8_t i=0;i<8;i++)
      {
         Serial.printf("load_EEPROM_Settings i: %d status: %d level: %d expo: %d device: %d\n",i,curr_statusarray[i],curr_levelarray[i], curr_expoarray[i], curr_devicearray[i]);
      }

      /*
      for (uint8_t i=0;i<8;i++)
      {
         Serial.printf("i: %d curr_levelarray: %d curr_funktionarray %d: \n",i,curr_levelarray[i],curr_funktionarray[i]);
      }
       */
      digitalWriteFast(24, 1);
   }
   loopcounter++;
   // MARK:  -  sinc > 1000
   /*
   if (sincelastseccond > 1000)
   {
      
      sincelastseccond = 0;
      
      sendesekunde++;
      
      if (manuellcounter && (blink_cursorpos < 0xFFFF))
      {
         //display_setcursorblink(sendesekunde);
      }

      if (curr_screen )
      {
         //update_sendezeit();
         //display_setcursorblink(sendesekunde);
      }
      //Serial.printf("update Kanalscreen CC\n");
      //Serial.printf("motorsekunde: %d programmstatus: %d manuellcounter: %d\n",motorsekunde, programmstatus, manuellcounter);
      Serial.printf("paketcounter \t %d  \t  startcounter: \t  %d  \t loopcounter: \t  %d  \t adccounter:  \t %d\n",paketcounter,startcounter, loopcounter , adccounter);

      if (sendesekunde == 60)
      {
         sendeminute++;
         sendesekunde = 0;
         if (curr_screen == 0)
         {
         refresh_screen();
         }
      }
      if (sendeminute == 60)
      {
         sendestunde++;
         sendeminute = 0;
      }
      //Serial.printf("sendesekunde: %d programmstatus: %d servostatus: %d manuellcounter: %d curr_screen: %d\n",sendesekunde, programmstatus,servostatus,  manuellcounter, curr_screen);

      
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
         if (curr_screen == 0)
         {
            //update_time();
         }
         
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
         if (curr_screen == 0)
         {
            //update_time();
         }
      }
      displaystatus |= (1<<UHR_UPDATE);//XX
   } // 1000
   */
    // MARK:  -  sinc > 500
   if (zeitintervall > 500) 
   {   
      sendbuffer[0] = 0xA0;
    //  uint8_t senderfolg = RawHID.send(sendbuffer, 50);
      sekundencounter++;
      if (sekundencounter%2)
      {
          
         sendesekunde++;
         
         if (manuellcounter && (blink_cursorpos < 0xFFFF))
         {
            //display_setcursorblink(sendesekunde);
         }

         if (curr_screen )
         {
            //update_sendezeit();
            //display_setcursorblink(sendesekunde);
         }
         //Serial.printf("update Kanalscreen CC\n");
         //Serial.printf("motorsekunde: %d programmstatus: %d manuellcounter: %d\n",motorsekunde, programmstatus, manuellcounter);
         //Serial.printf("paketcounter \t %d  \t  startcounter: \t  %d  \t loopcounter: \t  %d  \t adccounter:  \t %d\n",paketcounter,startcounter, loopcounter , adccounter);

         if (sendesekunde == 60)
         {
            sendeminute++;
            sendesekunde = 0;
            if (curr_screen == 0)
            {
               Serial.printf("refresh_screen sendeminute: %d\n",sendeminute);
               servostatus &=  ~(1<<RUN); 
               
               refresh_screen();
               servostatus |=  (1<<RUN); 
            }
         }
         if (sendeminute == 60)
         {
            sendestunde++;
            sendeminute = 0;
         }
         //Serial.printf("sendesekunde: %d programmstatus: %d servostatus: %d manuellcounter: %d curr_screen: %d\n",sendesekunde, programmstatus,servostatus,  manuellcounter, curr_screen);

         
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
            if (curr_screen == 0)
            {
               //update_time();
            }
            
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
            if (curr_screen == 0)
            {
               //update_time();
            }
         }
         displaystatus |= (1<<UHR_UPDATE);//XX
      }      //Serial.printf("pot0 %d pot1 %d\n",impulstimearray[0], impulstimearray[1]);
      if (manuellcounter && (blink_cursorpos < 0xFFFF))
      {
         display_setcursorblink(updatecounter);
      }
      updatecounter++;
                                
      //Serial.printf("send usb: pot0 %d\n",pot0);
      
      //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
      manuellcounter++;
      //Serial.printf("manuellcounter: %d\n",manuellcounter);
      zeitintervall = 0;
      //Serial.printf("paketcounter %d  startcounter: %d loopcounter: %d \n",paketcounter,startcounter, loopcounter);
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
            curr_screen=0;
            startcounter=0;
            settingstartcounter=0;
            
            /*
             lcd_gotoxy(0,2);
             lcd_putc(' ');
             lcd_putc(' ');
             lcd_putc(' ');
             */
         }
         
         //
      }
   
   
   
   
   }// zeitintervall
   
// MARK:  -  ADC_OK
   /* 
    // Mixing
    for (i=0;i<4;i++) // 50 us
    {
       // Mixing lesen
       
       uint8_t mixcanal = Mix_Array[2*i];
       if (mixcanal ^ 0x88) // 88 bedeutet OFF
       {
          // Setting nicht OFF
          uint8_t mixart = Mix_Array[2*i+1] & 0x07; // Art des mixings
          uint8_t canala = mixcanal & 0x07;         // beteiligter erster Kanal
          uint8_t canalb = (mixcanal & 0x70) >>4;   // beteiligter zweiter Kanal
          
          switch (mixart) // mixart ist gesetzt
          {
             case 1: // V-Mix
             {
                // Originalwert fuer jeden Kanal lesen
                canalwerta = Servo_ArrayInt[canala];// Wert fuer ersten Kanal
                canalwertb = Servo_ArrayInt[canalb];// Wert fuer zweiten Kanal
                
                // Wert mixen und neu speichern
                Servo_ArrayInt[canala] = canalwerta + canalwertb;
                Servo_ArrayInt[canalb] = canalwerta - canalwertb;
             
             }break;
          } // switch
       }
    }
    */
   // Impulsfolge ist fertig, Zeit nutzen. 
   // Zuerst potis lesen
   if (servostatus & (1<<ADC_OK)) //     20us pro kanal ohne printf
   {
      adccounter++;
      //manuellcounter++;
      //Serial.printf("+A+");
      OSZI_C_LO();
      displaycounter++;
      for (uint8_t i=0;i<NUM_SERVOS;i++)
      {
         //Serial.printf("A\n");
         //Serial.printf("i: %d pin: %d\n",i,adcpinarray[i]);
         if (adcpinarray[i] < 0xFF) // PIN belegt
         {
            if (adcpinarray[i] == 0xEF) // last
            {
               //Serial.printf("A: i: %d\n",i);
               impulstimearray[i] = 1000; // 
            }
            else 
            {
               
               //Serial.printf("+B+");
               uint16_t potwert = adc->adc0->analogRead(adcpinarray[i]);
               //uint16_t potwert = adc->adc0->analogRead(14);
               //float ppmfloat = PPMLO + quot *(float(potwert) - POTLO);

               float ppmfloat = PPMLO + quotarray[i] *(float(potwert) - potgrenzearray[i][0]);
               uint16_t ppmint = uint16_t(ppmfloat);
               if (i == 0)
               {
                  pot0 = ppmint;
               }
               if (i < 4)
               {
               //Serial.printf("pot %d pot1 %d\t",i, ppmint);
               sendbuffer[ADCOFFSET + 2*i] = (ppmint & 0x00FF); // LO
               sendbuffer[ADCOFFSET + 2*i + 1] = (ppmint & 0xFF00)>>8; // Hi
               }
               
               //Serial.printf("pot0 %d pot1 %d\n",pot0, pot1);
               uint16_t expo  = 0;  
               uint16_t ppmabs  = 0;  
               uint16_t expoint  = 0;  
               
               if (i < 2)
               {
                  float mittefloat = float( servomittearray[i]);
                  
                  float ppmapsfloat = (abs(ppmfloat - mittefloat))/expoquot;
                  int16_t ppmabs = uint16_t(ppmapsfloat);
                  //int16_t ppmabs = abs( ppmint - servomittearray[i]); // Abweichung PPM von mitte
                  
                  if (ppmabs > 0x200) // 512
                  {
                     //Serial.printf("servo %d ppmabs zu gross: %d\n",ppmabs);
                     ppmabs = 0x200;
                  }
                  
                  expo  = expoarray[0][ppmabs] * expoquot;
                  
                  if (ppmint < servomittearray[i])
                  {
                     expoint = servomittearray[i] - expo;
                  }
                  else
                  {
                     expoint = servomittearray[i] + expo;
                  }
                  
                  if (displaycounter == 14)
                  {
                     //Serial.printf("displaycounter: %d\n", displaycounter);
                     //Serial.printf("servo %d potwert: %d  ppmint: %d ppmabs: %d expo: %d expoint: %d quot: %.3f quotarray[i]: %.2f\n",i,potwert,ppmint,ppmabs, expo, expoint, quot, quotarray[i]);
                  }
                  {
                    // if (i<2)
                     {
                      //  Serial.printf("servo %d potwert: %d    ppmint: %d ppmabs: %d expo: %d expoint: %d displaycounter: %d\n",i,potwert,ppmint,ppmabs, expo, expoint,displaycounter);
                     }
                  }
                  impulstimearray[i] = expoint;
                  
                }                 
               //Serial.printf("pot0 %d pot1 %d\n",impulstimearray[0], impulstimearray[1]);
               //impulstimearray[i] = ppmint;
               //impulstimearray[i] = potwert;
            }
         }
         
         //Serial.printf("C\n");
      } // for i
      if (displaycounter == 50)
      {
         
         displaycounter=0;
         //Serial.printf("-\n");
      }
      // MARK:  - MIXING
// Mixing abarbeiten
      for (uint8_t mixindex=0;mixindex<4;mixindex++) // 50 us
      {
         //uint8_t mix0 = mixingsettingarray[0][mixindex][0];
         uint8_t mix0 = curr_mixstatusarray[mixindex];
         if ((mixindex == 0) && (displaycounter == 20))
         {
      //      Serial.printf("ADC mixindex: %d: mix0 : %d curr_mixstatusarray: %d\n",mixindex, mixingsettingarray[0][mixindex][0], curr_mixstatusarray[mixindex]);
            
         }
         
         
         if (mix0 & 0x08) // ON
         {
             uint8_t mixart = (mix0 & 0x30) >> 4;
            //uint8_t mix1 = mixingsettingarray[0][mixindex][1];
            uint8_t mix1 = curr_mixkanalarray[mixindex];
            
            uint8_t kanala = (mix1 & 0x03);
            uint8_t kanalb = (mix1 & 0x30) >> 4;
            
            if ((displaycounter == 20) )
               {
                 //Serial.printf("mix on mix0: %d mix1: %d \n",mix0,mix1 );
               
               }
          
            switch (mixart) // mixart ist gesetzt
            {
               case 1: // V-Mix
               {
                  // Originalwert fuer jeden Kanal lesen
                  uint16_t kanalwerta = impulstimearray[kanala];// Wert fuer ersten Kanal
                  uint16_t kanalwertb = impulstimearray[kanalb];// Wert fuer zweiten Kanal

                  uint16_t mixkanalwerta = impulstimearray[kanala];// Wert fuer ersten Kanal
                  uint16_t mixkanalwertb = impulstimearray[kanalb];// Wert fuer zweiten Kanal
                  
                  uint16_t mittea = servomittearray[kanala];
                  uint16_t mitteb = servomittearray[kanalb];
                  
                  //Serial.printf("mixindex: %d kanalwerta: %d kanalwertb: %d mittea: %d mitteb: %d\n",mixindex,kanalwerta,kanalwertb, mittea, mitteb); 
                  uint16_t diffa = 0;
                  if(kanalwerta > mittea)
                  {
                     diffa = kanalwerta - mittea;
                     mixkanalwerta = mittea + diffa;
                     mixkanalwertb = mitteb + diffa;
                     
                  }
                  else 
                  {
                     diffa = mittea - kanalwerta;
                     mixkanalwerta = mittea - diffa;
                     mixkanalwertb = mitteb - diffa;
                     
                     //diffa |= 0xF000; // bit 15
                  }
                  uint16_t diffb = 0;
                  if(kanalwertb > mitteb)
                  {
                     diffb = kanalwertb - mitteb;
                     mixkanalwerta += diffb;
                     mixkanalwertb -= diffb;
                     
                  }
                  else 
                  {
                     diffb = mitteb - kanalwertb;
                     //diffa |= 0xF000; // bit 15
                     mixkanalwerta -= diffb;
                     mixkanalwertb += diffb;
                  }
                  
                  if (displaycounter == 20)
                     {
                 // Serial.printf("mixindex: %d kanalwerta : \t%d \tkanalwertb : \t%d\t mixkanalwerta: \t%d\t mixkanalwertb: \t%d\t diffa: \t%d\t diffb: \t%d\t \n",mixindex,kanalwerta ,kanalwerta , mixkanalwerta, mixkanalwertb, diffa, diffb); 
                     }
                  
                  // Wert mixen und neu speichern
                  
                  impulstimearray[kanala] = mixkanalwerta;
                  impulstimearray[kanalb] = mixkanalwertb;
                  
               }break;
            } // switch
            
         }// if on
         
      }//for i
       servostatus &= ~(1<<ADC_OK);
      
      // MARK - Tastatur ADC
      Tastenwert=(adc->adc0->analogRead(TASTATURPIN))>>2;
      if (curr_screen )
      {
         //Serial.printf("C");
      }
      //Tastenwert = 0;
      if (Tastenwert>10)
      {
         if (!(tastaturstatus & (1<<TASTEOK)))
         {
            //Serial.printf("*AA*");
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
            //Serial.printf("*BB*");
            if (Tastenwertdiff < 6)
            {
               //Serial.printf("*C*");
               if (tastaturcounter < ADCTIMEOUT)
               {
                  //Serial.printf("D");
                  tastaturcounter++;
                  
                  if (tastaturcounter == ADCTIMEOUT) // Messung ist OK
                  {
                     
                     Tastenindex = Tastenwahl(Tastenwert); // taste pressed
                     //Serial.printf("Tastenwert: %d Tastenindex: %d\n",Tastenwert,Tastenindex);
                     tastaturstatus |= (1<<TASTEOK);
                     tastaturstatus |= (1<<AKTIONOK); // nur eine Aktion zulassen bis zum naechsten Tastendruck
                     programmstatus |= (1<< LEDON);
                     
                     display_set_LED(1);
                     
                  }
               }
             
            } // if Tastenwertdiff 
             
            else
            {
               //Serial.printf("F");
               tastaturcounter = 0;
            }
            //Serial.printf("TASTEOK end\n");
         }   // TASTEOK 
         //Serial.printf("Tastenwert end\n");
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
      
     // zeit fuer display-updates
      // 1. variable Daten auf homescreen updaten
      {
         //OSZI_D_LO();
         if (curr_screen == 0)
         {
            update_time(updatecounter & 0x0f);
         }
         
         
         
         
         if (updatecounter & 0x80)
         {
            //pot0 = sendbuffer[9]<<8 | sendbuffer[8];
            //Serial.printf("send usb: pot0 %d\n",pot0);
   //         uint8_t senderfolg = RawHID.send(pot0, 50);
            //Serial.printf("senderfolg: %d\n",senderfolg);
         }
         updatecounter++;
         //OSZI_D_HI();
         
         displaystatus &= ~(1<<UHR_UPDATE);
         
     //    Serial.printf("X");
         
      }
      //usb_rawhid_send((void*)sendbuffer, 50);
      //uint8_t senderfolg = RawHID.send(sendbuffer, 50);
      //Serial.printf("usb senderfolg: %d \n");
      //Serial.printf("+B+\n");
      servostatus |= (1<<USB_OK);
   }
   
#pragma mark - Tasten   
   
   if (tastaturstatus & (1<<TASTEOK))
   {
      //Serial.printf("U Tastenindex: %d\n",Tastenindex);
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
                 // tastaturstatus |= (1<<UPDATEOK);

               }
               manuellcounter=0;
             }

         }break;
            
         case 2://
         {
#pragma mark - Taste 2
            if (tastaturstatus & (1<<AKTIONOK))
            {
               tastaturstatus &=  ~(1<<AKTIONOK);
               tastaturstatus |= (1<<UPDATEOK);
               
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
                     
                     
                     
#pragma mark 2 SETTINGSCREEN                     
                  case SETTINGSCREEN: // T2 Settings
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        Serial.printf("T2 SETTINGSCREEN no blink");
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
                        Serial.printf("T2 SETTINGSCREEN mauellcounter>0 blink");
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
                        Serial.printf("T2 SETTINGSCREEN mauellcounter>0 end");
                     } 
                  }break;
#pragma mark 2 KANALSCREEN                    
                  case KANALSCREEN: // Kanalsettings
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
                        
                        Serial.printf("T2 KANALSCREEN curr_cursorspalte: %d\n",curr_cursorspalte);
                        
                        switch(curr_cursorzeile) // zeile
                        {
                           case 0: // Kanalnummer
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
                                    
                                 case 1: // Richtung toggle
                                 {
                                    eepromsavestatus |= (1<<SAVE_EXPO);
                                    //if (curr_settingarray[curr_kanal][1] & 0x80)
                                    if (curr_statusarray[curr_kanal] & 0x80)
                                    {
                                       curr_statusarray[curr_kanal] &= ~0x80;
                                    }
                                    else
                                    {
                                       curr_statusarray[curr_kanal] |= 0x80;
                                    }
                                 }break;
                                    
                                 case 2: // Funktion
                                 {
                                     //Bezeichnung von: FunktionTable[curr_funktionarray[curr_kanal]]
                                    // Funktion ist bit 0-2, Steuerdevice ist bit 4-6!!
                                    eepromsavestatus |= (1<<SAVE_FUNKTION);
                                    if (curr_devicearray[curr_kanal] )
                                    {
                                       curr_devicearray[curr_kanal] -= 0x01;
                                    }
                                    break;
                                    uint8_t tempfunktion = curr_funktionarray[curr_kanal]&0x07; //bit 0-2
                                    Serial.printf("T2 curr_funktionarray vor: %d\n",curr_funktionarray[curr_kanal]);
                                    Serial.printf("T2 zeile %d spalte %d  tempfunktion vor: %d\n",curr_cursorzeile,curr_cursorspalte,tempfunktion);
 
                                    if (tempfunktion) // noch nicht 0
                                    {
                                       tempfunktion--;
                                    }
                                    
                                    
                                    curr_funktionarray[curr_kanal] = (curr_funktionarray[curr_kanal]&0xF0)|tempfunktion; // cycle in FunktionTable: Bit 4-7 BitOR mit tempfunktion
                                    Serial.printf("T2 zeile %d spalte %d  tempfunktion nach: %d\n",curr_cursorzeile,curr_cursorspalte,tempfunktion);
                                    Serial.printf("T2 curr_funktionarray nach: %d\n",curr_funktionarray[curr_kanal]);

                                 }break;
                                    
                                    
                                    
                              }// switch tempspalte
                              
                           }break;
                              
                           case  1: // Zeile level
                           {
                              eepromsavestatus |= (1<<SAVE_LEVEL);
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Levelwert A
                                 {
                                    Serial.printf("2 Levelwert A curr level: %d\n",curr_levelarray[curr_kanal] );
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
                                    uint8_t expowert = (curr_expoarray[curr_kanal] & 0x70)>>4;
                                    Serial.printf("T2 Expowert A curr expo vor: %d expowert %d\n",curr_expoarray[curr_kanal],expowert );
                                    if ((curr_expoarray[curr_kanal] & 0x70)>>4) // noch nicht 0
                                    {
                                       curr_expoarray[curr_kanal] -= 0x10;
                                    }
                                    expowert = (curr_expoarray[curr_kanal] & 0x70)>>4;
                                    Serial.printf("T2 Expowert A curr expo nach: %d expowert %d\n",curr_expoarray[curr_kanal],expowert );
  
                                 }break;
                                    
                                 case 1: // Expowert B
                                 {
                                    if ((curr_expoarray[curr_kanal] & 0x07)) // noch nicht 0
                                    {
                                       curr_expoarray[curr_kanal] -= 0x01;
                                    }
                                 }break;
                                    
                                 case 2: //
                                 {
                                   //curr_cursorspalte = 1; // fehler, back
                                 }break;
                                    
                              }// switch curr_cursorspalte
                           }break;
                              
                           case  4:
                           {
                           }break;
                              //
                              
                        }// switch
                        manuellcounter=0;
                     } // if manuellcounter
                  }break; // canalscreen
#pragma mark 2 MIXSCREEN                      
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
                        uint8_t mixindex = (curr_mixstatusarray[curr_cursorzeile] & 0xC0) >> 6; // bit 6,7

                        Serial.printf("\nT2 mixscreen curr_cursorspalte: %d curr_cursorzeile: %d\n",curr_cursorspalte, curr_cursorzeile);
                        eepromsavestatus |= (1<<SAVE_MIX);
                        switch (curr_cursorspalte)
                        {
                           case 0: // T2 Mix weiterschalten
                           {
                              //Serial.printf("T2 mixscreen mixindex: %d\n",mixindex);

                              //Serial.printf("T2 mixscreen curr_mixstatusarray(%d) vor: %d\n",curr_cursorzeile,curr_mixstatusarray[curr_cursorzeile]); 
                              if ((curr_mixstatusarray[curr_cursorzeile] & 0x30) > 0x10  )
                              {
                                 curr_mixstatusarray[curr_cursorzeile] -= 0x10;
                                 
                                  
                              }
                              //Serial.printf("mixscreen curr_mixstatusarray(%d) nach: %d\n",curr_cursorzeile,curr_mixstatusarray[curr_cursorzeile]);

                           }break;
                              
                           case 1: // T2 ON toggle
                           {
                              //uint8_t mixindex = (curr_mixstatusarray[0] & 0xC0) >> 6; // bit 6,7
                              if (curr_mixstatusarray[curr_cursorzeile] & 0x08)
                              {
                                 curr_mixstatusarray[curr_cursorzeile] &= ~0x08;
                                 
                              }
                              else
                              {
                                 curr_mixstatusarray[curr_cursorzeile] |= 0x08;
                                // mixingsettingarray[0][mixindex][0] |= 0x08;
                              }
                              mixingsettingarray[0][mixindex][0] = curr_mixstatusarray[curr_cursorzeile];
                           }break;
   
                              
                              
                           case 2: // Kanal A zurueckschalten
                           {
                              
                               uint8_t kanala = curr_mixkanalarray[curr_cursorzeile] & 0x07;
                              
                              if (kanala) //
                              {
                                 curr_mixkanalarray[curr_cursorzeile] -= 0x01;
                              
                              }
                              
                              
                           }break;
                              
                           case 3: // Kanal B zurueckschalten
                           {
                              uint8_t kanalb = curr_mixkanalarray[curr_cursorzeile] & 0x70;
                             
                             if (kanalb) //
                             {
                                curr_mixkanalarray[curr_cursorzeile]  -= 0x10;
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
                                    if (((curr_statusarray[1]& 0x70))) // bit 4-6
                                    {
                                       curr_statusarray[1]-= 0x10;
                                    }
                                 }break;
                                    
                                 case 1: // R_V index 3
                                 {
                                    if (((curr_statusarray[3]& 0x07)))
                                    {
                                       curr_statusarray[3]-= 0x01;
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
                                    if (((curr_statusarray[0]& 0x07)))
                                    {
                                       // Kanalnummer fuer Device decrement
                                       curr_statusarray[0]-= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_H index 2
                                 {
                                    if (((curr_statusarray[2]& 0x07)))
                                    {
                                       curr_statusarray[2]-= 0x01;
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
                                    if (((curr_statusarray[4]& 0x07)))
                                    {
                                       // Kanalnummer fuer Device increment
                                       curr_statusarray[4]-= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // S_R index 5
                                 {
                                    if (((curr_statusarray[5]& 0x07)))
                                    {
                                       curr_statusarray[5]-= 0x01;
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
            if (tastaturstatus & (1<<UPDATEOK))
            {
               
               //Serial.printf("H%d update curscreen: %d\n",Tastenindex,curr_screen);
               tastaturstatus &= ~(1<<UPDATEOK);
               
               update_screen();
               //Serial.printf("H%d update curscreen end %d\n");
            }

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
                  tastaturstatus |= (1<<UPDATEOK);

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
               tastaturstatus |= (1<<UPDATEOK);
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
                     
                     
                  case SETTINGSCREEN: // T4 Settings
                  {
                    // lcd_gotoxy(14,2);
                    // lcd_puts("*S4*");
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
               
               if (tastaturstatus & (1<<UPDATEOK))
               {
                 
                  //Serial.printf("H4 update curscreen: %d\n",curr_screen);
                  tastaturstatus &= ~(1<<UPDATEOK);
                  update_screen();
               }
 
            } // if AKTIONOK
            
         }break;
            
         case 5://
            {
#pragma mark Taste 5
               
               if (tastaturstatus & (1<<AKTIONOK))
               {
                  tastaturstatus &=  ~(1<<AKTIONOK);
                  tastaturstatus |= (1<<UPDATEOK);
                  servostatus &=  ~(1<<RUN);
                  switch (curr_screen)
                  {
#pragma mark Taste 5 HOMESCREEN
                        
                     case HOMESCREEN:
                     {
                        // lcd_gotoxy(14,2);
                        // lcd_puts("*H5*");
                        Serial.printf("*H5* \t");
                        
                        
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
                                 //Serial.printf("settingstartcounter: %d\n",settingstartcounter);
                                 if (settingstartcounter == 3)
                                 {
                                    //servostatus &=  ~(1<<RUN);
                                    OSZI_A_LO();
                                    //lcd_gotoxy(2,2);
                                    //lcd_putc('3');
                                    Serial.printf("*** settingstartcounter 3\n");
                                    programmstatus &= ~(1<< SETTINGWAIT);
                                    //           programmstatus |= (1<<UPDATESCREEN);
                                    settingstartcounter=0;
                                    startcounter=0;
                                    eepromsavestatus = 0;
                                    // Umschalten
                                    display_clear();
                                    //lcd_putc('D');
                                    // Serial.printf("*H5*D \t");
                                    setsettingscreen();
                                    //lcd_putc('E');
                                    //Serial.printf("*H5*E \t");
                                    curr_screen = SETTINGSCREEN;
                                    curr_cursorspalte=0;
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    blink_cursorpos=0xFFFF;
                                    // Serial.printf("*H5*F \n");
                                    manuellcounter = 1;
                                    //servostatus |=  (1<<RUN);
                                    OSZI_A_HI();
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
                        
                        
                     case SETTINGSCREEN: // T5 setting
                     {
#pragma mark  5 SETTINGSCREEN
                        //Serial.printf("display T5 Settingscreen curr_cursorzeile: %d\n",curr_cursorzeile);
                        
                        if (manuellcounter)
                        {
                           //servostatus &=  ~(1<<RUN); 
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
                                    //Serial.printf("T5 Kanalscreen start\n");
                                    OSZI_D_LO();
                                    setcanalscreen();
                                    OSZI_D_HI();
                                    manuellcounter=0;
                                    //Serial.printf("T5 zu Kanalscreen end tastaturstatus: %d\n",tastaturstatus);
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
                           //servostatus |=  (1<<RUN); 
                        } // if manuellcounter
                        //Serial.printf("T5 end\n");
                     }break;
                        
                     case KANALSCREEN: // Kanal
                     {
                        Serial.printf("T5 Kanalscreen\n");
#pragma mark  5 KANALSCREEN
                        if (manuellcounter)
                        {
                           blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                           manuellcounter=0;
                        } // if manuellcounter
                        
                        
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
                        
                        
                     }break; // case kanalscreen
                        
                        
                     case MIXSCREEN: // Mixing
                     {
#pragma mark  5 MIXSCREEN
                        if (manuellcounter)
                        {
                           //blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                           //manuellcounter=0;
                        //} // if manuellcounter
                        
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
                                    
                                 case 1: // OK
                                 {
                                    uint8_t mixindex = (curr_mixstatusarray[curr_cursorzeile] & 0xC0) >> 6;
                                    Serial.printf("T5 mixindex: %d: mix0 vor: %d curr_mixstatusarray%d\n",mixindex, mixingsettingarray[0][mixindex][0], curr_mixstatusarray[curr_cursorzeile]);
                                    if (curr_mixstatusarray[curr_cursorzeile] & 0x08)
                                    {
                                       Serial.printf("T5 08 da\n");
                                       curr_mixstatusarray[curr_cursorzeile] &= ~0x08;
                                       //mixingsettingarray[0][mixindex][0] = curr_mixstatusarray[curr_cursorzeile];
                                    }
                                    else
                                    {
                                       Serial.printf("T5 08 nicht da\n");
                                       curr_mixstatusarray[curr_cursorzeile] |= 0x08;
                                       //mixingsettingarray[0][mixindex][0] |= 0x08;
                                       
                                    }
                                    // update
                                    mixingsettingarray[0][mixindex][0] = curr_mixstatusarray[curr_cursorzeile];
                                    
                                    
                                   // Serial.printf("T5 mixindex: %d: mix0 nach: %d\n",mixindex, mixingsettingarray[0][mixindex][0]);
                                    //blink_cursorpos =  cursorpos[0][1]; // OK
                                 }break;
                                 case 2: // Kanal A
                                 {
                                    blink_cursorpos =  cursorpos[0][2]; // Kanal A
                                 }break;
                                 case 3: // Kanal B
                                 {
                                    blink_cursorpos =  cursorpos[0][3]; // Kanal N
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
                                    blink_cursorpos =  cursorpos[1][1]; // OK
                                    
                                 }break;
                                 case 2:
                                 {
                                    blink_cursorpos =  cursorpos[1][2];// Kanal A
                                 }break;
                                 case 3:
                                 {
                                    blink_cursorpos =  cursorpos[1][3];// Kanal B
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
                  
                  if (tastaturstatus & (1<<UPDATEOK))
                  {
                     
                     //Serial.printf("H%d update curscreen: %d\n",Tastenindex,curr_screen);
                     tastaturstatus &= ~(1<<UPDATEOK);
                     update_screen();
                  }
                  //servostatus |=  (1<<RUN);
                  
               } // if A
               
               
               //Serial.printf("T end T5\n");
            } break; // 5
            
            
         case 6:// cursor nach rechts
         {
#pragma mark Taste 6
            if (tastaturstatus & (1<<AKTIONOK))
            {
               tastaturstatus &=  ~(1<<AKTIONOK);
               tastaturstatus |= (1<<UPDATEOK);
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
                     
                  case SETTINGSCREEN: // T6 Settings
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
                           //Serial.printf("T6 kanalscreen cursorspalte vor: %d\n",curr_cursorspalte);

                           curr_cursorspalte++;
                           //Serial.printf("T6 kanalscreen cursorspalte nach: %d\n",curr_cursorspalte);

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
            if (tastaturstatus & (1<<UPDATEOK))
            {
               
               //Serial.printf("H%d update curscreen: %d\n",Tastenindex,curr_screen);
               tastaturstatus &= ~(1<<UPDATEOK);
               update_screen();
            }

            manuellcounter=0;
            
            
         }break;
            
            
         case 7://home, in wenn 3* click aus default
         {
#pragma mark Taste 7
            //manuellcounter=0; // timeout zuruecksetzen
            //lcd_gotoxy(14,2);
            //lcd_puts("*7*");
            //Serial.printf("*** T7 ***\n");
            if (curr_screen) // nicht homescreen
            {
               servostatus &=  ~(1<<RUN);
               if (tastaturstatus & (1<<AKTIONOK))
               {
                  //Serial.printf("*** T7 AKTIONOK***\n");
                  tastaturstatus &=  ~(1<<AKTIONOK);
                  tastaturstatus |= (1<<UPDATEOK);

                  
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
                        Serial.printf("T7 >Trimmscreen\n");
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
                        Serial.printf("T7 > Savescreen\n");
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
                        
                        //Serial.printf("T7  settingscreen\n");
                        programmstatus &= ~(1<< SETTINGWAIT);
                        if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                        {
                           display_clear();
                           
                           curr_cursorspalte=0;
                           curr_cursorzeile=8;
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
                              //Serial.printf("H\n");
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
                  
                  if (tastaturstatus & (1<<UPDATEOK))
                  {
                     //Serial.printf("H%d update curscreen: %d\n",Tastenindex,curr_screen);
                     tastaturstatus &= ~(1<<UPDATEOK);
                     update_screen();
                     
                  }
                  servostatus |=  (1<<RUN); 

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
                     servostatus &=  ~(1<<RUN); 
                     refresh_screen();
                     servostatus |=  (1<<RUN); 
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
               servostatus &=  ~(1<<RUN); 
               switch (curr_screen)
               {
                  case HOMESCREEN: // home
                  {
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                     {
                        
                        Serial.printf("T8 homescreen no blink curr_cursorzeile: %d\n",curr_cursorzeile);
                        /*
                        curr_cursorzeile=8;
                        blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                        manuellcounter=0;
                         */
                        
                     }
                     else if (manuellcounter)
                     {
                        Serial.printf("T8 homescreen blink\n");
                     }
                     // lcd_gotoxy(14,2);
                     // lcd_puts("*H8*");
                     
                     break; // 
                     
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
                  case SETTINGSCREEN: // T8 Settings
                  {
                     if ((blink_cursorpos == 0xFFFF) && manuellcounter) // kein Blinken
                     {
                        uint8_t cur = (posregister[curr_cursorzeile+1][curr_cursorspalte]&0xFF00)>>8;
                        //Serial.printf("H8 curr_cursorzeile: %d curr_cursorspalte: %d\n",curr_cursorzeile,curr_cursorspalte);
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
                     if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken, cursor bewegen
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
                              Serial.printf("T8 curr_cursorspalte: %d\n",curr_cursorspalte);
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Kanalnummer
                                 {
                                    if (curr_kanal < 7)
                                    {
                                       curr_kanal++;
                                    }
                                 }break;
                                 case 1: // Richtung toggle
                                 {
                                    eepromsavestatus |= (1<<SAVE_EXPO);
                                    if (curr_statusarray[curr_kanal] & 0x80) // Bit 7 gesetzt
                                    {
                                       curr_statusarray[curr_kanal] &= ~0x80;
                                    }
                                    else
                                    {
                                       curr_statusarray[curr_kanal] |= 0x80;
                                    }
                                 }break;
                                    
                                 case 2: // Funktion
                                 {
                                    eepromsavestatus |= (1<<SAVE_FUNKTION);
                                      if (curr_devicearray[curr_kanal] < 4)
                                    {
                                       curr_devicearray[curr_kanal] += 0x01;
                                    }
                                      }break;
                                    
                              }// switch tempspalte
                              
                           }break;
                              
                           case  1: // Zeile Level
                           {
                              eepromsavestatus |= (1<<SAVE_LEVEL);
                              
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Levelwert A
                                 {
                                    Serial.printf("8 Levelwert A curr level: %d\n",curr_levelarray[curr_kanal] );
                                    if (((curr_levelarray[curr_kanal] & 0x70)>>4)<4) // noch weiterer Wert da
                                    {
                                       curr_levelarray[curr_kanal] += 0x10;
                                    }
                                    
                                 }break;
                                    
                                 case 1: // Levelwert B
                                 {
                                    if (((curr_levelarray[curr_kanal] & 0x07))<4) // noch weiterer Wert da
                                    {
                                       curr_levelarray[curr_kanal] += 0x01;
                                    }
                                    
                                 }break;
                                    
                                 case 2: //
                                 {
                                    //curr_cursorspalte = 1; // fehler, back
                                    
                                 }break;
                              }// switch curr_cursorspalte
                           }break;
                              
                           case  2: // Expo
                           {
                              eepromsavestatus |= (1<<SAVE_EXPO);
                              //Serial.printf("T8 Expowert curr level: %d\n",curr_expoarray[curr_kanal] );
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Expowert A
                                 {
                                    uint8_t expowert = (curr_expoarray[curr_kanal] & 0x70)>>4;
                                    //Serial.printf("T8 Expowert A curr expo vor: %d expowert: %d\n",curr_expoarray[curr_kanal] , expowert);
                                    if (((curr_expoarray[curr_kanal] & 0x70)>>4)<5) // noch mehr Werte da
                                    {
                                       //Serial.printf("8 Expowert change\n");
                                       curr_expoarray[curr_kanal] += 0x10;
                                    }
                                    expowert = (curr_expoarray[curr_kanal] & 0x70)>>4;
                                    Serial.printf("T8 Expowert A curr expo nach: %d expowert: %d\n",curr_expoarray[curr_kanal] , expowert);
                                }break;
                                    
                                 case 1: // Expowert B
                                 {
                                    if (((curr_expoarray[curr_kanal] & 0x07))<5)
                                    {
                                       curr_expoarray[curr_kanal] += 0x01;
                                    }
                                 }break;
                                    
                                 case 2: //
                                 {
                                   // curr_cursorspalte = 1; // fehler, back
                                    
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
                        /*
                         uint8_t modelindex = mix0 & 0x03; // bit 0,1
                         Serial.printf("modelindex: %d \n",modelindex);
                         uint8_t mixart = (mix0 & 0x30) >> 4; // bit 4,5
                         Serial.printf("mixart: %d \n",mixart);
                         uint8_t mixnummer = (mix0 & 0xC0) >> 6; // bit 6,7
                         Serial.printf("mixnummer: %d \n",mixnummer);
                         uint8_t mixon = (mix0 & 0x08) >> 3; // Bit 3
                         Serial.printf("mixon: %d \n",mixon);

                         */
                        Serial.printf("\nT8 mixscreen curr_cursorspalte: %d curr_cursorzeile: %d\n",curr_cursorspalte, curr_cursorzeile);
                        eepromsavestatus |= (1<<SAVE_MIX);
                        switch (curr_cursorspalte)
                        {
                           case 0: //T8 Mix weiterschalten
                           {
                              uint8_t mixnummer = (curr_mixstatusarray[curr_cursorzeile] & 0xC0) >> 6; // bit 6,7
                              Serial.printf("T8 mixscreen mixnummer: %d\n",mixnummer);
                              Serial.printf("T8 mixscreen curr_mixstatusarray(%d) vor: %d\n",curr_cursorzeile,curr_mixstatusarray[curr_cursorzeile]);
                              if ((curr_mixstatusarray[curr_cursorzeile] & 0x30) < 0x30  )
                              {
                                 if ((curr_mixstatusarray[curr_cursorzeile] & 0x30) == 0) // noch OFF
                                 {
                                    Serial.printf("mixscreen > ON\n");
                                       //curr_mixstatusarray[curr_cursorzeile] |= 0x30; // ON
                                  }
                                 curr_mixstatusarray[curr_cursorzeile] += 0x10;
                              }
                              
                              Serial.printf("mixscreen curr_mixstatusarray(%d) nach: %d\n",curr_cursorzeile,curr_mixstatusarray[curr_cursorzeile]);
     
                           }break;
                              
                           case 1: //T8 ON toggle
                           {
                              uint8_t mixnummer = (curr_mixstatusarray[curr_cursorzeile] & 0xC0) >> 6; // bit 6,7
                              if (curr_mixstatusarray[curr_cursorzeile] & 0x08)
                              {
                                 curr_mixstatusarray[curr_cursorzeile] &= ~0x08;
                              }
                              else
                              {
                                 curr_mixstatusarray[curr_cursorzeile] |= 0x08;
                              }
                           }break;

                              
                           case 2: // Kanal A weiterschalten
                           {
                              uint8_t kanala = curr_mixkanalarray[curr_cursorzeile] & 0x07;
                             
                             if (kanala < 7) //
                             {
                                curr_mixkanalarray[curr_cursorzeile] += 0x01;
                             }
                              
                              
                           }break;
                              
                           case 3: // Kanal B weiterschalten
                           {
                              uint8_t kanalb = curr_mixkanalarray[curr_cursorzeile] & 0x70;
                             
                             if (kanalb < 0x70) //
                             {
                                curr_mixkanalarray[curr_cursorzeile] += 0x10;
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
                                    if (((curr_statusarray[1]& 0x07))<8)
                                    {
                                       curr_statusarray[1]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_V index 3
                                 {
                                    if (((curr_statusarray[3]& 0x07))<8)
                                    {
                                       curr_statusarray[3]+= 0x01;
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
                                    if (((curr_statusarray[0]& 0x07))<8)
                                    {
                                       // Kanalnummer fuer Device increment
                                       curr_statusarray[0]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // R_H index 2
                                 {
                                    if (((curr_statusarray[2]& 0x07))<8)
                                    {
                                       curr_statusarray[2]+= 0x01;
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
                                    if (((curr_statusarray[4]& 0x07))<8)
                                    {
                                       // Kanalnummer fuer Device increment
                                       curr_statusarray[4]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // S_R index 5
                                 {
                                    if (((curr_statusarray[5]& 0x07))<8)
                                    {
                                       curr_statusarray[5]+= 0x01;
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
                  
                  //Serial.printf("H%d update curscreen: %d\n",Tastenindex,curr_screen);
                  tastaturstatus &= ~(1<<UPDATEOK);
                  update_screen();
               }
               servostatus |=  (1<<RUN); 
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
                  tastaturstatus |= (1<<UPDATEOK);

                  
                  programmstatus &= ~(1<<STOP_ON);
                  stopsekunde=0;
                  stopminute=0;
                  //update_time();
      //*            update_stopzeit();
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
      } // switch Tastenindex
      
  //    Serial.printf("V end Tastatur tastaturstatus: %d\n",tastaturstatus);
 //     programmstatus |= (1<<UPDATESCREEN);
      if (tastaturstatus & (1<<UPDATEOK))
      {
         
         //Serial.printf("Tastenindex update curscreen: %d\n",curr_screen);
         tastaturstatus &= ~(1<<UPDATEOK);
         //update_screen();
      }

      tastaturstatus &= ~(1<<TASTEOK);
      //Serial.printf("update Kanalscreen AA\n");
   } // if Tastaturok
       
   // 
   if (servostatus & (1<<USB_OK))
   {
      //Serial.printf("USB OK\n");
#pragma mark - start_usb
      //if (sinceusb > 100)   
      {
         //OSZI_D_LO();
         //Serial.printf("usb\n");
         sinceusb = 0;
         r = RawHID.recv(buffer, 0); 
         
         code = 0;
         if (r > 0) // 
         {
            //Serial.printf("usb r: %d\n",r);
         //   noInterrupts();
            
            code = buffer[0];
            /*
            for (uint8_t i = 0;i<32;i++)
            {
               Serial.printf("%d\t",buffer[i]);
            }
            */
            Serial.printf("\n***************************************  --->    rawhid_recv begin code HEX: %02X\n",code);
            //Serial.printf("code: %d\n",code);
            usb_recv_counter++;
          //  uint8_t device = buffer[32];
          //  sendbuffer[24] =  buffer[32];
            
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
                  
               case 0xF0: // Data
               {
                  
               }break;
                  
               // MARK: F4 Fix Sendersettings
               case 0xF4: // Fix Sendersettings
               {
                  /*
                   FUNKTION_OFFSET    0x60 // 96
                   DEVICE_OFFSET      0x70 // 122
                   AUSGANG_OFFSET     0x80 // 128
                   
                   */
                  Serial.printf("0xF4 Fix Sendersettings\n");
                  for (uint8_t i=0;i<64;i++)
                  {
                     Serial.printf("\t%d",buffer[i]);
                     if (i==15 )
                     {
                        //Serial.printf("\n");
                     }
                  }
                  Serial.printf("\n");
                  
                  
                  
                  uint8_t erfolg = decodeUSBChannelSettings(buffer);
                  
                                  
                  uint8_t modelindex =0;
                  modelindex = buffer[USB_DATA_OFFSET + modelindex * KANALSETTINGBREITE] & 0x03; // Bit 0,1 welches model soll gelesen werden
                  Serial.printf("modelindex: %d ",modelindex);
                  uint8_t kanalindex =  (buffer[USB_DATA_OFFSET + modelindex * KANALSETTINGBREITE] & 0x70) >> 4; // Bits 4,5,6
                  uint8_t pos=0;
                  Serial.printf("kanalindex: %d ",kanalindex);
                  uint8_t on = (buffer[USB_DATA_OFFSET + modelindex * KANALSETTINGBREITE] & 0x08) >>3; // Bit 3
                  Serial.printf("on: %d  ",on);
                  uint8_t richtung = (buffer[USB_DATA_OFFSET + modelindex * KANALSETTINGBREITE] & 0x80) >>7; // Bit 7
                  Serial.printf("richtung: %d \n",richtung);
                  
                  
                  uint8_t mixingerfolg = decodeUSBMixingSettings(buffer);
                  /*
                  Serial.printf("mix data0 : %d data 1: %d\n",buffer[USB_DATA_OFFSET + MODELSETTINGBREITE],buffer[USB_DATA_OFFSET + MODELSETTINGBREITE +1 ]);
                  
                  uint8_t mix0 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE] ; // Bit 3
                  uint8_t mix1 = buffer[USB_DATA_OFFSET + MODELSETTINGBREITE + 1]; // Bit 3
                  
                  uint8_t mixon = (mix0 & 0x08) >> 3; // Bit 3
                  Serial.printf("mixon: %d \n",mixon);
                  uint8_t mixart = (mix0 & 0x30) >> 4; // Bit 4,5
                  Serial.printf("mixart: %d \n",mixart);
                  
                  uint8_t mixkanala = mix1 & 0x07 ; // Bit 0-3
                  Serial.printf("mixkanala: %d \n",mixkanala);
                  uint8_t mixkanalb = (mix1 & 0x70) >> 4; // Bit 4-6
                  Serial.printf("mixkanalb: %d \n", mixkanalb);
*/
                  for (uint8_t i=0;i<8;i++)
                  {
                     Serial.printf("F4 load_EEPROM_Settings i: %d status: %d level: %d expo: %d device: %d\n",i,curr_statusarray[i],curr_levelarray[i], curr_expoarray[i], curr_devicearray[i]);
                  }

                  
                  
                  break;
                  /*
                  // funktion lesen
                  uint16_t readstartadresse = TASK_OFFSET  + FUNKTION_OFFSET + modelindex*SETTINGBREITE;
                  // startadresse fuer Settings des models
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x60 32
                  }
                  
                  // device lesen
                  readstartadresse = TASK_OFFSET  + DEVICE_OFFSET + modelindex*SETTINGBREITE;
                  //Im Sendbuffer ab pos 0x08 (8)
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x08 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x28 40
                  }
                  
                  // Ausgang lesen
                  readstartadresse = TASK_OFFSET  + AUSGANG_OFFSET + modelindex*SETTINGBREITE;
                  
                  //Im Sendbuffer ab pos 0x10 (16)
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x10 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x30 48
                     
                  }
                  
                  sendbuffer[1] = readstartadresse & 0x00FF;
                  sendbuffer[2] = (readstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = modelindex;
                  
                  sendbuffer[0] = 0xFD;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  */
               }
                  // MARK: F6 get teensysettings
               case 0xF6: //   // get teensysettings
               {
                  Serial.printf("0xF6 get teensysettings\n");
                  uint8_t getmodel = (buffer[USB_DATA_OFFSET] & 0x07);
                  uint8_t getkanal = (buffer[USB_DATA_OFFSET] & 0x70) >> 4;
                  Serial.printf("getmodel: %d getkanal: %d\n",getmodel, getkanal);
                  //uint8_t* usbarray[USB_DATENBREITE] = {};
               
                  // Kaanalsettings laden
                  uint8_t* temparray = encodeEEPROMChannelSettings(getmodel);
                  
                  uint8_t* mixarray = encodeEEPROMMixingSettings(getmodel);
                  //encodeCurrentChannelSettings(getkanal,getmodel);
                  sendbuffer[1] = sendesekunde; // randomwert fuer neu USB-Daten
                  sendbuffer[0] = 0xF7;
 
                  Serial.printf("*F7 sendbuffer:\n");
                  for (uint8_t i = 0;i<USB_DATENBREITE;i++)
                  {
                     Serial.printf("\t%d",sendbuffer[i]);
                  }
                  Serial.printf("*\n");             

                  uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 50);
                  Serial.printf("0xF7 senderfolg: %d\n",senderfolg);
                  break;
               }

                  
                  // MARK: FD read Sendersettings
               case 0xFD: // read Sendersettings
               {
                  Serial.printf("0xFD\n");
                  /*
                   FUNKTION_OFFSET    0x60 // 96
                   DEVICE_OFFSET      0x70 // 122
                   AUSGANG_OFFSET     0x80 // 128
                   
                   */
                  
                  
                  uint8_t modelindex =0;
                  modelindex = buffer[3]; // welches model soll gelesen werden
                  uint8_t pos=0;
                  
                  // funktion lesen
                  uint16_t readstartadresse = TASK_OFFSET  + FUNKTION_OFFSET + modelindex*SETTINGBREITE;
                  // startadresse fuer Settings des models
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x60 32
                  }
                  
                  // device lesen
                  readstartadresse = TASK_OFFSET  + DEVICE_OFFSET + modelindex*SETTINGBREITE;
                  //Im Sendbuffer ab pos 0x08 (8)
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x08 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x28 40
                  }
                  
                  // Ausgang lesen
                  readstartadresse = TASK_OFFSET  + AUSGANG_OFFSET + modelindex*SETTINGBREITE;
                  
                  //Im Sendbuffer ab pos 0x10 (16)
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x10 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x30 48
                     
                  }
                  
                  sendbuffer[1] = readstartadresse & 0x00FF;
                  sendbuffer[2] = (readstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = modelindex;
                  
                  sendbuffer[0] = 0xFD;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
               }
                  
                    
#pragma mark default
               default:
               {
                  //RawHID.send(sendbuffer, 50);
                  //usb_rawhid_send((void*)sendbuffer, 50);
                  //Serial.printf("usb send\n");
               }break; // default
                  
                  
                  
            } // switch code
            interrupts();
       //     code=0;
         }// r > 0
         /**   End USB-routinen   ***********************/
         //OSZI_D_HI();
      } // since usb

      
      servostatus &= ~(1<<USB_OK);
      //Serial.printf("USB OK end\n");
   }// usb
} // loop
