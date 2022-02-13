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


#include "Arduino.h"
#include <ADC.h>
#include <SPI.h>
#include "lcd.h"
#include "settings.h"

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
elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMicros sinceusb;
uint16_t cncdelaycounter = 0;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;

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

volatile float quot = (ppmhi - ppmlo)/(pothi - potlo);

// display
uint8_t testdata = 0;
volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

//volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
//volatile uint16_t                blink_cursorpos=0xFFF;


volatile uint16_t stopsekunde=0;
volatile uint16_t stopminute=0;
volatile uint16_t motorsekunde=0;
volatile uint16_t motorminute=0;
 
volatile uint8_t                 curr_model=0; // aktuelles modell
volatile uint8_t                 speichermodel=0;
volatile uint8_t                 curr_kanal=0; // aktueller kanal
volatile uint8_t                 curr_richtung=0; // aktuelle richtung
volatile uint8_t                 curr_impuls=0; // aktueller impuls

volatile uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t                     speichersetting=0;

volatile uint8_t                 curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
volatile uint8_t                 curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal


// Tastatur
volatile uint8_t                 Tastenindex=0;
volatile uint16_t                Tastenwert=0;
volatile uint16_t                Trimmtastenwert=0;
volatile uint8_t                 adcswitch=0;
volatile uint16_t                lastTastenwert=0;
volatile int16_t                 Tastenwertdiff=0;
volatile uint16_t                tastaturcounter=0;


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

   }
   else  // Paket beenden
   {
      servoimpulsTimer.end();
      servostatus |= (1<<PAUSE);
      servostatus |= (1<<ADC_OK); // ADCFenster starten
      OSZI_B_HI();
      //OSZI_C_LO();

   }
   kanalimpulsTimer.begin(kanalimpulsfunktion,IMPULSBREITE); // neuer Kanalimpuls
   
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
   /*
    
    */
  
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
   
   return -1;
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
   }
   
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
   digitalWriteFast(TASTATURPIN, INPUT);
   Serial.printf("W1: %d W2: %d W3: %d W4: %d W5: %d W6: %dW7: %d W8: %d W9: %d \n",WERT1, WERT2, WERT3, WERT4, WERT5, WERT6, WERT7, WERT8, WERT9);
}

// Add loop code
void loop()
{

   if (!(servostatus & (1<<RUN))) // first run
   {
      Serial.printf("first run\n");
      servostatus |= (1<<RUN);
   }
   
   if (sinceblink > 500) 
   {   
      //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
      
      sinceblink = 0;
      
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
         Tastenindex = Tastenwahl(Tastenwert); // taste pressed
         Serial.printf("Tastenwert: %d Tastenindex: %d\n",Tastenwert,Tastenindex);
      }
      //Serial.printf("servo potwert 0: %d 1: %d\n", impulstimearray[0],impulstimearray[1]); 
      impulscounter++;
      if (impulscounter > 5)
      {
         Serial.printf("servo potwert 0: %d 1: %d\n", impulstimearray[0],impulstimearray[1]); 
         /*
         for (uint8_t i=0;i<NUM_SERVOS;i++)
         {
            Serial.printf("\t%d\t%d",i,impulstimearray[i]);
         }
         Serial.printf("\n");
          */
         impulscounter = 0;
      }
   }// sinceblink
   
   
   if (servostatus & (1<<ADC_OK)) // 20us pro kanal ohne printf
   {
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
      
      
      // MARK:  Tastatur ADC
      Tastenwert=(adc->adc0->analogRead(TASTATURPIN))>>2;
      if (Tastenwert>5)
      {
         if (!(tastaturstatus & (1<<TASTEOK)))
         {
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
            
            //Tastenindex = Tastenwahl(Tastenwert);
            if (Tastenwertdiff < 6)
            {
               if (tastaturcounter < ADCTIMEOUT)
               {
                  tastaturcounter++;
                  
                  if (tastaturcounter == ADCTIMEOUT)
                  {
                     Tastenindex = Tastenwahl(Tastenwert); // taste pressed
                     tastaturstatus |= (1<<TASTEOK);
                     programmstatus |= (1<< LEDON);
                     display_set_LED(1);
                     
                  }
               }
            }
            else
            {
               tastaturcounter = 0;
            }
            
         }   
         
      }
      else
      {
         tastaturstatus &= ~(1<<TASTEOK);
         tastaturcounter = 0;
         Tastenindex = 0;
//           Trimmtastenwert=adc_read(TRIMMTASTATURPIN)>>2;
      }
      
      // end Tastatur
      OSZI_C_HI();
      servostatus &= ~(1<<ADC_OK);
      
      
      servostatus |= (1<<USB_OK);
   }
   
   
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
