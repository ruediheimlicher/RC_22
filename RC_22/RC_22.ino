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

// Set parameters




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

#define PAUSE        1
#define PAKET   2
#define IMPULS    3
#define ADC_OK 4
#define USB_OK 5
#define ENDEPAKET  7

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

IntervalTimer servoTimer;
IntervalTimer impulsTimer;
uint16_t impulsdelaycounter = 0;
uint16_t impulsdauer = 0;


IntervalTimer microTimer; 
uint16_t microcounter = 0;

#define IMPULSPIN  1

IntervalTimer              delayTimer;

volatile uint8_t           servoindex = 0;


// Utilities

volatile uint16_t impulstimearray[NUM_SERVOS] = {};

volatile uint8_t adcpinarray[NUM_SERVOS] = {};
// Prototypes
ADC *adc = new ADC(); // adc object

#define POTLO  300
#define POTHI  1000
#define PPMLO  500
#define PPMHI  1500

#define SHIFT 0xFFFFF
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
volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem
volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
volatile uint16_t                blink_cursorpos=0xFFFF;


volatile uint16_t stopsekunde=0;
volatile uint16_t stopminute=0;
volatile uint16_t motorsekunde=0;
volatile uint16_t motorminute=0;

volatile uint8_t                 curr_model=0; // aktuelles modell
uint8_t                     speichermodel=0;
volatile uint8_t                 curr_kanal=0; // aktueller kanal
volatile uint8_t                 curr_richtung=0; // aktuelle richtung
volatile uint8_t                 curr_impuls=0; // aktueller impuls

volatile uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t                     speichersetting=0;

volatile uint8_t                 curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
volatile uint8_t                 curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal



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

void servoimpulsfunction(void) // 
{ 

}

void servopaketfunktion(void) // start Abschnitt
{ 
   servostatus &= ~(1<<PAUSE);
   servostatus |= (1<<PAKET);// neues Paket starten
   servostatus |= (1<<IMPULS);
   servostatus |= (1<<IMPULS);
   OSZI_B_LO();
   
 //  microcounter = 0;
  // servoindex = 0; // neues Paket starten
   
}

void microtimerfunktion(void)
{
   OSZI_A_TOGG();
    if (servostatus & (1<<PAUSE))
    {
       
       //servostatus |= (1<<PAKET);
       //servostatus |= (1<<IMPULS);
       digitalWriteFast(IMPULSPIN,LOW);
       servoindex = 0;
    }
    else 
    {
       // impulsdelaycounter
       // impulsdauer
       if (servostatus & (1<<PAKET))  // Anfang neue Serie
       {
          if (servostatus & (1<<IMPULS)) // Start neuer Impuls
          {
             microcounter = 0;
             impulsdauer = impulstimearray[servoindex];
             digitalWriteFast(IMPULSPIN,HIGH);
             servostatus &= ~(1<<IMPULS);
          }
          
          if (microcounter > 50) // Impulsbreite
          {
             digitalWriteFast(IMPULSPIN,LOW);
             
             if (microcounter > impulsdauer) // next impuls einstellen
             {
                if (servoindex < NUM_SERVOS)
                {
                   servoindex++;
                   impulsdauer = impulstimearray[servoindex];
                   microcounter = 0;
                   servostatus |= (1<<IMPULS);
                }
                else 
                {
                   servostatus &= ~(1<<PAKET);
                   servostatus |= (1<<PAUSE); // pause beginnt
                   servostatus |= (1<<ADC_OK); // ADCFenster starten
                   OSZI_B_HI();
                   OSZI_C_LO();
                   microcounter = 0;
                   servoindex = 0;
                }
                
             }
          }
          microcounter++;
          
       } // PAKET
       
  
    }
    // digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
    

}

void servotimerfunction(void) // 1us ohne ramp
{ 
//   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
   
  
   
   if (timerstatus & (1<<TIMER_ON))
   {
      //OSZI_A_LO();
       
      //OSZI_A_HI();
      
      
      
   } // if timerstatus
   
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
   
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
     
   displayinit();
   
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
   
   
  
   
   pinMode(IMPULSPIN, OUTPUT);
   digitalWriteFast(IMPULSPIN,LOW);
   
   servoTimer.begin(servopaketfunktion, 80000);
   microTimer.begin(microtimerfunktion,2);
   
  //   pinMode(END_C1_PIN, INPUT_PULLUP); // 
   
//   attachInterrupt(digitalPinToInterrupt(END_A0_PIN), A0_ISR, FALLING);
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

    
 //  display_soft_init();
 //  display_clear();
   //sethomescreen();
   _delay_us(50);
}

// Add loop code
void loop()
{

   if (sinceblink > 500) 
   {   
      //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
      
      //sethomescreen();
      sinceblink = 0;
      
       if (digitalRead(LOOPLED) == 1)
      {
         digitalWriteFast(LOOPLED, 0);
          
      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
      }
      Serial.printf("servo potwert 0: %d 1: %d\n", impulstimearray[0],impulstimearray[1]); 
      
   }// sinceblink
   
   
   if (servostatus & (1<<ADC_OK)) // 50us ohne printf
   {
      for (uint8_t i=0;i<NUM_SERVOS;i++)
      {
         OSZI_C_LO();
         //Serial.printf("i: %d pin: %d\n",i,adcpinarray[i]);
         if (adcpinarray[i] < 0xFF) // PIN belegt
         {
            uint16_t potwert = adc->adc0->analogRead(adcpinarray[i]);
            float ppmfloat = PPMLO + quot *(float(potwert)-POTLO);
            uint16_t ppmint = uint16_t(ppmfloat);
            //uint16_t ppmval = PPMLO + (((potwert - POTLO)));
            if (i == 0)
            {
              // Serial.printf("servo %d potwert: %d  ppmval: %d ppmfloat: %.6f\n",i,potwert,ppmval,ppmfloat);    
               
            }                                    
            impulstimearray[i] = ppmint;
            //impulstimearray[i] = potwert;
            
         }
         
         OSZI_C_HI();
      }
      
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
            //     lcd.setCursor(10,1);
            //     lcd.print(String(usb_recv_counter));
            //     lcd.setCursor(14,1);
            //     lcd.print(String(code));
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
