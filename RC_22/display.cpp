/*----------------------------------------------------------------------------
 Copyright:
 Author:         Radig Ulrich
 Remarks:        
 known Problems: none
 Version:        21.11.2009
 Description:    Description:    EA DOG (M/L)128-6
------------------------------------------------------------------------------*/

//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
//#include <avr/io.h>
//#include <avr/delay.h>
#include "display.h"
#include "font.h"
#include "text.h"
#include "settings.h"

#include "Arduino.h"

unsigned char char_x;
unsigned char char_y;
unsigned char char_height_mul;
unsigned char char_width_mul;


//extern  uint8_t expowert;//
//extern  uint8_t expob;

//extern   uint8_t cursortab[10] = {0+OFFSET_6_UHR,24+OFFSET_6_UHR,40+OFFSET_6_UHR,52+OFFSET_6_UHR,64+OFFSET_6_UHR,76+OFFSET_6_UHR,88+OFFSET_6_UHR,100+OFFSET_6_UHR,108+OFFSET_6_UHR,0};

#define cursortab0 2
#define cursortab1 26
#define cursortab2 46
#define cursortab3 54
#define cursortab4 66
#define cursortab5 78
#define cursortab6 90
#define cursortab7 100
extern   uint8_t cursortab[10] = {cursortab0,cursortab1,cursortab2,cursortab3,cursortab4,cursortab5,cursortab6,cursortab7,cursortab0,cursortab0};
//extern  uint8_t itemtab[10] = {8+OFFSET_6_UHR,32+OFFSET_6_UHR,48+OFFSET_6_UHR,60+OFFSET_6_UHR,72+OFFSET_6_UHR,84+OFFSET_6_UHR,96+OFFSET_6_UHR,108+OFFSET_6_UHR,116+OFFSET_6_UHR,0+OFFSET_6_UHR};

#define itemtab0  10
#define itemtab1  34
#define itemtab2  50
#define itemtab3  62
#define itemtab4  74
#define itemtab5  88
#define itemtab6  110
#define itemtab7  118

extern  uint8_t itemtab[10] = {itemtab0,itemtab1,itemtab2,itemtab3,itemtab4,itemtab5,itemtab6,itemtab7,itemtab0,itemtab0};


extern  uint16_t      motorsekunde;

//extern  uint8_t       curr_settingarray[8][2];
extern  uint8_t       curr_levelarray[8];
extern  uint8_t       curr_expoarray[8];
extern  uint8_t       curr_mixarray[8];
extern  uint8_t       curr_funktionarray[8];
extern  uint8_t       curr_statusarray[8];
extern  uint8_t       curr_ausgangarray[8];

extern  uint8_t       curr_devicearray[8];

extern  int8_t        curr_trimmungarray[8];

extern  uint8_t       curr_mixstatusarray[8];
extern  uint8_t       curr_mixkanalarray[8];

extern  uint8_t       curr_screen;
extern  uint8_t       curr_page; // aktuelle page
extern  uint8_t       curr_col; // aktuelle colonne
extern  uint8_t       curr_model; // aktuelles modell

extern  uint8_t       curr_kanal; // aktueller kanal

extern  uint8_t       curr_richtung; // aktuelle richtung
extern  uint8_t       curr_impuls; // aktueller impuls

extern  uint8_t       eepromsavestatus;
extern  uint8_t       programmstatus;
extern  uint8_t       servostatus;

extern  uint8_t       curr_setting;
extern  uint8_t       curr_cursorzeile; // aktuelle zeile des cursors
extern  uint8_t       curr_cursorspalte; // aktuelle colonne des cursors

extern  uint8_t       last_cursorzeile; // letzte zeile des cursors
extern  uint8_t       last_cursorspalte; // letzte colonne des cursors
extern  uint16_t      blink_cursorpos;

extern  uint8_t                 curr_trimmkanal; // aktueller  Kanal fuerTrimmung
extern  uint8_t                 curr_trimmung; // aktuelle  Trimmung fuer Trimmkanal
extern  int8_t                vertikaltrimm;
extern  int8_t                horizontaltrimm;


//extern  uint16_t      motorsekunde;
extern  uint8_t       motorstunde;
extern  uint16_t      motorminute;
extern  uint16_t      motorsekunde;
extern  uint16_t      stopsekunde;
extern  uint16_t      motorminute;
extern  uint16_t      stopminute;
extern  uint16_t      batteriespannung;

extern  uint16_t sendesekunde;
extern  uint16_t sendeminute;
extern  uint8_t sendestunde;

extern  uint16_t      tastentransfer;

extern  uint16_t  posregister[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

extern  uint16_t  cursorpos[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer cursor (hex). geladen aus progmem

// 
extern  uint16_t              updatecounter; // Zaehler fuer Einschalten

extern void OSZI_D_LO();
extern void OSZI_D_HI();

/*
 write_dogm(0x40,A0);// Display start line set --> 0
 write_dogm(0xA1,A0);  // ADC set --> reverse
 write_dogm(0xC0,A0);  // Common output mode select --> normal
 write_dogm(0xA6,A0);  // Display --> normal
 write_dogm(0xA2,A0);  // LCD Bias set --> 1/9 (Duty 1/65)
 write_dogm(0x2F,A0);  // Power control set --> Booster, Regulator and Follower on
 write_dogm(0xF8,A0);  // Booster ratio set --> Set internal Booster to 4x
 write_dogm(0x00,A0);  // ...
 write_dogm(0x27,A0);  // Contrast set
 write_dogm(0x81,A0);  // ...
 write_dogm(actContrast,A0);
 write_dogm(0xAC,A0);  // Static indicator set --> no indicator
 write_dogm(0x00,A0);  // ...
 write_dogm(0xAF,A0);  // Display on/off

 */

//#define DATEN     0
//#define CMD      1

/*
#define HOMESCREEN      0
#define SETTINGSCREEN   1
#define KANALSCREEN     2
#define LEVELSCREEN     3
#define EXPOSCREEN      4
#define MIXSCREEN       5
#define ZUTEILUNGSCREEN 6
#define AUSGANGSCREEN   7
#define SAVESCREEN      8
#define TRIMMSCREEN      9

#define MODELLCURSOR 2
#define SETCURSOR    4
#define KANALCURSOR  6
#define MIXCURSOR    7

#define KANALTEXTCURSOR 1
#define KANALNUMMERCURSOR  3
*/

const char balken[8]=
{0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF};

const char marke[8]=
{0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};

const char marke_dick[8]=
{0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x03};



const  char DISPLAY_INIT[] =
{
   0x40,// Display start line set --> 0
   
//   0xA1, // ADC set --> reverse Horizontal gespiegelt wenn A1
   0xA0, // ADC set --> reverse Horizontal gespiegelt wenn A1
   
//   0xC0, // Common output mode select --> normal
   0xC8, // Common output mode select --> normal
   
   
   0xA6, // Display --> normal  A7 ist reverse
   0xA2, // LCD Bias set --> 1/9 (Duty 1/65)
   0x2F, // Power control set --> Booster, Regulator and Follower on
   0xF8, // Booster ratio set --> Set internal Booster to 4x
   0x00, // ...
   0x27, //
   0x81, // Contrast set
   0x18,
   0xAC, // Static indicator set --> no indicator
   0x00, // ...
   0xAF
};  // Display on/off


// 180° gedreht
//const char DISPLAY_INIT[] = {0x40, 0xA1, 0xC0, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x16, 0xAC, 0x00, 0xAF};

//char_height_mul 1, 2 or 4


char menubuffer[20];
char titelbuffer[20];


void resetRegister(void)
{
   uint8_t i=0,k=0;
   for(i=0;i<8;i++)
   {
      for (k=0;k<8;k++)
      {
         posregister[i][k]=0xFFFF;
      }
   }
}

void setheighmul(uint8_t wert)
{
   char_height_mul = wert ;
}

void sethomescreen(void)
{
   // Laufzeit
   Serial.printf("sethomescreen start\n");
   resetRegister();
   blink_cursorpos=0xFFFF;
   posregister[0][0] = itemtab[5] | (1 << 10);// Laufzeit Anzeige
   
   
   posregister[1][0] = (0+OFFSET_6_UHR) | (0x05 << 10); // Text Motorzeit
   posregister[1][1] = (0+OFFSET_6_UHR) | (0x06 << 10); // Anzeige Motorzeit
   
   
   posregister[2][0] = (60+OFFSET_6_UHR) | (0x05 << 10); // Text Stoppuhr
   posregister[2][1] = (60+OFFSET_6_UHR) | (0x06 << 10); // Anzeige Stoppuhr
   
   
   posregister[3][0] = (60+OFFSET_6_UHR) | (0x07 << 10); // Text Akku
   posregister[3][1] = (84+OFFSET_6_UHR) | (0x08 << 10); // Anzeige Akku

   posregister[4][0] = (0+OFFSET_6_UHR) | (2 << 10); // Name Modell
   posregister[4][1] = (80+OFFSET_6_UHR) | (3 << 10); // Text Setting
   posregister[4][2] = (100+OFFSET_6_UHR) | (3 << 10); // Anzeige Setting

   cursorpos[0][1] = cursortab[0] |    (8 << 10); //  cursorpos fuer Menu
   
   // positionen lesen
   // titel setzen

   char_x=OFFSET_6_UHR;
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_go_to(char_x+1,0);
   
   display_write_byte(DATEN,0xFF);
   //char_x++;
   
   //display_inverse(1);
   char_height_mul = 1;
   char_width_mul = 1;

   //display_write_prop_str(char_y,char_x,0,(unsigned char*)titelbuffer);
   display_write_str(TitelTable[0],1);
   //display_write_str(TitelTable[0],1);
   //display_write_str("ABC",1);
   //display_inverse(0);
   char_height_mul = 1;
   char_width_mul = 1;
   
   // Stoppuhrtext schreiben
   char_x = (posregister[2][0] & 0x00FF);
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   display_write_str(TitelTable[2],2);
   
   
   
   // Stoppzeit schreiben
   char_y= (posregister[2][1] & 0xFF00)>> 10;
   char_x = (posregister[2][1] & 0x00FF);
   char_height_mul = 2;
   char_width_mul = 2;
   display_write_stopzeit_BM(stopsekunde,stopminute);
   char_height_mul = 1;
   char_width_mul = 1;
   
   
   // Motorzeittext schreiben
   char_x = (posregister[1][0] & 0x00FF);
   char_y= ((posregister[1][0] & 0xFF00)>> 10);
   char_height_mul = 1;
   display_write_str(TitelTable[3],2);
   char_height_mul = 2;
   char_width_mul = 2;

   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
   // display_write_min_sek(motorsekunde,2);
   display_write_stopzeit(motorsekunde,motorminute, 2);
   
   char_height_mul = 1;
   char_width_mul = 1;
  
   
   // Modell schreiben
   char_y= (posregister[4][0] & 0xFF00)>> 10;
   char_x = posregister[4][0] & 0x00FF;
   //display_write_prop_str(char_y,char_x,0,titelbuffer,2);
   char_height_mul = 2;
   display_write_str(ModelTable[curr_model],1);

   char_height_mul = 1;
   //strcpy(titelbuffer, ((TitelTable[5])));
   char_y= (posregister[4][1] & 0xFF00)>> 10;
   char_x = posregister[4][1] & 0x00FF;
   display_write_str(TitelTable[5],2);
   char_y= (posregister[4][2] & 0xFF00)>> 10;
   char_x = posregister[4][2] & 0x00FF;
   display_write_int(curr_setting,2);
   
   
   
   char_height_mul = 1;
   char_width_mul = 1;

   // Batteriespannung
   char_y= ((posregister[3][0] & 0xFF00)>> 10)+1;
   char_x = posregister[3][0] & 0x00FF;
   char_height_mul = 1;
   display_write_str(TitelTable[6],2);
   
   char_height_mul = 1;
   char_width_mul = 1;

  // display_write_propchar(' ');
   char_y= (cursorpos[0][1] & 0xFF00)>> 10;
   char_x = cursorpos[0][1] & 0x00FF;
   display_write_symbol(pfeilvollrechts);
   char_x += 4;
   display_write_str(TitelTable[4],2);
   
   Serial.printf("sethomescreen end\n");
}// sethomescreen


void setsettingscreen(void)
{
  // Serial.printf("setsettingscreen start\n");
   
   resetRegister();
   blink_cursorpos=0xFFFF;
   
  
   // 2. Zeile
   posregister[0][0] =  itemtab[0] |    (2 << 10); //Modellname
   posregister[0][1] =  itemtab[0] |    (2 << 10); //

   posregister[0][2] =  itemtab[5] |    (3 << 10); // settingtext
   posregister[0][3] =  itemtab[7] |    (3 << 10); // settingnummer
   
   posregister[1][0] =  itemtab[0] |    (4 << 10); // Kanaltext
   
   posregister[2][0] =  itemtab[0] |    (5 << 10); // mixtext

   posregister[3][0] =  itemtab[0] |    (6 << 10); // Zuteilungtext
 
   posregister[4][0] =  itemtab[0] |    (7 << 10); // Zuteilungtext
   
   cursorpos[0][0] = cursortab[0] |    (2 << 10); // modellcursor lo: tab hi: page
   // cursorpos fuer model zeile/colonne
   
   cursorpos[0][1] = cursortab[5] |    (3 << 10); //  cursorpos fuer setting
   cursorpos[1][0] = cursortab[0] |    (4 << 10);  // cursorpos fuer kanal
   cursorpos[2][0] = cursortab[0] |    (5 << 10);  // cursorpos fuer mix
   cursorpos[3][0] = cursortab[0] |    (6 << 10);  // cursorpos fuer zuteilung
   cursorpos[4][0] = cursortab[0] |    (7 << 10);  // cursorpos fuer zuteilung
  
   
   char_x=itemtab[0];
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(SettingTable[0],1);
   char_height_mul = 2;
   char_width_mul = 1;
   
   // Modell Name
   
   //display_write_prop_str(char_y,char_x,0,menubuffer,2);
   
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_x = posregister[0][0] & 0x00FF;

   display_write_str(ModelTable[curr_model],1);
   char_height_mul = 2;
   char_width_mul = 1;

   char_y= (cursorpos[0][0] & 0xFF00)>> 10;
   char_x = cursorpos[0][0] & 0x00FF;
   display_write_symbol(pfeilvollrechts);
   
   // 2. Zeile Set mit Nummer
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
   
   char_height_mul = 1;
   display_write_str(SettingTable[2],2);
   

   // Kanal-Zeile
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[3],2);
   
   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
//   display_write_int(curr_kanal,2);

   
   // Mix-Zeile
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[4],2);
   
   // Zuteilung-Zeile
   char_y= (posregister[3][0] & 0xFF00)>> 10;
   char_x = posregister[3][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[5],2);
 
   // Output-Zeile
   char_y= (posregister[4][0] & 0xFF00)>> 10;
   char_x = posregister[4][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[6],2);
   
   //Serial.printf("setsettingscreen end\n");
}// setsettingscreen


void setcanalscreen(void)
{
   //OSZI_D_LO();
  // Serial.printf("setcanalscreen curr_screen: %d\n",curr_screen);
   for (uint8_t i=0;i<10;i++)
   {
     // Serial.printf("%d cur %d item %d\n",i,cursortab[i],itemtab[i]); 
   }
   /*
    #define itemtab0  12
    #define itemtab1  36
    #define itemtab2  52
    #define itemtab3  64
    #define itemtab4  76
    #define itemtab5  90
    #define itemtab6  110
    #define itemtab7  118

    */
   resetRegister();
   blink_cursorpos=0xFFFF;
   
   // bit 0-6: kolonne  bit 7: zeile
   
   posregister[0][0] =  itemtab[0] |    (1 << 10); // Kanaltext    Zeile 1
   posregister[0][1] =  itemtab[1] |    (1 << 10); // Kanalnummer
   posregister[0][2] =  itemtab[2] |    (1 << 10); // Richtungtext
   posregister[0][3] =  itemtab[3] |    (1 << 10); // RichtungPfeil
   posregister[0][4] =  itemtab[4] |    (1 << 10); // funktion
   posregister[0][5] =  itemtab[5] |    (1 << 10); // typ symbol
   posregister[0][6] =  itemtab[6] |    (1 << 10); // 
   posregister[0][7] =  itemtab[7] |    (1 << 10); // 
   for (uint8_t pos = 0;pos < 6; pos++)
   {
      //Serial.printf("pos: %d posregister: %d  itemtab: %d \n",pos, posregister[0][pos],itemtab[pos]);
   }
   // level
   posregister[1][0] =  itemtab[2] |    (2 << 10); // Leveltext
   
   posregister[1][1] =  itemtab[3] |    (2 << 10); // Level A text
   posregister[1][2] =  itemtab[0] |    (2 << 10); // Level A wert
   posregister[1][3] =  itemtab[5] |    (2 << 10); // Level B text
   posregister[1][4] =  itemtab[6] |    (2 << 10); // Level B text
   posregister[1][4] =  itemtab[6] |    (2 << 10); // Level B wert
   
   // expo
   posregister[2][0] =  itemtab[2] |    (3 << 10); // expotext
   
   posregister[2][1] =  itemtab[3] |    (3 << 10); // expo A text
   posregister[2][2] =  itemtab[0] |    (3 << 10); // expo A wert
   posregister[2][3] =  itemtab[6] |    (3 << 10); // expo B text
   posregister[2][4] =  itemtab[6] |    (3 << 10); // expo B wert
   
   // typ
   posregister[3][0] =  itemtab[6] |    (4 << 10); // typtext
   posregister[3][1] =  itemtab[8] |    (4 << 10); // typ symbol
   //posregister[3][2] =  itemtab[3] |    (8 << 10); //
   //posregister[3][3] =  itemtab[4] |    (8 << 10); //

   
   cursorpos[0][0] =cursortab[0] |   (1 << 10); // cursorpos fuer Kanal zeile/colonne
   cursorpos[0][1] =cursortab[2] |   (1 << 10); // cursorpos fuer Richtung
   cursorpos[0][2] =cursortab[5] |   (1 << 10); // cursorpos fuer Funktion

   
   cursorpos[1][0] =cursortab[0] |   (2 << 10); // cursorpos fuer Levelwert A
   cursorpos[1][1] =cursortab[7] |   (2 << 10);// cursorpos fuer Levelwert B

   cursorpos[2][0] =cursortab[0] |   (3 << 10); // cursorpos fuer Expowert A
   cursorpos[2][1] =cursortab[7] |   (3 << 10); // cursorpos fuer Expowert B

   
   cursorpos[3][0] =cursortab[0] |   (4 << 10); // cursorpos fuer Expo
   cursorpos[3][0] =cursortab[0] |   (1 << 10); // cursorpos fuer Art
   
   cursorpos[4][0] =cursortab[0] |   (1 << 10); // cursorpos fuer Art
   
//   Serial.printf("posregister[1]: %d posregister[2]%d ",posregister[1][0],posregister[2][0]);
//   Serial.printf("posregister[1]: %d posregister[2]%d ",posregister[1][1],posregister[2][1]);
//  Serial.printf("posregister[1]: %d posregister[2]%d ",posregister[1][2],posregister[2][2]);
 
   // Kanal bezeichnung zeigen
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_x = posregister[0][0] & 0x00FF;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(KanalTable[0],2); // Kan:
   
   // kanalnummer
   char_y= (posregister[0][1] & 0xFF00)>> 10;
   char_x = posregister[0][1] & 0x00FF;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_int(curr_kanal,2);

// Richtung Bezeichnung anzeigen 
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
   display_write_str(KanalTable[1],2); // Ri:
   
   // Richtungspfeil
   char_y= (posregister[0][3] & 0xFF00)>> 10;
   char_x = posregister[0][3] & 0x00FF;
   
   if (curr_kanal < 4) 
   {
      if (((curr_statusarray[curr_kanal] & 0x70)>>4)%2 == 0) // waagrecht, gerade Kanalnummer, 0 - 3
      {
         if (curr_statusarray[curr_kanal] & 0x80) // Bit 7 fuer Richtung
         {
            display_write_propsymbol(proprichtungright);
            //display_write_propsymbol(proprichtungup);
         }
         else
         {
            display_write_propsymbol(proprichtungleft);
         }
         
      }
      else if (((curr_statusarray[curr_kanal] & 0x70)>>4)%2 == 1)// senkrecht
      {
         if  (curr_statusarray[curr_kanal] & 0x80) // Bit 7 fuer Richtung
         {
            display_write_propsymbol(proprichtungup);
         }
         else
         {
            display_write_propsymbol(proprichtungdown);
         }
         
      }
   } // if kanal < 4

   
   
   
   char_height_mul = 1;

   // Funktion Bezeichnung anzeigen
   char_y= ((posregister[0][4] & 0xFF00)>> 10);
   char_x = posregister[0][4] & 0x00FF;
  // display_write_str(KanalTable[2],2); Fkt. nicht zeigen
   
   
   // Funktion anzeigen
   uint8_t funktionwert = curr_devicearray[curr_kanal];
   // **   char* funktionstring = FunktionTable[curr_devicearray[curr_kanal]];
   
    
   // Niveau-Bezeichnung "Level" anzeigen
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   display_write_str(KanalTable[3],1); // Level

   // Niveau-Bezeichnung "Expo" anzeigen
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   char_width_mul = 1;
   display_write_str(KanalTable[4],1); // "Expo"
   
    
   char_height_mul = 1;
 
   // Level A text
   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
   //display_write_str(KanalTable[4],1); // A
 
   // Level A wert
   char_y= (posregister[1][2] & 0xFF00)>> 10;
   char_x = posregister[1][2] & 0x00FF;
   char_width_mul = 1;
   display_write_int(8-((curr_levelarray[curr_kanal] & 0x70)>>4),1);
   display_write_str("/8\0",1);
   
   char_width_mul = 1;
   char_height_mul = 1;
   
   // Level B text
   char_y= (posregister[1][3] & 0xFF00)>> 10;
   char_x = posregister[1][3] & 0x00FF;
   
   // Level B wert
   char_width_mul = 1;
   char_y= (posregister[1][4] & 0xFF00)>> 10;
   char_x = posregister[1][4] & 0x00FF;
   display_write_int((8-(curr_levelarray[curr_kanal] & 0x07)),1);
   display_write_str("/8\0",1);
   // Bezeichnung Expo anzeigen
   
   // expo A text
   char_y= (posregister[2][1] & 0xFF00)>> 10;
   char_x = posregister[2][1] & 0x00FF;
   
   // expo A wert
   char_y= (posregister[2][2] & 0xFF00)>> 10;
   char_x = posregister[2][2] & 0x00FF;
   char_width_mul = 1;
   display_write_int((curr_expoarray[curr_kanal] & 0x70)>>4,1);
   display_write_str("/8\0",1);
   
   
   // expo B text
   char_y= (posregister[2][3] & 0xFF00)>> 10;
   char_x = posregister[2][3] & 0x00FF;
   char_width_mul = 1;
   
   // expo B wert
   char_y= (posregister[2][4] & 0xFF00)>> 10;
   char_x = posregister[2][4] & 0x00FF;
   char_width_mul = 1;
 //  display_write_int((curr_expoarray[curr_kanal] & 0x07),1);
  // display_write_str("/8\0",1);


     
    
}

void setausgangscreen(void)
{
   resetRegister();
   uint8_t delta=2;
   curr_impuls= 0;
   blink_cursorpos=0xFFFF;
   posregister[0][0] =  itemtab[0] |    (3 << 10); //
   posregister[0][1] =  itemtab[1] |    (3 << 10); //
//   posregister[0][2] =  itemtab[4] |    (1 << 10); //
//   posregister[0][3] =  itemtab[5] |    (1 << 10); //
   
   // level
   posregister[1][0] =  itemtab[0] |    (4 << 10); //
   posregister[1][1] =  itemtab[1] |    (4 << 10); //
//   posregister[1][2] =  itemtab[3] |    (1 << 10); //
//   posregister[1][3] =  itemtab[5] |    (1 << 10); //
//   posregister[1][4] =  itemtab[6] |    (1 << 10); //
   
   //
   posregister[2][0] =  itemtab[0] |    (5 << 10); //
   posregister[2][1] =  itemtab[1] |    (5 << 10); //
//   posregister[2][2] =  itemtab[3] |    (5 << 10); //
//   posregister[2][3] =  itemtab[5] |    (5 << 10); //
//   posregister[2][4] =  itemtab[6] |    (5 << 10); //

   posregister[3][0] =  itemtab[0] |    (6 << 10); //
   posregister[3][1] =  itemtab[1] |    (6 << 10); //
   //   posregister[2][2] =  itemtab[3] |    (5 << 10); //
   //   posregister[2][3] =  itemtab[5] |    (5 << 10); //
   //   posregister[2][4] =  itemtab[6] |    (5 << 10); //

   posregister[4][0] =  itemtab[0] |    (7 << 10); //
   posregister[4][1] =  itemtab[1] |    (7 << 10); //
   //   posregister[2][2] =  itemtab[3] |    (5 << 10); //
   //   posregister[2][3] =  itemtab[5] |    (5 << 10); //
   //   posregister[2][4] =  itemtab[6] |    (5 << 10); //

//   posregister[5][0] =  itemtab[0] |    (8 << 10); //
 //  posregister[5][1] =  itemtab[1] |    (8 << 10); //

   // typ
   
   
   cursorpos[0][0] =cursortab[0] |   (3 << 10); // cursorpos fuer
   cursorpos[1][0] =cursortab[0] |   (4 << 10); // cursorpos fuer
   cursorpos[2][0] =cursortab[0] |   (5 << 10); // cursorpos fuer
   cursorpos[3][0] =cursortab[0] |   (6 << 10); // cursorpos fuer
   cursorpos[4][0] =cursortab[0] |   (7 << 10); // cursorpos fuer
  // cursorpos[5][0] =cursortab[0] |   (8 << 10); // cursorpos fuer

   cursorpos[0][1] =cursortab[1] |   (3 << 10); // cursorpos fuer
   cursorpos[1][1] =cursortab[1] |   (4 << 10); // cursorpos fuer
   cursorpos[2][1] =cursortab[1] |   (5 << 10); // cursorpos fuer
   cursorpos[3][1] =cursortab[1] |   (6 << 10); // cursorpos fuer
   cursorpos[4][1] =cursortab[1] |   (7 << 10); // cursorpos fuer
  // cursorpos[5][1] =cursortab[1] |   (8 << 10); // cursorpos fuer

   
   //strcpy(menubuffer,(&(SettingTable[6]))); // Ausgang
   char_y= 1;
   char_x = 10+OFFSET_6_UHR;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(SettingTable[6],2);
   
   uint8_t spaltenarray[4] = {itemtab[0],itemtab[1]-delta,itemtab[2],itemtab[4]};
   
   //
   //strcpy(menubuffer,(&(AusgangTable[0]))); // Impuls
   char_y= 2;
   char_x = spaltenarray[0];
   display_write_str(AusgangTable[0],1);
 
   //strcpy(menubuffer, (&(AusgangTable[1]))); // Kanal
   char_y= 2;
   char_x = spaltenarray[1];
   display_write_str(AusgangTable[1],1);

   //strcpy(menubuffer, (&(AusgangTable[2]))); // Device
   char_y= 2;
   char_x = spaltenarray[2];
   display_write_str(AusgangTable[2],1);
  
   //strcpy(menubuffer, (&(AusgangTable[3]))); // Funktion
   char_y= 2;
   char_x = spaltenarray[3];
   display_write_str(AusgangTable[3],1);
   
  

   //
  }

void setmixscreen(void)
{
   uint8_t deltaplus =3;
   uint8_t deltaminus =3;
   uint8_t d1 = 6;
   uint8_t d2 = 3;
   uint8_t d5 = 6;
   
   resetRegister();
   blink_cursorpos=0xFFFF;
   
   // Mix 0
   posregister[0][0] =  itemtab[0] |    (4 << 10); // Art
   posregister[0][1] =  (itemtab[1]+d1) |    (4 << 10); // ON
   posregister[0][2] =  (itemtab[2]-deltaminus) |    (4 << 10); //
   posregister[0][3] =  (itemtab[3]-deltaminus) |    (4 << 10); //
   posregister[0][4] =  itemtab[4] |    (4 << 10); //
   posregister[0][5] =  (itemtab[5]+d5) |    (4 << 10); //
   posregister[0][6] =  itemtab[6] |    (4 << 10); // Level B wert
   posregister[0][7] =  itemtab[7] |    (4 << 10); // Level B wert
   
   // Mix 1
   posregister[1][0] =  itemtab[0] |    (5 << 10); //
   posregister[1][1] =  (itemtab[1]+d1) |    (5 << 10); //
   posregister[1][2] =  (itemtab[2]-deltaminus) |    (5 << 10); //
   posregister[1][3] =  (itemtab[3]-deltaminus) |    (5 << 10); //
   posregister[1][4] =  itemtab[4] |    (5 << 10); //
   posregister[1][5] =  (itemtab[5]+d5) |   (5 << 10); //
   posregister[1][6] =  itemtab[6] |    (5 << 10); // Level B wert
   posregister[1][7] =  itemtab[7] |    (5 << 10); // Level B wert

  
   posregister[2][0] =  itemtab[0] |    (6 << 10); // Leveltext
   posregister[2][1] =  itemtab[1] |    (6 << 10); // Level A text
   posregister[2][2] =  (itemtab[2]-deltaminus)  |    (6 << 10); // Level A wert
   posregister[2][3] =  itemtab[3] |    (6 << 10); // Level B text
   posregister[2][4] =  itemtab[4] |    (6 << 10); // Level B wert
   posregister[2][5] =  (itemtab[5]+deltaplus)  |  (6 << 10); // Level B wert
   posregister[2][6] =  itemtab[6] |    (6 << 10); // Level B wert
   posregister[2][7] =  itemtab[7] |    (6 << 10); // Level B wert
   
   
   cursorpos[0][0] =cursortab[0] |   (4 << 10); // cursorpos fuer Typ
   cursorpos[0][1] =(cursortab[1] + 6) |   (4 << 10); // cursorpos fuer OK
   cursorpos[0][2] =(cursortab[3] -3) |   (4 << 10); // cursorpos fuer
   cursorpos[0][3] =(cursortab[6] -2) |   (4 << 10); // cursorpos fuer Mix 0
  
   cursorpos[1][0] =cursortab[0] |   (5 << 10); // cursorpos fuer Typ
   cursorpos[1][1] =(cursortab[1] + 6) |   (5 << 10); // cursorpos fuer OK
   cursorpos[1][2] =(cursortab[3] -3) |   (5 << 10); // cursorpos fuer
   cursorpos[1][3] =(cursortab[6] -2) |   (5 << 10); // cursorpos fuer Mix 0

   //strcpy(menubuffer, (&(MixTable[0]))); // titel
   char_y= 1;
   char_x = itemtab[0] ;
   char_height_mul = 1;
   char_width_mul = 1;
   //display_write_str(MixTable[0],2);
   display_write_str("MIXING",2);
   
   // Tabellenkopf anzeigen
   char_y= 2;
   char_x = itemtab[0];
   display_write_str("Typ",1);

   char_y= 2;
   char_x = itemtab[2]-deltaminus;
   //char_x = itemtab[4];
   display_write_str("Rev",1); // Seite A
   
   char_y= 2;
   char_x = itemtab[5]+deltaplus;
   //char_x = itemtab[5];
   display_write_str("Par",1);

   char_height_mul = 1;
   
   
   // Mix 0 anzeigen
   
   /*
    // index gerade  : mixb mit (0x70)<<4, mixa mit 0x07
    // index ungerade: typ mit 0x03
    default:
    0x01, Kanal 0,1
    0x01, Typ 1: V-Mix
    0x23, Kanal 2,3
    0x02, Typ 2: Butterfly
*/
}

void setzuteilungscreen(void)
{
   // aktuelle Zuteilung der Kanaele/Funktionen zu den Devices feststellen
   /*
    // in updatescreen verschoben
   uint8_t kanalindex=0;
   
   for (kanalindex=0;kanalindex<8;kanalindex++)
   {
      uint8_t deviceindex = ((curr_funktionarray[kanalindex] & 0x70)>>4); // aktuelles device fuer kanal, bit 4-6
      // Kanal an deviceindex einsetzen
      curr_statusarray[deviceindex] = kanalindex ;
   }
*/
   uint8_t delta=6;
   resetRegister();
   blink_cursorpos=0xFFFF;
   
   // vertikal l
   posregister[0][0] =  22 |    (3 << 10); //
   posregister[0][1] =  32 |    (3 << 10); //
   
   // vertikal r
   posregister[0][2] =  82 |    (3 << 10); //
   posregister[0][3] =  92 |    (3 << 10); //
   
   
   // horizontal l
   posregister[1][0] =  10 |    (4 << 10); //
   posregister[1][1] =  10 |    (5 << 10); //
  
   // horizontal r
   posregister[1][2] =  70 |    (4 << 10); //
   posregister[1][3] =  70 |    (5 << 10); //

   // Schieber l
   posregister[2][0] =  20 |    (8 << 10); //
   posregister[2][1] =  30 |    (8 << 10); //
   
   // Schieber r
   posregister[2][2] =  80 |    (8 << 10); //
   posregister[2][3] =  90 |    (8 << 10); //
   
   
   cursorpos[0][0] = 12 |  (3 << 10); //
   cursorpos[0][1] = 72 |  (3 << 10); //

   cursorpos[1][0] = 0 |  (4 << 10); //
   cursorpos[1][1] = 60 |  (4 << 10); //

   cursorpos[2][0] = 10 |  (8 << 10); //
   cursorpos[2][1] = 70 |  (8 << 10); //

   
   //strcpy(menubuffer, (&(ZuteilungTable[0]))); // titel
   char_y= 1;
   char_x = itemtab[0] ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(ZuteilungTable[0],2);
   uint8_t page=0;
   for(page=3;page < 5;page++)
   {
      if (page==3)
      {
         uint8_t col=0;
         for (col=30; col< 52; col++)
         {
            display_go_to(col,page);
            if ((col == 40)|| (col==41))
            {
               display_write_byte(DATEN,0xFF);
            }
            else
            {
               display_write_byte(DATEN,0xC0);
            }
         }
         for (col=90; col< 112; col++)
         {
            display_go_to(col,page);
            if ((col == 100)|| (col==101))
            {
               display_write_byte(DATEN,0xFF);
            }
            else
            {
               display_write_byte(DATEN,0xC0);
            }
         }

      }
      display_go_to(40,page);
      display_write_byte(DATEN,0xFF);
      char_x++;
      display_write_byte(DATEN,0xFF);
      display_go_to(100,page);
      char_x++;
      display_write_byte(DATEN,0xFF);
      display_write_byte(DATEN,0xFF);
      
   }
   for(page=6;page < 8;page++)
   {
      display_go_to(60,page);
      display_write_byte(DATEN,0xFF);
      char_x++;
      display_write_byte(DATEN,0xFF);
      display_go_to(70,page);
      char_x++;
      display_write_byte(DATEN,0xFF);
      display_write_byte(DATEN,0xFF);

   }
   /*
   strcpy(menubuffer, (&(DeviceTable[1]))); // L_V
   char_y= (posregister[0][0] & 0xFF00)>> 10;;
   char_x = posregister[0][0] & 0x00FF ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(menubuffer,1);

   strcpy(menubuffer, (&(DeviceTable[3]))); // R_V
   char_y= (posregister[0][2] & 0xFF00)>> 10;;
   char_x = posregister[0][2] & 0x00FF ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(menubuffer,1);
  
   strcpy(menubuffer, (&(DeviceTable[2]))); // L_H
   char_y= (posregister[1][0] & 0xFF00)>> 10;;
   char_x = posregister[1][0] & 0x00FF ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(menubuffer,1);

   strcpy(menubuffer, (&(DeviceTable[0]))); // L_R
   char_y= (posregister[1][2] & 0xFF00)>> 10;;
   char_x = posregister[1][2] & 0x00FF ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(menubuffer,1);
 
   strcpy(menubuffer, (&(DeviceTable[4]))); // S_L
   char_y= (posregister[2][0] & 0xFF00)>> 10;;
   char_x = posregister[2][0] & 0x00FF ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(menubuffer,1);

   strcpy(menubuffer, (&(DeviceTable[5]))); // S_R
   char_y= (posregister[2][2] & 0xFF00)>> 10;;
   char_x = posregister[2][2] & 0x00FF ;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(menubuffer,1);
  */


   


}// setzuteilungscreen

void setsavescreen(void)
{
   resetRegister();
   blink_cursorpos=0xFFFF;
   
   
   
   posregister[0][0] =  itemtab[0] |     (4 << 10); // Aenderungen sichern
   posregister[0][1] =  itemtab[3] |    (4 << 10); //
//   posregister[0][2] =  itemtab[4] |    (3 << 10); // Abbrechen
//   posregister[0][3] =  itemtab[6] |    (3 << 10); //
   
   
   cursorpos[0][0] = cursortab[0] |    (4 << 10); // sichern
   cursorpos[0][1] = cursortab[3] |    (4 << 10); //  abbrechen
  
   strcpy(menubuffer, ((SichernTable[0]))); // titel
   char_y= 1;
   char_x = itemtab[0] ;
   char_height_mul = 2;
   char_width_mul = 1;
   //display_write_propchar('V',2);
   //display_write_propchar('?',2);
   //display_write_propchar(60,2);
   display_write_str(menubuffer,2);
   display_write_symbol(fragezeichen);

   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_x = posregister[0][0] & 0x00FF;
   strcpy(menubuffer, ((SichernTable[1]))); // sichern
   char_height_mul = 1;
   display_write_str(menubuffer,2);
   
   
   char_y= (posregister[0][1] & 0xFF00)>> 10;
   char_x = posregister[0][1] & 0x00FF;
   strcpy(menubuffer, ((SichernTable[2]))); // Abbrechen
   display_write_str(menubuffer,2);

   blink_cursorpos = cursorpos[0][0];

  
   
}

void settrimmscreen(void)
{
   resetRegister();
   blink_cursorpos=0xFFFF;
   
    
   posregister[0][0] =  itemtab[0] |   (2 << 10); //Vertikal links
   posregister[0][1] =  itemtab[4] |   (2 << 10); //Vertikal rechts
   
   posregister[1][0] =  itemtab[0] |    (5 << 10); // Horizontal links
   posregister[1][1] =  itemtab[4] |    (5 << 10); // Horizontal rechts
   
   cursorpos[0][0] = cursortab[0] |    (2 << 10); // cursorpos fuer Vertikal links
   cursorpos[0][1] = cursortab[4] |    (2 << 10); //  cursorpos fuer Vertikal rechts
   cursorpos[1][0] = cursortab[0] |    (4 << 10);  //cursorpos fuer Horizontal links
   cursorpos[1][1] = cursortab[0] |    (5 << 10);  //cursorpos fuer Horizontal rechts
   
   cursorpos[3][0] = cursortab[0] |    (6 << 10);  // cursorpos fuer save

   char_x = posregister[0][0] & 0x00FF;
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_height_mul = 1;
   char_width_mul = 1;

   


} //settrimmscreen

void update_motorzeit(void)
{   
   
   /*
   char_x = posregister[0][0] & 0x00FF;
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_height_mul = 1;
   char_width_mul = 1;
   */
   //display_write_min_sek(sendesekunde, 2);
   //display_write_zeit(motorsekunde&0xFF,motorminute,motorstunde, 2);
   
    
   // Motorzeit aktualisieren
   char_height_mul = 2;
   char_width_mul = 2;
   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
   //display_write_min_sek(motorsekunde,2);
   display_write_stopzeit(motorsekunde,motorminute, 2);
   
    
}

void update_blinkzeit(void)
{
   
}

void update_sendezeit(void)
{
   
   char_x = posregister[0][0] & 0x00FF;
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_zeit(sendesekunde&0xFF,sendeminute,sendestunde, 2);
  
}

void update_stopzeit(void)
{
   
   // Stoppzeit aktualisieren
   char_y= (posregister[2][1] & 0xFF00)>> 10;
   char_x = posregister[2][1] & 0x00FF;
   char_height_mul = 2;
   char_width_mul = 2;
   display_write_stopzeit(stopsekunde,stopminute, 2);
   
  
}


void update_time(uint8_t code)
{
   switch (code)
   {
   case 0:
      {
         //sendezeit aktualisieren
         
         char_x = posregister[0][0] & 0x00FF;
         char_y= (posregister[0][0] & 0xFF00)>> 10;
         char_height_mul = 1;
         char_width_mul = 1;
         
         display_write_zeit(sendesekunde&0xFF,sendeminute,sendestunde, 2);

      }break;
      
   case 1: // Stopzeit
      {
         // Stoppzeit aktualisieren
         char_y= (posregister[2][1] & 0xFF00)>> 10;
         char_x = posregister[2][1] & 0x00FF;
         char_height_mul = 2;
         char_width_mul = 2;
         display_write_stopzeit(stopsekunde,stopminute, 2);

      }break;
      
   case 2: // Motorzeit
      {
         // Motorzeit aktualisieren
         char_height_mul = 2;
         char_width_mul = 2;
         char_y= (posregister[1][1] & 0xFF00)>> 10;
         char_x = posregister[1][1] & 0x00FF;
         display_write_stopzeit(motorsekunde,motorminute, 2);

      }break;
      
   case 3: // Batteriespannung aktualisieren
      {
         
         // Batteriespannung aktualisieren
         char_y= (posregister[3][1] & 0xFF00)>> 10;
         char_x = posregister[3][1] & 0x00FF;
         char_height_mul = 1;
         char_width_mul = 1;
         display_write_spannung(batteriespannung/10,2);
  
      }break;
      case 4: // // Akkubalken aktualisieren
      {
         // Akkubalken anzeigen
         char_height_mul = 1;
         char_width_mul = 1;
         display_akkuanzeige(batteriespannung);

      }
         
      default:
      {
         return;//servostatus |= (1<<USB_OK);
      }break;
          
   }// switch
 
}

void update_akku(void)
{
   // Akkubalken anzeigen
   char_height_mul = 1;
   char_width_mul = 1;
   display_akkuanzeige(batteriespannung);

}

void update_batteriespannung(void)
{
   // Batteriespannung aktualisieren
   char_y= (posregister[3][1] & 0xFF00)>> 10;
   char_x = posregister[3][1] & 0x00FF;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_spannung(batteriespannung/10,2);
}

void set_parthomescreen(uint8_t code)
{
   switch (code)
   {
   case 0: // Positionen setzen
      {
         resetRegister();
         blink_cursorpos=0xFFFF;
         posregister[0][0] = itemtab[5] | (1 << 10);// Laufzeit Anzeige
         
         posregister[1][0] = (0+OFFSET_6_UHR) | (0x05 << 10); // Text Motorzeit
         posregister[1][1] = (0+OFFSET_6_UHR) | (0x06 << 10); // Anzeige Motorzeit
         
         posregister[2][0] = (60+OFFSET_6_UHR) | (0x05 << 10); // Text Stoppuhr
         posregister[2][1] = (60+OFFSET_6_UHR) | (0x06 << 10); // Anzeige Stoppuhr
         
         posregister[3][0] = (60+OFFSET_6_UHR) | (0x07 << 10); // Text Akku
         posregister[3][1] = (84+OFFSET_6_UHR) | (0x08 << 10); // Anzeige Akku

         posregister[4][0] = (0+OFFSET_6_UHR) | (2 << 10); // Name Modell
         posregister[4][1] = (80+OFFSET_6_UHR) | (3 << 10); // Text Setting
         posregister[4][2] = (100+OFFSET_6_UHR) | (3 << 10); // Anzeige Setting

         cursorpos[0][1] = cursortab[0] |    (8 << 10); //  cursorpos fuer Menu
         
      }break;
      case 1: // Titel schreiben
      {
         char_height_mul = 1;
         char_width_mul = 1;

         //display_write_prop_str(char_y,char_x,0,(unsigned char*)titelbuffer);
         display_write_str(TitelTable[0],1);
         //display_write_str(TitelTable[0],1);
         //display_write_str("ABC",1);
         //display_inverse(0);
         char_height_mul = 1;
         char_width_mul = 1;
         
         // Stoppuhrtext schreiben
         char_x = (posregister[2][0] & 0x00FF);
         char_y= (posregister[2][0] & 0xFF00)>> 10;
         display_write_str(TitelTable[2],2);
         // Motorzeittext schreiben
         char_x = (posregister[1][0] & 0x00FF);
         char_y= ((posregister[1][0] & 0xFF00)>> 10);
         char_height_mul = 1;
         display_write_str(TitelTable[3],2);


      } break;
      case 2: // Zeiten schreben
      {
         // Stoppzeit schreiben
         char_y= (posregister[2][1] & 0xFF00)>> 10;
         char_x = (posregister[2][1] & 0x00FF);
         char_height_mul = 2;
         char_width_mul = 2;
         display_write_stopzeit_BM(stopsekunde,stopminute);
         // Motorzeit schreiben
         char_y= (posregister[1][1] & 0xFF00)>> 10;
         char_x = posregister[1][1] & 0x00FF;
         // display_write_min_sek(motorsekunde,2);
         display_write_stopzeit(motorsekunde,motorminute, 2);

         char_height_mul = 1;
         char_width_mul = 1;
       
      }break;
      case 3:
      {
         // Modell schreiben
         char_y= (posregister[4][0] & 0xFF00)>> 10;
         char_x = posregister[4][0] & 0x00FF;
         //display_write_prop_str(char_y,char_x,0,titelbuffer,2);
         char_height_mul = 2;
         display_write_str(ModelTable[curr_model],1);

         char_height_mul = 1;
         //strcpy(titelbuffer, ((TitelTable[5])));
         char_y= (posregister[4][1] & 0xFF00)>> 10;
         char_x = posregister[4][1] & 0x00FF;
         display_write_str(TitelTable[5],2);
         char_y= (posregister[4][2] & 0xFF00)>> 10;
         char_x = posregister[4][2] & 0x00FF;
         display_write_int(curr_setting,2);

      }break;
      case 4:
      {
         char_height_mul = 1;
         char_width_mul = 1;

         // Batteriespannung
         char_y= ((posregister[3][0] & 0xFF00)>> 10)+1;
         char_x = posregister[3][0] & 0x00FF;
         char_height_mul = 1;
         display_write_str(TitelTable[6],2);
         
         char_height_mul = 1;
         char_width_mul = 1;

        // display_write_propchar(' ');
         /*
          // nicht unbediungt
         char_y= (cursorpos[0][1] & 0xFF00)>> 10;
         char_x = cursorpos[0][1] & 0x00FF;
         display_write_symbol(pfeilvollrechts);
         char_x += 4;
         display_write_str(TitelTable[4],2);
          */
      }break;
         
   }//switch code
}

uint8_t refresh_screen(void)
{
   display_clear();
   uint8_t fehler=0;
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   fehler=1;
   //Serial.printf("****************  refresh_screen: %d\n",curr_screen);
   
   switch (curr_screen)
   {
         
      case HOMESCREEN: // homescreen
      {
#pragma mark refresh HOMESCREEN
         
         fehler=2;
         //updatecounter++;
         
  
         char_x=OFFSET_6_UHR;
         char_y = 1;
         char_height_mul = 1;
         char_width_mul = 1;
         //display_go_to(char_x+1,0);
         
         //display_write_byte(DATEN,0xFF);
         char_x++;
         
         char_height_mul = 1;
         char_width_mul = 1;

         display_write_str(TitelTable[0],1);
         Serial.printf("refresh A\n"); 
         char_height_mul = 1;
         char_width_mul = 1;
         
         // Stoppuhrtext schreiben
         char_x = (posregister[2][0] & 0x00FF);
         char_y= (posregister[2][0] & 0xFF00)>> 10;
         display_write_str(TitelTable[2],2);
         Serial.printf("refresh B\n"); 
         // Stoppzeit schreiben
         char_y= (posregister[2][1] & 0xFF00)>> 10;
         char_x = (posregister[2][1] & 0x00FF);
         char_height_mul = 2;
         char_width_mul = 2;
         display_write_stopzeit_BM(stopsekunde,stopminute);
         Serial.printf("refresh C\n"); 
         char_height_mul = 1;
         char_width_mul = 1;
         
         
         // Motorzeittext schreiben
         char_x = (posregister[1][0] & 0x00FF);
         char_y= ((posregister[1][0] & 0xFF00)>> 10);
         char_height_mul = 1;
         display_write_str(TitelTable[3],2);
         Serial.printf("refresh D\n"); 
         char_height_mul = 2;
         char_width_mul = 2;

         char_y= (posregister[1][1] & 0xFF00)>> 10;
         char_x = posregister[1][1] & 0x00FF;
         // display_write_min_sek(motorsekunde,2);
         display_write_stopzeit(motorsekunde,motorminute, 2);
         Serial.printf("refresh E\n"); 
         char_height_mul = 1;
         char_width_mul = 1;
        
         
         // Modell schreiben
         char_y= (posregister[4][0] & 0xFF00)>> 10;
         char_x = posregister[4][0] & 0x00FF;
         //display_write_prop_str(char_y,char_x,0,titelbuffer,2);
         char_height_mul = 2;
         display_write_str(ModelTable[curr_model],1);
         Serial.printf("refresh F\n"); 
         char_height_mul = 1;
         //strcpy(titelbuffer, ((TitelTable[5])));
         char_y= (posregister[4][1] & 0xFF00)>> 10;
         char_x = posregister[4][1] & 0x00FF;
         display_write_str(TitelTable[5],2);
         char_y= (posregister[4][2] & 0xFF00)>> 10;
         char_x = posregister[4][2] & 0x00FF;
         display_write_int(curr_setting,2);
         Serial.printf("refresh G\n"); 
         
         
         char_height_mul = 1;
         char_width_mul = 1;

         // Batteriespannung
         char_y= ((posregister[3][0] & 0xFF00)>> 10)+1;
         char_x = posregister[3][0] & 0x00FF;
         char_height_mul = 1;
         display_write_str(TitelTable[6],2);
         Serial.printf("refresh H\n"); 
         
         char_height_mul = 1;
         char_width_mul = 1;

        // display_write_propchar(' ');
       
         char_y= (cursorpos[0][1] & 0xFF00)>> 10;
         char_x = cursorpos[0][1] & 0x00FF;
         display_write_symbol(pfeilvollrechts);
         Serial.printf("refresh I\n"); 
         char_x += 4;
         display_write_str(TitelTable[4],2);
         Serial.printf("refresh end\n");
      }break;
         
      case SETTINGSCREEN: // Setting
      {
         
#pragma mark refresh SETTINGSCREEN
         char_height_mul = 1;
         char_width_mul = 1;
         
         // Zeit aktualisieren
         char_y= 1;
         char_x = itemtab[5];
     //    display_write_min_sek(motorsekunde,2);
         
         char_height_mul = 1;
         if (programmstatus &(1<<UPDATESCREEN))
         {
            programmstatus &= ~(1<<UPDATESCREEN);
             
            // Modellname
         //   strcpy(menubuffer, (ModelTable[curr_model]));
            char_y= (posregister[0][0] & 0xFF00)>> 10;
            char_x = posregister[0][0] & 0x00FF;
            char_height_mul = 2;
            char_width_mul = 1;
            display_write_str(ModelTable[curr_model],1);
            char_height_mul = 1;
            
            // settingnummer
            char_height_mul = 1;
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = posregister[0][3] & 0x00FF;
            display_write_int(curr_setting,2);
         }
         
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            if (curr_cursorzeile==0) // Modellname
            {
               if (curr_cursorspalte == 0)
               {
                  char_height_mul = 2;
                  display_write_symbol(pfeilvollrechtsklein);
               }
               else // Set Nummer
               {
                  char_height_mul = 1;
                  display_write_symbol(pfeilvollrechts);
               }
               
            }
            else // alle anderen
            {
               char_height_mul = 1;
               display_write_symbol(pfeilvollrechts);
            }
            
         }
         
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            if (sendesekunde%2)
            {
               if (curr_cursorzeile==0)
               {
                  if (curr_cursorspalte == 0) // Modellname
                  {
                     char_height_mul = 2;
                     display_write_symbol(pfeilvollrechtsklein);
                  }
                  else // Set  Nummer
                  {
                     char_height_mul = 1;
                     display_write_symbol(pfeilvollrechts);
                  }
                  
                  
               }
               else
               {
                  char_height_mul = 1;
                  display_write_symbol(pfeilvollrechts);
               }
               
               
            }
            else
            {
               if (curr_cursorzeile==0)
               {
                  char_height_mul = 2;
               }
               else
               {
                  char_height_mul = 1;
               }
               
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         
         
      }break;
         
      case KANALSCREEN: // Kanal
      {
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            
#pragma mark refresh KANALSCREEN
            Serial.printf("refresh KANALSCREEN: %d\n",curr_screen);
            // kanalnummer
            char_y= (posregister[0][1] & 0xFF00)>> 10;
            char_x = posregister[0][1] & 0x00FF;
            char_height_mul = 1;
            char_width_mul = 1;
            display_write_int(curr_kanal,2);
            
            // Richtungspfeil anzeigen
            
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = posregister[0][3] & 0x00FF;
            char_height_mul = 1;
            char_width_mul = 1;
            
            
            // Funktion anzeigen // Bit 0-2 !!
            // !! Funktion ist bit 0-2 , Steuerdevice ist bit 4-6!!
            //strcpy(menubuffer, ((FunktionTable[(curr_funktionarray[curr_kanal]&0x07)])));
            
            
            char_y= (posregister[0][4] & 0xFF00)>> 10;
            char_x = posregister[0][4] & 0x00FF;
            //display_write_str(FunktionTable[(curr_funktionarray[curr_kanal]&0x07)],2);
            
            // levelwert A anzeigen
            char_y= (posregister[1][2] & 0xFF00)>> 10;
            char_x = posregister[1][2] & 0x00FF;
            display_write_int((8-(curr_levelarray[curr_kanal] & 0x07)),1);
            //display_write_int(8-((curr_settingarray[curr_kanal][0] & 0x70)>>4),1);
            
            
            display_write_str("/8\0",1);
            
            // levelwert B anzeigen
            char_y= (posregister[1][4] & 0xFF00)>> 10;
            char_x = posregister[1][4] & 0x00FF;
            
            display_write_int(8-((curr_levelarray[curr_kanal] & 0x70)>>4),1);
            display_write_str("/8\0",1);
            
            
            // expowert A anzeigen
            char_y= (posregister[2][2] & 0xFF00)>> 10;
            char_x = posregister[2][2] & 0x00FF;
            display_write_int((curr_expoarray[curr_kanal] & 0x07),1);
            
            
            // expowert B anzeigen
            char_y= (posregister[2][4] & 0xFF00)>> 10;
            char_x = posregister[2][4] & 0x00FF;
            display_write_int((curr_expoarray[curr_kanal] & 0x70)>>4,1);
            
            
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = posregister[0][3] & 0x00FF;
            
            if (curr_kanal < 4) 
            {
               if (((curr_statusarray[curr_kanal] & 0x70)>>4)%2 == 0) // waagrecht, gerade Kanalnummer, 0 - 3
                  
               {
                  if (curr_statusarray[curr_kanal] & 0x80) // Bit 7 fuer Richtung
                  {
                     display_write_propsymbol(proprichtungright);
                  }
                  else
                  {
                     display_write_propsymbol(proprichtungleft);
                  }
                  
               }
               else // senkrecht
               {
                  if  (curr_statusarray[curr_kanal] & 0x80) // Bit 7 fuer Richtung
                  {
                     display_write_propsymbol(proprichtungup);
                  }
                  else
                  {
                     display_write_propsymbol(proprichtungdown);
                  }
                  
               }
            } // if kanal < 4
            // Typ anzeigen
            char_y= (posregister[3][1] & 0xFF00)>> 10;
            char_x = posregister[3][1] & 0x00FF;
            char_height_mul = 1;
            char_width_mul = 1;
            
            // PGM_P typsymbol = (&(steuertyp[curr_funktionarray[curr_kanal]]));
            
            uint8_t kanaltyp =(curr_expoarray[curr_kanal] & 0x0C)>>2;
            char typsymbol = steuertyp[kanaltyp];
            //typsymbol=pitch;
            
            display_write_propsymbol(typsymbol);
            //strcpy(menubuffer, (&(KanalTypTable[kanaltyp]))); // Art wert
            //display_write_propsymbol(pitch);
            
            char_height_mul = 1;
            char_width_mul = 1;
            
            display_kanaldiagramm_var(64+OFFSET_6_UHR, 6, curr_levelarray[curr_kanal], curr_expoarray[curr_kanal], 1);
         
            
            char_height_mul = 1;
            
           // display_trimmanzeige_horizontal (14+OFFSET_6_UHR, 7,-15);

         }
         
         // Blinken
         //Serial.printf("update_screen blink_cursorpos: %d z: %d",blink_cursorpos,sendesekunde);
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            if ((curr_cursorspalte <=3)&& (curr_cursorzeile<=2)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }
            
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            if ((curr_cursorspalte <=1)&& (curr_cursorzeile==1)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         
      }break;
         
      case MIXSCREEN:
      {
#pragma mark refresh MIXSCREEN
         //uint8_t delta=6;
         /*
          // index gerade  : mixb mit (0x70)<<4, mixa mit 0x07
          
          // index ungerade: typ mit 0x03
          default:
          0x01, Kanal 0,1
          0x01, Typ 1: V-Mix
          0x23, Kanal 2,3
          0x02, Typ 2: Butterfly
          
          Kanalnummer 8: > OFF
          
          */
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            
            // Mix 0
            uint8_t mixtyp = curr_mixarray[1]& 0x03; // von ungeradem Index
            mixtyp &= 0x03; // nur 4 Typen
            
            //strcpy(menubuffer, (&(MixTypTable[mixtyp]))); // Typtext
            
            char_y= (posregister[0][0] & 0xFF00)>> 10;
            char_x = posregister[0][0] & 0x00FF;
            display_write_str(MixTypTable[mixtyp],2); // Mix-Typ
            uint8_t canalnummera=0,canalnummerb=0;
            
            if (mixtyp)
            {
               //Kanal A
               char_y= (posregister[0][2] & 0xFF00)>> 10;
               char_x = posregister[0][2] & 0x00FF;
               
               // Funktion anzeigen
               // Funktion fuer Seite A:
               canalnummera = ((curr_mixarray[0] & 0xF0)>>4);
                // index in curr_funktionarray: Kanalnummer von Seite A: (curr_mixarray[0] & 0x70)>>4]], Bit 4,5
              
               if (canalnummera < 8)
               {
                  display_write_int(canalnummera,2); // Kanalnummer A, von geradem Index
                  display_write_str(": \0",2);
               
                  //strcpy(menubuffer, (&(FunktionTable[canalnummera]))); // Funktion
                  display_write_str(FunktionTable[canalnummera],1);
               }
               else
               {
                  display_write_str(" - ",1);
                  display_write_str("  OFF \0",1);
               }

               
               //Kanal B
               char_y= (posregister[0][5] & 0xFF00)>> 10;
               char_x = (posregister[0][5] & 0x00FF);
               
               // Funktion anzeigen
               // Funktion fuer Seite B:
               canalnummerb = (curr_mixarray[0] & 0x0F);
               // index in curr_funktionarray: Kanalnummer von Seite B: (curr_mixarray[0] & 0x70)]], Bit 0,1
               if (canalnummerb < 8)
               {
                  display_write_int((curr_mixarray[0] & 0x0F),2);// Kanalnummer B, von geradem Index
                  display_write_str(": \0",2);

                  //strcpy(menubuffer, (&(FunktionTable[canalnummerb]))); // Funktion
                  display_write_str(FunktionTable[canalnummerb],1);
               }
               else
               {
                  display_write_str(" - ",1);
                  display_write_str("  OFF \0",1);
               }
               
            }
            else // Mix 0 ist OFF
            {
               char_y= (posregister[0][2] & 0xFF00)>> 10;
               char_x = posregister[0][2] & 0x00FF;
               display_write_str(" -\0",1);
               display_write_str("   OFF     \0",1);
               char_y= (posregister[0][5] & 0xFF00)>> 10;
               char_x = (posregister[0][5] & 0x00FF);
               display_write_str(" -\0",1);
               display_write_str("   OFF    \0",1);
               
            }
            
            
            // Mix 1
            mixtyp = curr_mixarray[3]& 0x03; // von ungeradem Index
            mixtyp &= 0x03; // nur 4 Typen
            //strcpy(menubuffer, (&(MixTypTable[mixtyp]))); // Leveltext
            char_y= (posregister[1][0] & 0xFF00)>> 10;
            char_x = posregister[1][0] & 0x00FF;
            display_write_str(MixTypTable[mixtyp],2); // Mix-Typ
            
            if (mixtyp)
            {
               
               //Kanal A
               char_y= (posregister[1][2] & 0xFF00)>> 10;
               char_x = posregister[1][2] & 0x00FF;
               
               // Funktion anzeigen
               // Funktion fuer Seite A:
               canalnummera = ((curr_mixarray[2] & 0xF0)>>4);
               if (canalnummera < 8)
               {
                  display_write_int(((curr_mixarray[2] & 0xF0)>>4),2); // Kanalnummer A, von geradem Index
                  display_write_str(": ",2);
                  
               // index in curr_funktionarray: Kanalnummer von Seite A: (curr_mixarray[0] & 0x70)>>4]], Bit 4,5
                  //strcpy(menubuffer, (&(FunktionTable[canalnummera]))); // Funktion
                  display_write_str(FunktionTable[canalnummera],1);
               }
               else
               {
                  display_write_str(" - ",1);
                  display_write_str("  OFF \0",1);
               }
               
               
               //Kanal B
               char_y= (posregister[1][5] & 0xFF00)>> 10;
               char_x = (posregister[1][5] & 0x00FF);
               canalnummerb = (curr_mixarray[2] & 0x0F);
               
               // Funktion anzeigen
               // Funktion fuer Seite B:
               
               // index in curr_funktionarray: Kanalnummer von Seite B: (curr_mixarray[0] & 0x70)]], Bit 0,1
               if (canalnummerb<8)
               {
                  display_write_int((curr_mixarray[2] & 0x0F),2);// Kanalnummer B, von geradem Index
                  display_write_str(": ",2);

                  //strcpy(menubuffer, (&(FunktionTable[canalnummerb]))); // Funktion
                  display_write_str(FunktionTable[canalnummerb],1);
               }
               else
               {
                  display_write_str(" - ",1);
                  display_write_str("  OFF ",1);
               }
            }
            else // Mix 1 ist OFF
            {
               char_y= (posregister[1][2] & 0xFF00)>> 10;
               char_x = posregister[1][2] & 0x00FF;
               display_write_str(" -\0",1);
               display_write_str("   OFF     \0",1);
               char_y= (posregister[1][5] & 0xFF00)>> 10;
               char_x = (posregister[1][5] & 0x00FF);
               display_write_str(" -\0",1);
               display_write_str("   OFF    \0",1);
               
            }
            
            
         } // if updaate
         
         
         
         // Cursor anzeigen
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            char_height_mul = 1;
            
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         
         
      }break;
         
      case ZUTEILUNGSCREEN:
      {
#pragma mark refresh ZUTEILUNGSCREEN
         
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            
            uint8_t kanalindex=0;
            for (kanalindex=0;kanalindex<8;kanalindex++)
            {
               uint8_t deviceindex = ((curr_funktionarray[kanalindex] & 0x70)>>4); // aktuelles device fuer kanal, bit 4-6
               // Kanal an deviceindex einsetzen
 //              curr_statusarray[deviceindex] = kanalindex ;
            }
            
            // Ungerade Nummern_ vertikal
            
            // Device 1: L_V
            
            // Display-Position Nummer
            char_y= (posregister[0][0] & 0xFF00)>> 10;
            char_x = (posregister[0][0] & 0x00FF);
            
            // Kanalnummer: aktueller Wert in curr_statusarray
            uint8_t canalnummer = ((curr_statusarray[1]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer ,
            
            // Display-Position Funktion
            char_y= (posregister[0][1] & 0xFF00)>> 10;
            char_x = (posregister[0][1] & 0x00FF);
            // Funktionnummer: aktueller wert in curr_funktionarray auf Zeile canalnummer
            uint8_t funktionnummer= curr_funktionarray[canalnummer]&0x07;
            
            // index der Devicenummer (1 fuer L_V) in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x10 | (curr_funktionarray[canalnummer]&0x0F);
            
            // Name der Funktion schreiben
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // L_V
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Device 3: R_V
            // Display-Position Nummer
            char_y = (posregister[0][2] & 0xFF00)>> 10;
            char_x = (posregister[0][2] & 0x00FF);

            // Kanalnummer: aktueller Wert in curr_statusarray
            canalnummer = ((curr_statusarray[3]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer R_V,
            
            // Display-Position Funktion
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = (posregister[0][3] & 0x00FF);
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x30 | (curr_funktionarray[canalnummer]&0x0F);

            // Name der Funktion schreiben
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // R_V
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Gerade Nummern_ horizontal

            // Device 0: L_H
            
            char_y = (posregister[1][0] & 0xFF00)>> 10;
            char_x = (posregister[1][0] & 0x00FF);
            canalnummer = ((curr_statusarray[0]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer L_H,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
 
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x00 | (curr_funktionarray[canalnummer]&0x0F);
            
            char_y = (posregister[1][1] & 0xFF00)>> 10;
            char_x = (posregister[1][1] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // L_H
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Device 2: R_H
            
            char_y = (posregister[1][2] & 0xFF00)>> 10;
            char_x = (posregister[1][2] & 0x00FF);
            canalnummer =((curr_statusarray[2]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer R_H,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x20 | (curr_funktionarray[canalnummer]&0x0F);
            
            char_y = (posregister[1][3] & 0xFF00)>> 10;
            char_x = (posregister[1][3] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // R_H
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Device 4: Schieber L
            
            // Position Nummer
            char_y = (posregister[2][0] & 0xFF00)>> 10;
            char_x = (posregister[2][0] & 0x00FF);
            canalnummer = ((curr_statusarray[4]& 0x07));
            display_write_int(canalnummer,1);// Schieber l,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            // Position Funktion
            char_y = (posregister[2][1] & 0xFF00)>> 10;
            char_x = (posregister[2][1] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // S_L
            display_write_str(FunktionTable[funktionnummer],1);
            
            // Device 5: Schieber R
            char_y = (posregister[2][2] & 0xFF00)>> 10;
            char_x = (posregister[2][2] & 0x00FF);
            canalnummer = ((curr_statusarray[5]& 0x07));
            display_write_int(canalnummer,1);// Schieber l,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
  //          curr_funktionarray[canalnummer] = 0x10 | (curr_funktionarray[canalnummer]&0x07);
            

            // Position Funktion
            char_y = (posregister[2][3] & 0xFF00)>> 10;
            char_x = (posregister[2][3] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // S_R
            display_write_str(FunktionTable[funktionnummer],1);
            
         }
         
         // Cursor anzeigen
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            char_height_mul = 1;
            
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         
         
      }break;
         
         
      case AUSGANGSCREEN:
      {
#pragma mark refresh AUSGANGSCREEN
         uint8_t cursoroffset=0;
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            uint8_t delta=2;
            uint8_t spaltenarray[4] = {itemtab[0],itemtab[1]-delta,itemtab[2],itemtab[4]};
            uint8_t impulsindex=0;
            uint8_t startindex = 0, endindex=5;
            
            startindex = 0;
            endindex = 5;
            char_y = 3;
            if (curr_impuls<4)
            {
               cursoroffset=0;
            }
            else // scrollen, curr_cursorzeile auf 1 <> impuls 4
            {
               if (cursoroffset==0) // curr_impuls neu > 3
               {
                  display_cursorweg();
               }
               //curr_cursorzeile=1;
               //startindex = 3;
               //endindex = 8;
               cursoroffset = 3;
               
               
            }
            
            //cursoroffset = 3;
            for (impulsindex= (startindex);impulsindex < (endindex );impulsindex++)
            {
               char_y = impulsindex+3;
               // Impulsnummer in Summensignal
               char_x = spaltenarray[0];
               display_write_int(impulsindex + cursoroffset,1);
               
               // Kanalnummer
               char_x = spaltenarray[1];
               
               // canalnummer ist im ausgangarray bit 0-2: Reihenfolge der Kanaele im Summensignal
               
               uint8_t canalnummer = curr_ausgangarray[impulsindex + cursoroffset]&0x07;
               //uint8_t canalnummer = curr_ausgangarray[impulsindex-cursoroffset]&0x07;
               
               display_write_int(canalnummer,1);
               
               // Devicenummer
               char_x = spaltenarray[2];
               //display_write_int((curr_funktionarray[canalnummer]&0x70)>>4,1);
               //uint8_t devicenummer = curr_statusarray[impulsindex]&0x07;
               
               //devicenummer ist im funktionarray bit 4-6. Wird in Zuteilung gesetzt
               uint8_t devicenummer = (curr_funktionarray[canalnummer]&0x70)>>4;
               
               //display_write_int(devicenummer,1);
 
               //strcpy(menubuffer, (&(DeviceTable[devicenummer])));
               display_write_str(DeviceTable[devicenummer],1);
               
               // Funktion
               char_x = spaltenarray[3];
               uint8_t funktionnummer =(curr_funktionarray[canalnummer]&0x07);
               //display_write_int(funktionnummer,1);
               
               //strcpy(menubuffer, (&(FunktionTable[funktionnummer])));
               display_write_str(FunktionTable[funktionnummer],1);
               
               /*
               //if (char_y == 3)
               {
               display_write_str(" ",1);
               display_write_int(curr_impuls,1);
               display_write_str(" ",1);
               display_write_int(curr_cursorzeile,1);

               }
                */
            }

            
         } // if update
         
         // Cursor anzeigen
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= ((cursorposition & 0xFF00)>> 10);;//-cursoroffset;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= ((blink_cursorpos & 0xFF00)>> 10);//-cursoroffset;
            char_x = blink_cursorpos & 0x00FF;
            char_height_mul = 1;
            
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;

      }break;

#pragma mark refresh SAVESCREEN

      case SAVESCREEN: // Kanal
      {
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            if ((curr_cursorspalte <=3)&& (curr_cursorzeile<=2)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }
            
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            if ((curr_cursorspalte <=1)&& (curr_cursorzeile==1)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }

            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         char_y= 5;
         char_x = 10+OFFSET_6_UHR;
         display_write_int(eepromsavestatus,2);
         
         
      }break;

#pragma mark refresh TRIMMSCREEN

      case TRIMMSCREEN: // Kanal
      {
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_y= 5;
         char_x = 10+OFFSET_6_UHR;
         
         display_trimmanzeige_horizontal (14+OFFSET_6_UHR, 7,1,-15);
      }break;


   }
   return fehler;
}

uint8_t update_screen(void)
{
   
   uint8_t fehler=0;
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   fehler=1;
   //Serial.printf("****************  update_screen: %d programmstatus: %d\n",curr_screen, programmstatus);
   
   switch (curr_screen)
   {
         
      case HOMESCREEN: // homescreen
      {
#pragma mark update HOMESCREEN
         
         fehler=2;
         //updatecounter++;
         /*
         //Laufzeit
         char_x = posregister[0][0] & 0x00FF;
         char_y= (posregister[0][0] & 0xFF00)>> 10;
         char_height_mul = 1;
         char_width_mul = 1;
         display_write_zeit(motorsekunde&0xFF,motorminute,motorstunde, 2);

         //display_write_min_sek(motorsekunde, 2);
          // Stoppzeit aktualisieren
         char_y= (posregister[2][1] & 0xFF00)>> 10;
         char_x = posregister[2][1] & 0x00FF;
         char_height_mul = 2;
         char_width_mul = 2;
 //        if (programmstatus &(1<<STOP_ON)) // loescht nicht bei reset
         {
            display_write_stopzeit_BM(stopsekunde,stopminute);
         }
 
         // Motorzeit aktualisieren
//         char_height_mul = 2;
//         char_width_mul = 2;
         char_y= (posregister[1][1] & 0xFF00)>> 10;
         char_x = posregister[1][1] & 0x00FF;
         {
            display_write_stopzeit_BM(motorsekunde,motorminute);
         }
 */
         // Batteriespannung aktualisieren
         char_y= (posregister[3][1] & 0xFF00)>> 10;
         char_x = posregister[3][1] & 0x00FF;
         char_height_mul = 1;
         char_width_mul = 1;
         display_write_spannung(batteriespannung/10,2);
         
         
         char_y= (cursorpos[0][1] & 0xFF00)>> 10;
         char_x = cursorpos[0][1] & 0x00FF;
         display_write_symbol(pfeilvollrechts);
         char_x += 4;
         display_write_str(TitelTable[4],2);

         /*
         char_x=4;
         char_y = 4;
         display_write_str("                    ",2);

         char_x=4;
         char_y = 4;
         
         //display_write_int(Tastenwert&0x00FF,2);
         display_write_int((tastentransfer&0xFF00)>> 10,2);
         display_write_int((tastentransfer&0x00FF),2);
         char_x=40;
         display_write_int((batteriespannung&0xFF00)>> 10,2);
         display_write_propchar('*',1);
         display_write_int((batteriespannung&0x00FF),2);
         */
         
         // Akkubalken anzeigen
 //        char_height_mul = 1;
 //        char_width_mul = 1;
//         display_akkuanzeige(batteriespannung);
         
     //    display_trimmanzeige_horizontal (4+OFFSET_6_UHR,3, 4,-15);
         
 //        display_trimmanzeige_vertikal (52+OFFSET_6_UHR,6, 4,vertikaltrimm);
         
      }break;
         
      case SETTINGSCREEN: // Setting
      {
         
#pragma mark update SETTINGSCREEN
         char_height_mul = 1;
         char_width_mul = 1;
         
         // Zeit aktualisieren
         char_y= 1;
         char_x = itemtab[5];
     //    display_write_min_sek(motorsekunde,2);
         
         char_height_mul = 1;
         if (programmstatus &(1<<UPDATESCREEN))
         {
            programmstatus &= ~(1<<UPDATESCREEN);
            
            
            // Modellname
         //   strcpy(menubuffer, (ModelTable[curr_model]));
            char_y= (posregister[0][0] & 0xFF00)>> 10;
            char_x = posregister[0][0] & 0x00FF;
            char_height_mul = 2;
            char_width_mul = 1;
            display_write_str(ModelTable[curr_model],1);
            char_height_mul = 1;
            
            // settingnummer
            char_height_mul = 1;
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = posregister[0][3] & 0x00FF;
            display_write_int(curr_setting,2);
         }
         
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            if (curr_cursorzeile==0) // Modellname
            {
               if (curr_cursorspalte == 0)
               {
                  char_height_mul = 2;
                  display_write_symbol(pfeilvollrechtsklein);
               }
               else // Set Nummer
               {
                  char_height_mul = 1;
                  display_write_symbol(pfeilvollrechts);
               }
               
            }
            else // alle anderen
            {
               char_height_mul = 1;
               display_write_symbol(pfeilvollrechts);
            }
            
         }
         
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            if (sendesekunde%2)
            {
               if (curr_cursorzeile==0)
               {
                  if (curr_cursorspalte == 0) // Modellname
                  {
                     char_height_mul = 2;
                     display_write_symbol(pfeilvollrechtsklein);
                  }
                  else // Set  Nummer
                  {
                     char_height_mul = 1;
                     display_write_symbol(pfeilvollrechts);
                  }
                  
                  
               }
               else
               {
                  char_height_mul = 1;
                  display_write_symbol(pfeilvollrechts);
               }
               
               
            }
            else
            {
               if (curr_cursorzeile==0)
               {
                  char_height_mul = 2;
               }
               else
               {
                  char_height_mul = 1;
               }
               
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         
         
      }break;
         
      case KANALSCREEN: // Kanal
      {
         
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            
#pragma mark update KANALSCREEN
            Serial.printf("update KANALSCREEN: %d\n",curr_screen);
           
            for (uint8_t i=0;i<8;i++)
            {
          //     Serial.printf("i: %d  curr_funktionarray %d: \n",i,curr_funktionarray[i]);
            }
            // kanalnummer
            char_y= (posregister[0][1] & 0xFF00)>> 10;
            char_x = posregister[0][1] & 0x00FF;
            char_height_mul = 1;
            char_width_mul = 1;
            display_write_int(curr_kanal,2);
            
            // Richtungspfeil anzeigen
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = posregister[0][3] & 0x00FF;
            char_height_mul = 1;
            char_width_mul = 1;
            
            // Funktion anzeigen
            char* funktionstring = FunktionTable[curr_devicearray[curr_kanal]];
            //Serial.printf("update Kanalscreen funktionstring: %s\n",funktionstring);
            char_y= (posregister[0][5] & 0xFF00)>> 10;
            char_x = posregister[0][5] & 0x00FF;
        display_write_str(funktionstring,2);
            
            
              
            // levelwert A anzeigen
            char_y= (posregister[1][2] & 0xFF00)>> 10;
            char_x = posregister[1][2] & 0x00FF;
            display_write_int(8-((curr_levelarray[curr_kanal] & 0x70)>>4),1);
            display_write_str("/8\0",1);
            
            // levelwert B anzeigen
            char_y= (posregister[1][4] & 0xFF00)>> 10;
            char_x = posregister[1][4] & 0x00FF;
            display_write_int((8-(curr_levelarray[curr_kanal] & 0x07)),1);       
            display_write_str("/8\0",1);
            
            
            // expowert A anzeigen
            char_y= (posregister[2][2] & 0xFF00)>> 10;
            char_x = posregister[2][2] & 0x00FF;
            uint8_t expowert = (curr_expoarray[curr_kanal] & 0x70)>>4;
            //Serial.printf("T8 Expowert A curr expo vor: %d expowert: %d\n",curr_expoarray[curr_kanal] , expowert);
            display_write_int((curr_expoarray[curr_kanal] & 0x70)>>4,1);
            display_write_str("/8\0",1);
            
            
            // expowert B anzeigen
            char_y= (posregister[2][4] & 0xFF00)>> 10;
            char_x = posregister[2][4] & 0x00FF;
            display_write_int((curr_expoarray[curr_kanal] & 0x07),1);
            display_write_str("/8\0",1);
            
              
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = posregister[0][3] & 0x00FF;
            
            if (curr_kanal < 4) 
            {
               if (((curr_statusarray[curr_kanal] & 0x70)>>4)%2 == 0) // waagrecht, gerade Kanalnummer, 0 - 3
               {
                  if (curr_statusarray[curr_kanal] & 0x80) // Bit 7 fuer Richtung
                  {
                     display_write_propsymbol(proprichtungright);
                  }
                  else
                  {
                     display_write_propsymbol(proprichtungleft);
                  }
                  
               }
               else if (((curr_statusarray[curr_kanal] & 0x70)>>4)%2 == 1)// senkrecht
               {
                  if  (curr_statusarray[curr_kanal] & 0x80) // Bit 7 fuer Richtung
                  {
                     display_write_propsymbol(proprichtungup);
                  }
                  else
                  {
                     display_write_propsymbol(proprichtungdown);
                  }
                  
               }
            } // if kanal < 4
            
            // Typ anzeigen
            char_y= (posregister[3][1] & 0xFF00)>> 10;
            char_x = posregister[3][1] & 0x00FF;
            char_height_mul = 1;
            char_width_mul = 1;
            
            // PGM_P typsymbol = (&(steuertyp[curr_funktionarray[curr_kanal]]));
            
    //        uint8_t kanaltyp =(curr_expoarray[curr_kanal] & 0x0C)>>2;
    //        char typsymbol = steuertyp[kanaltyp];
            //typsymbol=pitch;
            
    //        display_write_propsymbol(typsymbol);
            //strcpy(menubuffer, (&(KanalTypTable[kanaltyp]))); // Art wert
            //display_write_propsymbol(pitch);
            
            char_height_mul = 1;
            char_width_mul = 1;
            
            //display_kanaldiagramm (64+OFFSET_6_UHR, 6, curr_levelarray[curr_kanal], curr_expoarray[curr_kanal], 1);
            
            // uint8_t display_kanaldiagramm_var (uint8_t char_x0, uint8_t char_y0, uint8_t level, uint8_t expo, uint8_t typ )
            
            display_kanaldiagramm_var(64, 7, curr_levelarray[curr_kanal], curr_expoarray[curr_kanal], 1);
         
            
            char_height_mul = 1;
            
           // display_trimmanzeige_horizontal (14+OFFSET_6_UHR, 7,-15);
         } // if update
         // Blinken
         //Serial.printf("update_screen blink_cursorpos: %d z: %d",blink_cursorpos,sendesekunde);
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            /*
            if ((curr_cursorspalte <=3)&& (curr_cursorzeile<=2)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }
            */
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            if ((curr_cursorspalte <=1)&& (curr_cursorzeile==1)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         //Serial.printf("update Kanalscreen End\n");
      }break;
         
      case MIXSCREEN:
      {
#pragma mark update MIXSCREEN
         //uint8_t delta=6;
         /*
          // index gerade  : mixb mit (0x70)<<4, mixa mit 0x07
          
          // index ungerade: typ mit 0x03
          default:
          0x01, Kanal 0,1
          0x01, Typ 1: V-Mix
          0x23, Kanal 2,3
          0x02, Typ 2: Butterfly
          
          Kanalnummer 8: > OFF
          
          
          */
         /* new:
          curr_mixstatusarray[mixnummer] = mix0;
          curr_mixkanalarray[mixnummer] = mix1;
          uint8_t modelindex = mix0 & 0x03; // bit 0,1
          
          */
         /*
          const char funktion0[]  = "Seite";
          const char funktion1[]  = "Hoehe";
          const char funktion2[]  = "Quer ";
          const char funktion3[]  = "Motor";
          const char funktion4[]  = "QuerL";
          const char funktion5[]  = "QuerR";
          const char funktion6[]  = "Lande";
          const char funktion7[]  = "Aux  ";
          
          const char *FunktionTable[]  = {funktion0, funktion1, funktion2, funktion3, funktion4, funktion5, funktion6, funktion7};
          
          */

         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            //Serial.printf("\nupdate_screen curr_mixstatusarray[0]: %d curr_mixkanalarray[0]: %d\n",curr_mixstatusarray[0],curr_mixkanalarray[0]);
            
            // Mix 0 Byte 0 in curr_xy
            
            uint8_t modelindex = curr_mixstatusarray[0] & 0x03; // bit 0,1
            uint8_t mixart = (curr_mixstatusarray[0] & 0x30) >> 4; // bit 4,5
            uint8_t mixnummer = (curr_mixstatusarray[0] & 0xC0) >> 6; // bit 6,7
            uint8_t mixon = (curr_mixstatusarray[0] & 0x08) >> 3; // Bit 3

            uint8_t mixkanala=0,mixkanalb=0;
            
            //Serial.printf("update_screen Mix0 mixnummer: %d mixart: %d mixon: %d mixtyp: %s\n",mixnummer,mixart, mixon,MixTypTable[mixart]);

            {
                
               char_y= (posregister[0][0] & 0xFF00)>> 10;
               char_x = posregister[0][0] & 0x00FF;
               display_write_str(MixTypTable[mixart],2); // Mix-Typ

               char_y= (posregister[0][1] & 0xFF00)>> 10;
               char_x = posregister[0][1] & 0x00FF;
               if (mixon)
               {
                  display_write_str("OK",2); // Mix-Typ
               }
               else
               {
                  display_write_str("--",2); // Mix-Typ
               }
                
               //Kanal A
               char_y= (posregister[0][3] & 0xFF00)>> 10;
               char_x = posregister[0][3] & 0x00FF;
               
               // Funktion anzeigen
               // Funktion fuer Seite A:
               mixkanala = curr_mixkanalarray[mixnummer] & 0x07 ; // Bit 0-3
               //Serial.printf("update_screen mixkanala: %d \n",mixkanala);
               // index in curr_funktionarray: Kanalnummer von Seite A: (curr_mixarray[0] & 0x70)>>4]], Bit 4,5
               
                  //Serial.printf("update_screen mixkanala OK Fkt: %s\n ",FunktionTable[mixkanala]);
                  display_write_int(mixkanala,2); // Kanalnummer A, von geradem Index
                  display_write_str(":",2);
                  display_write_str(FunktionTable[mixkanala],1);
                
               
               //Kanal B
               char_y= (posregister[0][5] & 0xFF00)>> 10;
               char_x = (posregister[0][5] & 0x00FF);
               
               // Funktion anzeigen
               // Funktion fuer Seite B:
               // canalnummerb = (curr_mixarray[0] & 0x0F);
               mixkanalb = (curr_mixkanalarray[mixnummer] & 0x70) >> 4; // Bit 4-6
               
               //Serial.printf("update_screen mixkanalb: %d \n",mixkanalb);
               
               // index in curr_funktionarray: Kanalnummer von Seite B: (curr_mixarray[0] & 0x70)]], Bit 0,1
               if (mixkanalb < 8)
               {
                  //Serial.printf("update_screen mixkanalb OK Fkt: %s\n ",FunktionTable[mixkanalb]);
                  display_write_int(mixkanalb,2);// Kanalnummer B, von geradem Index
                  display_write_str(":",2);
                  
                  display_write_str(FunktionTable[mixkanalb],1);
               }
               else
               {
                  display_write_str(" - ",1);
                  display_write_str("  OFF \0",1);
               }
               
            } // if mixon
            /*
            else // Mix 0 ist OFF
            {
               char_y= (posregister[0][2] & 0xFF00)>> 10;
               char_x = posregister[0][2] & 0x00FF;
               display_write_str(" -\0",1);
               display_write_str("   OFF     \0",1);
               char_y= (posregister[0][5] & 0xFF00)>> 10;
               char_x = (posregister[0][5] & 0x00FF);
               display_write_str(" -\0",1);
               display_write_str("   OFF    \0",1);
               
            }
            */
           
            
            
            // Mix 1 Byte 1 in curr_xy
            
            modelindex = curr_mixstatusarray[1] & 0x03; // bit 0,1
            mixart = (curr_mixstatusarray[1] & 0x30) >> 4; // bit 4,5
            mixnummer = (curr_mixstatusarray[1] & 0xC0) >> 6; // bit 6,7
            mixon = (curr_mixstatusarray[1] & 0x08) >> 3; // Bit 3

            mixkanala=0;
            mixkanalb=0;
            
            //Serial.printf("update_screen Mix1 mixnummer: %d mixart: %d mixon: %d \n",mixnummer,mixart, mixon);
            //if (mixon)
            {
               char_y= (posregister[1][0] & 0xFF00)>> 10;
               char_x = posregister[1][0] & 0x00FF;
               display_write_str(MixTypTable[mixart],2); // Mix-Typ

               //Kanal A
               char_y= (posregister[1][2] & 0xFF00)>> 10;
               char_x = posregister[1][2] & 0x00FF;
               
               // Funktion anzeigen
               // Funktion fuer Seite A:
               mixkanala = curr_mixkanalarray[mixnummer] & 0x07 ; // Bit 0-3
               if (mixon)
               {
                  display_write_int(mixkanala,2); // Kanalnummer A, von geradem Index
                  display_write_str(":",2);
                  display_write_str(FunktionTable[mixkanala],1);
               }
               else
               {
                  display_write_str(" - ",1);
                  display_write_str("  OFF \0",1);
               }
               
               
               //Kanal B
               char_y= (posregister[1][5] & 0xFF00)>> 10;
               char_x = (posregister[1][5] & 0x00FF);
               mixkanalb = (curr_mixkanalarray[mixnummer] & 0x70) >> 4; // Bit 4-6
               
               // Funktion anzeigen
               // Funktion fuer Seite B:
               
               // index in curr_funktionarray: Kanalnummer von Seite B: (curr_mixarray[0] & 0x70)]], Bit 0,1
               if (mixon)
               {
                  display_write_int(mixkanalb,2);// Kanalnummer B, von geradem Index
                  display_write_str(":",2);
                  
                  //strcpy(menubuffer, (&(FunktionTable[canalnummerb]))); // Funktion
                  display_write_str(FunktionTable[mixkanalb],1);
               }
               else
               {
                  //display_write_str(" - ",1);
                  //display_write_str("  OFF ",1);
               }
            }
            /*
            else // Mix 1 ist OFF
            {
               
               char_y= (posregister[1][2] & 0xFF00)>> 10;
               char_x = posregister[1][2] & 0x00FF;
               display_write_str(" -\0",1);
               display_write_str("   OFF     \0",1);
               char_y= (posregister[1][5] & 0xFF00)>> 10;
               char_x = (posregister[1][5] & 0x00FF);
               display_write_str(" -\0",1);
               display_write_str("   OFF    \0",1);
               
            }
           */
            
         } // if update
         
         
         
         // Cursor anzeigen
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            char_height_mul = 1;
            
     /*       
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
       */     
         }
         char_height_mul = 1;
         
         //Serial.printf("update_screen end\n\n");
      }break;
         
      case ZUTEILUNGSCREEN:
      {
#pragma mark update ZUTEILUNGSCREEN
         
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            
            uint8_t kanalindex=0;
            for (kanalindex=0;kanalindex<8;kanalindex++)
            {
               uint8_t deviceindex = ((curr_funktionarray[kanalindex] & 0x70)>>4); // aktuelles device fuer kanal, bit 4-6
               // Kanal an deviceindex einsetzen
 //              curr_statusarray[deviceindex] = kanalindex ;
            }
            
            // Ungerade Nummern_ vertikal
            
            // Device 1: L_V
            
            // Display-Position Nummer
            char_y= (posregister[0][0] & 0xFF00)>> 10;
            char_x = (posregister[0][0] & 0x00FF);
            
            // Kanalnummer: aktueller Wert in curr_statusarray
            uint8_t canalnummer = ((curr_statusarray[1]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer ,
            
            // Display-Position Funktion
            char_y= (posregister[0][1] & 0xFF00)>> 10;
            char_x = (posregister[0][1] & 0x00FF);
            // Funktionnummer: aktueller wert in curr_funktionarray auf Zeile canalnummer
            uint8_t funktionnummer= curr_funktionarray[canalnummer]&0x07;
            
            // index der Devicenummer (1 fuer L_V) in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x10 | (curr_funktionarray[canalnummer]&0x0F);
            
            // Name der Funktion schreiben
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // L_V
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Device 3: R_V
            // Display-Position Nummer
            char_y = (posregister[0][2] & 0xFF00)>> 10;
            char_x = (posregister[0][2] & 0x00FF);

            // Kanalnummer: aktueller Wert in curr_statusarray
            canalnummer = ((curr_statusarray[3]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer R_V,
            
            // Display-Position Funktion
            char_y= (posregister[0][3] & 0xFF00)>> 10;
            char_x = (posregister[0][3] & 0x00FF);
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x30 | (curr_funktionarray[canalnummer]&0x0F);

            // Name der Funktion schreiben
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // R_V
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Gerade Nummern_ horizontal

            // Device 0: L_H
            
            char_y = (posregister[1][0] & 0xFF00)>> 10;
            char_x = (posregister[1][0] & 0x00FF);
            canalnummer = ((curr_statusarray[0]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer L_H,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
 
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x00 | (curr_funktionarray[canalnummer]&0x0F);
            
            char_y = (posregister[1][1] & 0xFF00)>> 10;
            char_x = (posregister[1][1] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // L_H
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Device 2: R_H
            
            char_y = (posregister[1][2] & 0xFF00)>> 10;
            char_x = (posregister[1][2] & 0x00FF);
            canalnummer =((curr_statusarray[2]& 0x07));
            display_write_int(canalnummer,1);// Kanalnummer R_H,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
            curr_funktionarray[canalnummer] = 0x20 | (curr_funktionarray[canalnummer]&0x0F);
            
            char_y = (posregister[1][3] & 0xFF00)>> 10;
            char_x = (posregister[1][3] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // R_H
            display_write_str(FunktionTable[funktionnummer],1);
            
            
            // Device 4: Schieber L
            
            // Position Nummer
            char_y = (posregister[2][0] & 0xFF00)>> 10;
            char_x = (posregister[2][0] & 0x00FF);
            canalnummer = ((curr_statusarray[4]& 0x07));
            display_write_int(canalnummer,1);// Schieber l,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            // Position Funktion
            char_y = (posregister[2][1] & 0xFF00)>> 10;
            char_x = (posregister[2][1] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // S_L
            display_write_str(FunktionTable[funktionnummer],1);
            
            // Device 5: Schieber R
            char_y = (posregister[2][2] & 0xFF00)>> 10;
            char_x = (posregister[2][2] & 0x00FF);
            canalnummer = ((curr_statusarray[5]& 0x07));
            display_write_int(canalnummer,1);// Schieber l,
            funktionnummer= curr_funktionarray[canalnummer]&0x07;
            // index der Devicenummer in curr_funktionarray einsetzen: bit 4-6
  //          curr_funktionarray[canalnummer] = 0x10 | (curr_funktionarray[canalnummer]&0x07);
            

            // Position Funktion
            char_y = (posregister[2][3] & 0xFF00)>> 10;
            char_x = (posregister[2][3] & 0x00FF);
            //strcpy(menubuffer, (&(FunktionTable[funktionnummer]))); // S_R
            display_write_str(FunktionTable[funktionnummer],1);
            
         }
         
         // Cursor anzeigen
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            char_height_mul = 1;
            
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         
         
      }break;
         
         
      case AUSGANGSCREEN:
      {
#pragma mark update AUSGANGSCREEN
         uint8_t cursoroffset=0;
         if (programmstatus & (1<< UPDATESCREEN))
         {
            programmstatus &= ~(1<< UPDATESCREEN);
            uint8_t delta=2;
            uint8_t spaltenarray[4] = {itemtab[0],itemtab[1]-delta,itemtab[2],itemtab[4]};
            uint8_t impulsindex=0;
            uint8_t startindex = 0, endindex=5;
            
            startindex = 0;
            endindex = 5;
            char_y = 3;
            if (curr_impuls<4)
            {
               cursoroffset=0;
            }
            else // scrollen, curr_cursorzeile auf 1 <> impuls 4
            {
               if (cursoroffset==0) // curr_impuls neu > 3
               {
                  display_cursorweg();
               }
               //curr_cursorzeile=1;
               //startindex = 3;
               //endindex = 8;
               cursoroffset = 3;
               
               
            }
            
            //cursoroffset = 3;
            for (impulsindex= (startindex);impulsindex < (endindex );impulsindex++)
            {
               char_y = impulsindex+3;
               // Impulsnummer in Summensignal
               char_x = spaltenarray[0];
               display_write_int(impulsindex + cursoroffset,1);
               
               // Kanalnummer
               char_x = spaltenarray[1];
               
               // canalnummer ist im ausgangarray bit 0-2: Reihenfolge der Kanaele im Summensignal
               
               uint8_t canalnummer = curr_ausgangarray[impulsindex + cursoroffset]&0x07;
               //uint8_t canalnummer = curr_ausgangarray[impulsindex-cursoroffset]&0x07;
               
               display_write_int(canalnummer,1);
               
               // Devicenummer
               char_x = spaltenarray[2];
               //display_write_int((curr_funktionarray[canalnummer]&0x70)>>4,1);
               //uint8_t devicenummer = curr_statusarray[impulsindex]&0x07;
               
               //devicenummer ist im funktionarray bit 4-6. Wird in Zuteilung gesetzt
               uint8_t devicenummer = (curr_funktionarray[canalnummer]&0x70)>>4;
               
               //display_write_int(devicenummer,1);
 
               //strcpy(menubuffer, (&(DeviceTable[devicenummer])));
               display_write_str(DeviceTable[devicenummer],1);
               
               // Funktion
               char_x = spaltenarray[3];
               uint8_t funktionnummer =(curr_funktionarray[canalnummer]&0x07);
               //display_write_int(funktionnummer,1);
               
               //strcpy(menubuffer, (&(FunktionTable[funktionnummer])));
               display_write_str(FunktionTable[funktionnummer],1);
               
               /*
               //if (char_y == 3)
               {
               display_write_str(" ",1);
               display_write_int(curr_impuls,1);
               display_write_str(" ",1);
               display_write_int(curr_cursorzeile,1);

               }
                */
            }

            
         } // if update
         
         // Cursor anzeigen
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= ((cursorposition & 0xFF00)>> 10);;//-cursoroffset;
            char_x = cursorposition & 0x00FF;
            char_height_mul = 1;
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= ((blink_cursorpos & 0xFF00)>> 10);//-cursoroffset;
            char_x = blink_cursorpos & 0x00FF;
            char_height_mul = 1;
            
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;

      }break;

#pragma mark update SAVESCREEN

      case SAVESCREEN: // Kanal
      {
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            if ((curr_cursorspalte <=3)&& (curr_cursorzeile<=2)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }
            
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            if ((curr_cursorspalte <=1)&& (curr_cursorzeile==1)) //Erste Zeile, Kanalnummer
            {
               char_height_mul = 1;
            }
            else
            {
               char_height_mul = 1;
            }

            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_height_mul = 1;
         char_y= 5;
         char_x = 10+OFFSET_6_UHR;
         display_write_int(eepromsavestatus,2);
         
         
      }break;

#pragma mark update TRIMMSCREEN

      case TRIMMSCREEN: // Kanal
      {
         if (blink_cursorpos == 0xFFFF) // Kein Blinken des Cursors
         {
            char_y= (cursorposition & 0xFF00)>> 10;
            char_x = cursorposition & 0x00FF;
            
            display_write_symbol(pfeilvollrechts);
            
         }
         else // Cursor blinkt an blink_cursorpos
         {
            
            char_y= (blink_cursorpos & 0xFF00)>> 10;
            char_x = blink_cursorpos & 0x00FF;
            
            if (sendesekunde%2)
            {
               display_write_symbol(pfeilvollrechts);
            }
            else
            {
               display_write_symbol(pfeilwegrechts);
            }
            
         }
         char_y= 5;
         char_x = 10+OFFSET_6_UHR;
         
         display_trimmanzeige_horizontal (14+OFFSET_6_UHR, 7,1,-15);
      }break;


   }
   return fehler;
}

//##############################################################################################
// Curser weg an curr position
//
//##############################################################################################
void display_cursorweg(void)
{
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   char_y= (cursorposition & 0xFF00)>> 10;
   char_x = cursorposition & 0x00FF;
   display_write_symbol(pfeilwegrechts);
}

void display_setcursorblink(uint8_t zeit)
{
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   char_y= (cursorposition & 0xFF00)>> 10;
   char_x = cursorposition & 0x00FF;
      if (zeit%2) // gerade
      {
         display_write_symbol(pfeilwegrechts);
      }
      else
      {
         display_write_symbol(pfeilvollrechts);
      }

   
}
//##############################################################################################
// Trimmanzeige
//
//##############################################################################################
void display_trimmanzeige_horizontal (uint8_t char_x0, uint8_t char_y0, uint8_t device, int8_t mitteposition)
{
   //mitteposition ist Abweichung von Mitte des Kanals, mit Vorzeichen
   
   uint8_t col=char_x0, page=char_y0, breite=100;
   
   // linke Begrenzung
   display_go_to(col,page);
   
   uint8_t i=0;
   for (i=0;i<breite;i++)
   {
      if ((i==0)||(i==breite-1) || (i==breite/2) || (i== breite/2+mitteposition)|| (i== breite/2+mitteposition-1)|| (i== breite/2+mitteposition+1))
      {
         display_write_byte(DATEN,0x7E);// Strich zeichnen
      }
      
        else
      {
         display_write_byte(DATEN,0x42);// obere und untere linie  zeichnen
      }
   }

}

void display_trimmanzeige_vertikal (uint8_t char_x0, uint8_t char_y0, uint8_t device, int8_t mitteposition)
{
   //mitteposition ist Abweichung von Mitte des Kanals, mit Vorzeichen
   
   uint8_t col=char_x0, page=char_y0, breite=6; uint8_t hoehe = 6;
   
   uint16_t markenlage =(hoehe*4 + mitteposition);
   //display_go_to(col,page);
   int8_t full =markenlage/8; // ganze pages
   int8_t part =markenlage%8; // rest
   uint8_t balkenbreite = 6;
   
   //full=2;
   //part=4;
   
   // linke Begrenzung
   /*
   display_go_to(0,3);
   display_write_int(mitteposition,2);
   display_write_propchar(' ',2);
   
   display_write_int(full,2);
   display_write_propchar(' ',2);
   display_write_int(part,2);
   
   return;
    */
   //display_go_to(col,page);
   for (page=char_y0;page > char_y0-(hoehe);page--)
   {
      if (page == char_y0-hoehe/2)// Mitte
      {
         display_go_to(char_x0-1,page);
         display_write_byte(DATEN,0x80);
         display_go_to(char_x0+breite,page);
         display_write_byte(DATEN,0x80);

         
      
      }

      for (col=char_x0;col<char_x0+breite;col++)
      {
         display_go_to(col,page);
         
         if ((col==char_x0)||(col==char_x0+breite-1) )
         {
            display_write_byte(DATEN,0xFF);// senkrechte Begrenzung zeichnen
         }
         
         else
         {
            
            uint8_t markenwert=0;
            
            if (page == (char_y0-full)) // Wert liegt in der page
            //if (page == (2)) // Wert liegt in der page
            {
               markenwert = marke[part];
            }
            if (page == char_y0) // untere Begrenzung
            {
               markenwert |= 0x80;
            }
           // else if (page == 1)  // obere Begrenzung

            else if (page == char_y0+1-hoehe)  // obere Begrenzung
            {
               markenwert |= 0x01;
            }
            
            else if (page == char_y0-hoehe/2)// Mitte
            {
               markenwert |= 0x80;
            }
            
            
            display_write_byte(DATEN,markenwert);
         }
         
      }
      
   }
  // display_go_to(char_x0,0);
   
   
}

//##############################################################################################
// Akkuanzeige
//
//##############################################################################################
void display_akkuanzeige (uint16_t spannung)
{
   uint16_t balkenhoehe =(spannung-MINSPANNUNG)*64/(MAXSPANNUNG-MINSPANNUNG);
   uint8_t col=0, page=0;
   uint8_t full =balkenhoehe/8; // page ist voll
   uint8_t part =balkenhoehe%8; // rest
   uint8_t balkenbreite = 12;
   uint8_t grenze = 4;
   //part=4;
   //full = 2;
   uint8_t char_x0 = 112+OFFSET_6_UHR;
   
   for (page=1;page<8;page++)
   {
      /*
      display_go_to(char_x+1,page);
      display_write_byte(DATEN,0xAA);
      display_go_to(char_x+balkenbreite,page);
      display_write_byte(DATEN,0xAA);
       */
      col=0;
      while(col++ < balkenbreite)
      {
         display_go_to(char_x0+col,page);
         
         if (page < (7-full)) // sicher
         {
            if (page == grenze) // Strich zeichnen
            {
               display_write_byte(DATEN,0x80);
            }
            else // leer lassen
            {
               display_write_byte(DATEN,00);
            }
         }
         else if (page == (7-full)) // grenzwertig
         {
            if ((full<grenze-1) && (sendesekunde%2)) // Blinken
            {
               display_write_byte(DATEN,0x00);
            }
            else
            {
               if (page == grenze) // Strich zeichnen wenn unter Grenze, sonst luecke zeichnen
               {
                  //display_write_byte(DATEN,(balken[part] | 0x08));
                  display_write_byte(DATEN,(balken[part] ^ 0x80)); // war 0x80 fuer duenneren Strich
               }
               else if (page > grenze) // kein
               {
                  display_write_byte(DATEN,(balken[part] ));
               }
               else
               {
                  display_write_byte(DATEN,(balken[part] ));
               }
            }
            
         }
         else // wird unsicher
         {
            if (page == grenze) // grenzwertig
            {
               display_write_byte(DATEN,0x7F); // Strich zeichnen
            }
            else
            {
               if ((full<grenze-1) && (sendesekunde%2)) // Blinken
               {
                  display_write_byte(DATEN,0x00);
               }
               else
               {
                  display_write_byte(DATEN,0xFF); // voller Balken
               }
            } // else if (page == grenze)
            
         } // else if(page == (7-full))
         
      } // while col
      

   } // for page
   
}

//##############################################################################################
//Diagramm
//
//##############################################################################################
uint8_t display_diagramm (uint8_t char_x, uint8_t char_y, uint8_t stufea, uint8_t stufeb, uint8_t typ )
{
   uint8_t pageA=0, pageB=0, col=0;
   uint16_t wertYA=0 , wertYB=0 ;

   uint8_t maxX=50, maxY=48;
   uint8_t endY= maxY*(4-stufea)/4; // punkte, nicht page
   uint8_t page=0;
   for (page=char_y;page>3;page--) //Ordinate
   {
      display_go_to(char_x-maxX,page);
      display_write_byte(DATEN,0xDB); // Strich zeichnen

      display_go_to(char_x,page);
      display_write_byte(DATEN,0xFF); // Strich zeichnen
      display_go_to(char_x+maxX,page);
      display_write_byte(DATEN,0xDB); // Strich zeichnen

   }
   //uint16_t steigung= 0xFF*maxY*(4-stufe)/4/maxX; // punkte, nicht page
   // Steigung = (4-stufe)/4  1:1 ist Stufe 0
   uint8_t k=0;
   for (col=1;col<maxX;col++)
   {
      wertYA = (4-stufea)*col*0x20/0x32/4;
      pageA = 7-(wertYA/8);
      wertYB = (4-stufeb)*col*0x20/0x32/4;
      pageB = 7-(wertYB/8);
      
      for (k=7; k >2; k--)
      {
         // Seite B ( rechts)
         display_go_to(char_x+col,k);
         if (k == pageB) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATEN,(1<<(7-wertYB%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATEN,(1<<(7-wertYB%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATEN,0x80); //Punkt zeichnen
         }
         
         // Seite A (links)
      
         display_go_to(char_x-col,k);
         if (k == pageA) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATEN,(1<<(7-wertYA%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATEN,(1<<(7-wertYA%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATEN,0x80); //Punkt zeichnen
         }
       

      
      }
      
      
      
      //display_go_to(char_x+col,page);
      //display_write_byte(DATEN,(1<<(7-wertY%8))); //Punkt zeichnen
   }
   
   
   return 1;
   
}


uint8_t display_kanaldiagramm (uint8_t char_x0, uint8_t char_y0, uint8_t level, uint8_t expo, uint8_t typ )
{
   uint8_t pageA=0, pageB=0, col=0;
   uint16_t wertYA=0 , wertYB=0 ;
   
   uint8_t maxX=50, maxY=48;
   //uint8_t endY= maxY*(4-stufea)/4; // punkte, nicht page
   uint8_t page=0;
   for (page=char_y0;page>3;page--) //Ordinate
   {
      display_go_to(char_x0-maxX,page);
      display_write_byte(DATEN,0xDB); // Strich zeichnen
      
      display_go_to(char_x0,page);
      display_write_byte(DATEN,0xFF); // Strich zeichnen
      display_go_to(char_x0+maxX,page);
      display_write_byte(DATEN,0xDB); // Strich zeichnen
      
   }
   //uint16_t steigung= 0xFF*maxY*(4-stufe)/4/maxX; // punkte, nicht page
   // Steigung = (4-stufe)/4  1:1 ist Stufe 0
   uint8_t k=0;
   uint8_t expoa=((expo & 0x30)>>4);
   uint8_t expob=(expo & 0x03);
   for (col=1;col<maxX;col++)
   {
      if (expoa==0) // linear
      {
         wertYA = (8-((level & 0x70)>>4))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            // expoa wirkt erst ab wert 1, array der Werte ist 0-basiert: Wert an expoa-1 lesen
            wertYA = ((expoarray25[expoa-1][col/2]))/2 +((expoarray25[expoa-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYA = ((expoarray25[expoa-1][col/2]));
         }
         wertYA =(8-((level & 0x70)>>4))*wertYA/8; // Level
      }
      pageA = 7-(wertYA/8);
      
      
      
      if (expob==0) // linear
      {
         wertYB = (8-(level & 0x07))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            wertYB = ((expoarray25[expob-1][col/2]))/2 + ((expoarray25[expob-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYB = ((expoarray25[expob-1][col/2]));
         }
         wertYB =(8-(level & 0x07))*wertYB/8; // Level
      }
      
      pageB = 7-(wertYB/8);
      
      for (k=7; k >2; k--)
      {
         // Seite B ( rechts)
         display_go_to(char_x0+col,k);
         if (k == pageB) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATEN,(1<<(7-wertYB%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATEN,(1<<(7-wertYB%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATEN,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATEN,0x00); //Punkte entfernen
         }
         
         // Seite A (links)
         
         display_go_to(char_x0-col,k);
         if (k == pageA) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATEN,(1<<(7-wertYA%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATEN,(1<<(7-wertYA%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATEN,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATEN,0x00); //Punkte entfernen
         }
         
         
         
      }
      
      
      
      //display_go_to(char_x+col,page);
      //display_write_byte(DATEN,(1<<(7-wertY%8))); //Punkt zeichnen
   }
   
   
   return expob;
   
}

uint8_t display_kanaldiagramm_var (uint8_t char_x0, uint8_t char_y0, uint8_t level, uint8_t expo, uint8_t typ )
{
   uint8_t pageA=0, pageB=0, col=0;
   uint16_t wertYA=0 , wertYB=0 ;
   
   uint8_t maxX=50, maxY=48;
   //uint8_t endY= maxY*(4-stufea)/4; // punkte, nicht page
   uint8_t page=0;
   for (page=char_y0;page>char_y0-4;page--) //Ordinate
   {
      display_go_to(char_x0-maxX,page);
      display_write_byte(DATEN,0xDB); // Strich zeichnen
      
      display_go_to(char_x0,page);
      display_write_byte(DATEN,0xFF); // Strich zeichnen
      display_go_to(char_x0+maxX,page);
      display_write_byte(DATEN,0xDB); // Strich zeichnen
      
   }
   //uint16_t steigung= 0xFF*maxY*(4-stufe)/4/maxX; // punkte, nicht page
   // Steigung = (4-stufe)/4  1:1 ist Stufe 0
   uint8_t k=0;
   uint8_t expoa=((expo & 0x70)>>4);
   uint8_t expob=(expo & 0x07);
   for (col=1;col<maxX;col++)
   {
      if (expoa==0) // linear
      {
      wertYA = (8-((level & 0x70)>>4))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            // expoa wirkt erst ab wert 1, array der Werte ist 0-basiert: Wert an expoa-1 lesen
            wertYA = ((expoarray25[expoa-1][col/2]))/2 + ((expoarray25[expoa-1][col/2+1]))/2;
       //     wertYA = ((expoarray25[expoa][col/2]))/2 + ((expoarray25[expoa][col/2+1]))/2;

         }
         else // gerade, Wert aus Array
         {
            wertYA = ((expoarray25[expoa-1][col/2]));
           // wertYA = ((expoarray25[expoa][col/2]));
         }
         wertYA =(8-((level & 0x70)>>4))*wertYA/8; // Level
      }
      pageA = char_y0-(wertYA/8);
      
      
      if (expob==0) // linear
      {
         wertYB = (8-(level & 0x07))*col*0x20/0x32/8;
      }
      else
      {
         if (col%2) // ungerade, interpolieren mit naechstem Wert
         {
            wertYB = ((expoarray25[expob-1][col/2]))/2 +pgm_read_byte(&(expoarray25[expob-1][col/2+1]))/2;
         }
         else // gerade, Wert aus Array
         {
            wertYB = ((expoarray25[expob-1][col/2]));
         }
         wertYB =(8-(level & 0x07))*wertYB/8; // Level
      }
      
      
      pageB = char_y0-(wertYB/8);
      
      
      
      for (k=char_y0; k >char_y0-5; k--)
      {
         // Seite B ( rechts)
         display_go_to(char_x0+col,k);
         if (k == pageB) // Auf dieser Page liegt der Wert
         {
            if (col%4==0)
            {
               display_write_byte(DATEN,(1<<(char_y0-wertYB%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATEN,(1<<(char_y0-wertYB%8)));
            }
            
            
         }
            
         else if (col%4==0)
         {
            display_write_byte(DATEN,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATEN,0x00); //Punkte entfernen
         }
          
         // Seite A (links)
         
         display_go_to(char_x0-col,k);
         if (k == pageA) // Auf dieser Page liegt der Wert
         {
            if (col%3==0)
            {
               display_write_byte(DATEN,(1<<(7-wertYA%8))|0x80); //Punkt zeichnen
            }
            else
            {
               display_write_byte(DATEN,(1<<(7-wertYA%8)));
            }
            
            
         }
         else if (col%3==0)
         {
            display_write_byte(DATEN,0x80); //Punkt zeichnen
         }
         else
         {
            display_write_byte(DATEN,0x00); //Punkte entfernen
         }
      
         
         
      }
      
      
      
      //display_go_to(char_x+col,page);
      //display_write_byte(DATEN,(1<<(7-wertY%8))); //Punkt zeichnen
   }
   
   
   return expob;
   
}







//##############################################################################################
//Writes one byte to DATEN or cmd register
//
//##############################################################################################
void display_back_char (void)
{
	char_x = char_x - (FONT_WIDTH*char_width_mul);
	if (char_x > 128+OFFSET_6_UHR) char_x = 0+OFFSET_6_UHR;
}

//##############################################################################################
//Writes one byte to DATEN or cmd register
//
//##############################################################################################
void display_write_byte(unsigned cmd_DATEN, unsigned char data) 
{
   if(cmd_DATEN == 0)
	{
		//A0_HI;
      digitalWriteFast(DOG_A0,1);
	}
	else
	{
		//A0_LO;
      digitalWriteFast(DOG_A0,0);
	}
   
   
   spi_out(data);
  
 }

//##############################################################################################
//Init LC-Display
//
//##############################################################################################
void display_init() 
{
   /*
	//Set TIMER0 (PWM OC2 Pin)
	HG_PWM_DDR |= (1<<HG_PWM_PIN);//PWM PORT auf Ausgang (OC2)
	TCCR2A |= (1<<WGM21|1<<WGM20|1<<COM2A1|1<<CS20);
	OCR2A = 50;
	
	//set outputs AO und RESET
	DDR_A0  |= (1<<PIN_A0);             
	DDR_RST |= (1<<PIN_RST);
  
	//Set SPI PORT
	DDRB |= (1<<SPI_Clock)|(1<<SPI_DO)|(1<<SPI_SS);
	PORTB |= (1<<SPI_SS);
	//Enable SPI, SPI in Master Mode
	SPCR = (1<<SPE)|(1<<MSTR); 	
  
    //Reset the Display Controller
	PORT_RST &= ~(1<<PIN_RST);                   
	PORT_RST |= (1<<PIN_RST);
	//asm("nop");

   
   
	_delay_us(10);
	//send 11 init commands to Display
   
	for (unsigned char tmp = 0;tmp < 14;tmp++)
	{
		display_write_byte(CMD,DISPLAY_INIT[tmp]);
      _delay_us(10);
	}
	display_clear();
	
	return;
    */
}

void display_soft_init()
{
	_delay_us(1);
	//send 11 init commands to Display
   
	for (unsigned char tmp = 0;tmp < 14;tmp++)
	{
		display_write_byte(CMD,DISPLAY_INIT[tmp]);
      _delay_us(10);
	}
	display_clear();
	
	return;
}

void display_set_LED(uint8_t state)
{
   if (state)
   {
      //SOFT_SPI_PORT |= (1<<DOG_PWM);
      digitalWriteFast(DOG_PWM,1);
   }
   else
   {
      //SOFT_SPI_PORT &= ~(1<<DOG_PWM);
      digitalWriteFast(DOG_PWM,0);
   }
}
//##############################################################################################
//Go to x,y
//
//##############################################################################################

void display_go_to (unsigned char x, unsigned char y)
{
   
	display_write_byte(CMD,DISPLAY_PAGE_ADDRESS | ((y) & 0x0F));
   
	display_write_byte(CMD,DISPLAY_COL_ADDRESS_MSB | ((x>>4) & 0x0F));
  
	display_write_byte(CMD,DISPLAY_COL_ADDRESS_LSB | ((x) & 0x0F));
	return;
}



//##############################################################################################
//Diplay clear
//
//##############################################################################################
void display_clear()
{
	unsigned char page, col;
	
	for(page=0;page<8;page++)
	{
		display_go_to(0+OFFSET_6_UHR,page);
	
		for (col=0+OFFSET_6_UHR;col<128+OFFSET_6_UHR;col++)
		{
         if (col%4)
         {
            display_write_byte(DATEN,0x00);
         }
         else
         {
            display_write_byte(0,0x00);
         }
		}
	}
}


//##############################################################################################
//Diplay clear
//
//##############################################################################################
void display_inverse(uint8_t inv)
{
   display_write_byte(1,0xA6+(inv & 0x01));
}

//##############################################################################################
//Diplay Memory (BMP MONO 64x128)
//
//##############################################################################################
void display_mem(void* pointer)
{
	unsigned char col,page;
	
	for(col=DISPLAY_OFFSET;col < 128 + DISPLAY_OFFSET;col++) 
	{
		for (page=8;page!=0;page--)
		{
			display_go_to(col,page-1);
				
			display_write_byte(0,~(pgm_read_byte(pointer++)));
		}
	}
}

//##############################################################################################
//Ausgabe eines Zeichens
//
//##############################################################################################
/*
void display_write_char(unsigned char c)
{
	unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
	PGM_P pointer = font[c-32];
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4; // HI byte
				tmp1 = tmp1 & 0x0f;     // LO byte
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
				
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
				
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
				
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				} // end >2

			
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATEN,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{	
				display_write_byte(DATEN,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}
*/
// inv
/*
void  display_write_inv_char(unsigned char c)
{
	unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
	PGM_P pointer = font[c-32];
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp4);
					}
				}
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATEN,~tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATEN,~tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}
*/

//##############################################################################################
//Ausgabe eines Prop Zeichens
//
//##############################################################################################
void display_write_propchar(unsigned char c, uint8_t prop)
{
	unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
   
	PGM_P pointer;
   uint8_t charsize = 8;
   switch (prop)
   {
      case 1:
      {
         pointer = propfont6[c-32];
         //charsize = 6;
      }break;
         
      case 2:
      {
         pointer = propfont8[c-32];
      }break;
      default:
      {
          pointer = propfont6[c-32];
      }
   }
	uint8_t  charbreite =   pgm_read_byte(pointer++);
   
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(charbreite*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((charsize/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4; // HI byte
				tmp1 = tmp1 & 0x0f;     // LO byte
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				} // end >2
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATEN,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATEN,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
      char_x++;
      display_write_byte(DATEN,0x00);
      if (char_height_mul > 1)
      {
         display_go_to(char_x,char_y);
         display_write_byte(DATEN,0x00);
      }
      //char_x++;
		char_x = char_x + (charbreite*char_width_mul);
	}
	return;
}

// invertiert

void display_write_inv_propchar(unsigned char c, uint8_t prop)
{
	unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
   
	PGM_P pointer;
   uint8_t charsize = 8;
   switch (prop)
   {
      case 1:
      {
         pointer = propfont6[c-32];
      }
      case 2:
      {
         pointer = propfont8[c-32];
         
      }break;
      default:
      {
         pointer = propfont6[c-32];
      }
   }
	uint8_t  charbreite =   pgm_read_byte(pointer++);
   
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(charbreite*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((charsize/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = pgm_read_byte(pointer++);
			
			if (char_height_mul > 1) // schreiben auf mehrere pages
			{
				tmp2 = (tmp1&0xf0)>>4; // HI byte
				tmp1 = tmp1 & 0x0f;     // LO byte
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,~tmp4);
					}
				} // end >2
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATEN,~tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATEN,~tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
      display_write_byte(DATEN,0xFF);
      if (char_height_mul > 1)
      {
         display_go_to(col,page-1);
         display_write_byte(DATEN,0xFF);
      }
      char_x++;
		char_x = char_x + (charbreite*char_width_mul);
	}
	return;
}




//##############################################################################################
// Ausgabe simple char
/*
void display_write_simplechar(unsigned char c)
{
	unsigned char col,tmp1,page;
	PGM_P pointer = font[c-32];
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH)+DISPLAY_OFFSET);col=col+1)
	{
		{
			tmp1 = pgm_read_byte(pointer++);
			
			display_go_to(col,char_y);
         display_write_byte(DATEN,tmp1);
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH);
	}
	return;
}
*/
//##############################################################################################
// Ausgabe simple prop char

void display_write_simple_propchar(unsigned char c, uint8_t prop, uint8_t offset)
{
	unsigned char col,page;
   uint16_t tmp1;
	PGM_P pointer = propfont8[c-32];
   
  
   //char_x = 60;
   uint8_t fontwidth =pgm_read_byte(pointer++);
   
  // tmp1 = pgm_read_byte(pointer++);
   
   //display_go_to(char_x,char_y-1);
   
   //display_write_int((tmp1 & 0xFF),2);
   //tmp1 <<=4;
   uint8_t line_y = char_y;

	uint8_t sub_y = char_y -1;
   //char_y = 2;
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(fontwidth)+DISPLAY_OFFSET);col=col+1)
	{
		{
			tmp1 = pgm_read_byte(pointer++);
         //uint8_t offset = 2;
         //tmp1 <<= 8;         // 8 bit nach oben zum voraus: Platz schaffen
         tmp1<<= (8-offset);          // offset nach unten
         uint8_t tmp3 = (tmp1&0xFF00)>> 10; // obere 8 bit, 8 bit nach unten, ergibt lo
         uint8_t tmp4 = (tmp1&0x00FF);//>>4; // obere 8 bit,  ergibt hi
         display_go_to(col,char_y);
         display_write_byte(DATEN,tmp3);

         display_go_to(col,char_y-1);
         display_write_byte(DATEN,tmp4);

         
         //display_write_byte(DATEN,(tmp1 & 0xFF00)>> 10);
		}
	}
   
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH);
	}
	return;
}






//##############################################################################################
//Ausgabe eines Zeichenkette
//
//##############################################################################################
void display_write_P (const char *Buffer,...)
{
	va_list ap;
	va_start (ap, Buffer);	
	
	int format_flag;
	char str_buffer[10];
	char str_null_buffer[10];
	char move = 0;
	char Base = 0;
	int tmp = 0;
	char by;
	char *ptr;
		
	//Ausgabe der Zeichen
    for(;;)
    {
       by = pgm_read_byte(Buffer++);
       if(by==0) break; // end of format string
       
       if (by == '%')
       {
          by = pgm_read_byte(Buffer++);
          if (isdigit(by)>0)
          {
             
             str_null_buffer[0] = by;
             str_null_buffer[1] = '\0';
             move = atoi(str_null_buffer);
             by = pgm_read_byte(Buffer++);
          }
          
          switch (by)
          {
             case 's':
                ptr = va_arg(ap,char *);
                while(*ptr) { display_write_propchar(*ptr++,1); }
                break;
             case 'b':
                Base = 2;
                goto ConversionLoop;
             case 'c':
                //Int to char
                format_flag = va_arg(ap,int);
                display_write_propchar (format_flag++,1);
                break;
             case 'i':
                Base = 10;
                goto ConversionLoop;
             case 'o':
                Base = 8;
                goto ConversionLoop;
             case 'x':
                Base = 16;
                //****************************
             ConversionLoop:
                //****************************
                itoa(va_arg(ap,int),str_buffer,Base);
                int b=0;
                while (str_buffer[b++] != 0){};
                b--;
                if (b<move)
                {
                   move -=b;
                   for (tmp = 0;tmp<move;tmp++)
                   {
                      str_null_buffer[tmp] = '0';
                   }
                   //tmp ++;
                   str_null_buffer[tmp] = '\0';
                   strcat(str_null_buffer,str_buffer);
                   strcpy(str_buffer,str_null_buffer);
                }
                display_write_str (str_buffer,1);
                move =0;
                break;
          }
          
       }	
       else
       {
          display_write_propchar ( by ,1);
       }
    }
	va_end(ap);
}

//##############################################################################################
//Ausgabe eines Strings
//
//##############################################################################################
void display_write_str(char *str, uint8_t prop)
{
	while (*str)
	{
		display_write_propchar(*str++,prop);
	}
}

void display_write_inv_str(char *str,uint8_t prop)
{
   display_go_to(char_y,char_x);
   display_write_byte(DATEN,0xFF);
   
   //char_x ++;
	while (*str)
	{
		display_write_inv_propchar(*str++,prop);
	}
}

//##############################################################################################
//Ausgabe einer Zahl
//
//##############################################################################################
void display_write_int(uint8_t zahl, uint8_t prop)
{
   char zahlbuffer[10];
   itoa(zahl,zahlbuffer,10);
   
	//while (*zahlbuffer)
	{
		display_write_str(zahlbuffer,prop);
      
	}
}

//##############################################################################################
//Ausgabe einer Dezimal-Zahl mit Nachkommastellen
//
//##############################################################################################

void display_write_dez(uint16_t zahl, uint8_t stellen, uint8_t prop)
{
   // zahl ist folge von ziffern ohne Punkt.
   #define ANZAHLELEMENTE 6
   char string[ANZAHLELEMENTE]={};
   int8_t i;                             // schleifenzähler
   int8_t flag=0;
   string[ANZAHLELEMENTE-1]='\0';                       // String Terminator
   for(i=ANZAHLELEMENTE-2; i>=0; i--)
   {
      if (i==ANZAHLELEMENTE-stellen-2)
      {
         string[i] = '.';
      }
      else
      {
         string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
         zahl /= 10;
      }
   }
   
   char c;
   i=0;
   while ( (c = string[i]) )
   {
      if (c>'0')
      {
         flag=1;
      }
      if (flag )
      {
  
         display_write_propchar(c,prop);
      }
      i++;
   }
}


//##############################################################################################
//Ausgabe Spannung
//
//##############################################################################################
void display_write_spannung(uint16_t rawspannung, uint8_t prop) // eine Dezimale
{
   uint16_t tempspannung = rawspannung;
   display_write_dez(tempspannung,1,prop);
   display_write_propchar('V',prop);
   
   /*
   uint8_t dezimale = temp%10;
   uint8_t wert = temp/10;
   char tempbuffer[6]={};
  
   {
     // tempbuffer[0] =' ';
   }
   tempbuffer[0] =wert%10+'0';
   tempbuffer[1] ='.';
   
   tempbuffer[2] =dezimale+'0';
   tempbuffer[3] ='V';
   tempbuffer[4] = '\0';
   display_write_str(tempbuffer,1);
*/
   
}


//##############################################################################################
//Ausgabe Minute:Sekunde
//
//##############################################################################################
void display_write_min_sek(uint16_t rawsekunde , uint8_t prop)
{
   uint8_t minute = rawsekunde/60%60;
   uint8_t sekunde = rawsekunde%60;
   uint8_t stunde = rawsekunde/3600;
   
   char tempbuffer[6]={};
   if (stunde)
   {
      char stdbuffer[4]={};
      stdbuffer[0] =stunde/10+'0';
      stdbuffer[1] =stunde%10+'0';
      stdbuffer[2] =':';
      stdbuffer[3] = '\0';
      display_write_str(stdbuffer,prop);
   }
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str(tempbuffer,prop);
   
}

void display_write_zeit(uint8_t sekunde,uint8_t minute,uint8_t stunde,uint8_t prop)
{
   
   char tempbuffer[6]={};
   if (stunde)
   {
      char stdbuffer[4]={};
      stdbuffer[0] =stunde/10+'0';
      stdbuffer[1] =stunde%10+'0';
      stdbuffer[2] =':';
      stdbuffer[3] = '\0';
      display_write_str(stdbuffer,prop);
   }
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str(tempbuffer,prop);
   
}

void display_write_stopzeit(uint8_t sekunde,uint8_t minute,uint8_t prop)
{
   
   char tempbuffer[6]={};
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str(tempbuffer,prop);
   
}

void display_write_stopzeit_BM(uint8_t sekunde,uint8_t minute)
{
   
   char tempbuffer[2][64]={};
   unsigned char col,page,tmp1,tmp2;
   col=char_x;
   page = char_y-1;
   display_go_to(col,page);
   
   uint8_t index=0, startposition=0, endposition=10, delta=11, breite=12;
   
   // Digit 1 minute zehner
   //PGM_P pointer = (PGM_P)zahlfont12[minute/10];
   //uint16_t* pointer = (uint16_t*)zahlfont12[minute/10];
   for (index=0;index< breite;index++)
   //for (index = 0;index < 10;index++)
   {
      tmp1 = (char)pgm_read_byte(&(zahlfont12[minute/10][2*index])); // byte col, lo
      tempbuffer[0][startposition+index] = tmp1;
      tmp2 = (char)pgm_read_byte(&(zahlfont12[minute/10][2*index+1])); // byte col, lo
      tempbuffer[1][startposition+index] = tmp2;
      
  }
   
   
   // Digit 2 minute einer
   startposition+= delta;
  // char_x +=delta;
   //pointer = (PGM_P)zahlfont12[minute%10];
   
   for (index=0;index<breite;index++)
   {
    tmp1 = (char)pgm_read_byte(&(zahlfont12[minute%10][2*index])); // byte col, lo
    tempbuffer[0][startposition+index] = tmp1;
    tmp2 = (char)pgm_read_byte(&(zahlfont12[minute%10][2*index+1])); // byte col, lo
    tempbuffer[1][startposition+index] = tmp2;
   }
   
   
   startposition+= delta;
   
   // Doppelpunkt
 
   for (index=0;index<4;index++)
{
   tmp1 = (char)pgm_read_byte(&(zahlfont12[10][2*index])); // byte col, lo
   tempbuffer[0][startposition+index] = tmp1;
   tmp2 = (char)pgm_read_byte(&(zahlfont12[10][2*index+1])); // byte col, lo
   tempbuffer[1][startposition+index] = tmp2;
   }
   
   
   startposition+= 5;
 

   // Digit 3 sekunde zehner
   
   for (index=0;index<breite;index++)
{
   tmp1 = (char)pgm_read_byte(&(zahlfont12[sekunde/10][2*index])); // byte col, lo
   tempbuffer[0][startposition+index] = tmp1;
   tmp2 = (char)pgm_read_byte(&(zahlfont12[sekunde/10][2*index+1])); // byte col, lo
   tempbuffer[1][startposition+index] = tmp2;
   }
   
   startposition+= delta;
   
    // Digit 4 sekunde zehner
   
   for (index=0;index<breite;index++)
   {
   tmp1 = (char)pgm_read_byte(&(zahlfont12[sekunde%10][2*index])); // byte col, lo
   tempbuffer[0][startposition+index] = tmp1;
   tmp2 = (char)pgm_read_byte(&(zahlfont12[sekunde%10][2*index+1])); // byte col, lo
   tempbuffer[1][startposition+index] = tmp2;
   }
   endposition = 52; // Ganzen Block zeichnen
   for (index=0;index<endposition;index++)
   {
      display_write_byte(0,tempbuffer[1][index]);
   }
   
   display_go_to(col,page+1);
   
   for (index=0;index<endposition;index++)
   {
      display_write_byte(0,tempbuffer[0][index]);
   }
   
   /*
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display_write_str(tempbuffer,prop);
   */
   
}

void display_write_stopzeit_BM1(uint8_t sekunde,uint8_t minute)
{
   
   char tempbuffer[2][48]={};
   unsigned char col,page,tmp1,tmp2;
   
   col=char_x;
   page = char_y-1;
   display_go_to(col,page);
   uint16_t index=0;
   
   // Digit 1 minute zehner
  // PGM_P pointer = (PGM_P)zahlfont12[minute/10];
   uint16_t* pointer = (uint16_t*)zahlfont12[minute/10];

   for (index=0;index<10;index++)
   {
      //tmp1 = pgm_read_byte(pointer++); // byte col, lo
     tmp1 = (char)pgm_read_byte(&(zahlfont12[0][2*index])); // byte col, lo
      tempbuffer[0][index] = tmp1;
    //  tmp2 = pgm_read_byte(pointer++); // byte col+1, hi
     tmp2 = (char)pgm_read_byte(&(zahlfont12[0][2*index+1])); // byte col, lo
      tempbuffer[1][index] = tmp2;
   }
   /*
   // Digit 2 minute einer
   pointer = (PGM_P)zahlfont12[minute%10];
   
   for (index=7;index<14;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   char_x++;
   // Doppelpunkt
   pointer = (PGM_P)zahlfont12[11];
   
   for (index=14;index<19;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   char_x++;
   // Digit 3 sekunde zehner
   pointer = (PGM_P)zahlfont12[sekunde/10];
   
   for (index=17;index<24;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   char_x++;
   // Digit 4 sekunde zehner
   pointer = (PGM_P)zahlfont12[sekunde%10];
   
   for (index=24;index<31;index++)
   {
      tmp1 = pgm_read_byte(pointer++); // byte col, lo
      tempbuffer[0][index] = tmp1;
      tmp2 = (pgm_read_byte(pointer++)); // byte col+1, hi
      tempbuffer[1][index] = tmp2;
   }
   */
   
   for (index=0;index<20;index++)
   {
      display_write_byte(0,tempbuffer[1][index]);
   }
   
   display_go_to(col,page+1);
   
   for (index=0;index<10;index++)
   {
      display_write_byte(0,tempbuffer[0][index]);
   }
   
   /*
    tempbuffer[0] =minute/10+'0';
    tempbuffer[1] =minute%10+'0';
    tempbuffer[2] =':';
    tempbuffer[3] =sekunde/10+'0';
    tempbuffer[4] =sekunde%10+'0';
    tempbuffer[5] = '\0';
    display_write_str(tempbuffer,prop);
    */
   
}



//##############################################################################################
// Ausgabe Pfeil rechts an pos x,y (page)
//##############################################################################################
void display_pfeilvollrechts(uint8_t col, uint8_t page)
{
   char_x = col;
   char_y = page;
   display_write_symbol(pfeilvollrechts);
   
}

//##############################################################################################
//Ausgabe eines Symbols
//
//##############################################################################################
void display_write_symbol(char* symbol)
{
	unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
	char* pointer = symbol;
   uint8_t symboldelay = 2;
   uint8_t count = 0;
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = (symbol[count++]);
			
			if (char_height_mul > 1)
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				}
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATEN,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATEN,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}
// **********
void display_write_symbol_T(char* symbol, uint8_t cols, uint8_t rows)
{
   // https://www.mikrocontroller.net/topic/144500?page=single
   unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
   char* pointer = symbol;
   uint8_t symboldelay = 2;
   uint8_t count = 0;
   for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
   {
      for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
      {
         tmp1 = (symbol[count++]);
         
         if (char_height_mul > 1)
         {
            tmp2 = (tmp1&0xf0)>>4;
            tmp1 = tmp1 & 0x0f;
            
            tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
            tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
            
            if (char_height_mul>2)
            {
               tmp3 = tmp2;
               tmp2 = (tmp1&0xf0)>>4;
               tmp1 = tmp1 & 0x0f;
               
               tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
               tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
               
               
               tmp4 = (tmp3&0xf0)>>4;
               tmp3 = tmp3 & 0x0f;
               
               tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
               tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
               
               display_go_to(col,page+1);
               for(counter = 0;counter<char_width_mul;counter++)
               {
                  display_write_byte(0,tmp3);
               }
               
               display_go_to(col,page+2);
               for(counter = 0;counter<char_width_mul;counter++)
               {
                  display_write_byte(0,tmp4);
               }
            }
            
            
            display_go_to(col,page);
            
            for(counter = 0;counter<char_width_mul;counter++)
            {
               display_write_byte(DATEN,tmp2);
            }
         }
         
         display_go_to(col,page-1);
         for(counter = 0;counter<char_width_mul;counter++)
         {
            display_write_byte(DATEN,tmp1);
         }
      }
   }
   
   if (char_x < (128 + DISPLAY_OFFSET))
   {
      char_x = char_x + (FONT_WIDTH*char_width_mul);
   }
   return;
}



// **************

void display_write_propsymbol( char* symbol)
{
	unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
	PGM_P pointer = symbol;
   uint8_t charsize = 8;
   uint8_t count = 0;
   uint8_t  charbreite =   pgm_read_byte(pointer++);
	
	
	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(charbreite*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((charsize/8)*char_height_mul));page = page +char_height_mul)
		{
         tmp1 =  (symbol[count++]);
			
			if (char_height_mul > 1)
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(0,tmp4);
					}
				}
            
            
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(DATEN,tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(DATEN,tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (charbreite*char_width_mul);
	}
	return;
}


/*
 http://www.mikrocontroller.net/attachment/5130/spi.c
 void write_spi (unsigned char DATEN_out){       //msb first
 unsigned char loop, mask;
 for (loop=0,mask=0x80;loop<8;loop++, mask=mask>>1)
 {sclk=0;
 if (DATEN_out & mask) mosi=1;
 else mosi=0;
 sclk=1;
 }
 sclk=0;
 }

 */

// progmem_image - prog_uint8_t array of columns aka the bitmap image
// x             - x start coordinate on the screen (in pixel)
// y             - y start coordinate on the screen (in pixel)
// pages         - height of image in pages
// columns       - width of image in pixels
// style         - Bit2: sets inverse mode
/*
void lcd_draw_image_xy_P(char* progmem_image, uint8_t x, uint8_t y, uint8_t pages, uint8_t columns, uint8_t style) 
{
  uint16_t i,j;
 uint8_t data   = 0;
 uint8_t inv    = style & INVERT_BIT;
  uint8_t offset = y & 0x7; //Optimized modulo 8
  //If there is an offset, we must use an additional page
  if(offset)  
   pages++;
 //If there is not enough vertical space -> cut image
 if(pages > LCD_RAM_PAGES - lcd_get_position_page())   
   pages = LCD_RAM_PAGES - lcd_get_position_page();
 //Goto starting point and draw
  lcd_moveto_xy((y>>3), x);
  for (j=0; j<pages; j++) {
     for (i=0; i<columns && (lcd_get_position_column() < LCD_WIDTH); i++){
     data = 0;
     if (!offset || j+1 != pages)
       data = progmem_image[j*columns + i]) << offset;
        if(j > 0 && offset)
           data |= (progmem_image[(j-1)*columns + i]) >> (8-offset);
        if(inv)   lcd_data(~data);
     else       lcd_data(data);
       }
     if(j+1 != pages)
        lcd_move_xy(1,-columns);
    }
 }

*/
/*
 * File:        dogl.c
 * Project:     Mini Anzeige Modul
 * Author:      Nicolas Meyertˆns
 * Version:     siehe setup.h
 * Web:         www.PIC-Projekte.de

 * Diese Funktion gibt eine Zeichenkette auf dem Bildschirm aus.
 * Mit einer Angabe der Adresse, kann dies an jedem beliebigen Ort auf
 * dem Display stattfinden. Des Weiteren kann zwischen normaler und
 * invertierter Darstellung gew‰hlt werden.
 *
 * page:        Adresse - Zeile (0..7)
 * column:      Adresse - Spalte (0..127)
 * inverse:     Invertierte Darstellung wenn true sonst normal
 * *pChain:     Die Zeichnkette ansich, welche geschrieben werden soll
 */

/*
 * Auswahl einer Adresse. Diese Funktion wird in der Regel nicht vom
 * Anwender selbst aufgerufen, sondern nur durch die Funktionen dieser
 * C-Datei.
 *
 * page:    Adresse - Zeile (0..7)
 * column:  Adresse - Spalte (0..127)
 */
void setAddrDOGL(uint8_t page, uint8_t column)
{
   if( page<8 && column<128 )
   {
      
      display_write_byte(CMD,0xB0 + page);
      display_write_byte(CMD,0x10 + ((column&0xF0)>>4) );
      display_write_byte(CMD,0x00 +  (column&0x0F) );
      
   }
}

void display_write_prop_str(uint8_t page, uint8_t column, uint8_t inverse, const uint8_t *pChain, uint8_t prop)
{
   uint8_t l=0,k;
   //setAddrDOGL(page-1,column);
   // Space schreiben
   /*
   if(inverse)
   {
      display_write_byte(DATEN,~propfont6[0][1]);
      column++;
   }
   else
   {
      display_write_byte(DATEN,propfont6[0][1]);
      column++;
   }
   */
   while(*pChain)
   {
      display_write_propchar(*pChain++,prop);
      /*
      PGM_P pointer = propfont6[*pChain-32];
      char blank =propfont6[0][1];
      
      
      switch (prop)
      {
         case 2:
         {
            pointer = propfont8[*pChain-32];
         }break;
      }
      //PGM_P pointer = propfont6[*pChain-32];
      uint8_t  fontbreite =   pgm_read_byte(pointer++);
      unsigned char tmp1=0;
      for(k=1; k <= fontbreite; k++)
      {
         if( column > 127)
            break;
         
         display_go_to(column,page-1);
         tmp1 = pgm_read_byte(pointer++);
         column++;
         
         if(inverse)
         {
            display_write_byte(DATEN,~tmp1);
         }
         else
         {
            display_write_byte(DATEN,tmp1);
         }
      }
      if( column > 127)
      {
         return ;
      }
      
      // Space schreiben
      if(inverse)
      {
         display_write_byte(DATEN,~0x00);
         column++;
      }
      else
      {
         display_write_byte(DATEN,0x00);
         column++;
      }
      */
      /*
       * Ab dem (126-32). Eintrag in der Font Library folgen die Zeichen
       * des MiniAnzeigeModuls. Diese Eintr‰ge kommen NICHT als String in
       * diese Funktion und haben somit keinen Terminator! Es muss beim
       * Zeichnen eines Zeichens also darauf geachtet werden, dass der
       * Pointer nicht einen Schritt weiter geht sondern direkt die
       * Schleife beendet. Das geschieht hier:
       */
      if( *pChain > 126 )
         break;
      pChain++;
   } // while *char
   
   return ;
}


//##############################################################################################

void r_uitoa8(int8_t zahl, char* string)
{
   uint8_t i;
   
   string[3]='\0';                  // String Terminator
   for(i=3; i>=0; i--)
   {
      string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
}
//##############################################################################################


uint8_t spi_out(uint8_t Datenout)
{
   uint8_t spidelay = 1; // xxx
   cli();
  // OSZI_B_LO;
   digitalWriteFast(DOG_CS,0); ; // Chip enable
   _delay_us(spidelay);
   uint8_t DATENin=0xFF;
   uint8_t pos=0;
   //SCL_LO; // SCL LO
   digitalWriteFast(DOG_SCL,0); 
   uint8_t tempDATEN=Datenout;
   _delay_us(spidelay);
   for (pos=8;pos>0;pos--)
   {
      if (tempDATEN & 0x80)
      {
         //DATEN_HI;
         //SOFT_SPI_PORT |= (1<<DOG_DATEN);
         digitalWriteFast(DOG_DATA,1); 
      }
      else
      {
         //DATEN_LO;
         //SOFT_SPI_PORT &= ~(1<<DOG_DATEN);
         digitalWriteFast(DOG_DATA,0); 
      }
      _delay_us(spidelay);
      tempDATEN<<= 1;
      _delay_us(spidelay);
      //SCL_HI;
      //SOFT_SPI_PORT |= (1<<DOG_SCL);
      digitalWriteFast(DOG_SCL,1); 
      _delay_us(spidelay);
      //SCL_LO;
      //SOFT_SPI_PORT &= ~(1<<DOG_SCL);
      digitalWriteFast(DOG_SCL,0);
      _delay_us(spidelay);
   }
 //  OSZI_B_HI;
   
   //CS_HI;// Chip disable
   digitalWriteFast(DOG_CS,1);
   _delay_us(spidelay);
   
   return DATENin;
}

