//
//  settings.h
//  Stepper32
//
//  Created by Ruedi Heimlicher on 07.05.2020.
//  Copyright © 2020 Ruedi Heimlicher. All rights reserved.
//

#ifndef settings_h
#define settings_h

#define LOOPLED 13

#define BLINK_PIN 24


#define TIMER0_STARTWERT   0x40

// Stepper A

#define BEEP_PIN 25

#define IMPULS_PIN         8
#define IMPULS_ENABLE_PIN  9

#define AKKU_PIN        23
#define AKKU_OFF_PIN   34


#define TIMER_ON    1 // Bit fuer timerfunktion start

#define pot0_PIN   14
#define pot1_PIN   15
#define pot2_PIN   16
#define pot3_PIN   17
#define pot4_PIN   18
#define pot5_PIN   19




// neu 3.5:

// bits von programmstatus

#define MOTOR_ON        1
#define STOP_ON         2
#define LOCALTASK       3
#define EXTERNTASK      4


// Bits von masterstatus
#define  USBPRESENT_BIT             4 //  USB eingesteckt
#define  AKKU_LOW_BIT               5 //  Akku ist low
#define  TIMEOUT_BIT                6 //  Beep nach ablauf des Timeout
#define  HALT_BIT                   7 //  Bit 7

#define TIMEOUT   12000             // 2800 ticks fuer 1 minute

// Bits von eepromstatus



// Bits von displaystatus
#define UHR_UPDATE         0
#define BATTERIE_UPDATE    1

#define USB_DATENBREITE 64
#define EE_PARTBREITE 32


// Tastatur

#define TASTATUR_PIN           35
#define TRIMMTASTATUR_PIN      36


#define MANUELLTIMEOUT        32 // Loopled-counts bis Manuell zurueckgesetzt wird. 50: ca. 30s

#define ADCTIMEOUT            1

//#define MITTE_TASK         0x01 // Mitte lesen
//#define KANAL_TASK         0x02 // Level und Expo lesen
//#define MIX_TASK           0x03 // Mix lesen

#define LOOPDELAY          5

#define LEDON              2   // Bit for Led-Background3


// USB Konstanten

#define USB_DATA_OFFSET 4
// EEPROM Speicherorte

#define TASK_OFFSET        0 // war 0x2000, Ort fuer Einstellungen

#define SETTINGBREITE      4 // 0x100; // 256 Bytes, Breite des Settingblocks fuer ein model

#define  ANZAHLMODELLE        5
#define  KANALSETTINGBREITE   4
#define  MODELSETTINGBREITE   32 // nur Kanalsettings. Anschliessend MixingSettings

#define  EEPROM_MODELSETTINGBREITE 64 //Kanalsettings und MixingSettings

#define  STATUS_OFFSET     0 
#define  LEVEL_OFFSET      1 //0x20 // 32
#define  EXPO_OFFSET       2 //0x30 // 48
#define  FUNKTION_OFFSET   3 // 96


#define DEVICE_OFFSET      0x70 // 122
#define AUSGANG_OFFSET     0x80 // 128

#define SAVE_STATUS      0
#define SAVE_LEVEL      1
#define SAVE_MIX        2
#define SAVE_EXPO       3
#define SAVE_FUNKTION   4
#define SAVE_DEVICE     5
#define SAVE_AUSGANG    6


#define LCD_CLK         30
#define LCD_DS          31
#define LCD_DS          32


#define OSZI_PULS_A        26
#define OSZI_PULS_B        27
#define OSZI_PULS_C        28
#define OSZI_PULS_D        29


#define EEPROM_WRITE_BYTE_TASK     1
#define EEPROM_WRITE_PAGE_TASK     2
#define EEPROM_READ_BYTE_TASK      3
#define EEPROM_READ_PAGE_TASK      4
#define EEPROM_AUSGABE_TASK        5

#define EEPROM_WRITE_START_OK    0xB0


#define THREAD_COUNT_BIT   0


#define RUN          0
#define PAUSE        1
#define PAKET        2
#define IMPULS       3
#define ADC_OK       4
#define USB_OK       5
#define ENDEPAKET    7

// USB
#define DATAOFFSET   8
#define ADCOFFSET   48
// Ramp
#define RAMP_OK      1 // Ramp einschalten
#define RAMPFAKTOR   2 // Verlaengerung der delayzeit
#define RAMPZEIT     800 // Mindestdauer fuer Ramp

#define RAMPSTARTBIT    1
#define RAMPENDBIT      2
#define RAMPEND0BIT     3 // Beginn der Endrampe
#define RAMPOKBIT       7
#define RAMPSCHRITT     10

// new
#define STARTIMPULSDAUER         100
#define ENDIMPULSDAUER           20
#define TASTENENDIMPULSDAUER     20

#define RAMPSCHRITT        4
# define RAMPDELAY         80 // delay fuerr Reduktion Impulsdauer
// revision
#define TASTE1     67
#define TASTE2     109
#define TASTE3     163
#define TASTE4     253
#define TASTE5     360
#define TASTE6     484
#define TASTE7     628
#define TASTE8     742
#define TASTE9     827
#define TASTEL     899
#define TASTE0     946
#define TASTER     993

// Tastatur2
/*
// Werte von Teensy2
#define WERT1    19    // 1 oben  Taste 2
#define WERT2    30    //  A links oben Taste  1
#define WERT3    49    // 2 links  Taste 4
#define WERT4    68    // 3 unten  Taste 8
#define WERT6    110   // 4 rechts  Taste 6
#define WERT9    215   // 5 Mitte  Taste 5
#define WERT5    88       //    B links unten Taste 7
#define WERT7    139      //   C rechts oben Taste 3
#define WERT8    168      // D rechts unten Taste 9
*/

//Tastatur2 Teensy3.5
#define    WERT1   76   //    1   oben      Taste   2
#define    WERT2   124   //    A    links oben     Taste   1
#define    WERT3   200   //    2   links      Taste   4
#define    WERT4   276   //    3   unten      Taste   8
#define    WERT6   442   //    4   rechts      Taste   6
#define    WERT9   861   //    5   Mitte      Taste   5
#define    WERT5   354   //    B    links unten     Taste   7
#define    WERT7   557   //    C    rechts oben     Taste   3
#define    WERT8   672   //    D    rechts unten     Taste   9
// revision end


//Trimmtastatur Teensy 3.5
/*
0   975   1012
1   937   956
2   857   897
3   760   808
4   475   617
5   330   402
6   158   244
7   93   125
 */
#define    TRIMMWERT0   125  //    1   oben      Taste   2
#define    TRIMMWERT1   244   //    2   links      Taste   4
#define    TRIMMWERT2   402   //    3   unten      Taste   8
#define    TRIMMWERT3   617   //    4   rechts      Taste   6
#define    TRIMMWERT4   808   //    5   Mitte      Taste   5
#define    TRIMMWERT5   897   //    B    links unten     Taste   7
#define    TRIMMWERT6   956  //    C    rechts oben     Taste   3
#define    TRIMMWERT7   1012   //    D    rechts unten     Taste   9


#define VORZEICHEN_X   0
#define VORZEICHEN_Y   1



#endif /* settings_h */
