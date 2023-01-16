//
//  potgrenzen.h
//  Stepper32
//
//  Created by Ruedi Heimlicher on 07.05.2020.
//  Copyright © 2020 Ruedi Heimlicher. All rights reserved.
//

#ifndef potgrenzen
#define potgrenzen

#define pot0_PIN   14
#define pot1_PIN   15
#define pot2_PIN   16
#define pot3_PIN   17
#define pot4_PIN   18
#define pot5_PIN   19

//#define POT0LO 920
//#define POT0HI 3230

#define POT0LO 620  // Min wert vom ADC Pot 0
#define POT0HI 3400 // Max wert vom ADC Pot 0


#define POT1LO 620  // Min wert vom ADC Pot 1
#define POT1HI 3480 // Max wert vom ADC Pot 1



#define POTLO   1300
#define POTHI  2900


//Impulslaenge, ms
#define PPMLO  850  // Minwert ms fuer Impulslaenge
#define PPMHI  2150 // Maxwert ms fur Impulslaenge

#define NULLBAND  200


#endif /* settings_h */
