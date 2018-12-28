//
// File			moteurs.h
// Class library header
//
// Details		<#details#>
//
// Project	 	robot
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author		Thibaud Lemaire
// 				Thibaud LEMAIRE
//
// Date			25/05/2014 21:48
// Version		<#version#>
//
// Copyright	© Thibaud Lemaire, 2014
// License	    <#license#>
//
// See			ReadMe.txt for references
//


// Core library - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430 G2 and F5529, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

#ifndef moteurs_h

#define moteurs_h

// Paramètres des moteurs
#define VmaxDefaut 800

class moteurs {
  
public:
    moteurs(int pattePwm, int patteAvancer, int patteReculer);
    void init();
    void setVmax(int newVmax);
    void avancer(int vitesse);
    void reculer(int vitesse);
    void stop();
    void roueLibre();
    void inverserSens();
    void changerSens(bool avancer);
    void changerVitesse(int vitesse);
  
private:
    int pinPwm;
    int pinAvancer;
    int pinReculer;
    int Vmax;
};

#endif
