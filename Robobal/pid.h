//
// File			pid.h
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
// Date			27/09/2014 17:01
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

#ifndef pid_h

#define pid_h


class pid {
  
public:
    pid(int kp, int ki, int kd);
    void majConsigne(double newConsigne);
    void majPid(double mesure, double dt);
    void majP(int newP);
    void majI(int newI);
    void majD(int newD);
    double getCorrection();
    int getP();
    int getI();
    int getD();
    double contraindre(double valeur, double max, double min);
  
private:
    int KI;
    int KP;
    int KD;
    double erreurPrecedente;
    double sommeErreurs;
    double consigne;
    double termeP;
    double termeI;
    double termeD;
    double correction;
};

#endif
