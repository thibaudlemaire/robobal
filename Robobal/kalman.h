//
// File			Kalman.h
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
// Date			07/05/2014 12:06
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

#ifndef kalman_h
#define kalman_h

/// Filtre de Kalman pour déterminer l'inclinaison du robot 
class kalman {
public:
    kalman(); // Constructeur
    void acq(double angleMesure, double omegaMesure, double dt); // Mise à jour du modèle
    
    // Lecture de l'état du modèle :
    double getAngle();
    
    // Réglages du filtre :
    void setQangle(double newQangle);
    void setQbiais(double newQbiais);
    void setRmesure(double newRmesure);
    double getQangle();
    double getQbiais();
    double getRmesure();
    
private:
    // Paramètres du filtre :
    double Qangle; // Variance du bruit de process pour les accéléromètres
    double Qbiais; // Variance du bruit de process pour le biais du gyromètre
    double Rmesure; // Variance du bruit de mesure
    
    // Vecteur d'état :
    double angle; // Angle filtré
    double biais; // Biais du gyromètre
    
    // Variables récursives :
    double omega; // Taux de giration corrigé
    double P[2][2]; // Matrice de covariance de l'erreur
};

#endif
