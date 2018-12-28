// 
// robot 
//
// Robot balancier
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Thibaud Lemaire
// 				Thibaud LEMAIRE
//
// Date			07/05/2014 01:43
// Version		<#version#>
// 
// Copyright	© Thibaud Lemaire, 2014
// License		<#license#>
//
// See			ReadMe.txt for references
//

// Core library for code-sense
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"   
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif

#include "kalman.h"
#include "moteurs.h"
#include "capteur.h"
#include "communication.h"
#include "LiquidCrystal.h"
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include "pid.h"

// Déclarations :
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
capteur capt;
kalman kalman;
communication com;
SoftwareSerial bt(14, 15);
moteurs moteurDroit(9, 13, 6);
moteurs moteurGauche(10, 8, 7);
pid pid(1,0,0);


bool newDatas;      // Drapeau de demande de mesure
bool moteurOn;
int compteur = 0;   // Compteur pour la fréquence d'échantillonage
int curseur = 0; // Curseur de réglage des gains
float AZ, AY, RX, angle;    // Mesures inertielles

// Fonction appelée sur interruption toute les 100us
void cligno()
{
    // Mise en place d'un compteur pour echantilloner toutes les 10ms
    if(compteur >= 100)
    {
        newDatas = true;
        compteur = 0;
    }
    else
        compteur++;
}

/// Fonction d'initialisation
void setup()
{
    newDatas = false;
    
    
    // Initialisation
    Serial.begin(9600);         // Liaison série physique
    bt.begin(38400);            // Liaison série du Bluetooth
    Timer1.initialize(100);     // Module PWM et interruptions
    capt.init();                // Initialisation des capteurs inertiels
    moteurGauche.init();        // Moteurs
    moteurDroit.init();
    lcd.begin(16, 2);           // Afficheur LCD
    
    // Attente de la disponibilité de l'afficheur
    delay(500);
 
    // Mise en place des légendes
    lcd.print("A");
  
    // Début de l'échantillonage
    Timer1.attachInterrupt(cligno);

}


/// Boucle principale
void loop()
{
    // Télécommande des moteurs via Bluetooth
    if(bt.available())
    {
        int car = bt.read();
        if(car == 115) // s
        {
            moteurOn = false;
        }
        else if ( car == 97) // a
        {
            moteurOn = true;
        }
        else if ( car == 112) // p
        {
            curseur = 0;
        }
        else if ( car == 105) // i
        {
            curseur = 1;
        }
        else if ( car == 100) // d
        {
            curseur = 2;
        }
        else if ( car == 43) // +
        {
            if (curseur == 0)
                pid.majP(pid.getP() + 1);
            else if (curseur == 1)
                pid.majI(pid.getI() + 1);
            else if (curseur == 2)
                pid.majD(pid.getD() + 1);
        }
        else if ( car == 45) // -
        {
            if (curseur == 0)
                pid.majP(pid.getP() - 1);
            else if (curseur == 1)
                pid.majI(pid.getI() - 1);
            else if (curseur == 2)
                pid.majD(pid.getD() - 1);
        }
        lcd.setCursor(0,1);
        lcd.print("P");
        lcd.print(pid.getP());
        lcd.print(" I");
        lcd.print(pid.getI());
        lcd.print(" D");
        lcd.print(pid.getD());
        lcd.print("     ");
    }
    
    // Acquisition et traitement des données
    if(newDatas)
    {
        // Acquisitions bruttes
        AZ=float(capt.getAccelX());
        AY=-float(capt.getAccelZ());
        RX=-float(capt.getGyroY())/131;
        
        // Traitement par Kalman
        angle = 360*atan(AZ/AY)/6.28;
        kalman.acq(angle, RX, 0.01);
    
        // Affichage des résultats
        lcd.setCursor(1, 0);
        lcd.print(kalman.getAngle());
        lcd.print("     ");
        
        // MAJ du PID
        pid.majPid(kalman.getAngle(), 0.01);
        lcd.setCursor(7, 0);
        lcd.print("C");
        lcd.print(pid.getCorrection());
        lcd.print("     ");
        double correction = pid.getCorrection();
        
        if(moteurOn == true)
        {
            if(correction >= 0)
            {
                moteurGauche.changerSens(true);
                moteurDroit.changerSens(true);
                moteurGauche.changerVitesse(correction);
                moteurDroit.changerVitesse(correction);
            }
            else
            {
                moteurGauche.changerSens(false);
                moteurDroit.changerSens(false);
                moteurGauche.changerVitesse(-correction);
                moteurDroit.changerVitesse(-correction);
            }
        }
        else
        {
            moteurGauche.stop();
            moteurDroit.stop();
        }
        
        // RAZ du drapeau
        newDatas = false;
    }
}
