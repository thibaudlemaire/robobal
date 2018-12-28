//
// moteurs.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		robot
//
// Created by 	Thibaud Lemaire, 25/05/2014 21:48
// 				Thibaud LEMAIRE
//
// Copyright 	© Thibaud Lemaire, 2014
// License     <#license#>
//
// See 			moteurs.h and ReadMe.txt for references
//


// Library header
#include "moteurs.h"
#include <TimerOne.h>

// Constructeur des controles moteur
moteurs::moteurs(int pattePwm, int patteAvancer, int patteReculer)
{
    pinPwm = pattePwm;
    pinAvancer = patteAvancer;
    pinReculer = patteReculer;
    Vmax = VmaxDefaut;
}

// Initialisation des ES
void moteurs::init()
{
    pinMode(pinPwm, OUTPUT);
    pinMode(pinAvancer, OUTPUT);
    pinMode(pinReculer, OUTPUT);
}

// Fonction avancer
void moteurs::avancer(int vitesse)
{
    // Limitation de la vitesse
    if(vitesse > Vmax)
    {
        vitesse = Vmax;
    }
    
    // Passage en marche avant
    digitalWrite(pinAvancer, HIGH);
    digitalWrite(pinReculer, LOW);
    
    // Modification de la vitesse
    Timer1.pwm(pinPwm, vitesse);
}

// Fonction reculer
void moteurs::reculer(int vitesse)
{
    // Limitation de la vitesse
    if(vitesse > Vmax)
    {
        vitesse = Vmax;
    }
    
    // Passage en marche arrière
    digitalWrite(pinAvancer, LOW);
    digitalWrite(pinReculer, HIGH);
    
    // Modification de la vitesse
    Timer1.pwm(pinPwm, vitesse);
}

// Stoppe le moteurs
void moteurs::stop()
{
    Timer1.pwm(pinPwm, 0);
    digitalWrite(pinAvancer, HIGH);
    digitalWrite(pinReculer, HIGH);
}
// Place le moteur en roue libre
void moteurs::roueLibre()
{
    Timer1.pwm(pinPwm, 0);
    digitalWrite(pinAvancer, LOW);
    digitalWrite(pinReculer, LOW);
}

// Inverse le sens de rotation du moteur
void moteurs::inverserSens()
{
    if ((digitalRead(pinAvancer) == 1) && (digitalRead(pinReculer) == 0))
    {
        digitalWrite(pinAvancer, LOW);
        digitalWrite(pinReculer, HIGH);
    }
    else if ((digitalRead(pinAvancer) == 0) && (digitalRead(pinReculer) == 1))
    {
        digitalWrite(pinAvancer, HIGH);
        digitalWrite(pinReculer, LOW);
    }
}

// Modifie uniquement le sens de rotation du moteur
void moteurs::changerSens(bool avancer)
{
    digitalWrite(pinAvancer, avancer);
    digitalWrite(pinReculer, !avancer);
}

// Modifie uniquement la vitesse du moteur
void moteurs::changerVitesse(int vitesse)
{
    // Limitation de la vitesse
    if(vitesse > Vmax)
    {
        vitesse = Vmax;
    }
    Timer1.pwm(pinPwm, vitesse);
}

// Changement de la limite de vitesse
void moteurs::setVmax(int newVmax)
{
    Vmax = newVmax;
}
