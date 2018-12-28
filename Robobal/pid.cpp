//
// pid.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		robot
//
// Created by 	Thibaud Lemaire, 27/09/2014 17:01
// 				Thibaud LEMAIRE
//
// Copyright 	© Thibaud Lemaire, 2014
// License     <#license#>
//
// See 			pid.h and ReadMe.txt for references
//


// Library header
#include "pid.h"

// Code

// Constructeur
pid::pid(int kp, int ki, int kd)
{
    KP=kp;
    KI=ki;
    KD=kd;
    erreurPrecedente = 0;
    sommeErreurs = 0;
    consigne = 0;
    termeP = 0;
    termeI = 0;
    termeD = 0;
    correction = 0;
}

// Fonction majConsigne, change la consigne d'inclinaison
void pid::majConsigne(double newConsigne)
{
    consigne = newConsigne;
}

// Fonction majPid, met à jour la correction PID
void pid::majPid(double mesure, double dt)
{
    double erreur = (consigne - mesure);
    double variationErreur = int((erreur - erreurPrecedente)*100); // Partie entière
    sommeErreurs += erreur * dt;
    termeP = KP * erreur;
    termeI = KI * contraindre(sommeErreurs, 10, -10);
    termeD = KD * variationErreur / dt /100;    // /100 rétabli l'echelle
    correction = termeP + termeI + termeD;
    erreurPrecedente = erreur;
}

// Fonction majP, modifie le gain proportionnel
void pid::majP(int newP)
{
    KP = newP;
}

// Fonction majI, modifie le gain intégral
void pid::majI(int newI)
{
    KI = newI;
}

// Fonction majD, modifie le gain dérivé
void pid::majD(int newD)
{
    KD = newD;
}

// Fonction getCorrection, renvoie le gain du correcteur
double pid::getCorrection()
{
    return contraindre(correction, 1023, -1023); // Rapports cyclique extrèmes
}

int pid::getP()
{
    return KP;
}

int pid::getI()
{
    return KI;
}

int pid::getD()
{
    return KD;
}

// Fonction contraindre
double pid::contraindre(double valeur, double max, double min)
{
    if(valeur > max)
        return max;
    else if (valeur < min)
        return min;
    else
        return valeur;
}
