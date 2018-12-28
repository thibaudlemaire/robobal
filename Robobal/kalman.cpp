//
// Kalman.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		robot
//
// Created by 	Thibaud Lemaire, 07/05/2014 12:06
// 				Thibaud LEMAIRE
//
// Copyright 	© Thibaud Lemaire, 2014
// License     license
//
// See 			kalman.h and ReadMe.txt for references
//


// Library header
#include "kalman.h"

/// Constructeur du filtre
kalman::kalman()
{
    // Valeurs par défaut des variances
    Qangle = 0.001;
    Qbiais = 0.003;
    Rmesure = 0.03;
    
    // Initialisation du modèle
    angle = 0;
    biais = 0;
    
    // En supposant l'état initial connu, la matrice de covariance de l'erreur est nulle
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;

}

/// Mise à jour du modèle
void kalman::acq(double angleMesure, double omegaMesure, double dt) {
    // Déclaration des variables
    double S; // Estimation de l'erreur
    double K[2]; // Gains de Kalman
    double y; // Innovation
    
    // Estimation de l'angle par intégration
    omega = omegaMesure - biais;
    angle += dt * omega;
    
    // Mise à jour de la covariance de l'erreur
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Qangle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Qbiais * dt;
    
    // Estimation de l'erreur
    S = P[0][0] + Rmesure;
    
    // Calcul des gains de Kalman
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    // Calcul de l'angle après filtrage et du biais
    y = angleMesure - angle; // Innovation
    angle += K[0] * y;
    biais += K[1] * y;
    
    // Mise à jour de la matrice de covariance de l'erreur
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
    
}

// Fonctions de réglage du filtre :
double kalman::getAngle()
{
    return angle;
}

void kalman::setQangle(double newQangle)
{
    Qangle = newQangle;
}

void kalman::setQbiais(double newQbiais)
{
    Qbiais = newQbiais;
}

void kalman::setRmesure(double newRmesure)
{
    Rmesure = newRmesure;
}

double kalman::getQangle()
{
    return Qangle;
}

double kalman::getQbiais()
{
    return Qbiais;
}

double kalman::getRmesure()
{
    return Rmesure;
}
