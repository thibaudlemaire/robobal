//
// capteur.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		robot
//
// Created by 	Thibaud Lemaire, 27/05/2014 11:07
// 				Thibaud LEMAIRE
//
// Copyright 	© Thibaud Lemaire, 2014
// License     <#license#>
//
// See 			capteur.h and ReadMe.txt for references
//


// Library header
#include "capteur.h"
#include "Wire.h"

// Code
capteur::capteur()
{
    Wire.begin();     // Ouverture de l'I2C en maître
}

void capteur::init()
{
    Wire.beginTransmission(25);
    Wire.write(byte(0x20));         // Adresse
    Wire.write(byte(0b01100111));   // 200Hz, Normal Power Mode, XYZ activé
    Wire.endTransmission();
    
    Wire.beginTransmission(25);
    Wire.write(byte(0x23));         // Adresse
    Wire.write(byte(0b00001000));   // Haute résolution (12 bits)
    Wire.endTransmission();
    
    Wire.beginTransmission(107);
    Wire.write(byte(0x20));         // Adresse
    Wire.write(byte(0b01001111));   // Frequence 190 Hz, Gyro ON
    Wire.endTransmission();
    
    Wire.beginTransmission(107);
    Wire.write(byte(0x23));         // Adresse
    Wire.write(byte(0b00000000));   // LSB@lower, pleine echelle 250dps
    Wire.endTransmission();
    
    delay(70);                   // Attente de prise en compte, mini 65ms
}

void capteur::tare()
{
    
}

int capteur::getAccelX()
{
    int reading;
    
    Wire.beginTransmission(25);
    Wire.write(byte(0xAD));      // Accel Y
    Wire.endTransmission();      // stop transmitting
    
    // step 4: request reading from sensor
    Wire.requestFrom(25, 2);    // request 2 bytes from slave
    
    if(2 <= Wire.available())    // if two bytes were received
    {
        reading = Wire.read();  // receive high byte (overwrites previous reading)
        reading = reading << 8;    // shift high byte to be high 8 bits
        reading |= Wire.read(); // receive low byte as lower 8 bits

    }
    return reading;
}

int capteur::getAccelZ()
{
    int reading;
    
    Wire.beginTransmission(25);
    Wire.write(byte(0xA9));      // Accel Y
    Wire.endTransmission();      // stop transmitting
    
    // step 4: request reading from sensor
    Wire.requestFrom(25, 2);    // request 2 bytes from slave
    
    if(2 <= Wire.available())    // if two bytes were received
    {
        reading = Wire.read();  // receive high byte (overwrites previous reading)
        reading = reading << 8;    // shift high byte to be high 8 bits
        reading |= Wire.read(); // receive low byte as lower 8 bits
        
    }
    return reading;

}

int capteur::getGyroY()
{
    int reading;
    Wire.beginTransmission(107);
    Wire.write(byte(0xAB));      // giro X
    Wire.endTransmission();      // stop transmitting
    
    // step 4: request reading from sensor
    Wire.requestFrom(107, 2);    // request 2 bytes from slave device #112
    
    // step 5: receive reading from sensor
    if(2 <= Wire.available())    // if two bytes were received
    {
        reading = Wire.read();  // receive high byte (overwrites previous reading)
        reading = reading << 8;    // shift high byte to be high 8 bits
        reading |= Wire.read(); // receive low byte as lower 8 bits
    }
    return reading;

}


