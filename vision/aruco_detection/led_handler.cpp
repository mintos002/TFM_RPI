/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   led_handler.cpp
 * Author: minto
 * 
 * Created on 6 de junio de 2018, 19:10
 */

#include "led_handler.h"

LedHandler::LedHandler() {
    wiringPiSetup();
    pinMode(G, OUTPUT);
    pinMode(Y, OUTPUT);
    pinMode(R, OUTPUT);
    digitalWrite(G, 0);
    digitalWrite(Y, 0);
    digitalWrite(R, 0);
}

void LedHandler::startLed(int led) {
    switch (led) {
        case 0:
            digitalWrite(G, HIGH);
            digitalWrite(Y, LOW);
            digitalWrite(R, LOW);
            break;
        case 1:
            digitalWrite(G, LOW);
            digitalWrite(Y, HIGH);
            digitalWrite(R, LOW);
            break;
        case 2:
            digitalWrite(G, LOW);
            digitalWrite(Y, LOW);
            digitalWrite(R, HIGH);
            break;
    }
}

void LedHandler::startLeds() {
    digitalWrite(G, HIGH);
    digitalWrite(Y, HIGH);
    digitalWrite(R, HIGH);
}

void LedHandler::stopLeds() {
    digitalWrite(G, LOW);
    digitalWrite(Y, LOW);
    digitalWrite(R, LOW);
}