/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   led_handler.h
 * Author: minto
 *
 * Created on 6 de junio de 2018, 19:10
 */

#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#define G 0
#define Y 2
#define R 3

#include <wiringPi.h>
#include <thread>

class LedHandler {
public:
    LedHandler();
    void startLed(int led);
    void startLeds();
    void stopLeds();
private:

};

#endif /* LED_HANDLER_H */

