/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: minto
 *
 * Created on 5 de marzo de 2018, 12:33
 */

#include <cstdlib>
#include <stdio.h>
#include "communication.h" // has printing also
//#include "print_out.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
//    Communication com("192.168.1.128");
    print_debug("START MAIN");
    bool comack;
    Communication robot("158.42.206.10");
//    Communication robot("192.168.238.142");
    comack = robot.start();
    
//    robot.run();
    print_debug("FIN MAIN");
    
    return 0;
}

