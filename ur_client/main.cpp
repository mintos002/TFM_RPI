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
#include <iostream>
#include <condition_variable>
#include "communication.h"
#include "ur_driver.h" // has printing also
//#include "print_out.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
//    Communication com("192.168.1.128");
    print_debug("START MAIN");
    condition_variable msg_cond;
    condition_variable rt_msg_cond;
    UrRealtimeCommunication* com = new UrRealtimeCommunication(rt_msg_cond, "158.42.206.10");
    com->start();
    int w=0;
    while(w<500){
        w++;
        usleep(1000000);
    };
//    robot.run();
    print_debug("FIN MAIN");
    
    return 0;
}

