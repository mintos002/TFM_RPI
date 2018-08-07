/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RtCommunication.h
 * Author: minto
 *
 * Created on 22 de abril de 2018, 19:52
 */

#ifndef RT_COMMUNICATION_H
#define RT_COMMUNICATION_H

#include "print_out.h"
#include "rt_data_handler.h"
#include "led_handler.h"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <condition_variable> 
#include <thread>
#include <mutex>

class RtCommunication {
public:
    RtCommunication(std::condition_variable& msg_cond, std::string host, unsigned int safety_count_max = 12);
    RtDataHandler *rt_data_handler;
    bool connected;
    bool start();
    void halt();
    bool setSpeed(double q0, double q1, double q2, double q3, double q4, double q5, double acc = 100.);
    bool movej(double j1, double j2, double j3, double j4, double j5, double j6, double acc = 1.0, double v = 1.2/*, double t = 4*/);
    void movejp(double x, double y, double z, double rx, double ry, double rz, double acc = 0.6, double v = 1.0);

    bool set_digital_out(int op, bool on);
    bool addCommandToQueue(std::string inp);
    void setSafetyCountMax(uint inp);
    std::string getLocalIp();
    LedHandler ledH;
    
private:
    unsigned int safety_count_max_;
    unsigned int safety_count;
    int sockfd;
    int flag;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    bool keepalive;
    void run();

    std::string local_ip;
    std::thread comThread;
    std::recursive_mutex command_string_lock;
    std::string command;

};

#endif /* RT_COMMUNICATION_H */

