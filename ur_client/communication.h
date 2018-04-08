/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   communication.h
 * Author: minto
 *
 * Created on 5 de marzo de 2018, 12:48
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "print_out.h"
#include "data_handler.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <fcntl.h>
#include <sys/types.h>

class Communication {
public:
    Communication(/*std::condition_variable& msg_cond,*/ std::string host);
    bool connected;
    DataHandler* robot_state;
    bool start();
    void halt();
    std::string receive(int);
private:
    int pri_sockfd, sec_sockfd; // port number
    struct sockaddr_in pri_serv_addr, sec_serv_addr; // sockets
    struct hostent *server; //  server IP
    bool keepalive;
    std::thread comThread;
    int flag;
    void run();
};

#endif /* COMMUNICATION_H */

