/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rt_communication.cpp
 * Author: minto
 * 
 * Created on 22 de abril de 2018, 19:52
 */

#include <opencv2/core/affine.hpp>

#include "rt_communication.h"

RtCommunication::RtCommunication(std::condition_variable& msg_cond, std::string host, unsigned int safety_count_max) {
    rt_data_handler = new RtDataHandler(msg_cond);
    bzero((char *) &serv_addr, sizeof (serv_addr));
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        print_fatal("ERROR opening socket");
    }
    server = gethostbyname(host.c_str());
    if (server == NULL) {
        print_fatal("ERROR, no such host");
    }
    serv_addr.sin_family = AF_INET;
    bcopy((char *) server->h_addr, (char *) &serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(30003);
    flag = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof (int));
    setsockopt(sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof (int));
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof (int));
    fcntl(sockfd, F_SETFL, O_NONBLOCK);
    connected = false;
    keepalive = false;
    safety_count = safety_count_max + 1;
    safety_count_max_ = safety_count_max;
}

bool RtCommunication::start() {
    fd_set writefds;
    struct timeval timeout;

    keepalive = true;
    print_debug("Realtime port: Connecting...");
    ledH.startLed(0);
    
    connect(sockfd, (struct sockaddr *) &serv_addr, sizeof (serv_addr));
    FD_ZERO(&writefds);
    FD_SET(sockfd, &writefds);
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    select(sockfd + 1, NULL, &writefds, NULL, &timeout);
    unsigned int flag_len;
    getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &flag, &flag_len);
    if (flag < 0) {
        print_fatal("Error connecting to RT port 30003");
        return false;
    }
    sockaddr_in name;
    socklen_t namelen = sizeof (name);
    int err = getsockname(sockfd, (sockaddr*) & name, &namelen);
    if (err < 0) {
        print_fatal("Could not get local IP");
        close(sockfd);
        return false;
    }
    char str[18];
    inet_ntop(AF_INET, &name.sin_addr, str, 18);
    local_ip = str;
    comThread = std::thread(&RtCommunication::run, this);
    print_debug("rt_communication: FIN START...");
    return true;
}

void RtCommunication::halt() {
    keepalive = false;
    comThread.join();
    ledH.stopLeds();
    print_info("Real-time connection (30003) is shuted down.");
}

bool RtCommunication::addCommandToQueue(std::string inp) {
    int bytes_written;
    if (inp.back() != '\n') {
        inp.append("\n");
    }
    if (connected){
        bytes_written = write(sockfd, inp.c_str(), inp.length());
        ledH.startLed(0);
    } else {
        print_error("Could not send command \"" + inp + "\". The robot is not connected! Command is discarded");
        ledH.startLed(2);
        return false;
    }
    return true;
}

bool RtCommunication::setSpeed(double q0, double q1, double q2, double q3, double q4, double q5, double acc) {
    char cmd[1024];
    if (rt_data_handler->getVersion() >= 3.3) {
        sprintf(cmd,
                "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n",
//                "stopj(100.)",
                q0, q1, q2, q3, q4, q5, acc);
    } else if (rt_data_handler->getVersion() >= 3.1) {
        sprintf(cmd,
                "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f)\n",
                q0, q1, q2, q3, q4, q5, acc);
    } else {
        sprintf(cmd,
                "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.02)\n",
                q0, q1, q2, q3, q4, q5, acc);
    }
    return addCommandToQueue((std::string) (cmd));
    if (q0 != 0. or q1 != 0. or q2 != 0. or q3 != 0. or q4 != 0. or q5 != 0.) {
        //If a joint speed is set, make sure we stop it again after some time if the user doesn't
        safety_count = 0;
    }
}

bool RtCommunication::movej(double j1, double j2, double j3, double j4, double j5, double j6, double acc, double v/*, double t*/) {
    char cmd[1024];
//    if(t == void){
        sprintf(cmd,
            "movej([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f],%1.5f, %1.5f)\n",
            j1, j2, j3, j4, j5, j6, acc, v);
//    } else {*/
//    sprintf(cmd,
//            "movej([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f],%1.5f, %1.5f, %1.5f)\n",
//            j1, j2, j3, j4, j5, j6, acc, v, t);
//    }
    return addCommandToQueue((std::string) (cmd));
    print_debug(cmd);
}


void RtCommunication::movejp(double x, double y, double z, double rx, double ry, double rz, double acc, double v) {
    char cmd[1024];

    sprintf(cmd,
            "movej(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f],%1.5f, %1.5f)\n",
            x, y, z, rx, ry, rz, acc, v);
//        sprintf(cmd,
//            "movej(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f])\n",
//            x, y, z, rx, ry, rz);
    
    addCommandToQueue((std::string) (cmd));
    print_debug(cmd);
}

bool RtCommunication::set_digital_out(int op, bool on) {
    char cmd[1024];

    if (on)
        sprintf(cmd,
            "set_digital_out(%d, True)\n",
            op);
    else
        sprintf(cmd,
            "set_digital_out(%d, False)\n",
            op);
    
    return addCommandToQueue((std::string) (cmd));
}

void RtCommunication::run() {
    uint8_t buf[2048];
    int bytes_read;
    bzero(buf, 2048);
    struct timeval timeout;
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sockfd, &readfds);
    print_debug("Realtime port: Got connection");
    connected = true;
    while (keepalive) {
        print_debug("rt_communication: RUN --> while keepalive");
        while (connected && keepalive) {
//            print_debug("rt_communication: RUN --> while connected && keepalive");
            ledH.startLed(0);
            timeout.tv_sec = 0; //do this each loop as selects modifies timeout
            timeout.tv_usec = 500000; // timeout of 0.5 sec
            select(sockfd + 1, &readfds, NULL, NULL, &timeout);
            bytes_read = read(sockfd, buf, 2048);
            if (bytes_read > 0) {
                setsockopt(sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof (int));
                rt_data_handler->unpack(buf);
                if (safety_count == safety_count_max_) {
                    setSpeed(0., 0., 0., 0., 0., 0.);
                }
                safety_count += 1;
//                std::string res;
//                sprintf(res, "^^ SAFETY COUNT: %u /n^^ SCM: %u", safety_count, safety_count_max_);
//                std::cout << "^^ SAFETY COUNT: " <<  safety_count << std::endl << "SCM: " << safety_count_max_ << std::endl;
//                print_debug(res);
            } else {
//                print_warning("No bytes readed!!!!!!!!");
                connected = false;
                close(sockfd);
            }
        }
        if (keepalive) {
            //reconnect
//            print_debug("rt_communication: RUN --> if keepalive");
            
            print_warning("Realtime port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...");
            ledH.startLed(1);
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0) {
                print_fatal("ERROR opening socket");
            }
            flag = 1;
            setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof (int));
            setsockopt(sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof (int));
            setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof (int));
            fcntl(sockfd, F_SETFL, O_NONBLOCK);
            while (keepalive && !connected) {
//                print_debug("rt_communication: RUN --> while !connected && keepalive");

                std::this_thread::sleep_for(std::chrono::seconds(10));
                fd_set writefds;

                connect(sockfd, (struct sockaddr *) &serv_addr, sizeof (serv_addr));
                FD_ZERO(&writefds);
                FD_SET(sockfd, &writefds);
                select(sockfd + 1, NULL, &writefds, NULL, NULL);
                unsigned int flag_len;
                getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &flag, &flag_len);
                if (flag < 0) {
                    print_error("Error re-connecting to RT port 30003. Is controller started? Will try to reconnect in 10 seconds...");
                } else {
                    connected = true;
                    print_info("Realtime port: Reconnected");
                }
            }
        }
    }
    setSpeed(0., 0., 0., 0., 0., 0.);
    close(sockfd);
}

void RtCommunication::setSafetyCountMax(uint inp) {
    safety_count_max_ = inp;
}

std::string RtCommunication::getLocalIp() {
    return local_ip;
}
