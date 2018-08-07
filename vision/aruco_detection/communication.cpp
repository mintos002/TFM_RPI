/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   communication.cpp
 * Author: minto
 * 
 * Created on 5 de marzo de 2018, 12:48
 */

#include "communication.h"

Communication::Communication(std::condition_variable& msg_cond, std::string host) {
    robot_state = new DataHandler(msg_cond);
    server = gethostbyname(host.c_str()); //Save host in server
    if (server == NULL) print_error("No such host"); //if NO server print ERROR

    // Create the sockets (Internet domain ipv4, stream-oriented, TCP protocol)
    pri_sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sec_sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (pri_sockfd < 0) print_error("Can not open primary socket");
    if (sec_sockfd < 0) print_error("Can not open secondary socket");

    // Prepare the server address structure
    bzero((char *) &pri_serv_addr, sizeof (pri_serv_addr));
    bzero((char *) &sec_serv_addr, sizeof (sec_serv_addr));
    pri_serv_addr.sin_family = AF_INET;
    sec_serv_addr.sin_family = AF_INET;
    bcopy((char *) server->h_addr, (char *) &pri_serv_addr.sin_addr.s_addr, server->h_length);
    bcopy((char *) server->h_addr, (char *) &sec_serv_addr.sin_addr.s_addr, server->h_length);
    pri_serv_addr.sin_port = htons(30001);
    sec_serv_addr.sin_port = htons(30002);

    flag = 1;
    // To make TCP connection faster:
    // Send data ASAP
    setsockopt(pri_sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
    setsockopt(sec_sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

    // Send ACK ASAP
    setsockopt(pri_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof(int));
    setsockopt(sec_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof(int));
    
    // manipulate socket-level options, to control permition of reuseling local address for this socket
    setsockopt(pri_sockfd, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof(int));
    setsockopt(sec_sockfd, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof(int));    
    
    keepalive = false;
    connected = false;
    print_debug("Declaration of communication object SUCCESSFUL");
}

bool Communication::start() {
    keepalive = true;
    uint8_t buffer[512];
    unsigned int bytes_read;
    std::string cmd;
    bzero(buffer, 512);
    print_debug("Acquire firmare version: Connecting...");
    if (connect(pri_sockfd, (struct sockaddr *) &pri_serv_addr, sizeof (pri_serv_addr)) < 0) {
        print_fatal("Error connecting to get firmware version");
        return false;
    }
    print_debug("Acquire firmware version: Got connection");
    bytes_read = read(pri_sockfd, buffer, 512);
    //  SETSOCKOPT:  int setsockopt(int socket, int level, int option_name, const void *option_value, socklen_t option_length);
    setsockopt(pri_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof(int));
    robot_state->unpack(buffer,bytes_read);
    //wait for some traffic so the UR socket doesn't die in version 3.1.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    char tmp[64];
    sprintf(tmp, "Firmware version detected: %.7f", robot_state->getVersion());
    print_debug(tmp);
    close(pri_sockfd);

    print_debug("Switching to secondary interface for masterboard data: Connecting...");

    fd_set writefds;
    struct timeval timeout;

    connect(sec_sockfd, (struct sockaddr *) &sec_serv_addr, sizeof(sec_serv_addr));
    FD_ZERO(&writefds);
    FD_SET(sec_sockfd, &writefds);
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    select(sec_sockfd + 1, NULL, &writefds, NULL, &timeout);
    unsigned int flag_len;
    getsockopt(sec_sockfd, SOL_SOCKET, SO_ERROR, &flag, &flag_len);
    if (flag < 0) {
        print_fatal("Error connecting to secondary interface");
        return false;
    }
    print_debug("Secondary interface: Got connection");
    comThread = std::thread(&Communication::run, this);
    return true;
}

void Communication::halt() {
//    print_info("En Halt communication");
    keepalive = false;
//    print_info("En Halt communication2");
    comThread.join();
    print_info("Connection 30002 is shuted down.");    
}

void Communication::run() {
    print_debug("IN Communication::run()");
    uint8_t buffer[2048];
    int bytes_read;
    bzero(buffer, 2048);
    struct timeval timeout;
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sec_sockfd, &readfds);
    connected = true;
    while (keepalive) {
//        print_debug("init while");
        while (connected && keepalive) {
//            print_debug("## while connected && keepalive");
            timeout.tv_sec = 0; //do this each loop as selects modifies timeout
            timeout.tv_usec = 500000; // timeout of 0.5 sec
            select(sec_sockfd + 1, &readfds, NULL, NULL, &timeout);
            bytes_read = read(sec_sockfd, buffer, 2048); // usually only up to 1295 bytes
            if (bytes_read > 0) {
//                print_debug("## while connected && keepalive (bytes_read > 0)");
                setsockopt(sec_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof(int));
                robot_state->unpack(buffer, bytes_read);
            } else {
//                print_debug("## while connected && keepalive (bytes_read <= 0)");
                connected = false;
                robot_state->setDisconnected();
                close(sec_sockfd);
            }
        }
        if (keepalive) {
//            print_debug("## keepalive");
            //reconnect
            print_warning("Secondary port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...");
            sec_sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sec_sockfd < 0) {
                print_fatal("ERROR opening secondary socket");
            }
            flag = 1;
            setsockopt(sec_sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
            setsockopt(sec_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag, sizeof(int));
            setsockopt(sec_sockfd, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof(int));
            fcntl(sec_sockfd, F_SETFL, O_NONBLOCK);
            while (keepalive && !connected) {
//                print_debug("## while !connected && keepalive");
                std::this_thread::sleep_for(std::chrono::seconds(10));
                fd_set writefds;

                connect(sec_sockfd, (struct sockaddr *) &sec_serv_addr, sizeof(sec_serv_addr));
                FD_ZERO(&writefds);
                FD_SET(sec_sockfd, &writefds);
                select(sec_sockfd + 1, NULL, &writefds, NULL, NULL);
                unsigned int flag_len;
                getsockopt(sec_sockfd, SOL_SOCKET, SO_ERROR, &flag, &flag_len);
                if (flag < 0) {
                    print_error("Error re-connecting to port 30002. Is controller started? Will try to reconnect in 10 seconds...");
                } else {
                    connected = true;
                    print_info("Secondary port: Reconnected");
                }
            }
        }
    }
//    print_debug("OUT while");

    //wait for some traffic so the UR socket doesn't die in version 3.1.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    close(sec_sockfd);
}