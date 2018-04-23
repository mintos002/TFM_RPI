/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   print_out.cpp
 * Author: minto
 * 
 * Created on 5 de marzo de 2018, 20:46
 */

#include "print_out.h"

void print_info(std::string msg)
{
    printf("INFO: %s\n", msg.c_str());
}
void print_debug(std::string msg)
{
    printf("DEBUG: %s\n", msg.c_str());
}
void print_fatal(std::string msg)
{
    printf("FATAL: %s\n", msg.c_str());
}
void print_error(std::string msg)
{
    printf("ERROR: %s\n", msg.c_str());
}
void print_warning(std::string msg)
{
    printf("WARNING: %s\n", msg.c_str());
}

