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

void print_info(const char *msg)
{
    printf("INFO: %s\n", msg);
}
void print_debug(const char *msg)
{
    printf("DEBUG: %s\n", msg);
}
void print_fatal(const char *msg)
{
    printf("FATAL: %s\n", msg);
}
void print_error(const char *msg)
{
    printf("ERROR: %s\n", msg);
}
void print_warning(const char *msg)
{
    printf("WARNING: %s\n", msg);
}

