/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   print_out.h
 * Author: minto
 *
 * Created on 5 de marzo de 2018, 20:46
 */

#ifndef PRINT_OUT_H
#define PRINT_OUT_H

#include <string>

void print_info(const char *msg);
void print_debug(const char *msg);
void print_fatal(const char *msg);
void print_error(const char *msg);
void print_warning(const char *msg);

#endif /* PRINT_OUT_H */

