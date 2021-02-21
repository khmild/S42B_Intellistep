#ifndef __SERIAL_H__
#define __SERIAL_H__

// Imports
#include "main.h"
#include "string.h"
#include <cstdlib> 

void initSerial();
void sendMessage(String message);
String readBuffer();
bool checkSerialData();
void runSerialParser();

#endif