#ifndef __SERIAL_H__
#define __SERIAL_H__

// Imports
#include "main.h"
#include "string.h"
#include <cstdlib>
#include "parser.h"
#include <cctype>

void initSerial();
void sendSerialMessage(String message);
String readSerialBuffer();
bool checkSerialData();
void runSerialParser();

#endif