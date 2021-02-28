#ifndef __PARSER_H__
#define __PARSER_H__

#include <Arduino.h>
#include "main.h"
#include "flash.h"

// Defines for strings that are used repeatedly
#define noValue "No value specified! Make sure to specify a value with a letter before it"

// Parse a string for commands
void parseString(String buffer);

// Parse a string for a value after a letter
String parseValue(String buffer, char letter);

#endif