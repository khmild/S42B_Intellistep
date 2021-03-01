#ifndef __PARSER_H__
#define __PARSER_H__

#include <Arduino.h>
#include "main.h"
#include "flash.h"

// Defines for strings that are used repeatedly
#define FEEDBACK_NO_VALUE "No value specified! Make sure to specify a value with a letter before it"
#define FEEDBACK_OK "ok"

// Parse a string for commands, returning the feedback on the command
String parseString(String buffer);

// Parse a string for a value after a letter
String parseValue(String buffer, char letter);

#endif