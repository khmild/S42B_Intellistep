#ifndef __PARSER_H__
#define __PARSER_H__

#include <Arduino.h>
#include "main.h"
#include "flash.h"
#include "config.h"

// Defines for strings that are used repeatedly
#define FEEDBACK_NO_VALUE        F("No value specified! Make sure to specify a value with a letter before it")
#define FEEDBACK_OK              F("ok")
#define FEEDBACK_CAN_NOT_ENABLED F("CAN functionality not enabled")
#define FEEDBACK_INVALID_STRING  F("Invalid string. Make sure that the string had double quotations on each side")


// Firmware feature prints
#define FIRMWARE_FEATURE_VERSION  String("Version: " + String(VERSION) + "\n")
#define FIRMWARE_BUILD_INFO       String("Compiled: " + String(__DATE__) + ", " + String(__TIME__) + "\n")
#define FIRMWARE_FEATURE_HEADER   String("Enabled features:")

// Firmware feature print definition
#ifdef ENABLE_OLED
    #define FIRMWARE_FEATURE_OLED     "\nOLED"
#else
    #define FIRMWARE_FEATURE_OLED     ""
#endif

#ifdef ENABLE_SERIAL
    #define FIRMWARE_FEATURE_SERIAL    "\nSerial"
#else
    #define FIRMWARE_FEATURE_SERIAL    ""
#endif

#ifdef ENABLE_CAN
    #define FIRMWARE_FEATURE_CAN    "\nCAN"
#else
    #define FIRMWARE_FEATURE_CAN    ""
#endif

#ifdef ENABLE_STALLFAULT
    #define FIRMWARE_FEATURE_STALLFAULT    "\nStallFault"
#else
    #define FIRMWARE_FEATURE_STALLFAULT    ""
#endif

#ifdef ENABLE_DYNAMIC_CURRENT
    #define FIRMWARE_FEATURE_DYNAMIC_CURRENT    "\nDynamic Current"
#else
    #define FIRMWARE_FEATURE_DYNAMIC_CURRENT    ""
#endif

#ifdef ENABLE_OVERTEMP_PROTECTION
    #define FIRMWARE_FEATURE_OVERTEMP_PROTECTION    "\nOvertemp Protection"
#else
    #define FIRMWARE_FEATURE_OVERTEMP_PROTECTION    ""
#endif

// Main firmware print string
#define FIRMWARE_FEATURE_PRINT String(FIRMWARE_FEATURE_VERSION + FIRMWARE_BUILD_INFO + FIRMWARE_FEATURE_HEADER + FIRMWARE_FEATURE_OLED + FIRMWARE_FEATURE_SERIAL + FIRMWARE_FEATURE_CAN + FIRMWARE_FEATURE_STALLFAULT + FIRMWARE_FEATURE_DYNAMIC_CURRENT + FIRMWARE_FEATURE_OVERTEMP_PROTECTION)

// Parse a string for commands, returning the feedback on the command
String parseCommand(String buffer);

// Parse a string for a value after a letter
String parseValue(String buffer, char letter);

// Parses a string for a string after a letter
String parseString(String buffer, char letter);

#endif