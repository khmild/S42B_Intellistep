#ifndef __PARSER_H__
#define __PARSER_H__

#include <Arduino.h>
#include "main.h"
#include "flash.h"
#include "config.h"

// Defines for strings that are used repeatedly
#define FEEDBACK_NO_VALUE        F("No value specified! Make sure to specify a value with a letter before it\n")
#define FEEDBACK_OK              F("ok\n")
#define FEEDBACK_CAN_NOT_ENABLED F("CAN functionality not enabled\n")
#define FEEDBACK_INVALID_STRING  F("Invalid string. Make sure that the string had double quotations on each side\n")


// Firmware feature prints
#define FIRMWARE_FEATURE_VERSION (String("Version: ") + String(VERSION) + String("\n"))
#define FIRMWARE_FEATURE_HEADER  (String("Enabled features:\n"))

// Firmware feature print definition
#ifdef ENABLE_OLED
    #define FIRMWARE_FEATURE_OLED     String("OLED\n")
#else
    #define FIRMWARE_FEATURE_OLED     String("")
#endif

#ifdef ENABLE_SERIAL
    #define FIRMWARE_FEATURE_SERIAL    String("Serial\n")
#else
    #define FIRMWARE_FEATURE_SERIAL    String("")
#endif

#ifdef ENABLE_CAN
    #define FIRMWARE_FEATURE_CAN    String("CAN\n")
#else
    #define FIRMWARE_FEATURE_CAN    String("")
#endif

#ifdef ENABLE_STALLFAULT
    #define FIRMWARE_FEATURE_STALLFAULT    String("StallFault\n")
#else
    #define FIRMWARE_FEATURE_STALLFAULT    String("")
#endif

#ifdef ENABLE_DYNAMIC_CURRENT
    #define FIRMWARE_FEATURE_DYNAMIC_CURRENT    String("Dynamic Current\n")
#else
    #define FIRMWARE_FEATURE_DYNAMIC_CURRENT    String("")
#endif

// Main firmware print string
#define FIRMWARE_FEATURE_PRINT (FIRMWARE_FEATURE_VERSION + FIRMWARE_FEATURE_HEADER + FIRMWARE_FEATURE_OLED + FIRMWARE_FEATURE_SERIAL + FIRMWARE_FEATURE_CAN + FIRMWARE_FEATURE_STALLFAULT + FIRMWARE_FEATURE_DYNAMIC_CURRENT)

// Parse a string for commands, returning the feedback on the command
String parseCommand(String buffer);

// Parse a string for a value after a letter
String parseValue(String buffer, char letter);

// Parses a string for a string after a letter
String parseString(String buffer, char letter);

#endif