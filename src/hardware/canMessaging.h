#ifndef __CANBUS_H__
#define __CANBUS_H__

#include "main.h"

// The header for the library
#include "CAN.h"

// Parser (called when the CAN commands are received)
#include "parser.h"

// Initialize the CAN bus
void initCAN();

// Send a command over the CAN bus
void sendCANCommand(char letter, int commandNumber);

// Receive a command over the CAN bus (a maximum of 8 values are allowed, so a command and 3 parameters)
void receieveCANCommand();

// Sets the CAN ID of the board
void setCANID(uint16_t canID);

// Gets the CAN ID of the board
uint16_t getCANID();

#endif