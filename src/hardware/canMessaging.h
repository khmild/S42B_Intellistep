#ifndef __CANBUS_H__
#define __CANBUS_H__

#include "main.h"

// The header for the library
#include "eXoCAN.h"

// Parser (called when the CAN commands are received)
#include "parser.h"

// Enumeration for CAN IDs with axis characters
typedef enum {
    MAINBOARD, // The mainboard of the controller
    HOST, // The computer that is connected
    X, X2, X3, X4, X5,
    Y, Y2, Y3, Y4, Y5,
    Z, Z2, Z3, Z4, Z5, 
    E, E2, E3, E4, E5, E6, E7
} AXIS_CAN_ID;

// Initialize the CAN bus
void initCAN();

// Sends a CAN string (raw int)
void txCANString(int ID, String string);

// Sends a CAN string (using an AXIS_CAN_ID)
void txCANString(AXIS_CAN_ID ID, String string);

// Reads out the CAN buffer to the CAN command buffer
void rxCANFrame();

// Checks to see if there is a full message in the CAN command buffer
void checkCANCmd();

// Sets the CAN ID of the board
void setCANID(AXIS_CAN_ID canID);

// Gets the CAN ID of the board
AXIS_CAN_ID getCANID();

#endif