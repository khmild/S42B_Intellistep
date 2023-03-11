#ifndef CANBUS_H
#define CANBUS_H

#include "main.h"

// The header for the library
#include "eXoCAN.h"

// Parser (called when the CAN commands are received)
#include "parser.h"

// Enumeration for CAN IDs with axis characters
typedef enum {
    NONE = -1,
    X_DRIVER_RX = 0x01,
    X_DRIVER_TX = 0x02,
    Y_DRIVER_RX = 0x05,
    Y_DRIVER_TX = 0x06,
    Z_DRIVER_RX = 0x07,
    Z_DRIVER_TX = 0x08,
    BELT_DRIVER_RX = 0x09,
    BELT_DRIVER_TX = 0x0A,
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