#ifndef __OLED_H
#define __OLED_H

// Imports
#include "pinMapping.h"
#include "SSD1306.h"
#include "cstring"
#include "motor.h"
#include "encoder.h"
#include "main.h"

// Variables
extern String topLevelMenuItems[];
extern const int topLevelMenuLength;


void initOLEDSPI();
void writeOLEDString(uint8_t x, uint8_t y, char* string);
void clearOLED();
void updateDisplay();
void displayMotorData();
void selectMenuItem();
void moveCursor();
void exitCurrentMenu();

#endif