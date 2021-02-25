#ifndef __OLED_H__
#define __OLED_H__

// Imports
#include "config.h"
#include "SSD1306.h"
#include "cstring"
#include "motor.h"
#include "encoder.h"
#include "main.h"
#include <algorithm>

// Variables
extern String topLevelMenuItems[];
extern const int topLevelMenuLength;

void initOLED();
void writeOLEDString(uint8_t x, uint8_t y, String string);
void clearOLED();
void updateDisplay();
void displayMotorData();
void selectMenuItem();
void moveCursor();
void exitCurrentMenu();
int getMenuDepth();

#endif