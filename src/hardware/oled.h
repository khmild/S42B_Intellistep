#pragma once

// Imports
#include "config.h"
#include "cstring"
#include "encoder.h"
#include "main.h"
#include <algorithm>
#include "stm32f1xx_ll_gpio.h"
#include "oledfont.h"

// Variables
extern const char* topLevelMenuItems[];
extern const int topLevelMenuLength;
class StepperMotor;

// Functions (for high level displaying)
void showBootscreen();
void updateDisplay();
void displayMotorData();
void selectMenuItem();
void moveCursor();
void exitCurrentMenu();
int getMenuDepth();

// BTT Display definitions
typedef enum {
    COMMAND,
    DATA
} OLED_MODE;

typedef enum {
    BLACK,
    WHITE
} OLED_COLOR;

#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
#define OB_SIZE 32

// Low level OLED commands
void initOLED();
void writeOLEDByte(uint8_t data, OLED_MODE mode);
void writeOLEDOn();
void writeOLEDOff();
void writeOLEDBuffer();
void setOLEDPixel(uint8_t x, uint8_t y, OLED_COLOR color);
void fillOLED(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_COLOR color, bool updateScreen);
void writeOLEDChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t fontSize, OLED_COLOR color, bool updateScreen);
void writeOLEDNum(uint8_t x, uint8_t y, uint32_t number, uint8_t len, uint8_t fontSize, bool updateScreen);
void writeOLEDString(uint8_t x, uint8_t y, const char *p, uint8_t fontSize, bool updateScreen);
void writeOLEDString(uint8_t x, uint8_t y, String string, bool updateScreen);
void clearOLED();
