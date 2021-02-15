#ifndef __OLED_H
#define __OLED_H

#include "pinMapping.h"
#include "SSD1306.h"

void initOLEDSPI();
void writeOLEDString(uint8_t x, uint8_t y, char* string);
void clearOLED();
void updateDisplay();

#endif