#include "oled.h"
#include "SPI.h"

// Main SPI setup function
void initOLEDSPI() {
    SPI.begin(OLED_CS_PIN);
}

// Convience function for writing a specific string to the OLED panel
void writeOLEDString(uint8_t x, uint8_t y, char* string) {
    SSD1306_SetCursor(x, y);
    SSD1306_WriteString(string, Font_7x10, WHITE);
}

// Convience function to clear the OLED display
void clearOLED() {
    SSD1306_Fill(BLACK);
}

// Function for displaying relevant data on the OLED panel, such as motor data or menu data
void updateDisplay() {
    
}