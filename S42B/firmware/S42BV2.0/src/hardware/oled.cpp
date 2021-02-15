#include "oled.h"
#include "SPI.h"
#include "cstring"

// The index of the current top level menu item
int topLevelMenuIndex = 0;

// The current value of the cursor
int currentCursorIndex = 0;

// The current menu depth. 0 would be the motor data, 1 the main menu, and 2 any submenus
int menuDepthIndex = 0;


// Main SPI setup function
void initOLEDSPI() {
    SPI.begin(OLED_CS_PIN);

    // Set all of the menu values back to their defaults (for if the screen needs to be reinitialized)
    topLevelMenuIndex = 0;
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

    // Decide on the current depth of the menu
    switch(menuDepthIndex){
        case 0:
            // Not actually in the menu, just displaying motor data for now
            displayMotorData();
            break;

        case 1:
            // In the top level of the menu. Make sure that the set top level menu index is within the range of the menu length
            // Values must first be determined using the mod function in order to prevent overflow errors
            // Then the strings are converted into character arrays by giving the address of the first character in the string
            writeOLEDString(0, 0,  &topLevelMenuItems[(topLevelMenuIndex)     % topLevelMenuLength][0]);
            writeOLEDString(0, 16, &topLevelMenuItems[(topLevelMenuIndex + 1) % topLevelMenuLength][0]);
            writeOLEDString(0, 32, &topLevelMenuItems[(topLevelMenuIndex + 2) % topLevelMenuLength][0]);
            writeOLEDString(0, 48, &topLevelMenuItems[(topLevelMenuIndex + 3) % topLevelMenuLength][0]);
            break;

        case 2:
            // We should be in a sub menu, this is where we have to figure out which submenu that should be
            switch(topLevelMenuIndex) {
                case 0:
                    // In the first menu, the calibration one. No need to do anything here, besides maybe displaying an progress bar (later?)
                    break;

                case 1:
                    // In the second menu, the motor mAs. This is dynamically generated and has increments every 100 mA from 0 mA (testing only) to 3500 mA
                    // ! Write yet
                    break;

                case 2:
                    // In the microstep menu, this is also dynamically generated. Get the current stepping of the motor, then display all of the values around it
                    // ! Write yet
                    break;

                case 3: 
                    // In the enable logic menu, a very simple menu. Just need to invert the displayed state
                    // ! Write yet
                    break;

                case 4:
                    // Another easy menu, just the direction pin. Once again, just need to invert the state
                    // ! Write yet
                    break;
            }
            break;
    }
    
}


// Gets the latest parameters from the motor and puts them on screen
void displayMotorData() {

    // RPM of the motor (RPM is capped at 2 decimal places)
    float currentRPM = getMotorRPM();
    writeOLEDString(0, 0, &(String("RPM: ") + String(currentRPM))[0]);

    // PID loop error
    float PIDError = getPIDError();
    writeOLEDString(0, 16, &(String("Err: ") + String(PIDError))[0]);

    // Current angle of the motor
    float currentAngle = getAngle();
    writeOLEDString(0, 32, &(String("Deg: ") + String(currentAngle))[0]);

    // Maybe a 4th line later?

}


// Function for moving the cursor up
void selectMenuItem() {
    // ! Write yet
}


// Function for moving the cursor down
void cursorDown() {
    // ! Check over yet
    currentCursorIndex++;
}


// Function for exiting the current menu
void exitCurrentMenu() {
    // ! Write yet
}


// A list of all of the top level menu items
String topLevelMenuItems[] = {
    "Calibrate", 
    "Motor mA",
    "Microstep",
    "Enable Logic"
    "Dir. Logic"
};

// Length of the list of top menu items (found by dividing the length of the list by how much space a single element takes up)
const int topLevelMenuLength = sizeof(topLevelMenuItems) / sizeof(topLevelMenuItems[0]);