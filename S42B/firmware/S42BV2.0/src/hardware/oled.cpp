#include "oled.h"
#include "SPI.h"
#include "cstring"
#include "math.h"

// The index of the current top level menu item
int topLevelMenuIndex = 0;

// The current value of the cursor
int currentCursorIndex = 0;

// The current menu depth. 0 would be the motor data, 1 the main menu, and 2 any submenus
int menuDepthIndex = 0;

// The SPI interface for the OLED
SPIClass OLED_SPI = SPIClass(1, 2, 3, OLED_CS_PIN);


// Main SPI setup function
void initOLED() {

    // Start the OLED SPI
    OLED_SPI.begin();

    SSD1306_Init(OLED_SPI);

    // Set all of the menu values back to their defaults (for if the screen needs to be reinitialized)
    topLevelMenuIndex = 0;
}

// Convience function for writing a specific string to the OLED panel
void writeOLEDString(uint8_t x, uint8_t y, String string) {
    SSD1306_SetCursor(x, y);
    SSD1306_WriteString(&string[0], Font_7x10, WHITE);
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
            clearOLED();
            writeOLEDString(0, 0, "->");
            writeOLEDString(2, 0,  &topLevelMenuItems[(topLevelMenuIndex)     % topLevelMenuLength][0]);
            writeOLEDString(2, 16, &topLevelMenuItems[(topLevelMenuIndex + 1) % topLevelMenuLength][0]);
            writeOLEDString(2, 32, &topLevelMenuItems[(topLevelMenuIndex + 2) % topLevelMenuLength][0]);
            writeOLEDString(2, 48, &topLevelMenuItems[(topLevelMenuIndex + 3) % topLevelMenuLength][0]);
            break;

        case 2:
            // We should be in a sub menu, this is where we have to figure out which submenu that should be
            switch(topLevelMenuIndex) {
                case 0:
                    // In the first menu, the calibration one. No need to do anything here, besides maybe displaying an progress bar or PID values (later?)
                    break;

                case 1:
                    // In the second menu, the motor mAs. This is dynamically generated and has increments every 100 mA from 0 mA (testing only) to 3500 mA
                    // ! Write yet
                    clearOLED();
                    //writeOLEDString(0, 0, "");
                    break;

                case 2:
                    // In the microstep menu, this is also dynamically generated. Get the current stepping of the motor, then display all of the values around it
                    // ! Write yet
                    clearOLED();
                    break;

                case 3:
                    // In the enable logic menu, a very simple menu. Just need to invert the displayed state
                    // ! Write yet
                    clearOLED();
                    break;

                case 4:
                    // Another easy menu, just the direction pin. Once again, just need to invert the state
                    // ! Write yet
                    clearOLED();
                    break;
            }
            break;
    }
}


// Gets the latest parameters from the motor and puts them on screen
void displayMotorData() {

    // Clear the old data off of the display
    clearOLED();

    // RPM of the motor (RPM is capped at 2 decimal places)
    float currentRPM = motor.getMotorRPM();
    writeOLEDString(0, 0, &(String("RPM: ") + String(currentRPM))[0]);

    // PID loop error
    float PIDError = motor.getPIDError();
    writeOLEDString(0, 16, &(String("Err: ") + String(PIDError))[0]);

    // Current angle of the motor
    float currentAngle = getEncoderAngle();
    writeOLEDString(0, 32, &(String("Deg: ") + String(currentAngle))[0]);

    // Maybe a 4th line later?

}


// Function for moving the cursor up
void selectMenuItem() {

    // Go down in the menu index if we're not at the bottom already
    if (menuDepthIndex < 2) {
        menuDepthIndex++;
    }

    // If we're in certain menus, the cursor should start at their current value
    if (menuDepthIndex == 2) {
        // Cursor settings are only needed in the submenus

        // Check the submenus available
        switch(topLevelMenuIndex) {

            case 0:
                // Nothing to see here, just the calibration display.
                break;

            case 1:
                // Motor mAs. Need to get the current motor mAs, then convert that to a cursor value
                currentCursorIndex = motor.getCurrent() / 100;
                break;

            case 2:
                // Motor microstepping. Need to get the current microstepping setting, then convert it to a cursor value
                currentCursorIndex = log2(motor.getMicrostepping());
                break;

            case 3:
                // Get if the enable pin is inverted
                // ! Write yet
                break;

            case 4:
                // Get if the direction pin is inverted
                // ! Write yet
                break;


        }

    }
}


// Function for moving the cursor down
void moveCursor() {

    // If we're on the motor display menu, do nothing for now
    if (menuDepthIndex == 0) {
        // Do nothing (maybe add a feature later?)
    }
    else if (menuDepthIndex == 1) {
        // We're in the top level menu, change the topLevelMenuIndex
        topLevelMenuIndex++;
    }
    else {
        // We have to be in the submenu, increment the cursor index (submenus handle the display themselves)
        currentCursorIndex++;
    }    
}


// Function for exiting the current menu
void exitCurrentMenu() {

    // Go up in the menu index if we're not already at the motor data screen
    if (menuDepthIndex > 0) {
        menuDepthIndex--;
    }
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