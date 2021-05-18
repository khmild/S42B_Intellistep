// Import the config (nedeed for the USE_OLED define)
#include "config.h"

// Only build if needed
#ifdef USE_OLED

// Import libraries
#include "oled.h"
#include "cstring"
#include "math.h"

// Screen data is stored in an array. Each set of 8 pixels is written one by one
// Each set of 8 is stored as an integer. The panel is written to horizontally, then vertically
// Array stores the values in width - height format with bits stored along vertical rows
uint8_t OLEDBuffer[128][8];

// The index of the current top level menu item
SUBMENU submenu = CALIBRATION;
SUBMENU lastSubmenu = CALIBRATION;

// The current value of the cursor
uint8_t currentCursorIndex = 0;

// The current menu depth
MENU_DEPTH menuDepth = MOTOR_DATA;
MENU_DEPTH lastMenuDepth = MOTOR_DATA;

// Displays the bootscreen
void showBootscreen() {
    writeOLEDString(0, 0, F("Intellistep"), false);
    writeOLEDString(0, 32, F("Version: "), false);
    writeOLEDString(0, 48, F(VERSION), true);
}


// Function for displaying relevant data on the OLED panel, such as motor data or menu data
void updateDisplay() {

    // Decide on the current depth of the menu
    switch(menuDepth) {
        case MOTOR_DATA:
            // Not actually in the menu, just displaying motor data for now
            displayMotorData();
            break;

        case TOP_LEVEL:
            // In the top level of the menu. Make sure that the set top level menu index is within the range of the menu length
            // Values must first be determined using the mod function in order to prevent overflow errors
            // Then the strings are converted into character arrays by giving the address of the first character in the string
            clearOLED();
            writeOLEDString(0, 0, "->", false);
            writeOLEDString(25, 0,  &submenuItems[(submenu)     % submenuCount][0], false);
            writeOLEDString(25, 16, &submenuItems[(submenu + 1) % submenuCount][0], false);
            writeOLEDString(25, 32, &submenuItems[(submenu + 2) % submenuCount][0], false);
            writeOLEDString(25, 48, &submenuItems[(submenu + 3) % submenuCount][0], true);
            break;

        case SUBMENUS:
            // We should be in a sub menu, this is where we have to figure out which submenu that should be
            switch(submenu) {
                case CALIBRATION:
                    // In the first menu, the calibration one. No need to do anything here, besides maybe displaying an progress bar or PID values (later?)
                    clearOLED();
                    writeOLEDString(0, 0, "Are you sure?", false);
                    writeOLEDString(0, 16, "Press select", false);
                    writeOLEDString(0, 32, "to confirm", true);
                    break;

                case CURRENT:
                    // In the second menu, the motor mAs. This is dynamically generated and has increments every 100 mA from 0 mA (testing only) to 3500 mA
                    clearOLED();

                    // Constrain the current setting within 0 and the maximum current
                    if (currentCursorIndex > (uint16_t)MAX_RMS_CURRENT / 100) {

                        // Loop back to the start of the list
                        currentCursorIndex = 0;
                    }

                    // Write the pointer
                    writeOLEDString(0, 0, "->", false);

                    // Write each of the strings
                    for (uint8_t stringIndex = 0; stringIndex <= 3; stringIndex++) {

                        // Check to make sure that the current isn't out of range of the max current
                        if ((currentCursorIndex + stringIndex) * 100 <= (uint16_t)MAX_RMS_CURRENT) {

                            // Value is in range, display the current on that line
                            writeOLEDString(25, stringIndex * 16, String((int) ((currentCursorIndex + stringIndex) * 100)) + String("mA"), false);
                        }
                        // else {
                            // Value is out of range, display a blank line for this line
                        // }
                    }

                    // Write out the info to the OLED display
                    writeOLEDBuffer();
                    break;

                case MICROSTEP:
                    // In the microstep menu, this is also dynamically generated. Get the current stepping of the motor, then display all of the values around it
                    clearOLED();
                    writeOLEDString(0, 0, "->", false);

                    // Loop the currentCursor index back if it's out of range
                    if (currentCursorIndex > log2(MAX_MICROSTEP_DIVISOR)) {
                        currentCursorIndex = log2(MIN_MICROSTEP_DIVISOR);
                    }
                    else if (currentCursorIndex < log2(MIN_MICROSTEP_DIVISOR)) {

                        // Make sure that the cursor index is in valid range
                        currentCursorIndex = log2(MIN_MICROSTEP_DIVISOR);
                    }

                    // Write each of the strings
                    for (uint8_t stringIndex = 0; stringIndex <= 3; stringIndex++) {

                        // Check to make sure that the current isn't out of range of the max current
                        if (pow(2, currentCursorIndex + stringIndex) <= MAX_MICROSTEP_DIVISOR) {

                            // Value is in range, display the current on that line
                            writeOLEDString(25, stringIndex * 16, String("1/") + String((int) pow(2, currentCursorIndex + stringIndex)) + String("th"), false);
                        }
                        // else {
                            // Value is out of range, display a blank line for this line
                        // }
                    }

                    // Write out the data to the OLED
                    writeOLEDBuffer();
                    break;

                case ENABLE_LOGIC:
                    // In the enable logic menu, a very simple menu. Just need to invert the displayed state
                    // Clear the OLED
                    clearOLED();

                    // Title
                    writeOLEDString(0, 0, "Enable logic:", false);

                    // Write the string to the screen
                    if (currentCursorIndex % 2 == 0) {

                        // The index is even, the logic is inverted
                        writeOLEDString(0, 24, "Inverted", true);
                    }
                    else {
                        // Index is odd, the logic is normal
                        writeOLEDString(0, 24, "Normal", true);
                    }
                    break;

                case DIR_LOGIC:
                    // Another easy menu, just the direction pin. Once again, just need to invert the state
                    clearOLED();

                    // Title
                    writeOLEDString(0, 0, "Dir logic:", false);

                    // Write the string to the screen
                    if (currentCursorIndex % 2 == 0) {

                        // The index is even, the logic is inverted
                        writeOLEDString(0, 24, "Inverted", true);
                    }
                    else {
                        // Index is odd, the logic is normal
                        writeOLEDString(0, 24, "Normal", true);
                    }
                    break;
            } // Submenu switch
            break;

            case WARNING:
                // Nothing to do
                break;

    } // Main switch

    // Save the last set value (for display functions that don't need to clear the OLED all of the time)
    lastMenuDepth = menuDepth;
    lastSubmenu = submenu;

} // Display menu function


// Gets the latest parameters from the motor and puts them on screen
void displayMotorData() {

    // Clear the old menu off of the display
    if (lastMenuDepth != 0) {
        clearOLED();
    }

    // RPM of the motor (RPM is capped at 2 decimal places)
    #ifdef ENCODER_SPEED_ESTIMATION

    // Check if the motor RPM can be updated. The update rate of the speed must be limited while using encoder speed estimation
    if (micros() - lastAngleSampleTime > SPD_EST_MIN_INTERVAL) {
        writeOLEDString(0, 0, (String("RPM: ") + padNumber(motor.getMotorRPM(), 2, 3)) + String("     "), false);
    }

    #else // ! ENCODER_SPEED_ESTIMATION

    // No need to check, just sample it
    writeOLEDString(0, 0, (String("RPM:   ") + padNumber(motor.getMotorRPM(), 2, 3)) + String("     "), false);

    #endif // ! ENCODER_SPEED_ESTIMATION

    // Angle error
    writeOLEDString(0, 16, (String("Err: ") + padNumber(motor.getAngleError(), 4, 2)) + String("     "), false);

    // Current angle of the motor
    writeOLEDString(0, 32, (String("Deg: ") + padNumber(getAbsoluteAngle(), 4, 2)) + String("     "), false);

    // Maybe a 4th line later?
    writeOLEDString(0, 48, (String("Temp:   ") + String(getEncoderTemp()) + String(" C")) + String("     "), true);
}


// Display an error message
void displayWarning(String firstLine, String secondLine, String thirdLine, bool updateScreen) {
    clearOLED();
    writeOLEDString(0, 0,  firstLine,  false);
    writeOLEDString(0, 16, secondLine, false);
    writeOLEDString(0, 32, thirdLine,  false);
    writeOLEDString(0, 48, F("Select to exit"), updateScreen);

    // Save the last menu used (for returning later), then move to the new index
    lastMenuDepth = menuDepth;
    menuDepth = WARNING;
}


// Function for moving the cursor up
void selectMenuItem() {

    // Go down to the top level
    if (menuDepth == MOTOR_DATA) {
        menuDepth = TOP_LEVEL;
        updateDisplay();
    }

    // Go down to the submenus and deterimine the starting index
    else if (menuDepth == TOP_LEVEL) {

        // Set that we're moving into the submenus
        menuDepth = SUBMENUS;

        // Check the submenus available
        switch(submenu % submenuCount) {

            case CALIBRATION:
                // Nothing to see here, just moving into the calibration
                motor.calibrate();

                // Exit the menu
                menuDepth = MENU_RETURN_LEVEL;
                break;

            case CURRENT:
                // Motor mAs. Need to get the current motor mAs, then convert that to a cursor value
                currentCursorIndex = constrain(round(motor.getRMSCurrent() / 100), 0, (uint16_t)MAX_RMS_CURRENT);

                // Enter the menu
                menuDepth = SUBMENUS;
                break;

            case MICROSTEP:
                // Motor microstepping. Need to get the current microstepping setting, then convert it to a cursor value. Needs to be -2 because the lowest index, 1/4 microstepping, would be at index 0
                currentCursorIndex = log2(motor.getMicrostepping()) - log2(MIN_MICROSTEP_DIVISOR);

                // Enter the menu
                menuDepth = SUBMENUS;
                break;

            case ENABLE_LOGIC:
                // Get if the enable pin is inverted
                if (motor.getEnableInversion()) {
                    // Value is true already, therefore start at "true"
                    currentCursorIndex = 0;
                }
                else {
                    // Otherwise set the value to false to start
                    currentCursorIndex = 1;
                }

                // Enter the menu
                menuDepth = SUBMENUS;
                break;

            case DIR_LOGIC:
                // Get if the direction pin is inverted
                if (motor.getReversed()) {
                    currentCursorIndex = 0;
                }
                else {
                    currentCursorIndex = 1;
                }

                // Enter the menu
                menuDepth = SUBMENUS;
                break;
        }

        // Update the display with the new information
        updateDisplay();
    }

    // If we're in certain menus, the cursor should start at their current value
    else if (menuDepth == SUBMENUS) {
        // Cursor settings are only needed in the submenus

        // Check the submenus available
        switch(submenu % submenuCount) {

            case CALIBRATION:
                // Nothing to see here, everything is handled on the initial press.
                break;

            case CURRENT: {
                // Motor mAs. Need to get the cursor value, then convert that to current value
                // Get the set value
                uint8_t rmsCurrentSetting = 100 * currentCursorIndex;

                // Check to see if the warning needs flagged
                if (rmsCurrentSetting % (uint16_t)MAX_RMS_CURRENT >= (uint16_t)WARNING_RMS_CURRENT) {

                    // Set the display to output the warning
                    menuDepth = WARNING;

                    // Actually draw the warning
                    displayWarning(F("Large current"), F("set. Are you"), F("sure?"), true);
                }
                else {
                    // Set the value
                    motor.setRMSCurrent(rmsCurrentSetting % (uint16_t)MAX_RMS_CURRENT);
                    
                    // Exit the menu
                    menuDepth = MENU_RETURN_LEVEL;
                }
           
                // We're done here, time to head out
                break;
            }

            case MICROSTEP: {
                // Motor microstepping. Need to get the current microstepping setting, then convert it to a cursor value. Needs to be -2 because the lowest index, 1/4 microstepping, would be at index 0
                // Get the set value
                uint8_t microstepSetting = pow(2, currentCursorIndex);

                // Check to see if the warning needs flagged
                if (microstepSetting >= WARNING_MICROSTEP) {

                    // Set the display to output the warning
                    menuDepth = WARNING;

                    // Actually draw the warning
                    displayWarning(F("Large stepping"), F("set. Are you"), F("sure?"), true);
                }
                else {
                    // Set the value
                    motor.setMicrostepping(microstepSetting);
                    
                    // Exit the menu
                    menuDepth = MENU_RETURN_LEVEL;
                }
                
                // We're done here, time to head out
                break;
            }

            case ENABLE_LOGIC:
                // Get if the enable pin is inverted
                if (currentCursorIndex % 2 == 0) {

                    // The index is even, the logic is inverted
                    motor.setEnableInversion(true);
                }
                else {
                    // Index is odd, the logic is normal
                    motor.setEnableInversion(false);
                }

                // Exit the menu
                menuDepth = MENU_RETURN_LEVEL;
                break;

            case DIR_LOGIC:
                // Get if the direction pin is inverted
                if (currentCursorIndex % 2 == 0) {

                    // The index is even, the direction is inverted
                    motor.setReversed(true);
                }
                else {
                    // Index is odd, the direction is normal
                    motor.setReversed(false);
                }

                // Exit the menu
                menuDepth = MENU_RETURN_LEVEL;
                break;
        }

        // Update the display with the new menu  
        updateDisplay();
    }

    // Warning level
    else if (menuDepth == WARNING) {

        // Check which menu was previously displayed
        switch(submenu) {

            // Need to set the current
            case CURRENT:

                // Calculate the setting from the cursor index
                motor.setRMSCurrent((100 * currentCursorIndex) % (uint16_t)MAX_RMS_CURRENT);

            // Need to set the microstep
            case MICROSTEP:

                // Set the value
                motor.setMicrostepping(pow(2, currentCursorIndex));

            default:
                // Nothing to do here, just move on
                break;
        }

        // Set the current menu to the last menu used
        menuDepth = MENU_RETURN_LEVEL;

        // Update the display
        updateDisplay();
    }
}


// Function for moving the cursor down
void moveCursor() {

    // If we're on the motor display menu, do nothing for now
    if (menuDepth == MOTOR_DATA) {
        // Do nothing (maybe add a feature later?)
    }
    else if (menuDepth == TOP_LEVEL) {
        // We're in the top level menu, change the topLevelMenuIndex (as long as we haven't exceeded the length of the list)
        if (submenu + 3 > submenuCount) {
            submenu = SUBMENU(0);
        }
        else {
            submenu = SUBMENU(submenu + 1);
        }
    }
    else if (menuDepth == SUBMENUS) {
        // We are in the submenu, increment the cursor index (submenus handle the display themselves)
        currentCursorIndex++;
    }

    // Update the display
    updateDisplay();
}


// Function for exiting the current menu
void exitCurrentMenu() {

    // Go up in the menu index if we're not already at the motor data screen
    if (menuDepth > (uint8_t)MOTOR_DATA) {
        menuDepth = MENU_DEPTH(menuDepth - 1);
    }

    // Update the display to match
    updateDisplay();
}


// Returns the depth of the menu (helpful for watching the select button)
MENU_DEPTH getMenuDepth() {
    return menuDepth;
}


// A list of all of the top level menu items
String submenuItems[] = {
    "Calibrate",
    "Motor mA",
    "Microstep",
    "En Logic",
    "Dir. Logic",
    ""
};


// Length of the list of top menu items (found by dividing the length of the list by how much space a single element takes up)
const uint8_t submenuCount = sizeof(submenuItems) / sizeof(submenuItems[0]);


// Function for padding numbers
String padNumber(float number, uint8_t leadingPlaceCount, uint8_t followingPlaceCount) {

    // The final string to be output
    String finalString = "";

    // Reserve memory for the string (needs an extra 1 for the decimal place)
    finalString.reserve(leadingPlaceCount + followingPlaceCount + 1);

    // Correct the number's rounding
    number = roundToPlace(number, followingPlaceCount);

    // If the value is negative, add a negative sign. Otherwise, add a blank space
    if (number < 0) {
        finalString += "-";
    }
    else {
        finalString += " ";
    }

    // Add any necessary zeros
    for (uint8_t index = (leadingPlaceCount - 1); index > 0; index--) {
        if (abs(number) < pow(10, index)) {
            finalString += "0";
        }
        else {
            // No need to continue, the other values will fail as well
            break;
        }
    }

    // Add the number to the string (must be absolute, otherwise sign is inserted)
    finalString += String(abs(number));

    // Return the string
    return finalString;
}


// Round to a specific decimal place
float roundToPlace(float number, uint8_t place) {
    return round(number * pow(10, place)) / pow(10, place);
}


// Sets up the OLED panel
void initOLED() {

    // Set all of the menu values back to their defaults (for if the screen needs to be reinitialized)
    submenu = CALIBRATION;

	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<2;

    GPIOA->CRH &= 0XFFFffFF0;
    GPIOA->CRH |= 0X00000003;
    GPIOA->ODR |= 1<<8;

  	GPIOB->CRH &= 0X0000FFFF;
  	GPIOB->CRH |= 0X33330000;
	GPIOB->ODR |= 0xF<<12;

	OLED_RST_PIN = 0;
	delay(100);
	OLED_RST_PIN = 1;
	writeOLEDByte(0xAE, COMMAND);//
	writeOLEDByte(0xD5, COMMAND);//
	writeOLEDByte(80,   COMMAND);  //[3:0],;[7:4],
	writeOLEDByte(0xA8, COMMAND);//
	writeOLEDByte(0X3F, COMMAND);//(1/64)
	writeOLEDByte(0xD3, COMMAND);//
	writeOLEDByte(0X00, COMMAND);//

	writeOLEDByte(0x40, COMMAND);// [5:0],.

	writeOLEDByte(0x8D, COMMAND);//
	writeOLEDByte(0x14, COMMAND);///
	writeOLEDByte(0x20, COMMAND);//
	writeOLEDByte(0x02, COMMAND);//[1:0],;
	writeOLEDByte(0xA1, COMMAND);//,bit0:0,0->0;1,0->127;
	writeOLEDByte(0xC0, COMMAND);//;bit3:0,;1, COM[N-1]->COM0;N:
	writeOLEDByte(0xDA, COMMAND);//
	writeOLEDByte(0x12, COMMAND);//[5:4]

	writeOLEDByte(0x81, COMMAND);//
	writeOLEDByte(0xEF, COMMAND);//1~255;
	writeOLEDByte(0xD9, COMMAND);//
	writeOLEDByte(0xf1, COMMAND);//[3:0],PHASE 1;[7:4],PHASE 2;
	writeOLEDByte(0xDB, COMMAND);//
	writeOLEDByte(0x30, COMMAND);//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
	writeOLEDByte(0xA4, COMMAND);//;bit0:1,;0,;
	writeOLEDByte(0xA6, COMMAND);//;bit0:1,;0,
	writeOLEDByte(0xAF, COMMAND);//
	delay(100);
	clearOLED();
}


// Write a single byte to the LCD panel
void writeOLEDByte(uint8_t data, OLED_MODE mode) {

    // Write the current mode and enable the screen's communcation
	OLED_RS_PIN = (uint8_t)mode;
	OLED_CS_PIN = 0;

    // Write each bit of the byte
	for(uint8_t i = 0; i < 8; i++) {

        // Prevent the screen from reading the data in
		OLED_SCLK_PIN = 0;

        // If the bit is 1 (true)
		if(data & 0x80) {

            // Write true
            OLED_SDIN_PIN = 1;
        }
		else {
            // Write false
            OLED_SDIN_PIN = 0;
        }

        // Signal that the data needs to be sampled again
		OLED_SCLK_PIN = 1;

        // Shift all of the bits so the next will be read
		data <<= 1;
	}

    // Disable the screen's communication
	OLED_CS_PIN = 1;
	OLED_RS_PIN = 1;
}


// Turn the display on
void writeOLEDOn() {
	writeOLEDByte(0X8D, COMMAND);  //SET
	writeOLEDByte(0X14, COMMAND);  //DCDC ON
	writeOLEDByte(0XAF, COMMAND);  //DISPLAY ON
}


// Turn the display off
void writeOLEDOff() {
	writeOLEDByte(0X8D, COMMAND);  //SET
	writeOLEDByte(0X10, COMMAND);  //DCDC OFF
	writeOLEDByte(0XAE, COMMAND);  //DISPLAY OFF
}


// Write the entire OLED buffer to the screen
void writeOLEDBuffer() {

    // Loop through each of the vertical lines
	for(uint8_t vertIndex = 0; vertIndex < 8; vertIndex++) {

		// Specify the line that is being written to
        writeOLEDByte(0xb0 + vertIndex, COMMAND);
		writeOLEDByte(0x00, COMMAND);
		writeOLEDByte(0x10, COMMAND); // DCDC OFF

        // Loop through the horizontal lines
		for(uint8_t horzIndex = 0; horzIndex < 128; horzIndex++) {
		    writeOLEDByte(OLEDBuffer[horzIndex][vertIndex], DATA);
        }
	}
}


// Wipes the output buffer, then writes the zeroed array to the screen
void clearOLED() {

    // Set all of the values in the buffer to 0
	memset(OLEDBuffer, 0X00, sizeof(OLEDBuffer));

    // Push the values to the display
	writeOLEDBuffer();
}


// Writes a point on the buffer
void setOLEDPixel(uint8_t x, uint8_t y, OLED_COLOR color) {

    // Create variables for later use
	uint8_t yPos;

    // Exit if the function would be writing to an index that is off of the screen
	if (x > 127 || y > 63) {
        return;
    }

    // Calculate the y position
	yPos = 7 - (y / 8);

    // Write white pixels if the point isn't inverted
	if(color) {
        OLEDBuffer[x][yPos] |= (1<<(7-(y%8)));
    }

    // Otherwise, write black pixels to the point
	else {
        OLEDBuffer[x][yPos] &= ~(1<<(7-(y%8)));
    }
}


// Fill an area of pixels, starting at x1 or y1 to x2 or y2
void fillOLED(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_COLOR color, bool updateScreen) {

    // Loop through all of the X points
	for(uint8_t x = x1; x <= x2; x++) {

        // Loop through all of the Y points
		for(uint8_t y = y1; y <= y2; y++) {
            setOLEDPixel(x, y, color);
        }
	}

    // Write the buffer to the screen if specified
    if (updateScreen) {
	    writeOLEDBuffer();
    }
}


// Writes a characer to the display
void writeOLEDChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t fontSize, OLED_COLOR color, bool updateScreen) {
	
    // Create the variables to be used
    uint8_t pixelData;
	uint8_t y0 = y;

    // Correct the character index
	chr = chr - ' ';

    // Loop through the data of each of the pixels of the character
    for (uint8_t pixelIndex = 0; pixelIndex < fontSize; pixelIndex++) {

        // Use the appropriate font data
		if (fontSize == 12) {
            pixelData = OLED_1206_Font[chr][pixelIndex];
        }
		else {
            pixelData = OLED_1608_Font[chr][pixelIndex];
        }

        // Loop through the bits, setting them on the buffer array
        for(uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {

            // Check if the pixel should be inverted (as compared to what was set)
			if (pixelData & 0x80) {
                setOLEDPixel(x, y, color);
            }
			else {
                setOLEDPixel(x, y, OLED_COLOR(!color));
            }

            // Shift all of the bits to the left, bringing the next bit in for the next cycle
			pixelData <<= 1;

            // Move down in the y direction so two pixels don't overwrite eachother
			y++;

            // When the character vertical's vertical column is filled, move to the next one
			if((y-y0) == fontSize) {
				y = y0;
				x++;
				break;
			}
		}
    }

    // Update the screen if desired
    if (updateScreen) {
        writeOLEDBuffer();
    }
	
}


// Writes a number to the OLED display
void writeOLEDNum(uint8_t x, uint8_t y, uint32_t number, uint8_t len, uint8_t fontSize, bool updateScreen) {

    // Create variables
	uint8_t digit;
	uint8_t enshow = 0;

    // Loop through each of the digits
	for (uint8_t digitIndex = 0; digitIndex < len; digitIndex++) {

        // Choose the digit out of the number
		digit = (number / (uint8_t)pow(10, len - (digitIndex + 1))) % 10;

        // Display the digit if everything is good
		if(enshow == 0 && digitIndex < (len-1)) {
			if(digit == 0) {
				writeOLEDChar(x+(fontSize/2)*digitIndex, y, ' ', fontSize, WHITE, false);
				continue;
			}
            else {
                enshow = 1;
            }
		}

        // Write out the character
	 	writeOLEDChar(x+(fontSize/2)*digitIndex, y, digit + '0', fontSize, WHITE, false);
	}

    // Update the screen if specified
    if (updateScreen) {
        writeOLEDBuffer();
    }
}


// Function to write a string to the screen
void writeOLEDString(uint8_t x, uint8_t y, const char *p, bool updateScreen) {

    // Check if we haven't reached the end of the string
    while(*p != '\0') {

        // Make sure that the x and y don't exceed the maximum indexes
        if(x > MAX_CHAR_POSX || y > MAX_CHAR_POSY){
            break;
        }

        // Display the character on the screen
        writeOLEDChar(x, y, *p, 16, WHITE, false);

        // Move over by half the font size
        x += (16 / 2);

        // Move to the next character
        p++;
    }

    // Update the screen if specified
    if (updateScreen) {
        writeOLEDBuffer();
    }
}


// Convience function for writing a specific string to the OLED panel
void writeOLEDString(uint8_t x, uint8_t y, String string, bool updateScreen) {
    writeOLEDString(x, y, string.c_str(), updateScreen);
}

#endif // ! USE_OLED