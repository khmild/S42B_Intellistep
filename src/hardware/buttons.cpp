// Import the config (needed for the ENABLE_OLED define)
#include "config.h"

// Imports
#include "buttons.h"
#include "config.h"
#include "Arduino.h"
#include "oled.h"

// Variable definitions
// Boolean for storing if the dip switches were installed the wrong way
bool dipInverted = false;
uint32_t lastButtonClickTime = 0;

// Initialize the button pins as inputs
void initButtons() {

  // Only build if specified
  #ifdef ENABLE_OLED

    // Down pin (moves the menu down)
    pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);

    // Select pin (selects the current menu item)
    pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);

    // Back pin (opens menu and also backs out of menus)
    pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);
  #endif

  // All of the dip switches
  pinMode(DIP_1_PIN, INPUT_PULLUP);
  pinMode(DIP_2_PIN, INPUT_PULLUP);
  pinMode(DIP_3_PIN, INPUT_PULLUP);
  pinMode(DIP_4_PIN, INPUT_PULLUP);

  // Read the microstepping of the dip switches
  readDipMicrostepping();
}

// Only include button code if using the OLED panel
#ifdef ENABLE_OLED

// Scan each of the buttons
void checkButtons(bool updateScreen, bool onlyAllowSelect) {

  // Make sure that the buttons don't repeat too fast
  if (millis() - lastButtonClickTime > BUTTON_REPEAT_INTERVAL) {

    // Check the select buttons
    if(checkButtonState(SELECT_BUTTON_PIN)) {

      // Update the last click time
      lastButtonClickTime = millis();

      // Select the current menu item
      selectMenuItem();
    }

    // Check the down button
    else if(checkButtonState(DOWN_BUTTON_PIN)) {

      // Update the last click time
      lastButtonClickTime = millis();

      // Move down
      if (!onlyAllowSelect) {
        moveCursor();
      }
    }

    // Check the back button
    else if(checkButtonState(BACK_BUTTON_PIN)) {

      // Update the last click time
      lastButtonClickTime = millis();

      // Back up
      if (!onlyAllowSelect) {
        exitCurrentMenu();
      }
    }
  }
}

// Check to see if a button is clicked
bool checkButtonState(PinName buttonPin) {

  // Check to see if the select button is clicked (pins are pulled high by default)
  if(digitalReadFast(buttonPin) == LOW){

    // Wait to make sure it wasn't a misread
    delay(10);

    // Check again
    if(digitalReadFast(buttonPin) == LOW){

      // The button is still clicked, it's valid
      return true;

    }
    else {

      // The button is no longer clicked, probably just a misread
      return false;
    }

  }
  else {

    // Button was never clicked
    return false;
  }
}
#endif // ! ENABLE_OLED

// Function for reading the microstepping set via the dip switches
void readDipMicrostepping() {

    // Disable interrupts during the check, it cannot be interrupted
    disableInterrupts();

    // Check if the dip switches are inverted
    if (dipInverted) {

        // If they were installed incorrectly, they have to be read opposite
        if (!digitalReadFast(DIP_4_PIN) && !digitalReadFast(DIP_3_PIN)) {

            // Set the microstepping to 1/32 if both dips are on
            motor.setMicrostepping(32);
        }
        else if (digitalReadFast(DIP_4_PIN) && !digitalReadFast(DIP_3_PIN)) {

            // Set the microstepping to 1/16 if the left dip is off and the right is on
            motor.setMicrostepping(16);
        }
        else if (!digitalReadFast(DIP_4_PIN) && digitalReadFast(DIP_3_PIN)) {

            // Set the microstepping to 1/8 if the right dip is off and the left on
            motor.setMicrostepping(8);
        }
        else {
            // Both are off, just revert to using full stepping
            motor.setMicrostepping(1);
        }
    }
    else {
        // Dips are not inverted, they are installed correctly
        if (!digitalReadFast(DIP_1_PIN) && !digitalReadFast(DIP_2_PIN)) {

            // Set the microstepping to 1/32 if both dips are on
            motor.setMicrostepping(32);
        }
        else if (digitalReadFast(DIP_1_PIN) && !digitalReadFast(DIP_2_PIN)) {

            // Set the microstepping to 1/16 if the left dip is off and the right is on
            motor.setMicrostepping(16);
        }
        else if (!digitalReadFast(DIP_1_PIN) && digitalReadFast(DIP_2_PIN)) {

            // Set the microstepping to 1/8 if the right dip is off and the left on
            motor.setMicrostepping(8);
        }
        else {
            // Both are off, just revert to using full stepping
            motor.setMicrostepping(1);
        }
    }

    // Update the timer based on the new microstepping
    updateCorrectionTimer();

    // All work is done, we can re-enable interrupts
    enableInterrupts();
}


// Check all of the dip switches
void checkDips() {

  // Read the microstepping
  readDipMicrostepping();

  // Adjust based on if inverted
  if (dipInverted) {

    // Check open/closed loop
    if (!digitalReadFast(DIP_2_PIN)) {
      enableStepCorrection();
    }
    else {
      disableStepCorrection();
    }
  }
  else {
    // Check open/closed loop
    if (!digitalReadFast(DIP_3_PIN)) {
      enableStepCorrection();
    }
    else {
      disableStepCorrection();
    }
  }
}


// Function for setting if the dip switches should be inverted
void setDipInverted(bool inverted) {
    dipInverted = inverted;
}

// Function for getting if the dip switches should be inverted
bool getDipInverted() {
    return dipInverted;
}