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

  // Down pin (moves the menu down)
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);

  // Select pin (selects the current menu item)
  pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);

  // Back pin (opens menu and also backs out of menus)
  pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);

  // All of the dip switches
  pinMode(DIP_CALIBRATED, INPUT_PULLUP);
  pinMode(DIP_CLOSED_LOOP, INPUT_PULLUP);
  pinMode(DIP_MICRO_1, INPUT_PULLUP);
  pinMode(DIP_MICRO_2, INPUT_PULLUP);

  // Read the microstepping of the dip switches
  readDipMicrostepping();
}

// Scan each of the buttons
void checkButtons(void) {

  // Make sure that the buttons don't repeat too fast
  if (millis() - lastButtonClickTime > BUTTON_REPEAT_INTERVAL) {

    // Check the select buttons
    if(checkButtonState(SELECT_BUTTON_PIN)) {

      // Update the last click time
      lastButtonClickTime = millis();

      // Select the current menu item
      selectMenuItem();

      // Update the display
      updateDisplay();
    }

    // Check the down button
    else if(checkButtonState(DOWN_BUTTON_PIN)) {

      // Update the last click time
      lastButtonClickTime = millis();

      // Move down
      moveCursor();

      // Update the screen
      updateDisplay();
    }

    // Check the back button
    else if(checkButtonState(BACK_BUTTON_PIN)) {

      // Update the last click time
      lastButtonClickTime = millis();

      // Back up
      exitCurrentMenu();

      // Update the screen
      updateDisplay();
    }
  }
}

// Check to see if a button is clicked
bool checkButtonState(uint32_t buttonPin) {

  // Check to see if the select button is clicked (pins are pulled high by default)
  if(digitalRead(buttonPin) == LOW){

    // Wait to make sure it wasn't a misread
    delay(10);

    // Check again
    if(digitalRead(buttonPin) == LOW){

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


// Function for reading the microstepping set via the dip switches
void readDipMicrostepping() {

    // Check if the dip switches are inverted
    if (dipInverted) {

        // If they were installed incorrectly, they have to be read opposite
        if (digitalRead(DIP_MICRO_2) && digitalRead(DIP_MICRO_1)) {

            // Set the microstepping to 1/32 if both dips are on
            motor.setMicrostepping(32);
        }
        else if (!digitalRead(DIP_MICRO_2) && digitalRead(DIP_MICRO_1)) {

            // Set the microstepping to 1/16 if the left dip is off and the right is on
            motor.setMicrostepping(16);
        }
        else if (digitalRead(DIP_MICRO_2) && !digitalRead(DIP_MICRO_1)) {

            // Set the microstepping to 1/8 if the right dip is off and the left on
            motor.setMicrostepping(8);
        }
        else {

            // Both are off, just revert to using quarter stepping
            motor.setMicrostepping(4);
        }
    }
    else {
        // Dips are not inverted, they are installed correctly
        if (digitalRead(DIP_MICRO_1) && digitalRead(DIP_MICRO_2)) {

            // Set the microstepping to 1/32 if both dips are on
            motor.setMicrostepping(32);
        }
        else if (!digitalRead(DIP_MICRO_1) && digitalRead(DIP_MICRO_2)) {

            // Set the microstepping to 1/16 if the left dip is off and the right is on
            motor.setMicrostepping(16);
        }
        else if (digitalRead(DIP_MICRO_1) && !digitalRead(DIP_MICRO_2)) {

            // Set the microstepping to 1/8 if the right dip is off and the left on
            motor.setMicrostepping(8);
        }
        else {

            // Both are off, just revert to using quarter stepping
            motor.setMicrostepping(4);
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