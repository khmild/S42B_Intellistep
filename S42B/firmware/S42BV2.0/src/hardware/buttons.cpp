#include "buttons.h"
#include "pinMapping.h"
#include "Arduino.h"
#include "oled.h"

// Initialize the button pins as inputs
void initButtons() {

  // Down pin (moves the menu down)
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);

  // Select pin (selects the current menu item)
  pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);

  // Back pin (opens menu and also backs out of menus)
  pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);

}

// Scan each of the buttons
void checkButtons(void) {

  // Check the select buttons
  if(checkButtonState(SELECT_BUTTON_PIN)) {
    
    // Select the current menu item
    selectMenuItem();
  }

  // Check the down button
  else if(checkButtonState(DOWN_BUTTON_PIN)) {

    // Move down
    moveCursor();

  }
  // Check the back button
  else if(checkButtonState(BACK_BUTTON_PIN)) {

    // Back up
    exitCurrentMenu();
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