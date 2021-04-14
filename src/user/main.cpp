/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "buttons.h"
#include "canMessaging.h"
#include "serial.h"
#include "flash.h"
#include "encoder.h"
#include "oled.h"
#include "stm32yyxx_ll_rcc.h"

// Create a new motor instance
StepperMotor motor = StepperMotor();

// Run the setup
void setup() {

    // Set processor up
    SystemInit();

    // Setup the system clock (includes overclocking)
    overclock(RCC_CFGR_PLLMULL16);

    // Initialize the LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize the OLED
    initOLED();

    // Give the user feeback that OLED starting has been successful
    writeOLEDString(0, 0, "Oled Init...OK", true);

    // Wait for 3 seconds so everything can boot and user can read the LCD
    delay(3000);

    // Initialize the buttons (for menu)
    initButtons();

    // Initialize the serial bus
    initSerial();

    // Initialize the CAN bus
    initCAN();

    // Initialize the encoder
    initEncoder();

    // Setup the motor for use
    motor.enable();

    // Test the flash if specified
    //#ifdef TEST_FLASH
    //    flash_test();
    //#endif

    // Clear the display, then write that we're using the closed loop mode
    //clearOLED();
    //writeOLEDString(0, 0, "Close Loop Mode");

    // Check if the board is calibrated. Need to force calibration if the board isn't calibrated
    if (!isCalibrated()) {

        // Display that the motor is not calibrated
        clearOLED();
        writeOLEDString(0, 0, "NOT", false);
        writeOLEDString(0, 16, "Calibrated!", false);
        writeOLEDString(0, 32, "Please calibrate", true);
        delay(3000);

        // Display that the select key can be clicked to run calibration
        clearOLED();
        writeOLEDString(0, 0, "Use the", false);
        writeOLEDString(0, 16, "select key", false);
        writeOLEDString(0, 32, "to calibrate", true);

        // Continuously check to see if the select key is clicked (depth index would increase when clicked)
        while(true) {

            // ! Only here for testing
            runSerialParser();

            // ! Only for testing
            //blink();

            // Check to see if any of the buttons are pressed
            checkButtons(false);

            // Check to see if the menu button has been clicked
            if (getMenuDepth() > 0) {

                // Calibrate the motor
                motor.calibrate();

                // Reboot the chip
                NVIC_SystemReset();
            }
        }
    }
    else {
        // There is a calibration, load it and move on to the loop

        // Load the values from flash
        loadSavedValues();

        // Let the user know that the calibration was successfully loaded
        clearOLED();
        writeOLEDString(0, 0, "Calibration", false);
        writeOLEDString(0, 16, "OK!", true);
        delay(1000);

        // Setup the motor timers and interrupts
        // ! setupMotorTimers();

        // Loop forever, checking the keys and updating the display
        while(true) {

            // Check to see if serial data is available to read
            runSerialParser();

            // Check the buttons
            checkButtons(true);

            // Only update the display if the motor data is being displayed, buttons update the display when clicked
            if (getMenuDepth() == 0) {
                displayMotorData();
            }
        }
    }
}


// Main loop
void loop() {
    blink();
}


// Overclocks the processor to the desired value
void overclock(uint32_t PLLMultiplier) {

    // Use PLL as the system clock instead of the HSE (the board's oscillator)
    RCC -> CFGR |= RCC_CFGR_PLLSRC;

    // Activate the HSE (board's oscillator)
    RCC -> CR |= RCC_CR_HSEON;

    // Set the multiplier to desired
    RCC -> CFGR |= PLLMultiplier;

    // Set the HSE to half speed before the PLL (so effectively multiplier/2 speed overall)
    RCC -> CFGR |= RCC_CFGR_PLLXTPRE_HSE;

    // Activate the PLL
    RCC -> CR |= RCC_CR_PLLON;

    // Wait until the PLL is configured
    while(!(RCC_CR_PLLRDY & RCC -> CR));

    // Use the PLL as the system clock
    RCC -> CFGR |= RCC_CFGR_SW_PLL;

    // Update the system clock with the new speed
    SystemCoreClockUpdate();
}


// ! Only here for testing
void blink() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}