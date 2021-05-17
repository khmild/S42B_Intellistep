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

    // Set up the HAL library
    HAL_Init();

    // Set processor up
    SystemInit();

    // Setup the system clock (includes overclocking)
    overclock(RCC_CFGR_PLLMULL16);

    // Initialize the LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize the encoder
    initEncoder();

    // Setup the motor for use
    motor.enable(true);
    //motor.setMicrostepping(16);
    //motor.setDesiredAngle(100);

    // Only run if the OLED is enabled
    #ifdef USE_OLED

        // Initialize the OLED
        initOLED();

        // Show the bootscreen
        showBootscreen();

        // Wait for 3 seconds so everything can boot and user can read the LCD
        delay(3000);

        // Initialize the buttons (for menu)
        initButtons();
    #endif

    // Initialize the serial bus
    #ifdef USE_SERIAL
        initSerial();
    #endif

    // Initialize the CAN bus
    #ifdef USE_CAN
        // Initialize the CAN bus
        initCAN();
    #endif

    // Test the flash if specified
    //#ifdef TEST_FLASH
    //    flash_test();
    //#endif

    // Clear the display, then write that we're using the closed loop mode
    //clearOLED();
    //writeOLEDString(0, 0, "Close Loop Mode");

    // Check if the board is calibrated. Need to force calibration if the board isn't calibrated
    if (!isCalibrated()) {

        // Only display to screen if the screen is enabled
        #ifdef USE_OLED

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
        #endif

        // Continuously check to see if the select key is clicked (depth index would increase when clicked)
        while(true) {

            // Only if serial is specified
            #ifdef USE_SERIAL
                // ! Only here for testing
                runSerialParser();
            #endif

            #ifdef USE_OLED
                // Check to see if any of the buttons are pressed
                checkButtons(false);
            #endif

            // ! Only for testing
            //blink();

            // Only if the OLED is needed
            #ifdef USE_OLED

                // Check to see if the menu button has been clicked
                if (getMenuDepth() > 0) {

                    // Calibrate the motor
                    motor.calibrate();

                    // Reboot the chip
                    NVIC_SystemReset();
                }
 
            #else

                // Just jump to calibrating the motor and reset the system afterward
                motor.calibrate();
                NVIC_SystemReset();

            #endif
        }
    }
    else {
        // There is a calibration, load it and move on to the loop

        // Load the values from flash
        //loadSavedValues();

        #ifdef USE_OLED

            // Let the user know that the calibration was successfully loaded
            clearOLED();
            writeOLEDString(0, 0, "Calibration", false);
            writeOLEDString(0, 16, "OK!", true);
            delay(1000);
            
            // Write out the first data to the screen (makes sure that the first write isn't interrupted)
            displayMotorData();
        #endif

        // Setup the motor timers and interrupts
        setupMotorTimers();
        
        //uint8_t count = 0;

        // Loop forever, checking the keys and updating the display
        while(true) {

            /*
            while(count < 16) {
                motor.step(CLOCKWISE);
                displayMotorData();
                delay(5);
                count++;
            }

            while (count > 0) {
                motor.step(COUNTER_CLOCKWISE);
                displayMotorData();
                delay(5);
                count--;
            }
            */

            // Check to see if serial data is available to read
            #ifdef USE_SERIAL
                runSerialParser();
            #endif

            #ifdef USE_OLED
                // Check the buttons
                checkButtons(true);

                // Only update the display if the motor data is being displayed, buttons update the display when clicked
                if (getMenuDepth() == MOTOR_DATA) {
                    displayMotorData();
                }
            #endif

            // We need a little delay to allow the motor time to process if it needs it
            delay(50);
        }
    }
}


// Main loop
void loop() {
    blink();
}


// Overclocks the processor to the desired value
void overclock(uint32_t PLLMultiplier) {

    // Tune the HSI
    //HSI_CalibrateMinError();

    // Initialization structure
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = PLLMultiplier;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Wait for the PLL to be configured
    while(!(RCC_CR_PLLRDY & RCC -> CR));

    // Use the PLL as the system clock
    RCC -> CFGR |= RCC_SYSCLKSOURCE_PLLCLK;

    // Update the system clock with the new speed
    SystemCoreClockUpdate();
}


// ! Only here for testing
void blink() {
    digitalWriteFast(LED_PIN, HIGH);
    delay(500);
    digitalWriteFast(LED_PIN, LOW);
    delay(500);
}