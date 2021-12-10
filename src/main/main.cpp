/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "buttons.h"
#include "canMessaging.h"
#include "serial.h"
#include "flash.h"
#include "encoder.h"
#include "oled.h"
#include "led.h"
#include "cube.h"

// Create a new motor instance
StepperMotor motor = StepperMotor();

// Create a variable for storing the starting time
// Used for timing display updates to be smooth regardless of processsing time
uint32_t startTime = 0;

// Run the setup
void setup() {

    // Resets of all peripherals, initializes the Flash interface and the Systick timer.
    HAL_Init();

    // Set processor up
    SystemInit();

    // Set priority grouping explicitly to 3.1(PRIO.SUBPRIO)
    // Possible value for PRIO from 0 to 7 and for SUBPRIO from 0 to 1 in setInterruptPriority()
    NVIC_SetPriorityGrouping(3);

    // Configure the system clock
    #if defined(SYSCLK_SRC_HSE_16)
        #if SYSCLK_FREQ == 72
            SystemClock_Config_HSE_16M_SYSCLK_72M();
        #else
            #error "Unsupported oscillator speed"
        #endif
    #elif defined(SYSCLK_SRC_HSE_8)
        #if SYSCLK_FREQ == 72
            SystemClock_Config_HSE_8M_SYSCLK_72M();
        #elif SYSCLK_FREQ == 128
            SystemClock_Config_HSE_8M_SYSCLK_128M();
        #else
            #error "Unsupported oscillator speed"
        #endif
    #elif defined(SYSCLK_SRC_HSI)
        #if SYSCLK_FREQ == 64
            SystemClock_Config_HSI_8M_SYSCLK_64M();
        #else
            #error "Unsupported oscillator speed"
        #endif
    #else
	    #error "Unsupported oscillator source"
    #endif

    #ifdef CHECK_MCO_OUTPUT
        MCO_GPIO_Init();
    #endif

    // Encoder speed debugging
    #ifdef CHECK_ENCODER_SPEED
        while(true) {
            GPIO_WRITE(LED_PIN, HIGH);
            motor.encoder.getRawAngleAvg();
            GPIO_WRITE(LED_PIN, LOW);
            delayMicroseconds(1);
        }
    #endif

    // Debugging for GPIO output switching
    #ifdef CHECK_GPIO_OUTPUT_SWITCHING
        PA_8_GPIO_Init();
        while(true) {
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);
            GPIO_WRITE(PA_8, HIGH);

            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
            GPIO_WRITE(PA_8, LOW);
        }
    #endif

    // Initialize the LED
    #ifdef ENABLE_LED
        initLED();
    #endif

    // Only run if the OLED is enabled
    #ifdef ENABLE_OLED

        // Initialize the OLED
        initOLED();

        // Show the bootscreen
        showBootscreen();

        // Wait for 3 seconds so everything can boot and user can read the LCD
        delay(3000);

        // Set the inversion (if specified)
        #ifdef INVERTED_DIPS
            setDipInverted(true);
        #else
            setDipInverted(false);
        #endif

        // Initialize the buttons (for menu)
        initButtons();
    #endif

    // Initialize the serial bus
    #ifdef ENABLE_SERIAL
        initSerial();
    #endif

    // Initialize the CAN bus
    #ifdef ENABLE_CAN
        // Initialize the CAN bus
        initCAN();
    #endif

    // Clear the display, then write that we're using the closed loop mode
    //clearOLED();
    //writeOLEDString(0, 0, "Close Loop Mode");

    // Check if the board is calibrated. Need to force calibration if the board isn't calibrated
    if (!isCalibrated()) {

        // Only needed for OLED changes
        #ifdef ENABLE_OLED
        // Note the start time
        uint32_t startTime = getCurrentMillis();

        // The currently displayed screen. It doesn't make sense to write the same screen over and over
        // Screen 0 is the first message, screen 1 the second. We init the screen index to 1 so the first
        // cycle through the logic will write the message to the screen.
        uint8_t screenIndex = 1;
        #endif // ! ENABLE_OLED

        // Continuously check to see if the select key is clicked (depth index would increase when clicked)
        while(true) {

            // Only display to screen if the screen is enabled
            #ifdef ENABLE_OLED

            // Calculate the elapsed time (should make the if comparisions faster and more consistent)
            uint32_t elapsedTime = (getCurrentMillis() - startTime);

            // Decide which screen to display (first check if the timer needs reset)
            if (elapsedTime > (2 * CALIBRATION_DISPLAY_TIME)) {

                // We exceeded the period of the screen switches, update the starting time for the cycle
                startTime = getCurrentMillis();
            }
            else if ((elapsedTime < CALIBRATION_DISPLAY_TIME) && screenIndex == 1) {
                // Display that the motor is not calibrated
                clearOLED();
                writeOLEDString(0, 0,               F("NOT"), false);
                writeOLEDString(0, LINE_HEIGHT * 1, F("Calibrated!"), false);
                writeOLEDString(0, LINE_HEIGHT * 2, F("Please"), false);
                writeOLEDString(0, LINE_HEIGHT * 3, F("calibrate"), true);
                screenIndex = 0;
            }
            // We know that we should be on the second screen, so we only need to check the screenIndex
            else if ((elapsedTime > CALIBRATION_DISPLAY_TIME) && screenIndex == 0) {
                // Display that the select key can be clicked to run calibration
                clearOLED();
                writeOLEDString(0, 0,               F("Use the"), false);
                writeOLEDString(0, LINE_HEIGHT * 1, F("select key"), false);
                writeOLEDString(0, LINE_HEIGHT * 2, F("to calibrate"), false);
                writeOLEDString(0, LINE_HEIGHT * 3, F("Requires power"), true);
                screenIndex = 1;
            }

            // Check to see if any of the buttons are pressed
            checkButtons(false, true);

            // Check to see if the menu button has been clicked
            if (getMenuDepth() > 0) {

                // Calibrate the motor (board reboots afterward)
                // Reboot the chip
                motor.calibrate();
            }

            #else // ! ENABLE_OLED

            // Just jump to calibrating the motor (board reboots afterward)
            // Reboot the chip
            motor.calibrate();
            #endif // ! ENABLE_OLED

            // ! Only for testing
            //blink();
        }
    }
    else {
        // There is a calibration, load it and move on to the loop

        // Only include the extensive flash printout if the OLED is enabled
        #ifdef ENABLE_OLED

            // Let the user know that the calibration was successfully loaded
            clearOLED();
            writeOLEDString(0, 0,               F("Calibration"), false);
            writeOLEDString(0, LINE_HEIGHT * 1, F("OK!"), false);

            // Write base string for flash loading
            writeOLEDString(0, LINE_HEIGHT * 2, F("Flash load"), false);

            // Attempt to load the parameters from flash
            if (loadParameters() == FLASH_LOAD_SUCCESSFUL) {
                writeOLEDString(0, LINE_HEIGHT * 3, F("successful"), true);
            }
            else {
                writeOLEDString(0, LINE_HEIGHT * 3, F("failed"), true);
            }

            // Note the starting time for the calibration message
            startTime = getCurrentMillis();
        #else
            // Nothing special, just try to load the flash data
            loadParameters();

            // We don't need to do anything with the startTime because the 0 it was initialized to will be way out of range for a delay
        #endif

        // Setup the motor timers and interrupts
        setupMotorTimers();
    }

    // Setup the motor for use (should be enabled at startup)
    // The jump when power is on
    //motor.setState(ENABLED, true);

    // Delay for move to power on position
    // Needs the motor timers
    delay(MOTOR_SETTLE_TIME);

    // Zero the encoder after motor is enabled
    motor.encoder.zero();

    // Only need to display info if OLED is enabled
    #ifdef ENABLE_OLED
    // Let the user read the message
    delay(POWERUP_DISPLAY_TIME - (startTime - getCurrentMillis()));

    // Clear the display
    clearOLED();

    // Write out the first data to the screen (makes sure that the first write isn't interrupted)
    displayMotorData();
    #endif
}


// Main loop
void loop() {

    // Note the loop start time
    uint32_t startTime = getCurrentMillis();

    // Check the dip switches
    checkDips();

    // Check to see if serial data is available to read
    #ifdef ENABLE_SERIAL
        runSerialParser();
    #endif

    #ifdef ENABLE_OLED
        // Check the buttons
        checkButtons(true);

        // Only update the display if the motor data is being displayed, buttons update the display when clicked
        if (getMenuDepth() == MOTOR_DATA) {
            displayMotorData();
        }
    #endif

    // If correction is disabled, then the enable pin is never checked
    // That is done here
    #ifndef STEP_CORRECTION
        motor.checkEnablePin();
    #endif

    // We need a little delay to allow the motor time to process if it needs it
    #ifdef ENABLE_BLINK
        // ! Only for testing
        blink();
    #else
        // We should delay the time remaining in the loop
        // We can take the set time for a loop and subtract the already taken time from it
        // This helps to stabilize the IO updates
        delay(MIN_IO_LOOP_TIME - (getCurrentMillis() - startTime)); // ! Maybe remove? This could make the updates much faster
    #endif
}


// ! Only here for testing
void blink() {
    GPIO_WRITE(LED_PIN, HIGH);
    delay(500);
    GPIO_WRITE(LED_PIN, LOW);
    delay(500);
}
