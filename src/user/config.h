#ifndef __CONFIG_H
#define __CONFIG_H

// Need the Arduino library for pin conversion
#include "Arduino.h"

// Version of the firmware (displayed on OLED) (follows semantic versioning)
#define VERSION "0.0.27"


// --------------  Settings  --------------

// Main feature enable
#define ENABLE_OLED
#define ENABLE_SERIAL
#define ENABLE_CAN

// Averages (number of readings in average)
#define RPM_AVG_READINGS     10
#define ACCEL_AVG_READINGS   10
#define ANGLE_AVG_READINGS   15
#define TEMP_AVG_READINGS    200

// If encoder estimation should be used
#define ENCODER_SPEED_ESTIMATION
#ifdef ENCODER_SPEED_ESTIMATION
    #define SPD_EST_MIN_INTERVAL 500 // The minimum sampling interval (us). Increase to get more steady readings at the cost of latency
#endif

// Serial configuration settings
#ifdef ENABLE_SERIAL
    #define STRING_START_MARKER '<'
    #define STRING_END_MARKER '>'
    #define SERIAL_BAUD 115200
#endif

// The CAN ID of this board
// X:2, X2:3...
// Y:7, Y2:8... 
// Z:11 Z2:12...
// E:17, E1:18...
#ifdef ENABLE_CAN
    #define DEFAULT_CAN_ID X
    #define CAN_BITRATE BR125K
#endif

// Motor characteristics
#define STEP_ANGLE 1.8 // ! Check to see for .9 deg motors as well
#define STEP_UPDATE_FREQ 78 // in Hz, to step the motor back to the correct position. Multiplied by the microstepping for actual update freq
#define MAX_MOTOR_SPEED 50 // deg/s

// Board characteristics
// ! Do not modify unless you know what you are doing!
#define BOARD_VOLTAGE           3.3 // The voltage of the main processor
#define CURRENT_SENSE_RESISTOR  0.2 // Value of the board's current calculation resistor. An incorrect value here will cause current inaccuracies
#define MAX_PEAK_BOARD_CURRENT  3500 // Maximum peak current in mA that the board can manage

// Motor settings
//#define ENABLE_DYNAMIC_CURRENT
#ifdef ENABLE_DYNAMIC_CURRENT
    // A dynamically controller current loop. Uses the equation: accel * accelCurrent + idleCurrent
    // Limited by the max dynamic current, which will limit the maximum that the dynamic loop can output
    // All current values are in RMS
    #define DYNAMIC_ACCEL_CURRENT 10 // Multiplied by deg/s/s, in mA
    #define DYNAMIC_IDLE_CURRENT  500 // In mA
    #define DYNAMIC_MAX_CURRENT   750 // In mA
#else
    // Classic, static current
    #define STATIC_RMS_CURRENT     500 // This is the rating of the motor from the manufacturer

    // Overtemperature protection (lowers motor current when motor temperature rises too high)
    #define ENABLE_OVERTEMP_PROTECTION
    #ifdef ENABLE_OVERTEMP_PROTECTION
        #define OVERTEMP_THRESHOLD_TEMP      70 // The temp to trigger a overtemp current reduction (C)
        #define OVERTEMP_INCREMENT           50 // The increment at which to reduce the current by (RMS mA)
        #define OVERTEMP_INTERVAL            30 // The minimum interval between current reductions (s)
        #define OVERTEMP_SHUTDOWN_TEMP       80 // The temp at which to completely shut down the motor, protecting it against burning up
        #define OVERTEMP_SHUTDOWN_CLEAR_TEMP 70 // Motor can begin movement again once this temp is reached
    #endif
#endif
#define MICROSTEP_MULTIPLIER    2 // The number of microsteps to move per step pulse
#define MIN_MICROSTEP_DIVISOR   1 // The minimum microstepping divisor
#define MAX_MICROSTEP_DIVISOR   32 // The maximum microstepping divisor
#define MOTOR_PWM_FREQ          50 // in kHz
#define IDLE_MODE               COAST // The mode to set the motor to when it's disabled

// Stallfault
//#define ENABLE_STALLFAULT
#ifdef ENABLE_STALLFAULT
    #define STEP_FAULT_TIME         1 // The maximum allowable time (sec) for a step fault (meaning motor is out of position)
    #define STEP_FAULT_ANGLE        10 // The maximum allowable deviation between the actual and set angles before StallFault is triggered

    // StallFault connection (to mainboard)
    // Pulls high on a stepper misalignment after the set period or angular deviation
    #define STALLFAULT_PIN  PA_13
#endif

// OLED settings
#ifdef ENABLE_OLED

    // Button settings
    //#define INVERTED_DIPS // Enable if your dips are inverted ("on" print is facing away from motor connector)
    #define BUTTON_REPEAT_INTERVAL 250 // Millis
    #define MENU_RETURN_LEVEL MOTOR_DATA // The level to return to after configuring a setting
    #define WARNING_MICROSTEP 32 // The largest microstep to warn on (the denominator of the fraction)
    
    // Warning thresholds
    #define WARNING_RMS_CURRENT 1000 // The RMS current at which to display a warning confirmation (mA)
    //#define WARNING_PEAK_CURRENT 1000 // The peak current at which to display a warning confirmation (mA)
#endif


// --------------  Pins  --------------
/*
// * = F103C8-CB    | DIGITAL | ANALOG | USART      | TWI       | SPI        | SPECIAL   |
//                  |---------|--------|------------|-----------|------------|-----------|
#define PA0  A0  // | 0       | A0     |            |           |            |           |
#define PA1  A1  // | 1       | A1     |            |           |            |           |
#define PA2  A2  // | 2       | A2     | USART2_TX  |           |            |           |
#define PA3  A3  // | 2       | A2     | USART2_RX  |           |            |           |
#define PA4  A4  // | 4       | A4     |            |           | SPI1_SS    |           |
#define PA5  A5  // | 5       | A5     |            |           | SPI1_SCK   |           |
#define PA6  A6  // | 6       | A6     |            |           | SPI1_MISO  |           |
#define PA7  A7  // | 7       | A7     |            |           | SPI1_MOSI  |           |
#define PA8  8   // | 8       |        |            |           |            |           |
#define PA9  9   // | 9       |        | USART1_TX  |           |            |           |
#define PA10 10  // | 10      |        | USART1_RX  |           |            |           |
#define PA11 11  // | 11      |        |            |           |            | USB_DN    |
#define PA12 12  // | 12      |        |            |           |            | USB_DP    |
#define PA13 13  // | 13      |        |            |           |            | SWD_SWDIO |
#define PA14 14  // | 14      |        |            |           |            | SWD_SWCLK |
#define PA15 15  // | 15      |        |            |           | SPI1_SS    |           |
//                  |---------|--------|------------|-----------|------------|-----------|
#define PB0  A8  // | 16      | A8     |            |           |            |           |
#define PB1  A9  // | 17      | A9     |            |           |            |           |
#define PB2  18  // | 18      |        |            |           |            | BOOT1     |
#define PB3  19  // | 19      |        |            |           | SPI1_SCK   |           |
#define PB4  20  // | 20      |        |            |           | SPI1_MISO  |           |
#define PB5  21  // | 21      |        |            |           | SPI1_MOSI  |           |
#define PB6  22  // | 22      |        | USART1_TX  | TWI1_SCL  |            |           |
#define PB7  23  // | 23      |        | USART1_RX  | TWI1_SDA  |            |           |
#define PB8  24  // | 24      |        |            | TWI1_SCL  |            |           |
#define PB9  25  // | 25      |        |            | TWI1_SDA  |            |           |
#define PB10 26  // | 26      |        | USART3_TX* | TWI2_SCL* |            |           |
#define PB11 27  // | 27      |        | USART3_RX* | TWI2_SDA* |            |           |
#define PB12 28  // | 28      |        |            |           | SPI2_SS*   |           |
#define PB13 29  // | 29      |        |            |           | SPI2_SCK*  |           |
#define PB14 30  // | 30      |        |            |           | SPI2_MISO* |           |
#define PB15 31  // | 31      |        |            |           | SPI2_MOSI* |           |
//                  |---------|--------|------------|-----------|------------|-----------|
#define PC13 32  // | 32      |        |            |           |            |           |
#define PC14 33  // | 33      |        |            |           |            | OSC32_IN  |
#define PC15 34  // | 34      |        |            |           |            | OSC32_OUT |
//                  |---------|--------|------------|-----------|------------|-----------|
#define PD0  35  // | 35      |        |            |           |            | OSC_IN    |
#define PD1  36  // | 36      |        |            |           |            | OSC_OUT   |
//                  |---------|--------|------------|-----------|------------|-----------|
*/


// OLED Mappings
#define OLED_CS_PIN   output(GPIOB_BASE, 12)
#define OLED_RST_PIN  output(GPIOA_BASE, 8)
#define OLED_RS_PIN   output(GPIOB_BASE, 13)
#define OLED_SCLK_PIN output(GPIOB_BASE, 15)	//(D0)
#define OLED_SDIN_PIN output(GPIOB_BASE, 14)	//(D1)

// Button mappings
#define DOWN_BUTTON_PIN         PA_3
#define BACK_BUTTON_PIN         PB_0
#define SELECT_BUTTON_PIN       PB_1

// Dip switch mappings
// PIN     |    Normal orientation      |  Reversed orientation
// DIP_1   |    Microstep 1             |  Calibration mode
// DIP_2   |    Microstep 2             |  Closed/Open loop
// DIP_3   |    Closed/Open loop        |  Microstep 2
// DIP_4   |    Calibration mode        |  Microstep 1
#define DIP_1_PIN  PA_15
#define DIP_2_PIN  PB_3
#define DIP_3_PIN  PB_11
#define DIP_4_PIN  PB_10

// LED pin
#define LED_PIN PC_13

// Motor mappings                                   [  A  ,   B  ]
static const PinName COIL_DIR_1_PINS[]           =  { PB_6, PB_8 };
static const PinName COIL_DIR_2_PINS[]           =  { PB_7, PB_9 };
static const PinName COIL_POWER_OUTPUT_PINS[]    =  { PB_5, PB_4 };

// Encoder SPI interface
#define ENCODER_CS_PIN    PA_4 // SPI1_SS
#define ENCODER_SCK_PIN   PA_5 // SPI1_SCK
#define ENCODER_MISO_PIN  PA_6 // SPI1_MISO
#define ENCODER_MOSI_PIN  PA_7 // SPI1_MOSI

// Stepping interface
#define STEP_PIN       PA_0
#define ENABLE_PIN     PA_2
#define DIRECTION_PIN  PA_1

// CAN bus pins
#define CAN_IN_PIN   PA_11
#define CAN_OUT_PIN  PA_12


// --------------  Internal defines  --------------
// Under the hood motor setup
#define SINE_VAL_COUNT (128)
#define SINE_MAX ((int16_t)(10000))

#define IS_POWER_2(N) (N & (N-1)) // Return 0 if a number is a power of 2.
#if IS_POWER_2(SINE_VAL_COUNT) != 0
    #error SINE_VAL_COUNT must be a power of 2 to use in fastSin() and fastCos() defines!!!
#endif

// Bitwise memory modification
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

// Low level GPIO configuration (for quicker manipulations than digitalWrites)
#define output(GPIO_BASE, n)   BIT_ADDR((GPIO_BASE + 12),n)
#define input(GPIO_BASE, n)    BIT_ADDR((GPIO_BASE + 8),n)

// Maybe an even faster digitalWrite, but only if really needed
/*
#define digitalWriteFaster(PinName pn, bool high) \
    if (high) { \
        WRITE_REG(get_GPIO_Port(STM_PORT(pn))->BSRR, (STM_LL_GPIO_PIN(pn) >> GPIO_PIN_MASK_POS) & 0x0000FFFFU); \
    } \
    else { \
        WRITE_REG(get_GPIO_Port(STM_PORT(pn))->BRR, (STM_LL_GPIO_PIN(pn) >> GPIO_PIN_MASK_POS) & 0x0000FFFFU); \
    }
*/

// --------------  Debugging  --------------
//#define TEST_FLASH

// Import the sanity check (needed so all files have the defines done in the sanity check file)
// Must be last so that it can use the defines above
#include "sanityCheck.h"

#endif //__CONFIG_H
