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

// Create a new timer instance
HardwareTimer *stepCorrectionTimer = new HardwareTimer(TIM1);
HardwareTimer *stepSkipCheckTimer = new HardwareTimer(TIM2);

// If the motor should move CCW or CW on next step update
STEP_DIR correctionStepDir = COUNTER_CLOCKWISE;

// Run the setup
void setup() {

    // Set everything up
    SystemInit();

    // Setup the system clock (includes overclocking)
    //overclock(RCC_CFGR_PLLMULL12);
    overclock(RCC_CFGR_PLLMULL16);

    // Initialize the LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Initialize the OLED
    initOLED();

    // Give the user feeback that OLED starting has been successful
    writeOLEDString(0, 0, "Oled Init...OK");

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

    // Initialize the encoder, motor, and the PWM timer
    //encoderInit();
    //motorInit();
    //TIM3_Init();

    // Check if the board is calibrated. Need to force calibration if the board isn't calibrated
    if (!isCalibrated()) {

        // Display that the motor is not calibrated
        clearOLED();
        writeOLEDString(0, 0, "NOT");
        writeOLEDString(0, 16, "Calibrated!");
        writeOLEDString(0, 32, "Please calibrate");
        delay(3000);

        // Display that the select key can be clicked to run calibration
        clearOLED();
        writeOLEDString(0, 0, "Use the");
        writeOLEDString(0, 16, "select key");
        writeOLEDString(0, 32, "to calibrate");

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
        writeOLEDString(0, 0, "Calibration");
        writeOLEDString(0, 16, "OK!");
        delay(1000);

        // Start displaying the motor data
        clearOLED();
        writeOLEDString(0, 0,  "RPM: 0.00");
        writeOLEDString(0, 16, "Err: 0.00");
        writeOLEDString(0, 32, "Deg: 0.00");

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
    /*

    EXTIX_Init();
    NVIC_EnableIRQ(EXTI1_IRQn);         //
    UART_Configuration(USART1, UART1_DEFAULT_BAUDRATE);     //
    CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_5tq, 6, CAN_Mode_LoopBack);//
    res = CAN1_Send_Msg(canbufTx, 8);                      //
    delayMs(100);
    if(res){                                            //
        printf("CAN1 Transport fail!\r\n");
    }
    else{
        printf("CAN1 Transport OK!\r\n");
    }
    TIM2_Cap_Init(0xFFFF,0);          //

     while(1) {
        #ifdef CLOSED_LOOP
            if(enableModeFlag){
                if(getEnablePin()) {
                    restart_init();
                }
                else{
                    resetStatusFlag++;
                    enmode = 0;
                }
            }
            else {
                if(!getEnablePin()){
                    restart_init();
                }
                else{
                    resetStatusFlag++;
                    enmode=0; //
                }
            }
            if(resetStatusFlag == 1){       //
                enmode = 0;
                resetStatusFlag++;           //1++
                PID_Cal_value_init();           //

                wap1 = 0;
                wap2 = 0;
                dataUpdateFlag = true;
            }
            else{
                if(resetStatusFlag > 3)
                    resetStatusFlag--;
            }

            usart_Receive_Process();                        //
            data_Process();
            test_uart();

            if(frameErrorFlag){
                frameErrorFlag =0;
                USART1_SendStr("Frame Err!\r\n");
            }
            if(flashStoreFlag){
                flashStoreFlag = false;

                //USART_Cmd(USART1, DISABLE);
                USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
                USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);

                flashWrite(DATA_STORE_ADDRESS, table1);//

                //USART_Cmd(USART1, ENABLE);
                USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
                USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
                //Reset_status_flag=1;
                //restart_init();
            }
            KeyScan();                                      //
            Oled_display();                                 //
            Motor_data_dis();                               //
            //TIM4_IRQHandler();
    #endif
#if 0
        if(KEY_Select==0){
            delay_ms(10);
            if(KEY_Select==0){
                if(k3_flag == 0){
                    k3_flag =1;
                    led1=!led1;			//
                    res=CAN1_Send_Msg(canbufTx,8);//
                    if(res)
                        printf("CAN1 Transport fail!\r\n");
                    else
                        printf("OK!\r\n");
                }
            }
        }
        else if(KEY_Confirm==0){
            delay_ms(10);
            if(KEY_Confirm==0){
                if(k4_flag == 0){
                    k4_flag =1;
                    mode=!mode;
                    CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_5tq,6,mode);	//

                    if(mode==0)//
                    {
                        printf("Nnormal Mode \r\n");
                    }else //
                    {
                        printf("LoopBack Mode\r\n");
                    }
                    led1=1;			//off
                    delay_ms(200);
                    led1=0;			//on
                    delay_ms(200);
                    led1=1;			//off
                    delay_ms(200);
                    led1=0;			//on
                    delay_ms(200);
                }
            }
        }
        else{
            k3_flag = 0;
            k4_flag = 0;
        }
        key=CAN1_Receive_Msg(canbufRx);
        if(key)//
        {
            printf("CAN Receive: ");
             for(i=0;i<key;i++){
                printf("%x ",canbufRx[i]);
             }
            printf("\r\n");
        }
        key=0;
#endif
    }
*/
}
/*
//
void restart_init(void)
{
    //
    if(resetStatusFlag !=0 ){
        CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);
        PID_Cal_value_init();           //
        SET_BIT(TIM2->CR1, TIM_CR1_CEN);
    }
    enmode=1;
    resetStatusFlag=0;
}

#ifdef TEST_FLASH
void flash_test(void)
{
    static char t=0;
    STMFLASH_Write(Data_Store_Arrdess,table,16);
    delay_ms(10);
    FLASH_Unlock();						//
    STMFLASH_Read(Data_Store_Arrdess,table1,16);
    FLASH_Lock();//
     printf("flash Read: ");
    for(t=0;t<16;t++){
        printf("%c ",table1[t]);
    }
    while(1);
}
#endif

*/
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


// Sets up the motor update timer
void setupMotorTimers() {

    // Setup the step pin
    pinMode(STEP_PIN, INPUT_PULLDOWN);

    // Attach the interupt to the step pin
    attachInterrupt(digitalPinToInterrupt(STEP_PIN), stepMotor, CHANGE);

    // Setup the timer for steps
    stepCorrectionTimer -> pause();
    stepCorrectionTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the interrupt
    stepCorrectionTimer -> setOverflow(CORRECTION_STEP_FREQ  /*motor.speedToHz(motor.compute(getEncoderAngle()))*/, HERTZ_FORMAT);
    stepCorrectionTimer -> attachInterrupt(stepMotor);

    // Setup the timer for step intervals
    stepSkipCheckTimer -> pause();
    stepSkipCheckTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the interrupt
    stepSkipCheckTimer -> setOverflow(STEPPER_SKIP_CHECK_RATE, HERTZ_FORMAT);
    stepSkipCheckTimer -> attachInterrupt(stepSkipCheckInterrupt);
    stepSkipCheckTimer -> resume();
}


// Need to declare a function to increment the motor for the step interrupt
void stepMotor() {
    motor.step(correctionStepDir, false);
}


// Update the interval on the step timer
void stepSkipCheckInterrupt() {

    // Check to see the state of the enable pin
    if (digitalRead(ENABLE_PIN) != motor.getEnableInversion()) {

        // The enable pin is off, the motor should be disabled
        motor.disable();
    }
    else {

        // Enable the motor (just energizes the coils to hold it in position)
        motor.enable();

        // Calculate the angular deviation
        float angularDeviation = getEncoderAngle() - motor.desiredAngle;

        // Check to make sure that the motor is in range (it hasn't skipped steps)
        if (abs(angularDeviation) > (motor.getFullStepAngle() / motor.getMicrostepping())) {

            // Set the stepper to move in the correct direction
            if (angularDeviation > (motor.getFullStepAngle() / motor.getMicrostepping())) {

                // Motor is at a position larger than the desired one
                correctionStepDir = CLOCKWISE;
            }
            else {
                // Motor is at a position larger than the desired one
                correctionStepDir = COUNTER_CLOCKWISE;
            }

            // Timer needs to be enabled, set it to the computed amount
            stepCorrectionTimer -> resume();
            stepCorrectionTimer -> setOverflow(CORRECTION_STEP_FREQ, HERTZ_FORMAT);
        }
        else {
            // Disable the timer for step correction, no need for it to run as we're in bounds
            stepCorrectionTimer -> pause();
        }
    }
}

// ! Only here for testing
void blink() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}