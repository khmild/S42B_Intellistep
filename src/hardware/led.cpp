#include "led.h"

// Function to setup the red LED
void initLED() {

    // Initialization structure
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Setup pin C13
    GPIO_InitStructure.Pin = GPIO_PIN_13;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}
