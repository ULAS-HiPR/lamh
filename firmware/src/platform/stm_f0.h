#ifdef F0
#ifndef STM_F072xB_H
#define STM_F072xB_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_gpio.h"

#define I2C_SCL_PIN GPIO_PIN_8
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_GPIO_PORT GPIOB 
#define I2C_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() 

// Extern handles for use by I2C/SPI handlers
extern I2C_HandleTypeDef hi2c1;

// Initialization functions
void MX_I2C1_Init();

#endif // STM_F4_H
#endif // F4