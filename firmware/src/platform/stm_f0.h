#ifdef F0
#ifndef STM_F072xB_H
#define STM_F072xB_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#define LED_PIN GPIO_PIN_5 
#define LED_GPIO_PORT GPIOA 
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() 

#define I2C_SCL_PIN GPIO_PIN_8
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_GPIO_PORT GPIOB 
#define I2C_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE() 

#define SERVO_ADDR 0x40 // default PCA9685 I2C address

#define CAN_RX_PIN GPIO_PIN_11
#define CAN_TX_PIN GPIO_PIN_12
#define CAN_GPIO_PORT GPIOA
#define CAN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define ACTIVE_PIN GPIO_PIN_2
#define ACTIVE_GPIO_PORT GPIOB
#define ACTIVE_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

// Extern handles for use by I2C/SPI handlers
extern I2C_HandleTypeDef hi2c1;

extern CAN_HandleTypeDef hcan;

// Initialization functions
void MX_I2C1_Init();
void MX_CAN_Init();
void MX_GPIO_Init();

#endif // STM_F072xB_H
#endif // F0
