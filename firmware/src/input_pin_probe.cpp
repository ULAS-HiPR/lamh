#include "stm32f0xx_hal.h"
#include <stdint.h>

#define INPUT_PROBE_PORT GPIOB
#define INPUT_PROBE_PIN GPIO_PIN_2

#define INPUT_PROBE_MAGIC 0x50423249u

typedef struct {
    uint32_t magic;
    uint32_t ticks;
    uint32_t samples;
    uint32_t high_count;
    uint32_t low_count;
    uint32_t rising_edges;
    uint32_t falling_edges;
    uint8_t raw_level;
    uint8_t active_low;
    uint8_t reserved[2];
} InputProbeStatus;

volatile InputProbeStatus input_probe = {
    INPUT_PROBE_MAGIC,
};

void SystemClock_Config(void);
void Error_Handler(void);

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void MX_InputProbe_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = INPUT_PROBE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(INPUT_PROBE_PORT, &GPIO_InitStruct);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_InputProbe_Init();

    GPIO_PinState previous = HAL_GPIO_ReadPin(INPUT_PROBE_PORT, INPUT_PROBE_PIN);

    while (1) {
        GPIO_PinState current = HAL_GPIO_ReadPin(INPUT_PROBE_PORT, INPUT_PROBE_PIN);

        input_probe.ticks = HAL_GetTick();
        input_probe.samples++;
        input_probe.raw_level = current == GPIO_PIN_SET ? 1 : 0;
        input_probe.active_low = current == GPIO_PIN_RESET ? 1 : 0;

        if (current == GPIO_PIN_SET) {
            input_probe.high_count++;
        } else {
            input_probe.low_count++;
        }

        if (current != previous) {
            if (current == GPIO_PIN_SET) {
                input_probe.rising_edges++;
            } else {
                input_probe.falling_edges++;
            }
            previous = current;
        }

        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
