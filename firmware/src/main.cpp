#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"

#include <I2C/I2C_STM.h>
#include <Servo/PCA9685.h>

void SystemClock_Config(void);
void Error_Handler(void);

// ============================================================
// Servo sweep task — mirrors the same pattern as default task
// ============================================================
namespace task {

class ServoSweep {
    public:
        ServoSweep(PCA9685Servo* servo) : _servo(servo) {}

        void run() {
            _taskHandle = osThreadNew(
                &ServoSweep::entry,  // FreeRTOS entry point
                this,                // pass this object as argument
                &_attributes
            );
        }

    private:
        static void entry(void* argument) {
            auto* self = static_cast<ServoSweep*>(argument);
            if (self) self->loop();
        }

        void loop() {
            // initialise the PCA9685 chip
            _servo->init();

            // start at centre so servo doesn't snap on boot
            _servo->set_position(90);
            osDelay(500);

            for (;;) {
                // sweep 0 → 180 degrees
                for (int8_t angle = 0; angle <= 180; angle += 2) {
                    _servo->set_position(angle);
                    osDelay(15);  // 15ms per step — adjust for sweep speed
                }

                // sweep 180 → 0 degrees
                for (int8_t angle = 180; angle >= 0; angle -= 2) {
                    _servo->set_position(angle);
                    osDelay(15);
                }
            }
        }

        PCA9685Servo*   _servo;
        osThreadId_t    _taskHandle;

        const osThreadAttr_t _attributes = {
            "servoSweep",       // name (visible in debugger)
            0,
            nullptr,
            0,
            nullptr,
            256 * 4,            // stack size — 1KB
            osPriorityNormal,   // priority
            0,
            0
        };
};

} // namespace task

// ============================================================
// Main
// ============================================================
int main(void) {
    HAL_Init();
    SystemClock_Config();
    osKernelInitialize();

    // create I2C handler — hi2c1 comes from platform/stm_f0.h
    // 0x40 << 1 because HAL expects the address pre-shifted
    I2C_Handler* i2c = new I2C_STM(&hi2c1, 0x40 << 1);

    // create servo on PCA9685 channel 0
    // change the second argument if your servo is on a different channel
    PCA9685Servo* servo = new PCA9685Servo(*i2c, 0);

    // create and start the sweep task
    static task::ServoSweep sweep_task(servo);
    sweep_task.run();

    osKernelStart();

    // never get here
    while (1) {
        HAL_Delay(1000);
    }
}

// ============================================================
// System Clock — copied from your existing main.cpp
// ============================================================
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}