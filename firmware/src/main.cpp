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
#include <servo_debug.h>
#include <stdint.h>

void SystemClock_Config(void);
void Error_Handler(void);

namespace {

uint8_t find_pca9685_address(I2C_HandleTypeDef* hi2c) {
    servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_START;
    servo_debug.pca9685_found = 0;

    for (uint16_t address = 0x40; address <= 0x7F; ++address) {
        if (address == 0x70) {
            continue; // avoid the default PCA9685 all-call address
        }

        servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_PROBE;
        servo_debug.scan_attempts++;
        servo_debug.scan_last_address = static_cast<uint8_t>(address);
        servo_debug.i2c_last_op = SERVO_DEBUG_I2C_OP_READY;
        servo_debug.i2c_last_address = static_cast<uint8_t>(address);

        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, static_cast<uint16_t>(address << 1), 2, 10);
        servo_debug.scan_last_status = static_cast<uint32_t>(status);
        servo_debug.scan_last_error = HAL_I2C_GetError(hi2c);
        servo_debug.i2c_last_status = servo_debug.scan_last_status;
        servo_debug.i2c_last_error = servo_debug.scan_last_error;

        if (status == HAL_OK) {
            servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_FOUND;
            servo_debug.pca9685_found = 1;
            servo_debug.pca9685_address = static_cast<uint8_t>(address);
            return static_cast<uint8_t>(address);
        }
    }

    servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_NOT_FOUND;
    servo_debug.pca9685_address = PCA9685Servo::DEFAULT_ADDRESS;
    return PCA9685Servo::DEFAULT_ADDRESS;
}

} // namespace

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
            servo_debug.stage = SERVO_DEBUG_STAGE_TASK_START;

            // initialise the PCA9685 chip
            _servo->init();

            // start at centre so servo doesn't snap on boot
            _servo->set_position(90);
            osDelay(500);

            for (;;) {
                // sweep 0 → 180 degrees
                for (int16_t angle = 0; angle <= 180; angle += 2) {
                    _servo->set_position(angle);
                    osDelay(15);  // 15ms per step — adjust for sweep speed
                }

                // sweep 180 → 0 degrees
                for (int16_t angle = 180; angle >= 0; angle -= 2) {
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
    servo_debug.magic = SERVO_DEBUG_MAGIC;
    servo_debug.stage = SERVO_DEBUG_STAGE_BOOT;
    HAL_Init();
    SystemClock_Config();
    servo_debug.stage = SERVO_DEBUG_STAGE_CLOCK_READY;
    MX_I2C1_Init();
    servo_debug.stage = SERVO_DEBUG_STAGE_I2C_READY;
    osKernelInitialize();

    // create I2C handler for the active STM platform
    uint8_t pca9685_address = find_pca9685_address(&hi2c1);
    I2C_Handler* i2c = new I2C_STM(&hi2c1, pca9685_address);

    // create servo on PCA9685 channel 0
    // change the second argument if your servo is on a different channel
    PCA9685Servo* servo = new PCA9685Servo(*i2c, 0, pca9685_address);

    // create and start the sweep task
    static task::ServoSweep sweep_task(servo);
    sweep_task.run();

    servo_debug.stage = SERVO_DEBUG_STAGE_RTOS_START;
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
#if defined(TIM6)
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
#else
    (void)htim;
#endif
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
