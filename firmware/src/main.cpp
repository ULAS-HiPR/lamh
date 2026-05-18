#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#elif F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#elif defined(LINUX)
#include "platform/linux.h"
#endif
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <data.h>
#include <sensor.h>
#include <I2C/I2C_STM.h>
#include <CAN/CAN_Handler.h>
#include <Servo/servo.h>
#include <Servo/PCA9685.h>
#include <servo_debug.h> //to get rid of later

#include "tasks/canards_controller.h"
#include "tasks/CAN_task.h"

#include <stdint.h>

void SystemClock_Config(void);
void Error_Handler(void);

const osMessageQueueAttr_t canQueue_attributes = {
  .name = "canQueue"
};

const osMessageQueueAttr_t loggingQueue_attributes = {
  .name = "loggingQueue"
};

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  osKernelInitialize();

  //bool init_status = true;

#ifndef LINUX
  I2C_Handler* i2c = new I2C_STM(&hi2c1, SERVO_ADDR << 1);
#else
  I2C_Handler* i2c = new I2C_Mock();
#endif
  Servo* servo = new PCA9685Servo(*i2c, 0, SERVO_ADDR << 1);
    // create servo on PCA9685 channel 0
    // change the second argument if your servo is on a different channel

  //I2C_Handler* i2c_handler = new I2C_STM(&hi2c1, 0x68 << 1);
  //SPI_Handler* spi_handler = new SPI_STM(&hspi1, GPIOA, GPIO_PIN_4);

  //Flash* flash_memory = new MX25L128();

  //Servo* servo = new Servo(i2c_handler, SERVO_PWM_CHANNEL);
  

  CAN_Handler* can = nullptr;
#ifdef LINUX
  g_can.init();
  can = &g_can;
#endif

  osMessageQueueId_t canQueueHandle =
    osMessageQueueNew(8, sizeof(char), &canQueue_attributes);
  osMessageQueueId_t loggingQueueHandle =
    osMessageQueueNew(8, sizeof(char), &loggingQueue_attributes);

  static task::Canards_Controller canards_controller(*servo, canQueueHandle, loggingQueueHandle);
  static task::CAN_task can_task(canQueueHandle, can);

  can_task.run();
  canards_controller.run();

  osKernelStart();

  // never get here 
  while (1)
  {
    HAL_Delay(1000);
  }
}



#ifndef LINUX
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
#endif /* LINUX */
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */