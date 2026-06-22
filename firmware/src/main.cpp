#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "platform/error_handler.h"
#include "platform/hal_time.h"

#include <data.h>
#include <sensor.h>
#include <I2C/I2C_STM.h>
#include <Servo/servo.h>
#include <Servo/PCA9685.h>
#include <CAN/CAN_Handler.h>
#if F4
#include <CAN/CAN_Mock.h>
#elif F0
#include <CAN/CAN_STM.h>
#endif

#include "tasks/canards_controller.h"
#include "tasks/CAN_task.h"

#include <stdint.h>


void SystemClock_Config(void);


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

  MX_GPIO_Init();
  MX_I2C1_Init();

  osKernelInitialize();
  printf("BOOT 1\n");

  //bool init_status = true;
  //shifting occuring in the I2C_STM class, so no need to shift here -> shouldnt do that imo
  I2C_Handler* i2c = new I2C_STM(&hi2c1, SERVO_ADDR);
  Servo* servo = new PCA9685Servo(*i2c, 0, SERVO_ADDR);

  #if F4
   CAN_Handler* canbus = new CAN_MOCK();
  #elif F0
    MX_CAN_Init();
    CAN_Handler* canbus = new CAN_STM(&hcan);
    bool can_init_status = canbus->init();
  #endif

  osMessageQueueId_t canInQueueHandle =
    osMessageQueueNew(8, sizeof(flight_data), &canQueue_attributes);
  osMessageQueueId_t canOutQueueHandle =
    osMessageQueueNew(8, sizeof(canards_raw), &loggingQueue_attributes);

  static task::Canards_Controller canards_controller(*servo, ACTIVE_PIN, ACTIVE_GPIO_PORT, canInQueueHandle, canOutQueueHandle);
  static task::CAN_task can_task(*canbus, canInQueueHandle, canOutQueueHandle);

  
  can_task.run();
  canards_controller.run();

  osKernelStart();

  // never get here 
  while (1)
  {
    HAL_Delay(1000);
  }
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
        Error_Handler();
    }
}

volatile uint32_t fault_pc = 0;
volatile uint32_t fault_lr = 0;
volatile uint32_t fault_sp = 0;

void HardFault_Handler(void)
{
    __asm volatile("mrs r0, MSP\n"
                   "b hard_fault_handler_c");
}

void hard_fault_handler_c(uint32_t *stack)
{
    fault_sp = (uint32_t)stack;
    fault_lr = stack[5];
    fault_pc = stack[6];

    while (1);
}

#ifdef F0
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
        HAL_IncTick();
    }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
#endif // F0
