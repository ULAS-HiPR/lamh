#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"

#include <data.h>
#include <sensor.h>

#include <I2C/I2C_STM.h>
#include <SPI/SPI_STM.h>

#include <IMU/MPU6050.h>
//#include <Radio/RA01H.h>
//#include <Servo/Servo.h>

#include "tasks/state_machine.h"
#include "tasks/telemetry.h"


void SystemClock_Config(void);
void Error_Handler(void);

const osMessageQueueAttr_t telemetryQueue_attributes = {
  .name = "telemetryQueue"
};

const osMessageQueueAttr_t loggingQueue_attributes = {
  .name = "loggingQueue"
};

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  osKernelInitialize();

  //SPI_Handler* spi_handler_imu = new SPI_STM(&hspi1, IMU_CS_PORT, IMU_CS_PIN);
  //SPI_Handler* spi_handler_radio = new SPI_STM(&hspi1, RA_CS_PORT, RA_CS_PIN);
  //PWM_Handler* pwm_handler = new PWM_STM(&htim1, SERVO_PWM_CHANNEL);
  I2C_Handler* i2c_handler = new I2C_STM(&hi2c1, 0x68 << 1);

  IMU* imu = new MPU6050(*i2c_handler);
  //Radio* radio = new RA01H(*spi_handler_radio, EXTRA_PIN_D0);
  //Servo* servo = new Servo();
  //pwm_handler, SERVO_PWM_CHANNEL);


  osMessageQueueId_t telemetryQueueHandle =
    osMessageQueueNew(16, sizeof(task::State_Machine::State), &telemetryQueue_attributes);

  osMessageQueueId_t loggingQueueHandle =
    osMessageQueueNew(16, sizeof(uint32_t), &loggingQueue_attributes);

  static task::State_Machine state_machine(*imu, telemetryQueueHandle, loggingQueueHandle);
    //servo, telemetryQueueHandle, loggingQueueHandle

  static task::Telemetry telem(telemetryQueueHandle);
    //radio, telemetryQueueHandle);

  state_machine.run();
  telem.run();
  
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // stay here
  }
}