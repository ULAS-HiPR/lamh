#pragma once
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include <cstdio>
#include <data.h>

//#include <IMU/IMU.h>
//#include <Servo/Servo.h>

namespace task{
class Canards_Controller {
    public:
        Canards_Controller(
                        //Servo& servo,
                      osMessageQueueId_t can_queue,
                      osMessageQueueId_t logger_queue)
            : 
              can_queue_(can_queue),
              logger_queue_(logger_queue),
              taskHandle_(nullptr){};
              
        void run();

    private:
        void StartCanardsController();
        static void StartCanardsControllerEntry(void *argument);
        void run_canards_controller(uint16_t sampleCount, imu_data imu, baro_data baro);
        void stop_action();
        bool safety_check(int state, imu_data imu);
        void get_up_direction();

        //Servo& servo_;
        osMessageQueueId_t can_queue_;
        osMessageQueueId_t logger_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "CanardsController",
            0,
            nullptr,
            0,
            nullptr,
            512 * 1,        // 2 KB stack
            osPriorityHigh,
            0,
            0
        };

        uint16_t UNROLL_POSITION = 1000;
        uint16_t UNROLL_HOLD_POSITION = 1200;
        uint16_t ROLL_POSITION = 2000;
        uint16_t ROLL_HOLD_POSITION = 1800;
        uint16_t STOP_POSITION = 1500;
    };

}

