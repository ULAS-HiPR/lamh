#pragma once
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>

#define EIGEN_NO_DEBUG
#define EIGEN_MPL2_ONLY
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include <Eigen/Dense>

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
        void run_canards_controller();
        void stop_action();

        Eigen::VectorXd x;
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
            512 * 4,        // 2 KB stack
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

